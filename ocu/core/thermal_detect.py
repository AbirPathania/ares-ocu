"""Finding warm and cold things in a thermal picture.

The thermal imager (thermal_scene.py) produces what a sensor would report: an apparent temperature in every
pixel. This reads that picture the way a detector does. It does not look at what is in the world; it looks at the
numbers.

  1. Take each pixel against its surroundings. The background is the local mean of the ground and objects the
     frame shows (not the sky), so the slow slope of a hillside warming in the sun is not a detection and a
     body standing on it is.
  2. A pixel counts when it is further from that background than the picture's own noise allows: three and a
     half times the robust spread of what is left (median absolute deviation), and never less than a degree.
  3. Connected pixels are one blob. A blob needs a few pixels to be a blob at all (the detection criterion), and
     more to be told from other things (the recognition criterion, after Johnson: a recognition needs roughly
     four pixels across the smaller side of the target, a detection about one and a half).
  4. A blob's size in metres comes from the pixels' own footprint on the ground (the imager records what each
     pixel spans), and its class from that size and its sign: tall, narrow and warm is a person; long, low and
     warm an animal; large and strongly different from the ground, warm or cold, a vehicle; lying on the
     ground and small, a ground anomaly, which is where something buried or recently disturbed shows.
  5. Its place on the ground is where its lowest pixels met it.

Nothing here knows what is really out there. A hot rock at midday is a warm blob, and is called what its size
makes it: an object, which is not reported. The confidence comes from how far above the noise it is and how many
pixels it has.

Recognition limits are the standard ones and not measurements of this sensor; the sizes that separate a person
from a deer from a truck are this build's choices, as are the thresholds. See DECISIONS.md, "Thermal detection".

No Qt import belongs in this file.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy import ndimage

#: How far from the background a pixel has to be, in robust standard deviations of the picture, and the least
#: it may be whatever the noise says.
SIGMA_GATE = 3.5
MIN_CONTRAST_K = 1.0

#: Smoothing window, in pixels, for the local background: wider than anything the camera is looking for.
BACKGROUND_WINDOW_PX = {"rover": 41, "scout": 21}

#: A blob smaller than this is noise: the detection criterion, a little over a pixel across.
MIN_BLOB_PIXELS = 3
#: A blob is split at half its own peak only when the peak is this many gates out.
SPLIT_PEAK_GATES = 2.0
#: Pixels across the smaller side of a blob for it to be told apart from others: the recognition criterion.
RECOGNITION_PIXELS = 5
#: A body that is told from clutter has at least this many pixels: a handful of warm pixels is a speck of ground.
MIN_BODY_PIXELS = 40

#: Sizes in metres (width, height, as seen), chosen. A person's outline is narrow and tall; the shoulders of
#: someone with a pack are up to a metre across. An animal is longer than it is tall. A vehicle is big.
PERSON_WIDTH_M = (0.30, 1.30)
PERSON_HEIGHT_M = (1.20, 2.30)
ANIMAL_WIDTH_M = (0.50, 3.00)
ANIMAL_HEIGHT_M = (0.60, 2.30)
VEHICLE_WIDTH_M = 1.80
VEHICLE_HEIGHT_M = 1.50
VEHICLE_MAX_WIDTH_M = 9.0
VEHICLE_MAX_HEIGHT_M = 4.5
#: A body fills most of its bounding box; a seam of warm ground, a gully edge or a row of rocks does not.
MIN_FILL = 0.3
#: A person or an animal fills more of its bounding box than a vehicle does (a vehicle is split by its lit and shaded
#: faces).
MIN_BODY_FILL = 0.4
TOO_TALL_M = 3.5  # a trunk, not a body

#: How warm a person or animal is against the ground it stands on, at the least. Skin and clothing are well above
#: the air; a rock in the sun can be as warm as the ground and no warmer than a vehicle.
BODY_MIN_WARM_K = 3.0
#: Against the ring just round it, the least a body stands out: less than against the picture, because a body on warm
#: ground is warmer than the ground beside it by less than it is warmer than the picture's median.
BODY_MIN_LOCAL_K = 2.5
VEHICLE_MIN_CONTRAST_K = 2.0

#: Largest a ground anomaly is taken to be, metres across: a disturbance a buried object makes, not a field.
ANOMALY_MAX_M = 4.0
#: At least this share of a blob's pixels must be bare ground for it to be a ground anomaly.
ANOMALY_GROUND_SHARE = 0.7

MAX_CONFIDENCE = 0.93


@dataclass(frozen=True)
class ThermalContact:
    kind: str  # "person", "animal", "vehicle", "anomaly", or "object" (seen, not classed, not reported)
    x: float
    y: float
    range_m: float
    width_m: float
    height_m: float
    contrast_k: float  # mean apparent difference from the background, signed
    pixels: int
    confidence: float
    box: tuple[float, float, float, float]  # (u0, v0, u1, v1), fractions of the frame
    entity: int  # the body the blob mostly lies on in the imager's record, -1 for none. Never used to decide anything.

    @property
    def reportable(self) -> bool:
        return self.kind != "object"


def _background(temp: np.ndarray, mask: np.ndarray, window: int) -> np.ndarray:
    """What the picture would show with nothing standing in it: for every row, the median of what the row shows.

    Rows are the right unit because a camera looking across ground sees each row at about one distance, so the
    row's median is the ground there, and a body that fills a small share of a row cannot move it. A mean over a
    window (the first thing tried) does move: a warm body raises its own background and leaves a cold halo of
    false blobs round it. The medians are smoothed down the picture so that the background is a gentle slope, and
    the window is the smoothing length.
    """
    rows = temp.shape[0]
    profile = np.full(rows, np.nan)
    for r in range(rows):
        values = temp[r][mask[r]]
        if values.size >= 8:
            profile[r] = np.median(values)
    good = np.isfinite(profile)
    if not good.any():
        return temp
    index = np.arange(rows)
    profile = np.interp(index, index[good], profile[good])
    profile = ndimage.gaussian_filter1d(profile, sigma=max(window / 8.0, 1.0), mode="nearest")
    return np.broadcast_to(profile[:, None], temp.shape).copy()


def noise_floor(residual: np.ndarray, mask: np.ndarray) -> float:
    """The picture's own spread, robust against what is being looked for: 1.4826 x the median absolute deviation."""
    values = residual[mask]
    if values.size < 16:
        return MIN_CONTRAST_K
    return float(1.4826 * np.median(np.abs(values - np.median(values))))


def detect(frame, sensor_kind: str = "scout") -> list[ThermalContact]:
    """Every warm or cold blob in a thermal frame, classed by size and sign. See the module docstring."""
    temp = frame.temp_c.astype(np.float64)
    painted = frame.painted if frame.painted is not None else np.ones(temp.shape, dtype=bool)
    if painted.sum() < 64:
        return []
    height, width = temp.shape
    background = _background(temp, painted, BACKGROUND_WINDOW_PX[sensor_kind])
    residual = np.where(painted, temp - background, 0.0)
    sigma = noise_floor(residual, painted)
    gate = max(SIGMA_GATE * sigma, MIN_CONTRAST_K)

    warm = residual > gate
    cold = residual < -gate
    contacts: list[ThermalContact] = []
    for mask, sign in ((warm, 1.0), (cold, -1.0)):
        if not mask.any():
            continue
        for rows, cols in _blobs(mask):
            parent = _describe(frame, residual, sigma, rows, cols, rows.size, sign, width, height)
            strong = _strongest(residual * sign, rows, cols, gate)
            if strong is not None:
                # a body on warm ground is one blob at the clutter's threshold; at half its own peak it stands alone
                inner = [
                    c for c in (_describe(frame, residual, sigma, r, k, r.size, sign, width, height) for r, k in _blobs(strong))
                    if c is not None and c.reportable
                ]
                if inner:
                    contacts.extend(inner)
                    continue
            if parent is not None:
                contacts.append(parent)
    return contacts


def _blobs(mask: np.ndarray):
    """(rows, cols) of every connected group of at least MIN_BLOB_PIXELS true pixels. See detect() for why they are closed first."""
    # a body's pixels are not all as far from the ground as each other (a sunlit side, a shaded side), so join
    # what is within a couple of pixels before counting blobs
    mask = ndimage.binary_closing(mask, structure=np.ones((3, 3)), iterations=2)
    labels, _ = ndimage.label(mask, structure=np.ones((3, 3)))
    for index, region in enumerate(ndimage.find_objects(labels), start=1):
        if region is None:
            continue
        rows, cols = np.nonzero(labels[region] == index)
        if rows.size < MIN_BLOB_PIXELS:
            continue
        yield rows + region[0].start, cols + region[1].start


def _strongest(signed: np.ndarray, rows: np.ndarray, cols: np.ndarray, gate: float) -> np.ndarray | None:
    """The part of a blob at least half as far from the background as its own 90th percentile, if that is well clear
    of the gate and is a smaller part than the whole; otherwise None."""
    values = signed[rows, cols]
    peak = float(np.percentile(values, 90.0))
    if peak < SPLIT_PEAK_GATES * gate:
        return None
    keep = values > 0.5 * peak
    if keep.all() or keep.sum() < MIN_BLOB_PIXELS:
        return None
    inner = np.zeros(signed.shape, dtype=bool)
    inner[rows[keep], cols[keep]] = True
    return inner


def _describe(frame, residual, sigma, rows, cols, pixels, sign, width, height) -> ThermalContact | None:
    slant = frame.slant_m[rows, cols]
    finite = np.isfinite(slant)
    if not finite.any():
        return None
    # What one pixel spans on the thing it sees: the range to the nearest surface times the pixel's angle. The
    # imager's own across_m/along_m are measured to the ground, which is infinite for a ray that passes over the
    # horizon on its way to a body, so they are not used here.
    across = float(np.mean(slant[finite])) * frame.sensor.ifov_rad
    along = float(np.mean(frame.along_m[rows, cols][np.isfinite(frame.along_m[rows, cols])] if np.isfinite(frame.along_m[rows, cols]).any() else [across]))
    box_w, box_h = int(cols.max() - cols.min() + 1), int(rows.max() - rows.min() + 1)
    width_m, height_px_m = box_w * across, box_h * across
    ground_share = float(np.mean(frame.ground[rows, cols]))
    contrast = float(np.mean(residual[rows, cols]))
    local = contrast - _surround(residual, frame.painted, rows, cols, box_w, box_h)
    snr = abs(contrast) / max(sigma, 1e-6)
    recognised = min(box_w, box_h) >= RECOGNITION_PIXELS

    # where it is: the nearest surface its pixels see, which for a body is its own face and for ground is the ground
    # the ray met (the imager's hx and hy are the ground behind a body, and are no use for one)
    sx = frame.sx if frame.sx is not None else frame.hx
    sy = frame.sy if frame.sy is not None else frame.hy
    px, py = sx[rows, cols], sy[rows, cols]
    good = np.isfinite(px) & np.isfinite(py)
    if not good.any():
        return None
    x, y = float(np.mean(px[good])), float(np.mean(py[good]))
    range_m = float(np.mean(slant[finite]))

    if ground_share >= ANOMALY_GROUND_SHARE:
        extent = max(width_m, box_h * along)
        kind = "anomaly" if extent <= ANOMALY_MAX_M else "object"
        height_m = 0.0
    else:
        height_m = height_px_m
        fill = pixels / float(box_w * box_h)
        kind = _classify_body(width_m, height_m, contrast, local, fill, pixels, recognised) if fill >= MIN_FILL else "object"

    size_term = min(1.0, pixels / 12.0)
    confidence = 0.30 + 0.63 * (1.0 - math.exp(-snr / 7.0)) * (0.5 + 0.5 * size_term)
    if kind == "anomaly":
        confidence = 0.12 + 0.40 * (1.0 - math.exp(-snr / 6.0)) * size_term  # a disturbance proves little by itself
    box = (float(cols.min()) / width, float(rows.min()) / height, float(cols.max() + 1) / width, float(rows.max() + 1) / height)
    entity = int(np.bincount(frame.entity[rows, cols][frame.entity[rows, cols] >= 0]).argmax()) if (frame.entity[rows, cols] >= 0).any() else -1
    return ThermalContact(kind, x, y, range_m, width_m, height_m, contrast, pixels, min(confidence, MAX_CONFIDENCE), box, entity)


def _surround(residual: np.ndarray, painted, rows: np.ndarray, cols: np.ndarray, box_w: int, box_h: int) -> float:
    """What the picture shows in a ring round a blob: the median residual of the pixels just outside it.

    A body stands out from what is directly round it. A warm patch of ground on a sunlit slope does not: the ground
    beside it is nearly as warm, so the blob's contrast against its surround is small even where its contrast
    against the whole picture is large.
    """
    margin = max(3, int(0.5 * max(box_w, box_h)))
    height, width = residual.shape
    r0, r1 = max(int(rows.min()) - margin, 0), min(int(rows.max()) + margin + 1, height)
    c0, c1 = max(int(cols.min()) - margin, 0), min(int(cols.max()) + margin + 1, width)
    blob = np.zeros((r1 - r0, c1 - c0), dtype=bool)
    blob[rows - r0, cols - c0] = True
    ring = ndimage.binary_dilation(blob, structure=np.ones((3, 3)), iterations=margin) & ~ndimage.binary_dilation(blob, structure=np.ones((3, 3)), iterations=1)
    valid = ring & (painted[r0:r1, c0:c1] if painted is not None else True)
    values = residual[r0:r1, c0:c1][valid]
    return float(np.median(values)) if values.size >= 8 else 0.0


def _classify_body(width_m: float, height_m: float, contrast: float, local: float, fill: float, pixels: int, recognised: bool) -> str:
    if height_m > TOO_TALL_M or not recognised:
        return "object"
    aspect = height_m / max(width_m, 1e-6)
    if contrast >= BODY_MIN_WARM_K and local >= BODY_MIN_LOCAL_K and fill >= MIN_BODY_FILL and pixels >= MIN_BODY_PIXELS:
        if PERSON_WIDTH_M[0] <= width_m <= PERSON_WIDTH_M[1] and PERSON_HEIGHT_M[0] <= height_m <= PERSON_HEIGHT_M[1] and aspect >= 1.3:
            return "person"
        if ANIMAL_WIDTH_M[0] <= width_m <= ANIMAL_WIDTH_M[1] and ANIMAL_HEIGHT_M[0] <= height_m <= ANIMAL_HEIGHT_M[1] and 0.3 <= aspect <= 1.3:
            return "animal"
    if (abs(contrast) >= VEHICLE_MIN_CONTRAST_K and VEHICLE_WIDTH_M <= width_m <= VEHICLE_MAX_WIDTH_M
            and VEHICLE_HEIGHT_M <= height_m <= VEHICLE_MAX_HEIGHT_M):
        return "vehicle"
    return "object"
