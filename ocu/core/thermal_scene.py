"""A thermal camera looking at the simulated world.

The imager resolves every pixel's ray with the same occlusion rule the visible renderer uses (`CameraRig.resolve_view`),
reads each hit's temperature from the day's field (`thermal_physics`), draws the things that stand on the ground from
their own surfaces, and then does what a camera does: band radiance with emissivity and reflected sky, atmospheric
transmission, a little optical blur, and noise equal to the sensor's noise-equivalent temperature difference.

No Qt import belongs in this file.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ocu.core import thermal_physics as tp

# --- the sensors (CANDIDATE: from ares-perception's sensor record, FLIR Boson 320 and Lepton class) ---------------------

@dataclass(frozen=True)
class ThermalSensor:
    name: str
    width: int
    height: int
    hfov_deg: float
    netd_k: float
    blur_px: float  # Gaussian sigma of the optics and detector together

    @property
    def ifov_rad(self) -> float:
        """What one pixel subtends, across the frame's width."""
        return math.radians(self.hfov_deg) / self.width

    def binned(self, factor: int) -> ThermalSensor:
        """The same sensor with factor x factor pixels averaged into one: fewer, larger pixels, and the noise of an
        average of factor squared of them."""
        return ThermalSensor(f"{self.name} (binned {factor}x{factor})", self.width // factor, self.height // factor, self.hfov_deg,
                             self.netd_k / factor, self.blur_px / factor)


ROVER_THERMAL = ThermalSensor("rover mast thermal", 320, 256, 50.0, 0.050, 0.6)
SCOUT_THERMAL = ThermalSensor("scout thermal", 160, 120, 57.0, 0.050, 0.6)
ROVER_THERMAL_HEIGHT_M = 1.5  # on the same mast as the visible forward camera
ROVER_THERMAL_PITCH_DOWN_DEG = 3.0

#: A body the camera sees from the front shows its face; from behind, hair. A surface standing upright reflects the
#: horizon and the ground more than the cold zenith: it sees air about this much colder.
UPRIGHT_REFLECTED_DEPRESSION_K = 4.0

# --- what stands on the ground ------------------------------------------------------------------------------------------

SKIN_CORE_C = 34.0
CLOTHING_COUPLING = 0.22  # how far a clothed surface sits from the air toward skin
CLOTHING_ALBEDO, CLOTHING_EMISSIVITY = 0.40, 0.95
HAIR_ALBEDO, HAIR_EMISSIVITY = 0.15, 0.97
SKIN_EMISSIVITY = 0.98
DEER_CORE_C = 38.5
COAT_COUPLING, FACE_COUPLING, EAR_COUPLING, LEG_COUPLING = 0.12, 0.45, 0.25, 0.08
COAT_ALBEDO, COAT_EMISSIVITY = 0.45, 0.97
PAINT_ALBEDO, PAINT_EMISSIVITY = 0.30, 0.88  # olive drab
GLASS_ALBEDO, GLASS_EMISSIVITY = 0.85, 0.92
TYRE_ALBEDO, TYRE_EMISSIVITY = 0.05, 0.95
SHEET_INTERIOR_CONDUCTANCE = 5.0  # sheet metal is held toward the cabin's temperature through this
CABIN_GREENHOUSE_K_PER_W_M2 = 0.014  # a closed cabin sits this much above the air for each W/m2 of sun on a horizontal surface


@dataclass(frozen=True)
class Band:
    """One horizontal slice of a body with its own outline and its own temperature model.

    `model` is "const" (a fixed temperature, `base_c`), "coupled" (the air plus a share of the way to the body's core,
    which is `base_c`, plus the sunlight on that face) or "sheet" (thin painted metal or glass: sun, sky, air and the
    cabin behind it, in balance).
    """

    z0: float  # fraction of the height
    z1: float
    shape: str  # "ellipse" or "box"
    scale_a: float  # outline relative to the entity's length / 2 and width / 2
    scale_b: float
    forward: float  # centre's offset along the heading, in metres
    emissivity: float
    name: str
    model: str = "const"
    base_c: float = 0.0
    albedo: float = 0.0
    interior_c: float = 0.0


def _sheet_temperature_c(albedo: float, emissivity: float, flux, air_c: float, sky_lw: float, h_conv: float, interior_c: float):
    """A thin painted sheet: the sun it absorbs and the sky it sees, against what it radiates, the air it touches and the
    cabin behind it. Solved by repeated linearisation; `flux` may be an array."""
    flux = np.asarray(flux, dtype=np.float64)
    t = np.full(flux.shape, air_c + tp.KELVIN)
    sigma = tp.STEFAN_BOLTZMANN
    for _ in range(8):
        total = 4.0 * emissivity * sigma * t**3 + h_conv + SHEET_INTERIOR_CONDUCTANCE
        t = ((1.0 - albedo) * flux + emissivity * sky_lw + 3.0 * emissivity * sigma * t**4 + h_conv * (air_c + tp.KELVIN)
             + SHEET_INTERIOR_CONDUCTANCE * (interior_c + tp.KELVIN)) / total
    return t - tp.KELVIN


def band_temperature(band: Band, flux, air_c: float, sky_lw: float, h_conv: float):
    """A band's temperature where the sunlight on the face the camera sees is `flux` (array, W/m2)."""
    flux = np.asarray(flux, dtype=np.float64)
    if band.model == "coupled":
        return band.base_c + (1.0 - band.albedo) * flux / (h_conv + 4.5)
    if band.model == "sheet":
        return _sheet_temperature_c(band.albedo, band.emissivity, flux, air_c, sky_lw, h_conv, band.interior_c)
    return np.full(flux.shape, band.base_c)


def body_bands(entity, air_c: float, sun_el: float, beam: float, diffuse: float, h_conv: float, facing_camera: bool) -> list[Band]:
    """The surfaces a person, a deer or a truck presents. Their temperatures are filled in per pixel (`band_temperature`)
    from the sunlight on the face each ray meets, so the sunny side of a body is hotter than the shaded one."""
    horizontal = beam * math.sin(max(sun_el, 0.0)) + diffuse
    if entity.kind == "person":
        clothing = air_c + CLOTHING_COUPLING * (SKIN_CORE_C - air_c)
        hair = air_c + 0.3 * (SKIN_CORE_C - air_c)
        face = float(np.clip(28.0 + 0.3 * (air_c - 6.0), 24.0, 34.0))
        head = (Band(0.82, 1.00, "ellipse", 0.62, 0.50, 0.0, SKIN_EMISSIVITY, "face", "const", face) if facing_camera
                else Band(0.82, 1.00, "ellipse", 0.62, 0.50, 0.0, HAIR_EMISSIVITY, "hair", "coupled", hair, HAIR_ALBEDO))
        return [
            Band(0.00, 0.47, "ellipse", 0.60, 0.55, 0.0, CLOTHING_EMISSIVITY, "legs", "coupled", clothing - 1.5, CLOTHING_ALBEDO),
            Band(0.47, 0.82, "ellipse", 1.00, 1.00, 0.0, CLOTHING_EMISSIVITY, "torso", "coupled", clothing, CLOTHING_ALBEDO),
            head,
        ]
    if entity.kind == "animal":
        coat = air_c + COAT_COUPLING * (DEER_CORE_C - air_c)
        legs = air_c + LEG_COUPLING * (DEER_CORE_C - air_c)
        face = air_c + FACE_COUPLING * (DEER_CORE_C - air_c)
        ear = air_c + EAR_COUPLING * (DEER_CORE_C - air_c)
        head = (Band(0.72, 0.92, "ellipse", 0.28, 0.40, 0.58, COAT_EMISSIVITY, "face", "const", face) if facing_camera
                else Band(0.72, 0.92, "ellipse", 0.28, 0.40, 0.58, COAT_EMISSIVITY, "head", "coupled", coat + 1.0, COAT_ALBEDO))
        return [
            Band(0.00, 0.40, "ellipse", 0.30, 0.45, 0.0, COAT_EMISSIVITY, "legs", "coupled", legs, COAT_ALBEDO + 0.35),
            Band(0.40, 0.72, "ellipse", 1.00, 1.00, 0.0, COAT_EMISSIVITY, "coat", "coupled", coat, COAT_ALBEDO),
            head,
            Band(0.92, 1.00, "ellipse", 0.10, 0.45, 0.58, COAT_EMISSIVITY, "ears", "const", ear),
        ]
    # a vehicle: engine off, so its surfaces are whatever the sun and the sky make of painted sheet and glass, and its cabin
    # is a greenhouse that warms with the sun
    cabin = air_c + CABIN_GREENHOUSE_K_PER_W_M2 * horizontal
    return [
        Band(0.00, 0.34, "box", 1.00, 1.00, 0.0, TYRE_EMISSIVITY, "wheels", "sheet", 0.0, 1.0 - 0.3 * (1.0 - TYRE_ALBEDO), air_c),
        Band(0.34, 0.62, "box", 1.00, 1.00, 0.0, PAINT_EMISSIVITY, "body", "sheet", 0.0, PAINT_ALBEDO, cabin),
        Band(0.62, 0.88, "box", 0.45, 0.92, 0.9, GLASS_EMISSIVITY, "cab", "sheet", 0.0, GLASS_ALBEDO, cabin),
        Band(0.88, 1.00, "box", 0.40, 0.88, 0.9, PAINT_EMISSIVITY, "roof", "sheet", 0.0, PAINT_ALBEDO, cabin),
    ]


def _band_hits(origin, direction, centre_xy, heading_rad, ground_z, height, band: Band, a: float, b: float):
    """Where rays leave a band's outline first. Returns distance (inf where they miss), hit height, and the outward
    horizontal normal at the hit and whether it was the top face. Directions are unit."""
    f = np.array([math.sin(heading_rad), math.cos(heading_rad)])
    s = np.array([math.cos(heading_rad), -math.sin(heading_rad)])
    cx, cy = centre_xy[0] + f[0] * band.forward, centre_xy[1] + f[1] * band.forward
    ax, bx = a * band.scale_a, b * band.scale_b
    z_lo, z_hi = ground_z + band.z0 * height, ground_z + band.z1 * height
    ox, oy, oz = origin
    u0 = (ox - cx) * f[0] + (oy - cy) * f[1]
    v0 = (ox - cx) * s[0] + (oy - cy) * s[1]
    du = direction[:, 0] * f[0] + direction[:, 1] * f[1]
    dv = direction[:, 0] * s[0] + direction[:, 1] * s[1]
    dz = direction[:, 2]
    n = direction.shape[0]
    best = np.full(n, np.inf)
    hit_z = np.zeros(n)
    nu, nv = np.zeros(n), np.zeros(n)  # normal in the entity's frame
    cap = np.zeros(n, dtype=bool)

    def consider(t, z, normal_u, normal_v, is_cap):
        nonlocal best, hit_z, nu, nv, cap
        better = (t > 1e-6) & (t < best) & (z >= z_lo - 1e-9) & (z <= z_hi + 1e-9)
        best = np.where(better, t, best)
        hit_z = np.where(better, z, hit_z)
        nu, nv = np.where(better, normal_u, nu), np.where(better, normal_v, nv)
        cap = np.where(better, is_cap, cap)

    with np.errstate(divide="ignore", invalid="ignore"):
        t_top = (z_hi - oz) / dz
        u, v = u0 + du * t_top, v0 + dv * t_top
        inside = (np.abs(u) <= ax) & (np.abs(v) <= bx) if band.shape == "box" else (u / ax) ** 2 + (v / bx) ** 2 <= 1.0
        top_ok = (dz < 0) & (t_top > 0) & inside
        consider(np.where(top_ok, t_top, np.inf), np.full(n, z_hi), 0.0, 0.0, True)
        if band.shape == "ellipse":
            qa = (du / ax) ** 2 + (dv / bx) ** 2
            qb = 2.0 * (u0 * du / ax**2 + v0 * dv / bx**2)
            qc = (u0 / ax) ** 2 + (v0 / bx) ** 2 - 1.0
            disc = qb**2 - 4.0 * qa * qc
            root = np.sqrt(np.where(disc >= 0.0, disc, np.nan))
            for sign in (-1.0, 1.0):
                t = (-qb + sign * root) / (2.0 * qa)
                pu, pv = u0 + du * t, v0 + dv * t
                length = np.hypot(pu / ax**2, pv / bx**2)
                consider(np.where(np.isfinite(t), t, np.inf), oz + dz * t, pu / ax**2 / np.maximum(length, 1e-12), pv / bx**2 / np.maximum(length, 1e-12), False)
        else:
            for axis, (axis_o, axis_d, half) in enumerate(((u0, du, ax), (v0, dv, bx))):
                for sign in (-1.0, 1.0):
                    t = (sign * half - axis_o) / axis_d
                    u, v = u0 + du * t, v0 + dv * t
                    ok = (np.abs(u) <= ax + 1e-9) & (np.abs(v) <= bx + 1e-9) & np.isfinite(t)
                    consider(np.where(ok, t, np.inf), oz + dz * t, sign if axis == 0 else 0.0, sign if axis == 1 else 0.0, False)
    world_nx = nu * f[0] + nv * s[0]
    world_ny = nu * f[1] + nv * s[1]
    return best, hit_z, world_nx, world_ny, cap


# --- the imager --------------------------------------------------------------------------------------------------------

@dataclass
class ThermalFrame:
    temp_c: np.ndarray  # (H, W) apparent temperature the sensor reports
    ground: np.ndarray  # (H, W) bool: a ground pixel nothing stands in front of
    entity: np.ndarray  # (H, W) int: index of the body this pixel sees, -1 for none
    hx: np.ndarray  # (H, W) where the ray met the scene
    hy: np.ndarray
    slant_m: np.ndarray  # (H, W) range
    along_m: np.ndarray  # (H, W) how far one pixel stretches along the line of sight on the ground
    across_m: np.ndarray
    sensor: ThermalSensor
    painted: np.ndarray | None = None  # (H, W) bool: the ray met ground, an object or a body, not sky
    sx: np.ndarray | None = None  # (H, W) where the nearest surface is: hx, hy for ground, the body's own face for a body
    sy: np.ndarray | None = None


def _gaussian_blur(image: np.ndarray, sigma: float) -> np.ndarray:
    if sigma <= 0.0:
        return image
    radius = max(1, int(math.ceil(3.0 * sigma)))
    weights = np.exp(-0.5 * (np.arange(-radius, radius + 1) / sigma) ** 2)
    weights /= weights.sum()
    out = image
    for axis in (0, 1):
        padded = np.pad(out, [(radius, radius) if a == axis else (0, 0) for a in range(2)], mode="edge")
        acc = np.zeros_like(out)
        for i, w in enumerate(weights):
            acc += w * np.take(padded, np.arange(i, i + out.shape[axis]), axis=axis)
        out = acc
    return out


class ThermalImager:
    def __init__(self, rig, field: tp.ThermalField) -> None:
        self.rig = rig
        self.field = field
        self._skin_cache: tuple[float, np.ndarray] | None = None

    def _skin(self, hour: float) -> np.ndarray:
        if self._skin_cache is None or abs(self._skin_cache[0] - hour) > 1.0 / 3600.0:
            self._skin_cache = (hour, self.field.skin_at(hour))
        return self._skin_cache[1]

    def render(self, sensor: ThermalSensor, x: float, y: float, camera_z: float, heading_deg: float, pitch_down_deg: float,
               hour: float, entities=(), poses=(), rng: np.random.Generator | None = None, max_range_m: float = 140.0, prune: bool = True) -> ThermalFrame:
        field, site = self.field, self.field.site
        w, h = sensor.width, sensor.height
        hits = self.rig.resolve_view(x, y, camera_z, heading_deg, w, h, sensor.hfov_deg, pitch_down_deg, max_range_m)
        painted, kind = hits["painted"], hits["kind"]
        n = w * h
        elev, azim = hits["elevation"].astype(np.float64), hits["azimuth"].astype(np.float64)
        bearing = hits["heading"] + azim
        direction = np.stack([np.sin(bearing) * np.cos(elev), np.cos(bearing) * np.cos(elev), np.sin(elev)], axis=1)
        slant = np.where(painted, hits["depth"] / np.maximum(np.cos(elev), 1e-3), np.inf)

        air_c = site.air_c(hour)
        air_k = air_c + tp.KELVIN
        surface_c = np.zeros(n)
        emissivity = np.full(n, 0.95)
        reflected_k = np.zeros(n)

        skin = self._skin(hour)
        ground = painted & (kind == 0)
        if painted.any():
            px, py = hits["hx"][painted], hits["hy"][painted]
            surface_c[painted] = field.sample(skin, px, py)
            emissivity[painted] = field.nearest(field.emissivity, px, py)
        # buried objects: the patch over each differs from the ground beside it by what its own column says
        if len(field.target_xy) and ground.any():
            delta = field.target_delta_at(hour)
            gx, gy = hits["hx"][ground], hits["hy"][ground]
            extra = np.zeros(gx.shape)
            for (tx, ty), d in zip(field.target_xy, delta, strict=True):
                dist = np.hypot(gx - tx, gy - ty)
                wgt = np.clip((tp.PATCH_RADIUS_M + tp.PATCH_EDGE_M / 2.0 - dist) / tp.PATCH_EDGE_M, 0.0, 1.0)
                extra += wgt * wgt * (3.0 - 2.0 * wgt) * d
            surface_c[ground] += extra
        depression = np.clip(-elev, 0.0, math.pi / 2.0)
        reflected_k[:] = np.where(kind == 0, tp.sky_brightness_k(air_k, depression), air_k - UPRIGHT_REFLECTED_DEPRESSION_K)

        # things that stand on the ground, drawn from their own surfaces and in front of what is behind them
        entity_index = np.full(n, -1, dtype=np.int16)
        nearest = slant.copy()
        surface_x, surface_y = hits["hx"].astype(np.float64).copy(), hits["hy"].astype(np.float64).copy()
        origin = (x, y, camera_z)
        beam, diffuse = site.sunlight(site.sun(hour)[0])
        sun_el, sun_az = site.sun(hour)
        sun_dir = (math.sin(sun_az), math.cos(sun_az))
        horizontal_flux = beam * math.sin(max(sun_el, 0.0)) + diffuse
        sky_lw = tp.ThermalSite.sky_longwave(air_c)
        h_conv = float(site.convection(hour))
        for index, (entity, pose) in enumerate(zip(entities, poses, strict=False)):
            ex, ey, eh = pose[0], pose[1], pose[2]
            distance = math.hypot(ex - x, ey - y)
            if distance > max_range_m + 5.0:
                continue
            z_ground = self.rig.elevation_at(ex, ey)
            heading_rad = math.radians(eh)
            fwd = (math.sin(heading_rad), math.cos(heading_rad))
            facing = (x - ex) * fwd[0] + (y - ey) * fwd[1] > 0.0
            # an animal's width is taken across its antlers or horns, which are not its body
            a, b = max(entity.length_m, 0.2) / 2.0, max(min(entity.width_m, 0.5) if entity.kind == "animal" else entity.width_m, 0.2) / 2.0
            # only rays that could reach this body: a window of bearing and elevation round it, not every pixel
            reach = math.hypot(a, b) * 1.6 + abs(entity.box_offset_m) + 0.7
            if prune and distance <= reach:
                continue
            half_width = math.asin(min(1.0, reach / max(distance, 1e-6))) + 0.02
            to_body = math.atan2(ex - x, ey - y)
            swing = (bearing - to_body + math.pi) % (2.0 * math.pi) - math.pi
            near, far = max(distance - reach, 0.5), distance + reach
            low = min(math.atan2(z_ground - camera_z, near), math.atan2(z_ground - camera_z, far)) - 0.02
            high = max(math.atan2(z_ground + entity.height_m - camera_z, near), math.atan2(z_ground + entity.height_m - camera_z, far)) + 0.02
            window = (np.abs(swing) < half_width) & (elev > low) & (elev < high) if prune else np.ones(n, dtype=bool)
            sub = np.nonzero(window)[0]
            if sub.size == 0:
                continue
            sub_dir = direction[sub]
            for band in body_bands(entity, air_c, sun_el, beam, diffuse, h_conv, facing):
                t, z, nx, ny, cap = _band_hits(origin, sub_dir, (ex + fwd[0] * entity.box_offset_m, ey + fwd[1] * entity.box_offset_m), heading_rad,
                                                z_ground, entity.height_m, band, a, b)
                closer = t < nearest[sub]
                if closer.any():
                    face_flux = beam * math.cos(max(sun_el, 0.0)) * np.clip(nx * sun_dir[0] + ny * sun_dir[1], 0.0, None) + diffuse / 2.0
                    flux = np.where(cap, horizontal_flux, face_flux)
                    hit = sub[closer]
                    surface_c[hit] = band_temperature(band, flux[closer], air_c, sky_lw, h_conv)
                    emissivity[hit] = band.emissivity
                    reflected_k[hit] = air_k - UPRIGHT_REFLECTED_DEPRESSION_K
                    nearest[hit] = t[closer]
                    surface_x[hit] = x + t[closer] * sub_dir[closer, 0]
                    surface_y[hit] = y + t[closer] * sub_dir[closer, 1]
                    entity_index[hit] = index
                    painted[hit] = True

        # radiometry: what leaves the surface, what the air in between adds and takes
        apparent_k = tp.RADIOMETER.apparent_k(surface_c + tp.KELVIN, emissivity, reflected_k)
        tau = tp.transmission(np.where(np.isfinite(nearest), nearest, 0.0))
        radiance = tau * tp.RADIOMETER.radiance(apparent_k) + (1.0 - tau) * tp.RADIOMETER.radiance(air_k)
        measured_k = tp.RADIOMETER.temperature(radiance)
        sky_k = tp.sky_brightness_k(air_k, np.clip(elev, 0.0, math.pi / 2.0))
        measured_k = np.where(painted, measured_k, sky_k)

        # the sensor
        image = _gaussian_blur(measured_k.reshape(h, w), sensor.blur_px)
        rng = rng or np.random.default_rng()
        image = image + rng.normal(0.0, sensor.netd_k, image.shape)

        along = np.where(painted, slant * sensor.ifov_rad / np.maximum(np.sin(depression), 0.02), np.inf)
        across = np.where(painted, slant * sensor.ifov_rad, np.inf)
        ground_pixels = (ground & (entity_index < 0) & (nearest <= slant + 1e-6)).reshape(h, w)
        return ThermalFrame(
            (image - tp.KELVIN).astype(np.float32), ground_pixels, entity_index.reshape(h, w), hits["hx"].reshape(h, w), hits["hy"].reshape(h, w),
            nearest.reshape(h, w).astype(np.float32), along.reshape(h, w).astype(np.float32), across.reshape(h, w).astype(np.float32), sensor,
            painted.reshape(h, w), surface_x.reshape(h, w), surface_y.reshape(h, w),
        )
