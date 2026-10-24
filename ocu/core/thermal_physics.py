"""The physics under the thermal picture: sun, sky, ground and radiance.

Thermal imagery is not the visible scene recoloured. A surface's infrared brightness is set by its temperature, its
emissivity and what it reflects, and its temperature is the result of an energy balance that depends on the time of day.
This module computes that balance for every cell of the world, over a day, and the radiometry that turns it into what a
camera records. DECISIONS.md ("Thermal imagery: a physical model, not a palette") says what it is and is not: a
simulation with textbook parameters, checked against analytic solutions, not validated against a real site.

No Qt import belongs in this file.
"""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass, field

import numpy as np

from ocu.core.terrain import Terrain, TerrainWorld

# --- physical constants -----------------------------------------------------------------------------------------

PLANCK_H = 6.62607015e-34
LIGHT_C = 2.99792458e8
BOLTZMANN_K = 1.380649e-23
STEFAN_BOLTZMANN = 5.670374419e-8
SOLAR_CONSTANT_W_M2 = 1361.0
KELVIN = 273.15

# --- radiometry: the camera's band ------------------------------------------------------------------------------

BAND_UM = (8.0, 14.0)  # the long-wave window an uncooled microbolometer looks through
BAND_SAMPLES = 240
TABLE_K = (150.0, 450.0, 0.02)  # lowest, highest and step of the radiance-to-temperature table


def _planck_band_radiance(temperature_k: np.ndarray, band_um=BAND_UM, samples: int = BAND_SAMPLES) -> np.ndarray:
    """Radiance (W/m2/sr) of a black body in the band, by trapezoid over wavelength."""
    lam = np.linspace(band_um[0], band_um[1], samples) * 1e-6
    t = np.asarray(temperature_k, dtype=np.float64)[..., None]
    spectral = 2.0 * PLANCK_H * LIGHT_C**2 / lam**5 / np.expm1(PLANCK_H * LIGHT_C / (lam * BOLTZMANN_K * t))
    return np.trapezoid(spectral, lam, axis=-1)


class Radiometer:
    """Band radiance of a black body and its inverse, from a table (a monotonic curve, so interpolation is exact enough)."""

    def __init__(self, band_um=BAND_UM) -> None:
        lo, hi, step = TABLE_K
        self._t = np.arange(lo, hi + step, step)
        self._l = _planck_band_radiance(self._t, band_um)
        self.band_um = band_um

    def radiance(self, temperature_k):
        return np.interp(temperature_k, self._t, self._l)

    def temperature(self, radiance):
        return np.interp(radiance, self._l, self._t)

    def apparent_k(self, surface_k, emissivity, reflected_k):
        """The temperature of the black body that would give the same band radiance as a surface of this emissivity,
        which also reflects an environment at `reflected_k`."""
        return self.temperature(emissivity * self.radiance(surface_k) + (1.0 - emissivity) * self.radiance(reflected_k))


RADIOMETER = Radiometer()

#: Clear sky in the window is far colder than the air, and colder looking straight up than near the horizon, where the
#: line of sight crosses more atmosphere. A ground surface reflects the sky at the elevation it is viewed from.
SKY_WINDOW_DEPRESSION_K = 50.0
SKY_WINDOW_EXPONENT = 0.6


def sky_brightness_k(air_k: float, elevation_rad):
    """Apparent temperature of the clear sky in the band, at an elevation above the horizon."""
    return air_k - SKY_WINDOW_DEPRESSION_K * np.clip(np.sin(elevation_rad), 0.0, 1.0) ** SKY_WINDOW_EXPONENT


#: Atmospheric extinction in the band, per metre, for clear dry air at altitude. A choice, not a measurement.
EXTINCTION_PER_M = 2.0e-4


def transmission(range_m):
    return np.exp(-EXTINCTION_PER_M * np.asarray(range_m, dtype=np.float64))


# --- the site, the sun and the sky -------------------------------------------------------------------------------


@dataclass(frozen=True)
class ThermalSite:
    """Where and when. A high-altitude border region in late October, clear sky: choices, not data.

    The sun the visible cameras draw (42 degrees up, 150 round) is what 20 October gives at 34 degrees north at about
    10:40 solar time, so the two agree when a mission starts.
    """

    latitude_deg: float = 34.0
    altitude_m: float = 3500.0
    day_of_year: int = 293  # 20 October
    air_mean_c: float = 6.0
    air_swing_k: float = 9.0  # half the daily range
    air_peak_hour: float = 15.0
    wind_mean_ms: float = 2.4
    wind_swing: float = 0.5  # share of the mean: the wind eases at night and peaks mid-afternoon as the boundary layer mixes
    wind_peak_hour: float = 14.0
    deep_c: float = 6.0  # the temperature a metre down, which barely moves in a day: the daily mean of the air
    start_hour: float = 10.67  # solar time at the start of a mission (10:40)

    def air_c(self, hour: float) -> float:
        return self.air_mean_c + self.air_swing_k * math.cos(2.0 * math.pi * (hour - self.air_peak_hour) / 24.0)

    def declination_rad(self) -> float:
        return math.radians(23.45) * math.sin(2.0 * math.pi * (284 + self.day_of_year) / 365.0)

    def sun(self, hour: float) -> tuple[float, float]:
        """(elevation above the horizon in radians, azimuth in radians clockwise from north) at a solar time."""
        phi, delta = math.radians(self.latitude_deg), self.declination_rad()
        hour_angle = math.radians(15.0 * (hour - 12.0))
        sin_el = math.sin(phi) * math.sin(delta) + math.cos(phi) * math.cos(delta) * math.cos(hour_angle)
        east = -math.cos(delta) * math.sin(hour_angle)
        north = math.sin(delta) * math.cos(phi) - math.cos(delta) * math.sin(phi) * math.cos(hour_angle)
        return math.asin(max(-1.0, min(1.0, sin_el))), math.atan2(east, north) % (2.0 * math.pi)

    def sunlight(self, elevation_rad: float) -> tuple[float, float]:
        """Clear-sky (beam normal, diffuse horizontal) in W/m2: the Hottel model with its altitude terms, as set out by
        Duffie and Beckman, for a mid-latitude summer atmosphere. Zero when the sun is down."""
        if elevation_rad <= math.radians(0.5):
            return 0.0, 0.0
        a = self.altitude_m / 1000.0
        a0 = 0.97 * (0.4237 - 0.00821 * (6.0 - a) ** 2)
        a1 = 0.99 * (0.5055 + 0.00595 * (6.5 - a) ** 2)
        k = 1.02 * (0.2711 + 0.01858 * (2.5 - a) ** 2)
        cos_z = math.sin(elevation_rad)
        tau_b = a0 + a1 * math.exp(-k / cos_z)
        tau_d = max(0.271 - 0.294 * tau_b, 0.0)
        extraterrestrial = SOLAR_CONSTANT_W_M2 * (1.0 + 0.033 * math.cos(2.0 * math.pi * self.day_of_year / 365.0))
        return extraterrestrial * tau_b, extraterrestrial * cos_z * tau_d

    @staticmethod
    def sky_longwave(air_c: float) -> float:
        """Downwelling long-wave radiation on a clear night or day, W/m2 (Swinbank)."""
        return 5.31e-13 * (air_c + KELVIN) ** 6

    def wind_ms(self, hour: float) -> float:
        return self.wind_mean_ms * (1.0 + self.wind_swing * math.cos(2.0 * math.pi * (hour - self.wind_peak_hour) / 24.0))

    def convection(self, hour: float, exposure=1.0):
        """Convective heat transfer coefficient, W/m2/K (McAdams), scaled by how exposed the surface is to the wind."""
        return exposure * (5.7 + 3.8 * self.wind_ms(hour))


# --- materials ---------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Material:
    """Textbook values (Oke, Boundary Layer Climates; Kahle on thermal inertia): not measured at this site."""

    name: str
    albedo: float
    emissivity: float
    conductivity: float  # W/m/K
    heat_capacity: float  # J/m3/K, volumetric
    latent_share: float  # share of absorbed sunlight spent evaporating water (plants only)
    sky_view: float = 1.0
    wind_exposure: float = 1.0

    @property
    def inertia(self) -> float:
        return math.sqrt(self.conductivity * self.heat_capacity)


MATERIALS = {
    "soil": Material("dry soil", 0.30, 0.95, 0.30, 1.3e6, 0.00),
    "track": Material("compacted track", 0.28, 0.95, 0.60, 1.5e6, 0.00),
    "scrub": Material("low scrub", 0.20, 0.98, 0.20, 0.9e6, 0.10),
    "rock": Material("rock", 0.25, 0.93, 2.80, 2.2e6, 0.00),
    "tree": Material("tree canopy", 0.15, 0.98, 0.15, 0.7e6, 0.35, wind_exposure=1.3),
    "bush": Material("shrub", 0.18, 0.98, 0.18, 0.8e6, 0.20, wind_exposure=1.15),
    "pothole": Material("sheltered bare soil", 0.25, 0.95, 0.30, 1.3e6, 0.00, sky_view=0.75, wind_exposure=0.6),
}
MATERIAL_ORDER = tuple(MATERIALS)
ROCK_MIN_HEIGHT_M = 0.12  # a boulder at least this tall is rock whatever the class grid says under it

_MATERIAL_OF_CLASS = {
    Terrain.UNKNOWN: "soil",
    Terrain.GROUND: "soil",
    Terrain.TRACK: "track",
    Terrain.SCRUB: "scrub",
    Terrain.ROCK: "rock",
    Terrain.TREE: "tree",
    Terrain.POTHOLE: "pothole",
    Terrain.BUSH: "bush",
}

# --- the column solver --------------------------------------------------------------------------------------------

#: Layer thicknesses, top first, in metres: fine where the daily wave is strong. The wave's damping depth in dry soil is
#: about 8 cm, so 63 cm is some eight of them.
LAYERS_M = (0.01, 0.02, 0.04, 0.08, 0.16, 0.32)
DEEP_REACH_M = 0.5  # the reservoir below the last layer sits this far past its centre


class ColumnSolver:
    """Heat conduction in a column under every cell, implicit in time, with an energy-balance surface.

    State is `temps` (layers, cells) in kelvin and `skin` (cells), the temperature of the surface itself. Each step the
    surface balance is linearised about the previous skin temperature, which turns it into a conductance to a forcing
    temperature, and the whole column is solved as a tridiagonal system, so any step is stable.
    """

    def __init__(self, conductivity, heat_capacity, layers_m=LAYERS_M, deep_k: float = 280.0) -> None:
        """`conductivity` and `heat_capacity` are (layers, cells), or (cells,) when every layer is alike."""
        self.dz = np.asarray(layers_m, dtype=np.float64)
        n_layers = len(self.dz)
        k = np.asarray(conductivity, dtype=np.float64)
        c = np.asarray(heat_capacity, dtype=np.float64)
        if k.ndim == 1:
            k = np.repeat(k[None, :], n_layers, axis=0)
        if c.ndim == 1:
            c = np.repeat(c[None, :], n_layers, axis=0)
        self.k, self.rho_c = k, c
        self.cells = k.shape[1]
        dz = self.dz[:, None]
        self.capacity = c * dz  # J/m2/K per layer
        self.between = 1.0 / (dz[:-1] / (2.0 * k[:-1]) + dz[1:] / (2.0 * k[1:]))  # W/m2/K between layer i and i+1
        self.skin_link = k[0] / (dz[0, 0] / 2.0)  # W/m2/K, surface to the top layer's centre
        self.deep_link = k[-1] / (dz[-1, 0] / 2.0 + DEEP_REACH_M)
        self.deep_k = deep_k
        self.temps = np.full((n_layers, self.cells), deep_k)
        self.skin = np.full(self.cells, deep_k)

    def set_state(self, temps: np.ndarray, skin: np.ndarray) -> None:
        self.temps, self.skin = temps.copy(), skin.copy()

    def heat_content(self) -> np.ndarray:
        """J/m2 above a zero kelvin reference, per cell."""
        return (self.capacity * self.temps).sum(axis=0)

    def step(self, dt: float, absorbed_sw, emissivity, longwave_down, air_k, convection, latent, insulated: bool = False) -> None:
        """One implicit step. `absorbed_sw`, `emissivity`, `convection` and `latent` are per cell; the sky and air are
        scalars. `insulated` closes the top and the bottom, for the conservation check."""
        sigma = STEFAN_BOLTZMANN
        tn = self.skin
        radiative = 4.0 * emissivity * sigma * tn**3
        total = radiative + convection + self.skin_link
        forcing = absorbed_sw + emissivity * longwave_down + 3.0 * emissivity * sigma * tn**4 + convection * air_k - latent
        t_force = forcing / (total - self.skin_link)
        g_top = self.skin_link * (total - self.skin_link) / total
        if insulated:
            g_top = np.zeros_like(g_top)
        g_deep = np.zeros_like(self.deep_link) if insulated else self.deep_link

        layers = len(self.dz)
        inv_dt = self.capacity / dt
        diag = inv_dt.copy()
        diag[:-1] += self.between
        diag[1:] += self.between
        diag[0] += g_top
        diag[-1] += g_deep
        rhs = inv_dt * self.temps
        rhs[0] += g_top * t_force
        rhs[-1] += g_deep * self.deep_k
        # Thomas algorithm, vectorised over cells
        cp = np.empty_like(diag)
        dp = np.empty_like(diag)
        cp[0] = -self.between[0] / diag[0] if layers > 1 else 0.0
        dp[0] = rhs[0] / diag[0]
        for i in range(1, layers):
            denom = diag[i] + self.between[i - 1] * cp[i - 1]
            cp[i] = -self.between[i] / denom if i < layers - 1 else 0.0
            dp[i] = (rhs[i] + self.between[i - 1] * dp[i - 1]) / denom
        out = np.empty_like(diag)
        out[-1] = dp[-1]
        for i in range(layers - 2, -1, -1):
            out[i] = dp[i] - cp[i] * out[i + 1]
        self.temps = out
        self.skin = (forcing + self.skin_link * out[0]) / total if not insulated else out[0]


# --- the day's field ----------------------------------------------------------------------------------------------

STEP_S = 300.0
FRAME_S = 600.0
FRAMES_PER_DAY = int(86400 / FRAME_S)
SPINUP_DAYS = 2
SHADOW_STEP_M = 0.5
SHADOW_MAX_REACH_M = 80.0
SHADOW_MIN_ELEVATION_RAD = math.radians(1.0)
PATCH_RADIUS_M = 0.30  # the footprint of disturbed ground over a buried object
PATCH_EDGE_M = 0.12  # how softly it blends into the ground round it
DISTURBED_CONDUCTIVITY = 0.70  # loosened fill conducts this share of what undisturbed soil does
DISTURBED_HEAT_CAPACITY = 0.95
BODY_HEIGHT_M = 0.10
BODY_CONDUCTIVITY = 0.25  # a plastic-cased charge
BODY_HEAT_CAPACITY = 1.9e6


def _layer_edges() -> np.ndarray:
    return np.concatenate([[0.0], np.cumsum(LAYERS_M)])


def _overlap(top: float, bottom: float) -> np.ndarray:
    """Share of each layer between two depths."""
    edges = _layer_edges()
    lo, hi = edges[:-1], edges[1:]
    return np.clip(np.minimum(hi, bottom) - np.maximum(lo, top), 0.0, None) / (hi - lo)


def target_layers(depth_m: float, base: Material) -> tuple[np.ndarray, np.ndarray]:
    """Conductivity and heat capacity of the column over a buried object: loosened fill above it, its body below."""
    fill = _overlap(0.0, depth_m)
    body = np.clip(_overlap(depth_m, depth_m + BODY_HEIGHT_M), 0.0, 1.0 - fill)
    soil = 1.0 - fill - body
    k = soil * base.conductivity + fill * base.conductivity * DISTURBED_CONDUCTIVITY + body * BODY_CONDUCTIVITY
    c = soil * base.heat_capacity + fill * base.heat_capacity * DISTURBED_HEAT_CAPACITY + body * BODY_HEAT_CAPACITY
    return k, c


def _shift(a: np.ndarray, ox: int, oy: int, fill: float) -> np.ndarray:
    """out[i, j] = a[i + ox, j + oy], `fill` where that is off the grid."""
    out = np.full_like(a, fill)
    n, m = a.shape
    xs, xd = (slice(ox, n), slice(0, n - ox)) if ox >= 0 else (slice(0, n + ox), slice(-ox, n))
    ys, yd = (slice(oy, m), slice(0, m - oy)) if oy >= 0 else (slice(0, m + oy), slice(-oy, m))
    if abs(ox) < n and abs(oy) < m:
        out[xd, yd] = a[xs, ys]
    return out


def shadow_mask(surface: np.ndarray, cell_m: float, elevation_rad: float, azimuth_rad: float) -> np.ndarray:
    """True where the sun's beam is blocked: something upwind stands higher than the ray rising from the cell's top.

    `surface` is ground plus whatever stands on it, indexed [x east, y north]; the sun is toward (sin az, cos az).
    """
    tan_el = math.tan(elevation_rad)
    standing = float(surface.max() - surface.min())
    reach = min(SHADOW_MAX_REACH_M, standing / max(tan_el, 1e-3))
    ux, uy = math.sin(azimuth_rad), math.cos(azimuth_rad)
    shaded = np.zeros(surface.shape, dtype=bool)
    seen = set()
    d = SHADOW_STEP_M
    while d <= reach:
        key = (round(d * ux / cell_m), round(d * uy / cell_m))
        if key != (0, 0) and key not in seen:
            seen.add(key)
            shaded |= _shift(surface, key[0], key[1], -1e9) - surface > d * tan_el
        d += SHADOW_STEP_M
    return shaded


@dataclass
class ThermalField:
    """The whole day, ready to be looked up at any time: skin temperature of every cell every ten minutes, and the
    columns over each buried object beside the ground they sit in."""

    site: ThermalSite
    cell_m: float
    extent: int
    skin_c: np.ndarray  # (frames, span, span) degrees C, float16
    emissivity: np.ndarray  # (span, span)
    material: np.ndarray  # (span, span) index into MATERIAL_ORDER
    target_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    target_c: np.ndarray = field(default_factory=lambda: np.zeros((FRAMES_PER_DAY, 0), dtype=np.float32))
    host_c: np.ndarray = field(default_factory=lambda: np.zeros((FRAMES_PER_DAY, 0), dtype=np.float32))

    @property
    def span(self) -> int:
        return self.skin_c.shape[1]

    def _frames(self, hour: float) -> tuple[int, int, float]:
        pos = (hour % 24.0) * 3600.0 / FRAME_S
        i = int(math.floor(pos)) % FRAMES_PER_DAY
        return i, (i + 1) % FRAMES_PER_DAY, pos - math.floor(pos)

    def skin_at(self, hour: float) -> np.ndarray:
        """Skin temperature of every cell at a solar time, degrees C, interpolated between stored frames."""
        a, b, t = self._frames(hour)
        return (self.skin_c[a].astype(np.float32) * (1.0 - t) + self.skin_c[b].astype(np.float32) * t)

    def target_delta_at(self, hour: float) -> np.ndarray:
        """Temperature of each buried object's patch minus the ground beside it, K."""
        a, b, t = self._frames(hour)
        d = self.target_c - self.host_c
        return d[a] * (1.0 - t) + d[b] * t

    def sample(self, field_2d: np.ndarray, xs, ys) -> np.ndarray:
        """Bilinear sample of a per-cell field at world points (cell centres sit half a cell past their index)."""
        fx = np.asarray(xs) / self.cell_m + self.extent - 0.5
        fy = np.asarray(ys) / self.cell_m + self.extent - 0.5
        x0, y0 = np.floor(fx), np.floor(fy)
        tx, ty = (fx - x0).astype(np.float32), (fy - y0).astype(np.float32)
        top = self.span - 1
        x0, y0 = np.clip(x0.astype(np.int64), 0, top), np.clip(y0.astype(np.int64), 0, top)
        x1, y1 = np.minimum(x0 + 1, top), np.minimum(y0 + 1, top)
        return (
            field_2d[x0, y0] * (1 - tx) * (1 - ty) + field_2d[x1, y0] * tx * (1 - ty) + field_2d[x0, y1] * (1 - tx) * ty + field_2d[x1, y1] * tx * ty
        )

    def nearest(self, field_2d: np.ndarray, xs, ys) -> np.ndarray:
        ix = np.clip(np.floor(np.asarray(xs) / self.cell_m).astype(np.int64) + self.extent, 0, self.span - 1)
        iy = np.clip(np.floor(np.asarray(ys) / self.cell_m).astype(np.int64) + self.extent, 0, self.span - 1)
        return field_2d[ix, iy]

    def save(self, path) -> None:
        np.savez_compressed(path, skin=self.skin_c, emissivity=self.emissivity, material=self.material, target_xy=self.target_xy,
                            target_c=self.target_c, host_c=self.host_c, cell_m=self.cell_m, extent=self.extent)

    @classmethod
    def load(cls, path, site: ThermalSite) -> ThermalField:
        z = np.load(path)
        return cls(site, float(z["cell_m"]), int(z["extent"]), z["skin"], z["emissivity"], z["material"], z["target_xy"], z["target_c"], z["host_c"])


def cache_key(world: TerrainWorld, site: ThermalSite, targets) -> str:
    """What a stored field depends on: the world's identity and size, the site, the targets, and the model's own numbers."""
    rock = float(np.asarray(world._rock_height).sum())
    parts = (world.seed, world._span, world.cell_size_m, round(rock, 2), site,
             tuple((round(t.x, 2), round(t.y, 2), t.depth_m) for t in targets), LAYERS_M, STEP_S, tuple(sorted((k, m) for k, m in MATERIALS.items())),
             DISTURBED_CONDUCTIVITY, DISTURBED_HEAT_CAPACITY, BODY_CONDUCTIVITY, BODY_HEAT_CAPACITY, SPINUP_DAYS)
    return hashlib.sha256(repr(parts).encode()).hexdigest()[:16]


def _material_grid(world: TerrainWorld) -> np.ndarray:
    index = {name: i for i, name in enumerate(MATERIAL_ORDER)}
    out = np.zeros(world._grid.shape, dtype=np.uint8)
    for terrain, name in _MATERIAL_OF_CLASS.items():
        out[world._grid == int(terrain)] = index[name]
    out[(world._rock_height >= ROCK_MIN_HEIGHT_M) & (world._grid != int(Terrain.POTHOLE))] = index["rock"]
    return out


def _coarsen(a: np.ndarray, how=np.mean) -> np.ndarray:
    n = (a.shape[-1] // 2) * 2
    a = a[..., :n, :n]
    return how(a.reshape(*a.shape[:-2], n // 2, 2, n // 2, 2), axis=(-3, -1))


def _upsample(a: np.ndarray, span: int) -> np.ndarray:
    big = np.repeat(np.repeat(a, 2, axis=-2), 2, axis=-1)
    pad = span - big.shape[-1]
    return np.pad(big, [(0, 0)] * (big.ndim - 2) + [(0, pad), (0, pad)], mode="edge") if pad else big


def _absorbed_sunlight(site: ThermalSite, frame: int, albedo, sky_view, normal, shaded) -> np.ndarray:
    """Sunlight absorbed per cell (W/m2) in a frame of the day: the beam on the tilted surface where not shaded, plus
    the sky's diffuse light on the share of sky the cell can see."""
    el, az = site.sun(frame * FRAME_S / 3600.0)
    beam, diffuse = site.sunlight(el)
    if beam <= 0.0:
        return np.zeros(albedo.shape)
    sun = np.array([math.cos(el) * math.sin(az), math.cos(el) * math.cos(az), math.sin(el)])
    lit = ~shaded if shaded is not None else 1.0
    return (1.0 - albedo) * (beam * np.clip(normal @ sun, 0.0, None) * lit + diffuse * (1.0 + normal[..., 2]) / 2.0 * sky_view)


def _columns(per_cell: np.ndarray, per_target: list[np.ndarray]) -> np.ndarray:
    """Layer properties for every cell (the same in each layer) followed by one column per buried object."""
    cells = np.repeat(per_cell.reshape(-1)[None, :], len(LAYERS_M), axis=0)
    return np.concatenate([cells, np.stack(per_target, axis=1)], axis=1) if per_target else cells


def build_field(rig, targets=(), site: ThermalSite | None = None, progress=None) -> ThermalField:
    """Solve the day. `rig` is the camera rig of the world (its height fields are what shadows are cast by), `targets`
    the buried objects, each given a column of its own in the sun and shadow of the cell it sits in."""
    site = site or ThermalSite()
    world = rig._world
    span, cell_m = world._span, world.cell_size_m
    elevation = rig._elevation.astype(np.float64)
    relief = rig._relief  # elevation, tree, rock, scrub heights
    standing = elevation + np.maximum(np.maximum(relief[..., 1], relief[..., 2]), relief[..., 3])
    material = _material_grid(world)
    by_name = {n: MATERIALS[n] for n in MATERIAL_ORDER}
    pick = lambda attr: np.array([getattr(by_name[n], attr) for n in MATERIAL_ORDER])[material]  # noqa: E731
    albedo, emis, latent, sky_view, exposure = (pick(a) for a in ("albedo", "emissivity", "latent_share", "sky_view", "wind_exposure"))
    gx, gy = np.gradient(elevation, cell_m)
    normal = np.stack([-gx, -gy, np.ones_like(gx)], axis=-1)
    normal /= np.linalg.norm(normal, axis=-1, keepdims=True)

    sun_by_frame = [site.sun(f * FRAME_S / 3600.0) for f in range(FRAMES_PER_DAY)]
    shadows = {}
    for f, (el, az) in enumerate(sun_by_frame):
        if el > SHADOW_MIN_ELEVATION_RAD:
            shadows[f] = shadow_mask(standing, cell_m, el, az)
        if progress:
            progress(0.2 * (f + 1) / FRAMES_PER_DAY)

    targets = list(targets)
    m = len(targets)
    host_ix = np.array([int(np.clip(math.floor(t.x / cell_m) + world._extent, 0, span - 1)) for t in targets], dtype=int)
    host_iy = np.array([int(np.clip(math.floor(t.y / cell_m) + world._extent, 0, span - 1)) for t in targets], dtype=int)
    layers = [target_layers(t.depth_m, by_name[MATERIAL_ORDER[int(material[ix, iy])]]) for t, ix, iy in zip(targets, host_ix, host_iy, strict=True)]
    deep_k = site.deep_c + KELVIN

    def make_solver(conductivity, heat_capacity):
        solver = ColumnSolver(_columns(conductivity, [k for k, _ in layers]), _columns(heat_capacity, [c for _, c in layers]), deep_k=deep_k)
        solver.set_state(np.full_like(solver.temps, deep_k), np.full(solver.cells, deep_k))
        return solver

    def with_targets(per_cell_2d: np.ndarray, host_flat: np.ndarray) -> np.ndarray:
        v = per_cell_2d.reshape(-1)
        return np.concatenate([v, v[host_flat]]) if m else v

    steps = int(FRAME_S / STEP_S)

    def advance(solver, frame, sw_2d, host_flat, e_all, exposure_all, l_all):
        sw = with_targets(sw_2d, host_flat)
        for k in range(steps):
            hour = frame * FRAME_S / 3600.0 + (k + 0.5) * STEP_S / 3600.0
            air = site.air_c(hour)
            solver.step(STEP_S, sw, e_all, site.sky_longwave(air), air + KELVIN, site.convection(hour, exposure_all), l_all * sw)

    # settle the deep layers on a half-resolution copy of the ground (they are smooth), two days
    cspan = span // 2
    c_albedo, c_emis, c_latent, c_view, c_exposure = (_coarsen(x) for x in (albedo, emis, latent, sky_view, exposure))
    c_normal = _coarsen(np.moveaxis(normal, -1, 0)).transpose(1, 2, 0)
    c_normal = c_normal / np.linalg.norm(c_normal, axis=-1, keepdims=True)
    c_shadows = {f: _coarsen(mk.astype(np.float32)) > 0.5 for f, mk in shadows.items()}
    c_host = np.clip(host_ix // 2, 0, cspan - 1) * cspan + np.clip(host_iy // 2, 0, cspan - 1)
    coarse = make_solver(_coarsen(by_name_array(material, "conductivity")), _coarsen(by_name_array(material, "heat_capacity")))
    ce, ch, cl = with_targets(c_emis, c_host), with_targets(c_exposure, c_host), with_targets(c_latent, c_host)
    for count in range(SPINUP_DAYS * FRAMES_PER_DAY):
        frame = count % FRAMES_PER_DAY
        advance(coarse, frame, _absorbed_sunlight(site, frame, c_albedo, c_view, c_normal, c_shadows.get(frame)), c_host, ce, ch, cl)
        if progress:
            progress(0.2 + 0.4 * (count + 1) / (SPINUP_DAYS * FRAMES_PER_DAY))

    fine = make_solver(by_name_array(material, "conductivity"), by_name_array(material, "heat_capacity"))
    cells, nl = span * span, len(LAYERS_M)
    ground_t = coarse.temps[:, : cspan * cspan].reshape(nl, cspan, cspan)
    ground_skin = coarse.skin[: cspan * cspan].reshape(cspan, cspan)
    fine.set_state(
        np.concatenate([_upsample(ground_t, span).reshape(nl, -1), coarse.temps[:, cspan * cspan :]], axis=1),
        np.concatenate([_upsample(ground_skin, span).reshape(-1), coarse.skin[cspan * cspan :]]),
    )

    host_flat = host_ix * span + host_iy
    e_all, h_all, l_all = with_targets(emis, host_flat), with_targets(exposure, host_flat), with_targets(latent, host_flat)
    skin_frames = np.empty((FRAMES_PER_DAY, span, span), dtype=np.float16)
    target_frames = np.empty((FRAMES_PER_DAY, m), dtype=np.float32)
    host_frames = np.empty((FRAMES_PER_DAY, m), dtype=np.float32)
    for frame in range(FRAMES_PER_DAY):
        advance(fine, frame, _absorbed_sunlight(site, frame, albedo, sky_view, normal, shadows.get(frame)), host_flat, e_all, h_all, l_all)
        skin_frames[frame] = (fine.skin[:cells].reshape(span, span) - KELVIN).astype(np.float16)
        if m:
            target_frames[frame] = fine.skin[cells:] - KELVIN
            host_frames[frame] = fine.skin[host_flat] - KELVIN
        if progress:
            progress(0.6 + 0.4 * (frame + 1) / FRAMES_PER_DAY)

    return ThermalField(site, cell_m, world._extent, skin_frames, emis.astype(np.float32), material,
                        np.array([[t.x, t.y] for t in targets], dtype=np.float64).reshape(-1, 2), target_frames, host_frames)


def by_name_array(material: np.ndarray, attr: str) -> np.ndarray:
    """A per-cell array of one material property."""
    return np.array([getattr(MATERIALS[n], attr) for n in MATERIAL_ORDER])[material]
