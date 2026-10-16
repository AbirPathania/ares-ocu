"""Camera views rendered from the simulated world.

What these tests care about is that a view *means* something: that a wall of
trees in front of the rover is in the picture and the ground behind it is
not, that a tricopter climbing sees more and sees it smaller, that
something east of an aircraft is on the right of its frame. A renderer can
produce a plausible picture while getting every one of those wrong, so the
assertions are about what is where, not about the pictures looking nice.
"""

import math

import numpy as np

from ocu.core.camera import SUN_AZIMUTH_DEG, CameraRig
from ocu.core.terrain import CELL_SIZE_M, Terrain, TerrainWorld

WIDTH, HEIGHT = 192, 76


def open_world() -> TerrainWorld:
    return TerrainWorld.blank(half_extent_m=60.0)


def paint(world: TerrainWorld, kind: Terrain, x0: float, x1: float, y0: float, y1: float) -> None:
    """Fill a rectangle of the world, in metres."""
    for cx in range(int(x0 / CELL_SIZE_M), int(x1 / CELL_SIZE_M) + 1):
        for cy in range(int(y0 / CELL_SIZE_M), int(y1 / CELL_SIZE_M) + 1):
            world.set_cell((cx, cy), kind)


def forward(world: TerrainWorld, heading: float, x: float = 0.0, y: float = 0.0) -> np.ndarray:
    return CameraRig(world).forward_view(x, y, heading, WIDTH, HEIGHT)


def centre(image: np.ndarray, half_width: int = 14) -> np.ndarray:
    mid = image.shape[1] // 2
    return image[:, mid - half_width : mid + half_width].reshape(-1, 3).astype(int)


def count_green(pixels: np.ndarray) -> int:
    r, g, b = pixels[:, 0], pixels[:, 1], pixels[:, 2]
    return int(((g > r + 6) & (g > b + 6)).sum())


def count_grey(pixels: np.ndarray) -> int:
    """Rock: no strong hue. Ground is tan, sky and haze are blue, foliage is
    green, so a pixel with almost no spread between channels is rock."""
    spread = pixels.max(axis=1) - pixels.min(axis=1)
    return int(((spread < 14) & (pixels.mean(axis=1) > 70) & (pixels.mean(axis=1) < 190)).sum())


# --- the rover's forward view ----------------------------------------------


def test_a_frame_is_the_size_asked_for_and_is_a_real_image():
    image = forward(open_world(), 0.0)
    assert image.shape == (HEIGHT, WIDTH, 3)
    assert image.dtype == np.uint8


def test_open_ground_is_sky_above_and_earth_below():
    image = forward(open_world(), 0.0).astype(int)
    sky, ground = image[2, WIDTH // 2], image[HEIGHT - 3, WIDTH // 2]
    assert sky[2] > sky[0], "the sky is bluer than it is red"
    assert ground[0] > ground[2], "dry ground is warmer than it is blue"


def test_the_horizon_sits_between_sky_and_ground():
    image = forward(open_world(), 0.0).astype(int)
    blueness = image[:, WIDTH // 2, 2] - image[:, WIDTH // 2, 0]
    assert blueness[: HEIGHT // 4].mean() > blueness[-HEIGHT // 4 :].mean() + 20


def test_a_wall_of_trees_ahead_is_in_the_picture_and_behind_is_not():
    world = open_world()
    paint(world, Terrain.TREE, -30.0, 30.0, 12.0, 18.0)
    rig = CameraRig(world)

    ahead = rig.forward_view(0.0, 0.0, 0.0, WIDTH, HEIGHT)
    behind = rig.forward_view(0.0, 0.0, 180.0, WIDTH, HEIGHT)
    assert count_green(centre(ahead)) > 150
    assert count_green(centre(behind)) == 0


def test_a_view_follows_the_heading():
    """Facing east at a wall to the east, then north at the same wall."""
    world = open_world()
    paint(world, Terrain.TREE, 12.0, 18.0, -30.0, 30.0)
    rig = CameraRig(world)
    assert count_green(centre(rig.forward_view(0.0, 0.0, 90.0, WIDTH, HEIGHT))) > 150
    assert count_green(centre(rig.forward_view(0.0, 0.0, 0.0, WIDTH, HEIGHT))) < 30


def test_something_nearer_hides_something_farther():
    """The occlusion rule, which is the whole reason for the method: a tree
    wall close in hides a rock wall behind it, and the same rock wall is
    visible when nothing is in front."""
    rocks_only = open_world()
    paint(rocks_only, Terrain.ROCK, -30.0, 30.0, 10.0, 12.0)

    with_trees = open_world()
    paint(with_trees, Terrain.ROCK, -30.0, 30.0, 10.0, 12.0)
    paint(with_trees, Terrain.TREE, -30.0, 30.0, 5.0, 7.0)

    visible = count_grey(centre(forward(rocks_only, 0.0)))
    hidden = count_grey(centre(forward(with_trees, 0.0)))
    assert visible > 20, "the rock wall has to be visible to begin with"
    assert hidden == 0, "a tree wall in front must hide it completely"


def test_a_ridge_hides_what_is_behind_it():
    """Terrain occludes too, not only things standing on it. Trees that would
    fill the view are not in it when a ridge is in the way."""
    trees = open_world()
    paint(trees, Terrain.TREE, -30.0, 30.0, 24.0, 28.0)
    assert count_green(centre(forward(trees, 0.0))) > 20

    ridge = open_world()
    paint(ridge, Terrain.TREE, -30.0, 30.0, 24.0, 28.0)
    for cx in range(-120, 121):
        for cy in range(int(9.0 / CELL_SIZE_M), int(11.0 / CELL_SIZE_M) + 1):
            index = ridge._index((cx, cy))
            if index:
                ridge._elevation[index] = 6.0
    assert count_green(centre(forward(ridge, 0.0))) == 0


def test_distance_fades_into_haze():
    near, far = open_world(), open_world()
    paint(near, Terrain.TREE, -30.0, 30.0, 6.0, 8.0)
    paint(far, Terrain.TREE, -30.0, 30.0, 52.0, 56.0)
    near_px, far_px = centre(forward(near, 0.0)), centre(forward(far, 0.0))

    def greenness(pixels):
        g = pixels[(pixels[:, 1] > pixels[:, 0] + 3) & (pixels[:, 1] > pixels[:, 2] + 3)]
        return float((g[:, 1] - g[:, 2]).mean()) if len(g) else 0.0

    assert greenness(near_px) > greenness(far_px) + 8, "distant foliage must wash out"


def test_the_same_inputs_give_the_same_picture():
    rig = CameraRig(open_world())
    a = rig.forward_view(1.0, 2.0, 33.0, WIDTH, HEIGHT)
    b = rig.forward_view(1.0, 2.0, 33.0, WIDTH, HEIGHT)
    assert np.array_equal(a, b)


def test_sensor_noise_is_applied_when_asked_for_and_only_then():
    rig = CameraRig(open_world())
    clean = rig.forward_view(0.0, 0.0, 0.0, WIDTH, HEIGHT)
    noisy_a = rig.forward_view(0.0, 0.0, 0.0, WIDTH, HEIGHT, sensor_noise=3.0)
    noisy_b = rig.forward_view(0.0, 0.0, 0.0, WIDTH, HEIGHT, sensor_noise=3.0)
    assert not np.array_equal(clean, noisy_a)
    assert not np.array_equal(noisy_a, noisy_b), "a live camera is different each frame"


def test_the_real_world_renders_without_incident_from_anywhere_on_it():
    """Including its edge, where the sampled fields have to clamp."""
    rig = CameraRig(TerrainWorld())
    for x, y, heading in [(0, 0, 0), (105, 105, 45), (-109, 0, 270), (0, -109, 180), (109, 109, 90)]:
        image = rig.forward_view(x, y, heading, WIDTH, HEIGHT)
        assert image.shape == (HEIGHT, WIDTH, 3)
        assert not np.isnan(image.astype(float)).any()


def test_tiny_frames_do_not_break_the_renderer():
    rig = CameraRig(open_world())
    assert rig.forward_view(0.0, 0.0, 0.0, 8, 4).shape == (4, 8, 3)
    assert rig.nadir_view(0.0, 0.0, 10.0, 0.0, 8, 4).shape == (4, 8, 3)


# --- the tricopters' nadir view ----------------------------------------------


def rock_centroid(image: np.ndarray) -> tuple[float, float] | None:
    """Where the rock is in the frame, as fractions from the centre: x to the
    right, y upward, both in [-1, 1]."""
    pixels = image.astype(int)
    spread = pixels.max(axis=2) - pixels.min(axis=2)
    mean = pixels.mean(axis=2)
    mask = (spread < 14) & (mean > 70) & (mean < 190)
    if mask.sum() < 4:
        return None
    rows, cols = np.nonzero(mask)
    h, w = mask.shape
    return ((cols.mean() + 0.5) / w - 0.5) * 2.0, (0.5 - (rows.mean() + 0.5) / h) * 2.0


def rock_world(x: float, y: float) -> TerrainWorld:
    world = open_world()
    paint(world, Terrain.ROCK, x - 1.5, x + 1.5, y - 1.5, y + 1.5)
    return world


def test_something_east_of_the_aircraft_is_on_the_right_when_facing_north():
    rig = CameraRig(rock_world(8.0, 0.0))
    centroid = rock_centroid(rig.nadir_view(0.0, 0.0, 14.0, 0.0, WIDTH, HEIGHT))
    assert centroid is not None
    assert centroid[0] > 0.2, "east is to the right of a north-facing camera"
    assert abs(centroid[1]) < 0.3


def test_something_in_the_direction_of_travel_is_toward_the_top_of_the_frame():
    """Facing east, a rock to the east is ahead, so it is up the frame, and
    it is not to either side. The frame is a wide strip, so 'ahead' has to be
    close enough to fall inside it: at this height it covers about five
    metres forward and twelve to either side."""
    rig = CameraRig(rock_world(3.5, 0.0))
    centroid = rock_centroid(rig.nadir_view(0.0, 0.0, 14.0, 90.0, WIDTH, HEIGHT))
    assert centroid is not None
    assert centroid[1] > 0.4
    assert abs(centroid[0]) < 0.3

    # And the same rock is not up the frame when the aircraft faces north:
    # it is to the right, which is the other half of the orientation check.
    sideways = rock_centroid(rig.nadir_view(0.0, 0.0, 14.0, 0.0, WIDTH, HEIGHT))
    assert sideways is not None and sideways[0] > 0.15 and abs(sideways[1]) < 0.3


def test_climbing_widens_the_footprint_so_things_move_toward_the_centre():
    """The real trade of altitude: more ground in the frame, at lower
    resolution. At twice the height the same rock sits half as far out."""
    rig = CameraRig(rock_world(8.0, 0.0))
    low = rock_centroid(rig.nadir_view(0.0, 0.0, 12.0, 0.0, WIDTH, HEIGHT))
    high = rock_centroid(rig.nadir_view(0.0, 0.0, 24.0, 0.0, WIDTH, HEIGHT))
    assert low is not None and high is not None
    assert high[0] < low[0]
    assert abs(high[0] / low[0] - 0.5) < 0.12


def test_something_outside_the_footprint_is_not_in_the_frame():
    rig = CameraRig(rock_world(40.0, 0.0))
    assert rock_centroid(rig.nadir_view(0.0, 0.0, 12.0, 0.0, WIDTH, HEIGHT)) is None


def dark_pixels(image: np.ndarray) -> int:
    return int((image.astype(int).mean(axis=2) < 75).sum())


def test_the_rover_is_drawn_where_it_is_and_only_if_it_is_in_frame():
    rig = CameraRig(open_world())
    without = rig.nadir_view(0.0, 0.0, 10.0, 0.0, WIDTH, HEIGHT)
    under = rig.nadir_view(0.0, 0.0, 10.0, 0.0, WIDTH, HEIGHT, rover=(0.0, 0.0, 0.0))
    far = rig.nadir_view(0.0, 0.0, 10.0, 0.0, WIDTH, HEIGHT, rover=(45.0, 45.0, 0.0))

    assert dark_pixels(under) > dark_pixels(without) + 6
    assert np.array_equal(far, without), "a vehicle outside the frame must not touch it"


def test_a_tree_throws_its_shadow_away_from_the_sun():
    """From above a crown is a green shape, and its shadow is the only thing
    that says it is tall. A point beyond the tree on the side away from the
    sun is darker than one the same distance out on the sunward side."""
    world = open_world()
    paint(world, Terrain.TREE, -3.0, 3.0, -3.0, 3.0)
    rig = CameraRig(world)

    altitude = 30.0
    image = rig.nadir_view(0.0, 0.0, altitude, 0.0, WIDTH, HEIGHT, fov_deg=84.0).astype(float)
    half_w = altitude * math.tan(math.radians(42.0))
    half_h = half_w * HEIGHT / WIDTH

    def brightness(wx: float, wy: float) -> float:
        col = int((wx / half_w * 0.5 + 0.5) * WIDTH)
        row = int((0.5 - wy / half_h * 0.5) * HEIGHT)
        return float(image[row - 1 : row + 2, col - 1 : col + 2].mean())

    sun = (math.sin(math.radians(SUN_AZIMUTH_DEG)), math.cos(math.radians(SUN_AZIMUTH_DEG)))
    reach = 6.0
    sunward = brightness(sun[0] * reach, sun[1] * reach)
    shaded = brightness(-sun[0] * reach, -sun[1] * reach)
    assert shaded < sunward * 0.8


def test_a_nadir_frame_is_ground_truth_not_the_map():
    """What is drawn at a point is what the world has there. A rock the rover
    has never scanned is still in a tricopter's picture."""
    world = open_world()
    paint(world, Terrain.ROCK, 4.0, 7.0, 4.0, 7.0)
    rig = CameraRig(world)
    assert rock_centroid(rig.nadir_view(5.5, 5.5, 8.0, 0.0, WIDTH, HEIGHT)) is not None


# --- the scouts' forward-looking camera ---------------------------------------


def scout(world, heading, x=0.0, y=0.0, agl=17.0, pitch=18.0, rover=None):
    return CameraRig(world).scout_view(
        x, y, agl, heading, WIDTH, HEIGHT, pitch_down_deg=pitch, rover=rover
    )


def sky_boundary_row(image: np.ndarray) -> float | None:
    """First row, from the top, that is ground rather than sky, down the
    middle of the frame; None if the frame is all one or the other."""
    column = image[:, WIDTH // 2 - 6 : WIDTH // 2 + 6].astype(int).mean(axis=1)
    skyish = column[:, 2] - column[:, 0] > 5  # bluer than it is red
    if skyish.all() or not skyish.any():
        return None
    return float(np.argmax(~skyish))


def test_a_scout_sees_where_it_is_facing_not_straight_down():
    world = open_world()
    paint(world, Terrain.TREE, -30.0, 30.0, 40.0, 46.0)
    rig = CameraRig(world)
    ahead = rig.scout_view(0.0, 0.0, 17.0, 0.0, WIDTH, HEIGHT)
    away = rig.scout_view(0.0, 0.0, 17.0, 180.0, WIDTH, HEIGHT)
    assert count_green(centre(ahead)) > 100, "a wall of trees 40 m ahead is in the picture"
    assert count_green(centre(away)) == 0


def test_the_horizon_sits_where_the_pitch_says_it_must():
    """The camera is tipped down by p, so the horizon is f*tan(p) above the
    centre of the frame. This is geometry, so it can be checked exactly."""
    world = open_world()
    focal = (WIDTH / 2.0) / math.tan(math.radians(84.0 / 2.0))
    for pitch in (0.0, 8.0, 15.0):
        row = sky_boundary_row(scout(world, 0.0, pitch=pitch, agl=3.0))
        expected = HEIGHT / 2.0 - focal * math.tan(math.radians(pitch))
        assert row is not None, pitch
        assert abs(row - expected) < 4.0, (pitch, row, expected)


def test_tipping_the_camera_down_moves_the_horizon_up_and_out_of_frame():
    world = open_world()
    levels = [sky_boundary_row(scout(world, 0.0, pitch=p, agl=3.0)) for p in (0.0, 10.0, 20.0)]
    assert levels[0] > levels[1], "more pitch, horizon higher in the frame"
    steep = scout(world, 0.0, pitch=40.0)
    assert sky_boundary_row(steep) is None and steep[2, WIDTH // 2, 0] >= steep[2, WIDTH // 2, 2] - 40, (
        "tipped far enough down there is no sky at all"
    )


def test_the_forward_view_of_a_feature_matches_its_bearing_and_distance():
    """A tall marker 30 m out, 10 m to the right of the line of sight, appears
    right of centre by atan(10/30) and not elsewhere."""
    world = open_world()
    paint(world, Terrain.TREE, 9.0, 11.0, 29.0, 31.0)
    image = scout(world, 0.0, agl=17.0, pitch=18.0)
    pixels = image.astype(int)
    green = (pixels[..., 1] > pixels[..., 0] + 6) & (pixels[..., 1] > pixels[..., 2] + 6)
    assert green.any()
    cols = np.nonzero(green)[1]
    focal = (WIDTH / 2.0) / math.tan(math.radians(84.0 / 2.0))
    expected = WIDTH / 2.0 + focal * (10.0 / 30.0)
    assert abs(cols.mean() - expected) < 6.0, (cols.mean(), expected)


def test_a_scout_sees_the_rover_when_it_is_in_front_of_it():
    world = open_world()
    heading = 0.0
    with_rover = scout(world, heading, rover=(0.0, 14.0, 0.0), agl=6.0, pitch=10.0)
    without = scout(world, heading, agl=6.0, pitch=10.0)
    assert not np.array_equal(with_rover, without)
    assert dark_pixels(with_rover) > dark_pixels(without) + 4


def test_a_scout_does_not_draw_the_rover_behind_it_or_out_of_the_frame():
    world = open_world()
    base = scout(world, 0.0, agl=6.0, pitch=10.0)
    behind = scout(world, 0.0, rover=(0.0, -14.0, 0.0), agl=6.0, pitch=10.0)
    wide = scout(world, 0.0, rover=(60.0, 5.0, 0.0), agl=6.0, pitch=10.0)
    assert np.array_equal(base, behind)
    assert np.array_equal(base, wide)


def test_something_between_the_scout_and_the_rover_hides_it():
    """The rover is only drawn where it is nearer than whatever else the pixel
    would show. A tree wall in front of it hides it."""
    open_ground = open_world()
    screened = open_world()
    paint(screened, Terrain.TREE, -10.0, 10.0, 8.0, 10.0)

    def visible(world):
        a = scout(world, 0.0, rover=(0.0, 20.0, 0.0), agl=8.0, pitch=8.0)
        b = scout(world, 0.0, agl=8.0, pitch=8.0)
        return int((a.astype(int) != b.astype(int)).any(axis=2).sum())

    assert visible(open_ground) > 3
    assert visible(screened) == 0


def test_a_scout_holds_height_over_the_ground_beneath_it_not_over_a_datum():
    """Terrain following. Raise the whole world by nine metres and nothing a
    scout sees may change, because what it holds is height *above the
    ground*, and the ground came up with it."""
    flat = open_world()
    raised = open_world()
    raised._elevation[:] = 9.0
    for world in (flat, raised):
        paint(world, Terrain.TREE, -4.0, 4.0, 30.0, 36.0)
        paint(world, Terrain.ROCK, 6.0, 8.0, 18.0, 20.0)

    on_flat = CameraRig(flat).scout_view(0.0, 0.0, 12.0, 0.0, WIDTH, HEIGHT, pitch_down_deg=14.0)
    on_raised = CameraRig(raised).scout_view(0.0, 0.0, 12.0, 0.0, WIDTH, HEIGHT, pitch_down_deg=14.0)
    assert np.array_equal(on_flat, on_raised)


def test_the_same_height_over_a_hill_sees_less_than_over_flat_ground():
    """The converse: a hill in front rises into the view, so a scout 12 m
    over the ground does not see what it would over a plain."""
    plain = open_world()
    hilly = open_world()
    paint(plain, Terrain.TREE, -20.0, 20.0, 50.0, 56.0)
    paint(hilly, Terrain.TREE, -20.0, 20.0, 50.0, 56.0)
    for cx in range(-120, 121):
        for cy in range(int(25.0 / CELL_SIZE_M), int(30.0 / CELL_SIZE_M)):
            index = hilly._index((cx, cy))
            if index:
                hilly._elevation[index] = 25.0

    seen_plain = count_green(centre(CameraRig(plain).scout_view(0.0, 0.0, 12.0, 0.0, WIDTH, HEIGHT, pitch_down_deg=6.0)))
    seen_hilly = count_green(centre(CameraRig(hilly).scout_view(0.0, 0.0, 12.0, 0.0, WIDTH, HEIGHT, pitch_down_deg=6.0)))
    assert seen_plain > 30
    assert seen_hilly == 0
