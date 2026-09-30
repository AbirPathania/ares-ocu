from ocu.core.geofence import Geofence


def test_unconfigured_geofence_never_breaches():
    fence = Geofence()
    assert fence.configured is False
    assert fence.check_breach(1000, 1000) is None


def test_square_geofence_contains_and_breach():
    fence = Geofence(vertices=[(0, 0), (10, 0), (10, 10), (0, 10)], breach_action="return")
    assert fence.contains(5, 5) is True
    assert fence.contains(50, 50) is False
    assert fence.check_breach(5, 5) is None
    assert fence.check_breach(50, 50) == "return"


def test_invalid_breach_action_rejected():
    try:
        Geofence(vertices=[(0, 0), (1, 0), (1, 1)], breach_action="explode")
        assert False, "expected ValueError"
    except ValueError:
        pass
