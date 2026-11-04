"""Route speed zones must also constrain retimed, frozen geometry."""

import pytest

from competition_planning.trajectory_parameterizer import retime_continuous_trajectory


def reference_line():
    points = [
        {"x": i * 0.1, "y": 0.0, "yaw": 0.0, "s": i * 0.1, "curvature": 0.0}
        for i in range(601)
    ]
    points[200]["ref_id"] = "turn_entry"
    points[240]["ref_id"] = "turn_exit"
    return {"ok": True, "frame_id": "map", "points": points}


def optimizer():
    return {"continuous_trajectory_optimizer": {
        "spatial_speed_envelope": True,
        "max_speed_mps": 0.5,
        "max_acceleration_mps2": 0.12,
        "max_deceleration_mps2": 0.18,
        "max_jerk_mps3": 0.2,
        "max_lateral_acceleration_mps2": 0.12,
        "max_curvature_rate_1pmps": 2.5,
    }}


def speed_profile(end_ref="turn_exit"):
    return {"default_max_speed_mps": 0.5, "segments": [{
        "id": "turn", "start_ref": "turn_entry", "end_ref": end_ref,
        "max_speed_mps": 0.3,
    }]}


def test_retime_respects_turn_zone_and_brakes_before_entry():
    artifact = reference_line()
    result = retime_continuous_trajectory(
        artifact, optimizer(), route_speed_profile=speed_profile(),
    )
    assert result.ok
    for original, actual in zip(artifact["points"], result.points):
        assert (actual.x, actual.y, actual.yaw, actual.s, actual.curvature) == (
            original["x"], original["y"], original["yaw"],
            original["s"], original["curvature"],
        )
    assert max(point.v for point in result.points[200:241]) <= 0.3 + 1e-9
    assert max(point.v for point in result.points[:150]) == pytest.approx(0.5)
    assert max(point.v for point in result.points[300:500]) == pytest.approx(0.5)
    assert result.points[199].v < 0.5
    assert result.points[0].v == result.points[-1].v == 0.0


def test_retime_rejects_missing_zone_endpoint():
    with pytest.raises(ValueError, match="end is not on path"):
        retime_continuous_trajectory(
            reference_line(), optimizer(), route_speed_profile=speed_profile("missing"),
        )


def test_retime_without_route_profile_keeps_previous_speed_limit():
    result = retime_continuous_trajectory(reference_line(), optimizer())
    assert result.ok
    assert max(point.v for point in result.points[200:241]) == pytest.approx(0.5)
