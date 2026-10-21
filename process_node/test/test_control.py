"""统一控制接口的停止优先级、单位边界与基线映射。"""

from dataclasses import replace

import pytest

from process_node.control import (
    BaselineControlConfig, BaselineController, ControlInput, ControlReference,
)
from process_node.motion import MotionEstimator
from test_motion import odom


def feedback():
    estimator = MotionEstimator()
    estimator.update(odom(1), received_at=10)
    return estimator.update(odom(1.1), received_at=10.1)


def test_reference_maps_radians_and_clamps_around_calibrated_center():
    controller = BaselineController(BaselineControlConfig(steer_center=20))
    reference = ControlReference(round(1.1e9), 0.5)
    command = controller.compute(ControlInput(reference, feedback()))
    assert command.status == 'OK'
    assert (command.steer, command.speed) == (-130, -100)
    command = controller.compute(ControlInput(replace(reference, heading_error_rad=3), feedback()))
    assert command.steer == 20 - 600


def test_stop_overrides_reference_and_missing_feedback():
    controller = BaselineController()
    reference = ControlReference(1, None, stop_requested=True, reason='没有通道')
    command = controller.compute(ControlInput(reference, MotionEstimator().feedback()))
    assert command.status == 'STOP_REQUESTED'
    assert command.speed == 0
    assert command.reason == '没有通道'


def test_missing_feedback_stops_by_default_and_observation_override_is_explicit():
    missing = MotionEstimator().feedback()
    inputs = ControlInput(ControlReference(1, 0.5), missing)
    assert BaselineController().compute(inputs).status == 'WAITING_ODOM'
    controller = BaselineController(BaselineControlConfig(require_motion_feedback=False))
    assert controller.compute(inputs).speed == -100


@pytest.mark.parametrize('angle', [None, float('nan'), float('inf')])
def test_invalid_direction_never_produces_motion(angle):
    command = BaselineController().compute(ControlInput(ControlReference(1, angle), feedback()))
    assert command.status == 'INVALID_REFERENCE'
    assert command.speed == 0


def test_baseline_does_not_silently_ignore_physical_speed_request():
    reference = ControlReference(1, 0.0, target_speed_mps=0.2)
    command = BaselineController().compute(ControlInput(reference, feedback()))
    assert command.status == 'UNSUPPORTED_REFERENCE'
    assert command.speed == 0


def test_invalid_reference_time_does_not_produce_motion():
    command = BaselineController().compute(ControlInput(ControlReference(-1, 0.0), feedback()))
    assert command.status == 'INVALID_REFERENCE'
    assert command.speed == 0
