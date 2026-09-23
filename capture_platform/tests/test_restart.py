import pytest
from rebot_capture.processes import wait_for_exit


def test_waits_for_device_cleanup_after_listener_has_closed():
    elapsed = []
    wait_for_exit([1], alive=lambda pid: len(elapsed) < 8, sleep=elapsed.append)
    assert len(elapsed) == 8


def test_timeout_never_proceeds_to_restart():
    with pytest.raises(RuntimeError, match='设备清理'):
        wait_for_exit([1], timeout=.5, alive=lambda pid: True, sleep=lambda _: None)


def test_already_exited_returns_without_delay():
    def unexpected_sleep(_):
        pytest.fail('Should not sleep after process exits')
    wait_for_exit([1], alive=lambda pid: False, sleep=unexpected_sleep)
