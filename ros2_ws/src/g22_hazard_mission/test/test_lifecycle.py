from unittest.mock import Mock

import pytest
import rclpy
from rclpy.executors import ExternalShutdownException

from g22_hazard_mission import run_node


@pytest.mark.parametrize('error', [None, KeyboardInterrupt(), ExternalShutdownException()])
def test_normal_and_interrupted_exits_release_ros_resources(monkeypatch, error):
    node = Mock()
    monkeypatch.setattr(rclpy, 'init', Mock())
    monkeypatch.setattr(rclpy, 'spin', Mock(side_effect=error))
    shutdown = Mock()
    monkeypatch.setattr(rclpy, 'try_shutdown', shutdown)
    run_node(lambda: node)
    node.destroy_node.assert_called_once()
    shutdown.assert_called_once()


def test_unexpected_spin_failure_is_not_hidden_and_still_cleans_up(monkeypatch):
    node = Mock()
    monkeypatch.setattr(rclpy, 'init', Mock())
    monkeypatch.setattr(rclpy, 'spin', Mock(side_effect=RuntimeError('spin failed')))
    shutdown = Mock()
    monkeypatch.setattr(rclpy, 'try_shutdown', shutdown)
    with pytest.raises(RuntimeError, match='spin failed'):
        run_node(lambda: node)
    node.destroy_node.assert_called_once()
    shutdown.assert_called_once()


def test_constructor_failure_still_shuts_down_ros(monkeypatch):
    monkeypatch.setattr(rclpy, 'init', Mock())
    shutdown = Mock()
    monkeypatch.setattr(rclpy, 'try_shutdown', shutdown)
    with pytest.raises(ValueError, match='invalid settings'):
        run_node(Mock(side_effect=ValueError('invalid settings')))
    shutdown.assert_called_once()


def test_cleanup_failure_does_not_leave_the_ros_context_running(monkeypatch):
    node = Mock()
    node.destroy_node.side_effect = RuntimeError('destroy failed')
    monkeypatch.setattr(rclpy, 'init', Mock())
    monkeypatch.setattr(rclpy, 'spin', Mock())
    shutdown = Mock()
    monkeypatch.setattr(rclpy, 'try_shutdown', shutdown)
    with pytest.raises(RuntimeError, match='destroy failed'):
        run_node(lambda: node)
    shutdown.assert_called_once()
