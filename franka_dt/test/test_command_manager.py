from unittest.mock import Mock

from franka_dt.command_manager import CommandManager
from franka_dt_interfaces.msg import DigitalTwinCommand
import pytest
import rclpy
from sensor_msgs.msg import JointState


@pytest.fixture
def command_manager():
    """Crea un Command Manager per il test e lo distrugge alla fine."""
    rclpy.init()
    node = CommandManager()

    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def create_command(positions, command_id='test-command-001'):
    """Crea un DigitalTwinCommand valido."""
    message = DigitalTwinCommand()
    message.command_id = command_id
    message.emission_timestamp.sec = 1
    message.emission_timestamp.nanosec = 0
    message.target_positions = positions

    return message


def test_rejects_wrong_number_of_positions(command_manager):
    """Rifiuta un comando che non contiene sette posizioni."""
    command_manager.publish_status = Mock()

    message = create_command([0.0])

    command_manager.command_callback(message)

    command_manager.publish_status.assert_called_once_with(
        'rejected',
        'invalid_joint_count',
        'test-command-001',
        1.0
    )

    assert command_manager.active_command is None


@pytest.mark.parametrize(
    'invalid_value',
    [float('nan'), float('inf'), float('-inf')]
)
def test_rejects_non_finite_positions(command_manager, invalid_value):
    """Rifiuta comandi contenenti posizioni non finite."""
    command_manager.publish_status = Mock()

    positions = [0.0] * 7
    positions[2] = invalid_value

    message = create_command(positions)

    command_manager.command_callback(message)

    command_manager.publish_status.assert_called_once_with(
        'rejected',
        'non_finite_position',
        'test-command-001',
        1.0
    )

    assert command_manager.active_command is None


def test_rejects_command_when_another_is_active(command_manager):
    """Rifiuta un nuovo comando se ce n'è già uno attivo."""
    command_manager.publish_status = Mock()
    command_manager.command_publisher.publish = Mock()

    first_command = {
        joint: 0.1
        for joint in command_manager.joint_names
    }

    command_manager.active_command = first_command.copy()
    command_manager.command_start_time = 100.0

    message = create_command(
        [0.2] * 7,
        command_id='test-command-002'
    )

    command_manager.command_callback(message)

    command_manager.publish_status.assert_called_once_with(
        'rejected',
        'command_already_active',
        'test-command-002',
        1.0
    )

    assert command_manager.active_command == first_command

    command_manager.command_publisher.publish.assert_not_called()


def test_detects_stable_target(command_manager, monkeypatch):
    """Verifica che il target venga rilevato come stabile."""
    command_manager.publish_status = Mock()

    target = {
        joint: 0.1
        for joint in command_manager.joint_names
    }

    command_manager.active_command = target.copy()
    command_manager.command_start_time = 100.0
    command_manager.target_reached_time = None
    command_manager.target_stable_logged = False

    message = JointState()
    message.name = command_manager.joint_names
    message.position = [0.1] * 7

    monkeypatch.setattr('time.monotonic', lambda: 101.0)

    command_manager.joint_state_callback(message)

    assert command_manager.target_reached_time == 101.0
    assert command_manager.active_command is not None

    for step in range(1, 14):
        timestamp = 101.0 + step * 0.04

        monkeypatch.setattr(
            'time.monotonic',
            lambda timestamp=timestamp: timestamp
        )

        command_manager.joint_state_callback(message)

    assert command_manager.target_stable_logged is True
    assert command_manager.active_command is not None

    command_manager.publish_status.assert_not_called()


def test_resets_stability_after_delayed_update(
    command_manager,
    monkeypatch
):
    """Azzera il conteggio se gli aggiornamenti sono troppo distanti."""
    command_manager.publish_status = Mock()

    target = {
        joint: 0.1
        for joint in command_manager.joint_names
    }

    command_manager.active_command = target.copy()
    command_manager.command_start_time = 99.0

    message = JointState()
    message.name = command_manager.joint_names
    message.position = [0.1] * 7

    monkeypatch.setattr('time.monotonic', lambda: 100.0)

    command_manager.joint_state_callback(message)

    assert command_manager.target_reached_time == 100.0

    monkeypatch.setattr('time.monotonic', lambda: 100.04)

    command_manager.joint_state_callback(message)

    assert command_manager.target_reached_time == 100.0
    assert command_manager.active_command is not None

    monkeypatch.setattr('time.monotonic', lambda: 100.10)

    command_manager.joint_state_callback(message)

    assert command_manager.target_reached_time == 100.10
    assert command_manager.last_target_update_time == 100.10
    assert command_manager.active_command is not None

    command_manager.publish_status.assert_not_called()


def test_logs_final_joint_errors(command_manager, monkeypatch):
    """Verifica il calcolo degli errori articolari finali."""
    command_manager.active_command = {
        joint: 0.1
        for joint in command_manager.joint_names
    }

    command_manager.active_command_id = 'test-command-001'
    command_manager.position_tolerance = 0.01

    command_manager.current_positions = {
        'fr3_joint1': 0.102,
        'fr3_joint2': 0.104,
        'fr3_joint3': 0.101,
        'fr3_joint4': 0.103,
        'fr3_joint5': 0.105,
        'fr3_joint6': 0.102,
        'fr3_joint7': 0.106
    }

    logger_mock = Mock()

    monkeypatch.setattr(
        command_manager,
        'get_logger',
        lambda: logger_mock
    )

    command_manager.log_final_errors()

    log_messages = [
        call.args[0]
        for call in logger_mock.info.call_args_list
    ]

    assert any(
        'fr3_joint7: errore = 0.006000 rad' in message
        for message in log_messages
    )

    assert any(
        'Errore massimo: 0.006000 rad' in message
        for message in log_messages
    )

    assert any(
        'Tolleranza utilizzata: 0.010000 rad' in message
        for message in log_messages
    )


def test_logs_errors_when_command_is_completed(command_manager):
    """Registra gli errori quando il comando viene completato."""
    command_manager.active_command = {
        joint: 0.1
        for joint in command_manager.joint_names
    }

    command_manager.active_command_id = 'test-command-001'

    command_manager.log_final_errors = Mock()
    command_manager.publish_status = Mock()

    command_manager.finish_command(
        'completed',
        'target_reached'
    )

    command_manager.log_final_errors.assert_called_once_with()

    command_manager.publish_status.assert_called_once_with(
        'completed',
        'target_reached',
        completion_state_timestamp_ns=None
    )

    assert command_manager.active_command is None


def test_does_not_log_errors_when_command_fails(command_manager):
    """Non registra gli errori quando il comando fallisce."""
    command_manager.active_command = {
        joint: 0.1
        for joint in command_manager.joint_names
    }

    command_manager.active_command_id = 'test-command-002'

    command_manager.log_final_errors = Mock()
    command_manager.publish_status = Mock()

    command_manager.finish_command(
        'failed',
        'timeout'
    )

    command_manager.log_final_errors.assert_not_called()

    command_manager.publish_status.assert_called_once_with(
        'failed',
        'timeout',
        completion_state_timestamp_ns=None
    )

    assert command_manager.active_command is None
