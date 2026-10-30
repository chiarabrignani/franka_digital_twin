from unittest.mock import Mock

from franka_dt.command_manager import CommandManager
import pytest
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


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


def test_rejects_wrong_number_of_positions(command_manager):
    """Rifiuta un comando che non contiene sette posizioni."""
    command_manager.publish_status = Mock()

    message = Float64MultiArray()
    message.data = [0.0]

    command_manager.command_callback(message)

    command_manager.publish_status.assert_called_once_with(
        'rejected',
        'invalid_joint_count'
    )

    assert command_manager.active_command is None


@pytest.mark.parametrize(
    'invalid_value',
    [float('nan'), float('inf'), float('-inf')]
)
def test_rejects_non_finite_positions(command_manager, invalid_value):
    """Rifiuta comandi contenenti posizioni non finite."""
    command_manager.publish_status = Mock()

    message = Float64MultiArray()
    message.data = [0.0] * 7
    message.data[2] = invalid_value

    command_manager.command_callback(message)

    command_manager.publish_status.assert_called_once_with(
        'rejected',
        'non_finite_position'
    )

    assert command_manager.active_command is None


def test_rejects_command_when_another_is_active(command_manager):
    """Rifiuta un nuovo comando se ce n'è già uno attivo."""
    command_manager.publish_status = Mock()
    command_manager.command_publisher.publish = Mock()

    # Simula un comando già in esecuzione
    first_command = {
        joint: 0.1
        for joint in command_manager.joint_names
    }
    command_manager.active_command = first_command.copy()
    command_manager.command_start_time = 100.0

    # Prova a inviare un secondo comando
    message = Float64MultiArray()
    message.data = [0.2] * 7

    command_manager.command_callback(message)

    # Il secondo comando deve essere rifiutato
    command_manager.publish_status.assert_called_once_with(
        'rejected',
        'command_already_active'
    )

    # Il primo comando deve rimanere invariato
    assert command_manager.active_command == first_command

    # Nessun comando deve essere inoltrato al controller
    command_manager.command_publisher.publish.assert_not_called()


def test_completes_command_after_stable_target(command_manager, monkeypatch):
    """Completa il comando dopo aver mantenuto il bersaglio."""
    # Evita pubblicazioni ROS effettive durante il test
    command_manager.publish_status = Mock()
    monkeypatch.setattr(
        command_manager.command_publisher,
        'publish',
        Mock()
    )

    # Imposta un comando obiettivo per tutti i giunti
    target = {
        joint: 0.1
        for joint in command_manager.joint_names
    }
    command_manager.active_command = target.copy()
    command_manager.command_start_time = 100.0
    command_manager.target_reached_time = None

    # Prepara uno stato del robot esattamente sul bersaglio
    message = JointState()
    message.name = command_manager.joint_names
    message.position = [0.1] * 7

    # Simula il raggiungimento del bersaglio
    monkeypatch.setattr('time.monotonic', lambda: 101.0)
    command_manager.joint_state_callback(message)

    assert command_manager.target_reached_time == 101.0
    assert command_manager.active_command is not None

    # Simula il mantenimento del bersaglio per 0,5 secondi
    monkeypatch.setattr('time.monotonic', lambda: 101.6)
    command_manager.joint_state_callback(message)

    command_manager.publish_status.assert_called_once_with(
        'completed',
        'target_reached'
    )
    assert command_manager.active_command is None
