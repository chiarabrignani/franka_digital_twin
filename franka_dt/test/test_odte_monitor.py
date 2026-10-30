"""Unit tests for the ODTE monitor."""

from collections import deque
import json
from unittest.mock import Mock

from franka_dt.odte_monitor import ODTEMonitor
from std_msgs.msg import String


def test_rejected_command_is_not_counted():
    """Ignore rejected commands in Digital -> Physical metrics."""
    monitor = ODTEMonitor.__new__(ODTEMonitor)

    monitor.accepted_commands = {}
    monitor.command_results = {}
    monitor.command_observations = Mock()
    monitor.get_logger = Mock()

    message = String()
    message.data = json.dumps({
        'status': 'rejected',
        'command_id': None,
    })

    monitor.command_status_callback(message)

    assert monitor.accepted_commands == {}
    assert monitor.command_results == {}
    monitor.command_observations.append.assert_not_called()


def test_executing_command_is_registered_only_once(monkeypatch):
    """Register an accepted command only once."""
    monitor = ODTEMonitor.__new__(ODTEMonitor)

    monitor.accepted_commands = {}
    monitor.command_results = {}
    monitor.command_observations = []
    monitor.get_logger = Mock()

    reception_time = 100.0
    monkeypatch.setattr(
        'franka_dt.odte_monitor.time.monotonic',
        lambda: reception_time,
    )

    message = String()
    message.data = json.dumps({
        'status': 'executing',
        'command_id': 'command-001',
    })

    monitor.command_status_callback(message)
    monitor.command_status_callback(message)

    assert len(monitor.accepted_commands) == 1
    assert 'command-001' in monitor.accepted_commands
    assert len(monitor.command_observations) == 1


def test_final_status_is_associated_with_command_id(monkeypatch):
    """Associate final command outcomes with the correct command ID."""
    monitor = ODTEMonitor.__new__(ODTEMonitor)

    monitor.accepted_commands = {
        'command-001': {'accepted_at': 100.0},
        'command-002': {'accepted_at': 101.0},
    }
    monitor.command_results = {}
    monitor.command_observations = [
        (100.0, 'command-001'),
        (101.0, 'command-002'),
    ]
    monitor.get_logger = Mock()

    monkeypatch.setattr(
        'franka_dt.odte_monitor.time.monotonic',
        lambda: 102.0,
    )

    completed_message = String()
    completed_message.data = json.dumps({
        'status': 'completed',
        'command_id': 'command-001',
        'elapsed_seconds': 2.0,
    })

    failed_message = String()
    failed_message.data = json.dumps({
        'status': 'failed',
        'command_id': 'command-002',
        'elapsed_seconds': 3.0,
    })

    monitor.command_status_callback(completed_message)
    monitor.command_status_callback(failed_message)

    assert monitor.command_results['command-001']['status'] == 'completed'
    assert monitor.command_results['command-001']['elapsed_seconds'] == 2.0

    assert monitor.command_results['command-002']['status'] == 'failed'
    assert monitor.command_results['command-002']['elapsed_seconds'] == 3.0

    assert len(monitor.accepted_commands) == 2


def test_expired_executing_command_is_removed():
    """Remove accepted commands older than the observation window."""
    monitor = ODTEMonitor.__new__(ODTEMonitor)

    monitor.window = 10.0
    monitor.accepted_commands = {
        'command-001': {'accepted_at': 100.0},
        'command-002': {'accepted_at': 105.0},
    }
    monitor.command_results = {}
    monitor.command_observations = deque([
        (100.0, 'command-001'),
        (105.0, 'command-002'),
    ])

    monitor.remove_old_command_observations(now=111.0)

    assert 'command-001' not in monitor.accepted_commands
    assert 'command-001' not in monitor.command_results

    assert 'command-002' in monitor.accepted_commands
    assert len(monitor.command_observations) == 1
    assert monitor.command_observations[0][1] == 'command-002'


def test_late_final_status_does_not_restore_expired_command(
    monkeypatch,
):
    """Ignore final outcomes for commands that have expired."""
    monitor = ODTEMonitor.__new__(ODTEMonitor)

    monitor.window = 10.0
    monitor.accepted_commands = {
        'command-001': {'accepted_at': 100.0},
    }
    monitor.command_results = {}
    monitor.command_observations = deque([
        (100.0, 'command-001'),
    ])
    monitor.get_logger = Mock()

    # La finestra temporale è scaduta.
    monitor.remove_old_command_observations(now=111.0)

    # Arriva in ritardo l'esito del comando ormai scaduto.
    monkeypatch.setattr(
        'franka_dt.odte_monitor.time.monotonic',
        lambda: 112.0,
    )

    message = String()
    message.data = json.dumps({
        'status': 'completed',
        'command_id': 'command-001',
        'elapsed_seconds': 2.0,
    })

    monitor.command_status_callback(message)

    assert 'command-001' not in monitor.accepted_commands
    assert 'command-001' not in monitor.command_results
    assert len(monitor.command_observations) == 0


def test_digital_to_physical_metrics():
    """Calculate D->P metrics from accepted commands and outcomes."""
    monitor = ODTEMonitor.__new__(ODTEMonitor)

    monitor.window = 10.0
    monitor.command_timeliness = 10.0
    monitor.heartbeat_timeout = 1.5
    monitor.expected_heartbeat_rate = 2.0
    monitor.start_time = 100.0
    monitor.heartbeat_observations = deque([108.0, 109.0])

    monitor.accepted_commands = {
        'command-001': {'accepted_at': 100.0},
        'command-002': {'accepted_at': 101.0},
        'command-003': {'accepted_at': 102.0},
        'command-004': {'accepted_at': 103.0},
    }

    monitor.command_observations = deque([
        (100.0, 'command-001'),
        (101.0, 'command-002'),
        (102.0, 'command-003'),
        (103.0, 'command-004'),
    ])

    monitor.command_results = {
        'command-001': {
            'status': 'completed',
            'elapsed_seconds': 2.0,
            'received_at': 104.0,
        },
        'command-002': {
            'status': 'completed',
            'elapsed_seconds': 12.0,
            'received_at': 105.0,
        },
        'command-003': {
            'status': 'failed',
            'elapsed_seconds': 3.0,
            'received_at': 106.0,
        },
    }

    monitor.last_heartbeat_time = 109.5
    monitor.publisher = Mock()
    monitor.get_logger = Mock()

    monitor.calculate_digital_to_physical(now=110.0)

    monitor.publisher.publish.assert_called_once()

    published_message = monitor.publisher.publish.call_args.args[0]
    metrics = json.loads(published_message.data)

    assert metrics['direction'] == 'digital_to_physical'
    assert metrics['accepted_commands'] == 4
    assert metrics['completed_commands'] == 2
    assert metrics['timeliness'] == 0.25
    assert metrics['reliability'] == 0.5
    assert metrics['availability'] == 0.1
    assert metrics['odte'] == 0.0125


def test_digital_to_physical_odte_is_zero_with_stale_heartbeat():
    """Set availability and ODTE to zero when heartbeat is stale."""
    monitor = ODTEMonitor.__new__(ODTEMonitor)

    monitor.window = 10.0
    monitor.command_timeliness = 10.0
    monitor.heartbeat_timeout = 1.5
    monitor.expected_heartbeat_rate = 2.0
    monitor.start_time = 90.0
    monitor.heartbeat_observations = deque()

    monitor.accepted_commands = {
        'command-001': {'accepted_at': 100.0},
    }

    monitor.command_observations = deque([
        (100.0, 'command-001'),
    ])

    monitor.command_results = {
        'command-001': {
            'status': 'completed',
            'elapsed_seconds': 2.0,
            'received_at': 101.0,
        },
    }

    # L'ultimo heartbeat è più vecchio del timeout.
    monitor.last_heartbeat_time = 100.0

    monitor.publisher = Mock()
    monitor.get_logger = Mock()

    monitor.calculate_digital_to_physical(now=102.0)

    monitor.publisher.publish.assert_called_once()

    published_message = monitor.publisher.publish.call_args.args[0]
    metrics = json.loads(published_message.data)

    assert metrics['timeliness'] == 1.0
    assert metrics['reliability'] == 1.0
    assert metrics['availability'] == 0.0
    assert metrics['odte'] == 0.0


def test_availability_below_expected_heartbeat_rate():
    """Availability is proportional to a lower heartbeat rate."""
    monitor = ODTEMonitor.__new__(ODTEMonitor)
    monitor.window = 10.0
    monitor.command_timeliness = 10.0
    monitor.expected_heartbeat_rate = 2.0
    monitor.start_time = 100.0
    monitor.heartbeat_observations = deque([102.0, 104.0, 106.0])
    monitor.accepted_commands = {
        'command-001': {'accepted_at': 100.0},
    }
    monitor.command_observations = deque([(100.0, 'command-001')])
    monitor.command_results = {
        'command-001': {
            'status': 'completed',
            'elapsed_seconds': 2.0,
            'received_at': 101.0,
        },
    }
    monitor.publisher = Mock()
    monitor.get_logger = Mock()

    monitor.calculate_digital_to_physical(now=110.0)

    metrics = json.loads(
        monitor.publisher.publish.call_args.args[0].data
    )
    assert metrics['availability'] == 0.15


def test_availability_below_expected_heartbeat_rate_1hz():
    """Availability is 0.5 when the measured rate is 1 Hz."""
    monitor = ODTEMonitor.__new__(ODTEMonitor)
    monitor.window = 10.0
    monitor.command_timeliness = 10.0
    monitor.expected_heartbeat_rate = 2.0
    monitor.start_time = 100.0
    monitor.heartbeat_observations = deque(
        [101.0, 102.0, 103.0, 104.0, 105.0,
         106.0, 107.0, 108.0, 109.0, 110.0]
    )
    monitor.accepted_commands = {
        'command-001': {'accepted_at': 100.0},
    }
    monitor.command_observations = deque([(100.0, 'command-001')])
    monitor.command_results = {
        'command-001': {
            'status': 'completed',
            'elapsed_seconds': 2.0,
            'received_at': 101.0,
        },
    }
    monitor.publisher = Mock()
    monitor.get_logger = Mock()

    monitor.calculate_digital_to_physical(now=110.0)

    metrics = json.loads(
        monitor.publisher.publish.call_args.args[0].data
    )
    assert metrics['availability'] == 0.5


def test_availability_is_capped_at_one():
    """Availability never exceeds one."""
    monitor = ODTEMonitor.__new__(ODTEMonitor)
    monitor.window = 10.0
    monitor.command_timeliness = 10.0
    monitor.expected_heartbeat_rate = 2.0
    monitor.start_time = 100.0
    monitor.heartbeat_observations = deque(
        [100.4 + 0.4 * i for i in range(25)]
    )
    monitor.accepted_commands = {
        'command-001': {'accepted_at': 100.0},
    }
    monitor.command_observations = deque([(100.0, 'command-001')])
    monitor.command_results = {
        'command-001': {
            'status': 'completed',
            'elapsed_seconds': 2.0,
            'received_at': 101.0,
        },
    }
    monitor.publisher = Mock()
    monitor.get_logger = Mock()

    monitor.calculate_digital_to_physical(now=110.0)

    metrics = json.loads(
        monitor.publisher.publish.call_args.args[0].data
    )
    assert metrics['availability'] == 1.0


def test_digital_to_physical_metrics_without_accepted_commands():
    """Publish D->P metrics even when no commands are accepted."""
    monitor = ODTEMonitor.__new__(ODTEMonitor)

    monitor.window = 10.0
    monitor.command_timeliness = 10.0
    monitor.expected_heartbeat_rate = 2.0
    monitor.start_time = 100.0

    # Dieci heartbeat positivi in una finestra di 10 secondi.
    monitor.heartbeat_observations = deque([
        101.0, 102.0, 103.0, 104.0, 105.0,
        106.0, 107.0, 108.0, 109.0, 110.0,
    ])

    # Nessun comando accettato.
    monitor.accepted_commands = {}
    monitor.command_observations = deque()
    monitor.command_results = {}

    monitor.publisher = Mock()
    monitor.get_logger = Mock()

    monitor.calculate_digital_to_physical(now=110.0)

    # Verifica che il monitor abbia pubblicato le metriche.
    monitor.publisher.publish.assert_called_once()

    published_message = monitor.publisher.publish.call_args.args[0]
    metrics = json.loads(published_message.data)

    assert metrics['direction'] == 'digital_to_physical'
    assert metrics['accepted_commands'] == 0
    assert metrics['completed_commands'] == 0
    assert metrics['timeliness'] == 0.0
    assert metrics['reliability'] == 0.0
    assert metrics['availability'] == 0.5
    assert metrics['odte'] == 0.0
