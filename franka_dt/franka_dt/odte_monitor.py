"""Monitor bidirectional Digital Twin entanglement metrics."""

from collections import deque

import json
import math
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String


class ODTEMonitor(Node):
    """Monitor synchronization metrics between physical and digital twins."""

    def __init__(self):
        """Initialize parameters, subscriptions, and publishers."""
        super().__init__('odte_monitor')

        self.declare_parameter('expected_rate_hz', 100.0)
        self.declare_parameter('desired_timeliness_ms', 50.0)
        self.declare_parameter('window_seconds', 10.0)
        self.declare_parameter('heartbeat_timeout', 1.5)
        self.declare_parameter('expected_heartbeat_rate_hz', 2.0)
        self.declare_parameter('command_timeliness_ms', 10000.0)

        self.expected_rate = float(
            self.get_parameter('expected_rate_hz').value
        )

        self.desired_timeliness = float(
            self.get_parameter('desired_timeliness_ms').value
        ) / 1000.0

        self.window = float(
            self.get_parameter('window_seconds').value
        )

        self.heartbeat_timeout = float(
            self.get_parameter('heartbeat_timeout').value
        )

        self.expected_heartbeat_rate = float(
            self.get_parameter('expected_heartbeat_rate_hz').value
        )

        self.command_timeliness = float(
            self.get_parameter('command_timeliness_ms').value
        ) / 1000.0

        if (
            self.expected_rate <= 0
            or self.desired_timeliness <= 0
            or self.window <= 0
            or self.heartbeat_timeout <= 0
            or self.expected_heartbeat_rate <= 0
            or self.command_timeliness <= 0
        ):
            raise ValueError(
                'I parametri ODTE devono essere positivi.'
            )

        # Ogni osservazione contiene:
        # (tempo monotono di ricezione, timeliness in secondi).
        self.observations = deque()
        self.first_observation_time = None

        # Stato e tempi di ricezione degli heartbeat del lato fisico.
        self.physical_heartbeat = False
        self.last_heartbeat_time = None
        self.start_time = time.monotonic()
        self.heartbeat_observations = deque()

        # Osservazioni dei comandi per il percorso Digital -> Physical.
        self.command_observations = deque()

        # Comandi accettati, indicizzati tramite command_id.
        self.accepted_commands = {}

        # Esiti finali dei comandi, indicizzati tramite command_id.
        self.command_results = {}

        # Riceve i dati temporali misurati da state_manager.
        self.subscription = self.create_subscription(
            String,
            '/digital_twin/state_timing',
            self.timing_callback,
            10,
        )

        self.heartbeat_subscription = self.create_subscription(
            Bool,
            '/digital_twin/physical_heartbeat',
            self.heartbeat_callback,
            10,
        )

        self.command_status_subscription = self.create_subscription(
            String,
            '/digital_twin/command_status',
            self.command_status_callback,
            10,
        )

        self.publisher = self.create_publisher(
            String,
            '/digital_twin/odte',
            10,
        )

        self.timer = self.create_timer(
            1.0,
            self.calculate_odte,
        )

        self.get_logger().info(
            'ODTE Monitor avviato: percorso Physical -> Digital, '
            'con monitoraggio heartbeat fisico.'
        )

    def heartbeat_callback(self, msg):
        """Registra lo stato e l'istante di ricezione dell'heartbeat."""
        now = time.monotonic()
        self.physical_heartbeat = msg.data

        if msg.data:
            self.last_heartbeat_time = now
            self.heartbeat_observations.append(now)

    def command_status_callback(self, msg):
        """Collect command events for Digital -> Physical metrics."""
        reception_time = time.monotonic()

        try:
            data = json.loads(msg.data)

        except (ValueError, TypeError):
            self.get_logger().warning(
                'Evento di comando ricevuto in formato non valido.'
            )
            return

        status = data.get('status')
        command_id = data.get('command_id')

        # I comandi rifiutati non sono comandi accettati.
        if status == 'rejected':
            return

        # Gli altri eventi devono essere associati a un comando.
        if not isinstance(command_id, str) or not command_id:
            self.get_logger().warning(
                'Evento di comando privo di command_id valido.'
            )
            return

        if status == 'executing':
            # Registra il comando una sola volta, al momento
            # della prima ricezione dello stato executing.
            if command_id not in self.accepted_commands:
                self.accepted_commands[command_id] = {
                    'accepted_at': reception_time,
                }

                # La coda contiene i tempi di accettazione,
                # non quelli degli esiti finali.
                self.command_observations.append(
                    (reception_time, command_id)
                )

            return

        if status not in ('completed', 'failed'):
            return

        # Ignora gli esiti di comandi che il monitor non ha osservato
        # come accettati.
        if command_id not in self.accepted_commands:
            return

        # Non conteggiare due volte un esito finale.
        if command_id in self.command_results:
            return

        try:
            elapsed_seconds = float(data['elapsed_seconds'])

        except (KeyError, TypeError, ValueError):
            self.get_logger().warning(
                'Durata del comando mancante o non valida.'
            )
            return

        if not math.isfinite(elapsed_seconds) or elapsed_seconds < 0:
            self.get_logger().warning(
                'Durata del comando non valida.'
            )
            return

        self.command_results[command_id] = {
            'status': status,
            'elapsed_seconds': elapsed_seconds,
            'received_at': reception_time,
        }

    def timing_callback(self, msg):
        """Process timing observations from the state manager."""
        reception_monotonic = time.monotonic()

        try:
            timing_data = json.loads(msg.data)

            generation_ros = float(
                timing_data['generation_timestamp']
            )
            reception_ros = float(
                timing_data['reception_timestamp']
            )
            processing_time = float(
                timing_data['processing_time_seconds']
            )

        except (ValueError, TypeError, KeyError):
            self.get_logger().warning(
                'Dati temporali ricevuti in formato non valido.'
            )
            return

        # Rifiuta timestamp o tempi di elaborazione non validi.
        if (
            not all(
                math.isfinite(value)
                for value in (
                    generation_ros,
                    reception_ros,
                    processing_time,
                )
            )
            or generation_ros <= 0
            or reception_ros < generation_ros
            or processing_time < 0
        ):
            return

        # Timeliness Physical -> Digital:
        # ricezione - generazione + elaborazione.
        timeliness = (
            reception_ros
            - generation_ros
            + processing_time
        )

        self.observations.append(
            (reception_monotonic, timeliness)
        )

        if self.first_observation_time is None:
            self.first_observation_time = reception_monotonic

        self.remove_old_observations(reception_monotonic)

    def remove_old_observations(self, now):
        """Remove observations older than the configured window."""
        while (
            self.observations
            and now - self.observations[0][0] > self.window
        ):
            self.observations.popleft()

    def remove_old_command_observations(self, now):
        """Remove accepted commands older than the time window."""
        while (
            self.command_observations
            and now - self.command_observations[0][0] > self.window
        ):
            _, command_id = self.command_observations.popleft()

            # Elimina sia il comando accettato sia il suo eventuale esito.
            self.accepted_commands.pop(command_id, None)
            self.command_results.pop(command_id, None)

    def remove_old_heartbeat_observations(self, now):
        """Remove heartbeat observations older than the time window."""
        while (
            self.heartbeat_observations
            and now - self.heartbeat_observations[0] > self.window
        ):
            self.heartbeat_observations.popleft()

    def calculate_odte(self):
        """Calculate and publish the current ODTE metrics."""
        now = time.monotonic()

        self.calculate_digital_to_physical(now)

        self.remove_old_observations(now)

        if len(self.observations) < 2:
            return

        # Calcola la durata effettiva coperta dalle osservazioni
        # presenti nella finestra.
        first_time = self.observations[0][0]
        last_time = self.observations[-1][0]
        elapsed = last_time - first_time

        if elapsed <= 0:
            return

        received_messages = len(self.observations)

        # Frequenza media calcolata sugli intervalli osservati.
        measured_rate = (received_messages - 1) / elapsed

        # Reliability: frequenza misurata rispetto a quella attesa.
        reliability = min(
            1.0,
            measured_rate / self.expected_rate,
        )

        # Timeliness: frazione di aggiornamenti entro la soglia.
        timeliness_values = [
            observation[1]
            for observation in self.observations
        ]

        timely_updates = sum(
            1
            for value in timeliness_values
            if 0 <= value <= self.desired_timeliness
        )

        timeliness = timely_updates / len(timeliness_values)

        # Nel percorso Physical -> Digital, A = 1
        # secondo la definizione del modello di riferimento.
        availability = 1.0

        odte = timeliness * reliability * availability

        metrics = {
            'direction': 'physical_to_digital',
            'timeliness': round(timeliness, 4),
            'reliability': round(reliability, 4),
            'availability': availability,
            'odte': round(odte, 4),
            'received_messages': received_messages,
            'measured_rate_hz': round(measured_rate, 2),
            'expected_rate_hz': self.expected_rate,
            'desired_timeliness_ms':
                self.desired_timeliness * 1000,
            'window_seconds': round(elapsed, 2),
            'timeliness_method':
                'generation_to_reception_plus_state_manager_processing',
        }

        message = String()
        message.data = json.dumps(metrics)
        self.publisher.publish(message)

        self.get_logger().info(
            f'T={timeliness:.3f} | '
            f'R={reliability:.3f} | '
            f'A={availability:.3f} | '
            f'ODTE={odte:.3f}'
        )

    def calculate_digital_to_physical(self, now):
        """Calculate Digital -> Physical ODTE metrics."""
        self.remove_old_command_observations(now)

        # Considera i comandi accettati ancora presenti nella finestra.
        accepted_count = len(self.accepted_commands)

        # Seleziona gli esiti dei comandi ancora presenti nella finestra.
        results = [
            result
            for command_id, result in self.command_results.items()
            if command_id in self.accepted_commands
        ]

        # Timeliness e Reliability.
        if accepted_count == 0:
            timeliness = 0.0
            reliability = 0.0
            successful_commands = 0

        else:
            # Timeliness: percentuale di comandi completati entro soglia.
            timely_commands = sum(
                1
                for result in results
                if (
                    result['status'] == 'completed'
                    and result['elapsed_seconds']
                    <= self.command_timeliness
                )
            )

            timeliness = timely_commands / accepted_count

            # Reliability: comandi completati con successo / accettati.
            successful_commands = sum(
                1
                for result in results
                if result['status'] == 'completed'
            )

            reliability = successful_commands / accepted_count

        # Availability: frequenza degli heartbeat nella finestra temporale.
        self.remove_old_heartbeat_observations(now)

        if not self.heartbeat_observations:
            availability = 0.0
        else:
            # Durata effettiva della finestra di osservazione.
            observed_window = min(
                self.window,
                max(0.0, now - self.start_time)
            )

            if observed_window <= 0.0:
                availability = 0.0
            else:
                measured_rate = (
                    len(self.heartbeat_observations) / observed_window
                )

                availability = min(
                    1.0,
                    measured_rate / self.expected_heartbeat_rate
                )

        odte = timeliness * reliability * availability

        metrics = {
            'direction': 'digital_to_physical',
            'timeliness': round(timeliness, 4),
            'reliability': round(reliability, 4),
            'availability': round(availability, 4),
            'odte': round(odte, 4),
            'accepted_commands': accepted_count,
            'completed_commands': successful_commands,
            'command_timeliness_ms':
                self.command_timeliness * 1000,
            'window_seconds': self.window,
            'timeliness_method':
                'command_execution_duration_proxy',
            'availability_method':
                'heartbeat_frequency_in_window',
        }

        message = String()
        message.data = json.dumps(metrics)
        self.publisher.publish(message)

        self.get_logger().info(
            f'D→P | T={timeliness:.3f} | '
            f'R={reliability:.3f} | '
            f'A={availability:.3f} | '
            f'ODTE={odte:.3f}'
        )


def main(args=None):
    """Initialize ROS 2 and run the ODTE monitor."""
    rclpy.init(args=args)
    node = ODTEMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
