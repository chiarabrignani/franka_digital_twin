"""Monitor bidirectional Digital Twin entanglement metrics."""

from collections import deque

import json
import math
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState


class ODTEMonitor(Node):
    """Monitor synchronization metrics between physical and digital twins."""

    def __init__(self):
        """Initialize parameters, subscriptions, and publishers."""
        super().__init__('odte_monitor')

        self.declare_parameter('expected_rate_hz', 100.0)
        self.declare_parameter('desired_timeliness_ms', 50.0)
        self.declare_parameter('window_seconds', 10.0)
        self.declare_parameter('heartbeat_timeout', 1.5)
        self.declare_parameter('command_timeliness_ms', 10000.0)
        self.declare_parameter('max_data_age', 2.0)
        self.declare_parameter('timestamp_tolerance', 0.02)

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

        self.command_timeliness = float(
            self.get_parameter('command_timeliness_ms').value
        ) / 1000.0

        self.max_data_age = float(
            self.get_parameter('max_data_age').value
        )

        self.timestamp_tolerance = float(
            self.get_parameter('timestamp_tolerance').value
        )

        if (
            self.expected_rate <= 0
            or self.desired_timeliness <= 0
            or self.window <= 0
            or self.heartbeat_timeout <= 0
            or self.command_timeliness <= 0
            or self.max_data_age <= 0
            or self.timestamp_tolerance <= 0
        ):
            raise ValueError(
                'I parametri ODTE devono essere positivi.'
            )

        self.joint_names = [
            'fr3_joint1',
            'fr3_joint2',
            'fr3_joint3',
            'fr3_joint4',
            'fr3_joint5',
            'fr3_joint6',
            'fr3_joint7',
        ]

        # Code temporanee degli stati PT e DT.
        # Ogni elemento contiene timestamp, posizioni e tempo di ricezione.
        self.pt_states = deque(maxlen=100)
        self.dt_states = deque(maxlen=100)

        # Ultimi istanti di ricezione degli stati.
        self.last_pt_receipt = None
        self.last_dt_receipt = None

        # Istante dell'ultimo confronto PT-DT valido.
        self.last_match_receipt = None

        # Stato corrente della sincronizzazione.
        self.synchronization_status = None

        # Metriche dell'ultimo confronto.
        self.joint_errors = {}
        self.max_error = None
        self.mae = None
        self.valid_comparisons = 0

        # Statistiche aggregate della finestra di osservazione.
        self.window_comparisons = 0
        self.window_error_sum = 0.0
        self.window_max_error = 0.0

        self.window_joint_max_errors = {
            joint: 0.0 for joint in self.joint_names
        }

        self.window_max_timestamp_difference = 0.0

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

        # Riceve gli aggiornamenti effettivamente elaborati dal Digital Model.
        self.state_processed_subscription = self.create_subscription(
            String,
            '/digital_twin/state_processed',
            self.state_processed_callback,
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

        # Stato del robot fisico.
        self.physical_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.physical_state_callback,
            10,
        )

        # Stato predetto dal Digital Model.
        self.digital_state_subscription = self.create_subscription(
            JointState,
            '/digital_twin/predicted_joint_states',
            self.digital_state_callback,
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
        """Registra ogni heartbeat, sia True sia False."""
        now = time.monotonic()
        self.physical_heartbeat = msg.data
        self.heartbeat_observations.append((now, msg.data))

        if msg.data:
            self.last_heartbeat_time = now

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

    def extract_joint_state(self, msg):
        """Estrae e valida lo stato dei sette giunti."""

        if len(msg.name) != len(msg.position):
            return None

        positions = dict(zip(msg.name, msg.position))

        if not all(joint in positions for joint in self.joint_names):
            return None

        values = [positions[joint] for joint in self.joint_names]

        if not all(math.isfinite(value) for value in values):
            return None

        stamp_ns = (
            msg.header.stamp.sec * 1_000_000_000
            + msg.header.stamp.nanosec
        )

        if stamp_ns <= 0:
            return None

        return {
            'positions': positions,
            'timestamp': stamp_ns / 1_000_000_000,
            'receipt_time': time.monotonic(),
        }

    def physical_state_callback(self, msg):
        """Registra uno stato valido del Physical Twin."""

        state = self.extract_joint_state(msg)

        if state is None:
            self.get_logger().warning(
                'Stato fisico ignorato: dati non validi.'
            )
            return

        self.last_pt_receipt = state['receipt_time']
        self.pt_states.append(state)
        self.match_states()

    def digital_state_callback(self, msg):
        """Registra uno stato valido del Digital Twin."""

        state = self.extract_joint_state(msg)

        if state is None:
            self.get_logger().warning(
                'Stato digitale ignorato: dati non validi.'
            )
            return

        self.last_dt_receipt = state['receipt_time']
        self.dt_states.append(state)
        self.match_states()

    def match_states(self):
        """Associa gli stati PT e DT tramite i timestamp."""

        while self.pt_states and self.dt_states:

            physical = self.pt_states[0]
            digital = self.dt_states[0]

            timestamp_difference = abs(
                physical['timestamp'] - digital['timestamp']
            )

            # Se la differenza rientra nella tolleranza,
            # associa i due stati.
            if timestamp_difference <= self.timestamp_tolerance:

                self.pt_states.popleft()
                self.dt_states.popleft()

                self.calculate_synchronization_errors(
                    physical,
                    digital
                )

            # Altrimenti scarta lo stato temporalmente più vecchio.
            elif physical['timestamp'] < digital['timestamp']:

                self.pt_states.popleft()

            else:

                self.dt_states.popleft()

    def calculate_synchronization_errors(
        self,
        physical,
        digital
    ):
        """Calcola gli errori articolari tra PT e DT."""

        errors = {}

        for joint in self.joint_names:

            errors[joint] = abs(
                physical['positions'][joint]
                - digital['positions'][joint]
            )

        mae = sum(errors.values()) / len(errors)
        max_error = max(errors.values())

        timestamp_difference = abs(
            physical['timestamp'] - digital['timestamp']
        )

        # Aggiorna le metriche dell'ultimo confronto.
        self.joint_errors = errors
        self.mae = mae
        self.max_error = max_error

        self.last_match_receipt = time.monotonic()
        self.valid_comparisons += 1

        # Aggiorna le statistiche aggregate.
        self.window_comparisons += 1
        self.window_error_sum += mae

        self.window_max_error = max(
            self.window_max_error,
            max_error
        )

        for joint in self.joint_names:

            self.window_joint_max_errors[joint] = max(
                self.window_joint_max_errors[joint],
                errors[joint]
            )

        self.window_max_timestamp_difference = max(
            self.window_max_timestamp_difference,
            timestamp_difference
        )

    def state_processed_callback(self, msg):
        """Registra la Timeliness degli aggiornamenti elaborati dal DT."""

        reception_monotonic = time.monotonic()

        try:
            data = json.loads(msg.data)

            update_id = int(data['update_id'])
            generation_ros = float(data['source_timestamp'])
            reception_ros = float(data['receipt_ros_timestamp'])
            processing_time = float(data['processing_time_seconds'])

        except (ValueError, TypeError, KeyError):
            self.get_logger().warning(
                'Evento PT-DT ricevuto in formato non valido.'
            )
            return

        values = (
            generation_ros,
            reception_ros,
            processing_time,
        )

        if (
            update_id <= 0
            or not all(math.isfinite(value) for value in values)
            or generation_ros <= 0
            or reception_ros < generation_ros - 1e-9
            or processing_time < 0
        ):
            self.get_logger().warning(
                'Evento PT-DT ignorato: '
                f'update_id={update_id}, '
                f'generation_ros={generation_ros:.9f}, '
                f'reception_ros={reception_ros:.9f}, '
                f'processing_time={processing_time:.9f}'
            )
            return

        # Generazione PT -> ricezione DT + elaborazione DT.
        timeliness = (
            max(0.0, reception_ros - generation_ros)
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
        """Conserva l'ultimo evento precedente alla finestra."""
        window_start = max(self.start_time, now - self.window)

        while (
            len(self.heartbeat_observations) >= 2
            and self.heartbeat_observations[1][0] <= window_start
        ):
            self.heartbeat_observations.popleft()

    def calculate_availability(self, now):
        """Calcola la frazione di tempo in cui il sistema è disponibile."""
        window_start = max(self.start_time, now - self.window)
        observed_window = now - window_start

        if observed_window <= 0:
            return 0.0

        available_duration = 0.0
        events = list(self.heartbeat_observations)

        for index, (event_time, is_available) in enumerate(events):
            if not is_available:
                continue

            next_event_time = (
                events[index + 1][0]
                if index + 1 < len(events)
                else now
            )

            # La disponibilità termina al prossimo evento oppure
            # alla scadenza del timeout, se non arrivano altri heartbeat.
            interval_end = min(
                next_event_time,
                event_time + self.heartbeat_timeout,
                now,
            )
            interval_start = max(event_time, window_start)

            if interval_end > interval_start:
                available_duration += interval_end - interval_start

        return min(1.0, max(0.0, available_duration / observed_window))

    def calculate_synchronization_diagnostics(self):
        """Valuta la disponibilità e la sincronizzazione PT-DT."""

        now = time.monotonic()

        # Calcola l'età degli ultimi dati ricevuti.
        physical_age = (
            None if self.last_pt_receipt is None
            else now - self.last_pt_receipt
        )

        digital_age = (
            None if self.last_dt_receipt is None
            else now - self.last_dt_receipt
        )

        # Verifica la disponibilità di entrambi i flussi.
        if (
            physical_age is None
            or digital_age is None
            or physical_age > self.max_data_age
            or digital_age > self.max_data_age
        ):
            new_status = 'DATA_UNAVAILABLE'

        # Verifica che esista un confronto recente tra PT e DT.
        elif (
            self.last_match_receipt is None
            or now - self.last_match_receipt > self.max_data_age
        ):
            new_status = 'TEMPORAL_MISMATCH'

        else:
            new_status = 'SYNCHRONIZED'

        # Registra un messaggio solo quando cambia lo stato.
        if new_status != self.synchronization_status:

            self.synchronization_status = new_status

            if new_status == 'DATA_UNAVAILABLE':
                self.get_logger().warning(
                    'Sincronizzazione PT-DT: DATA_UNAVAILABLE | '
                    f'età PT={physical_age} | '
                    f'età DT={digital_age}'
                )

            elif new_status == 'TEMPORAL_MISMATCH':
                self.get_logger().warning(
                    'Sincronizzazione PT-DT: TEMPORAL_MISMATCH | '
                    f'età PT={physical_age:.3f}s | '
                    f'età DT={digital_age:.3f}s'
                )

            else:
                self.get_logger().info(
                    'Sincronizzazione PT-DT: SYNCHRONIZED'
                )

        # Stampa le statistiche aggregate quando il sistema è sincronizzato.
        if (
            new_status == 'SYNCHRONIZED'
            and self.window_comparisons > 0
        ):

            window_mae = (
                self.window_error_sum / self.window_comparisons
            )

            worst_joint = max(
                self.window_joint_max_errors,
                key=self.window_joint_max_errors.get
            )

            worst_joint_error = self.window_joint_max_errors[worst_joint]

            self.get_logger().info(
                'Diagnostica PT-DT | '
                f'Confronti={self.window_comparisons} | '
                f'MAE medio={window_mae:.5f} rad | '
                f'Errore massimo={self.window_max_error:.5f} rad | '
                f'Giunto peggiore={worst_joint} '
                f'({worst_joint_error:.5f} rad) | '
                f'Delta timestamp massimo='
                f'{self.window_max_timestamp_difference * 1000:.2f} ms'
            )

            # Azzera le statistiche per la finestra successiva.
            self.window_comparisons = 0
            self.window_error_sum = 0.0
            self.window_max_error = 0.0

            self.window_joint_max_errors = {
                joint: 0.0 for joint in self.joint_names
            }

            self.window_max_timestamp_difference = 0.0

    def calculate_odte(self):
        """Calculate and publish the current ODTE metrics."""
        now = time.monotonic()

        # Diagnostica indipendente delle metriche ODTE.
        self.calculate_synchronization_diagnostics()

        self.calculate_digital_to_physical(now)

        self.remove_old_observations(now)

        if len(self.observations) < 2:
            return

        # Durata effettiva della finestra di osservazione.
        # Parte dal primo aggiornamento e cresce fino a window_seconds.
        observed_window = min(
            self.window,
            now - self.first_observation_time
        )

        if observed_window <= 0:
            return

        received_messages = len(self.observations)

        # Frequenza media: messaggi ricevuti / durata osservata.
        # In questo modo vengono considerati anche gli intervalli
        # senza aggiornamenti, fino all'istante attuale.
        measured_rate = received_messages / observed_window

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
            'window_seconds': round(observed_window, 2),
            'timeliness_method':
                'generation_to_reception_plus_digital_model_processing',
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

        # Availability: frazione di tempo in cui il sistema fisico
        # risulta disponibile, considerando heartbeat e timeout.
        self.remove_old_heartbeat_observations(now)
        availability = self.calculate_availability(now)

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
                'available_time_fraction_from_heartbeat_and_timeout',
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
