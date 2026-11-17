import uuid

from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from unique_identifier_msgs.msg import UUID
from action_msgs.msg import GoalStatus

import json
import math
import time

from rcl_interfaces.msg import SetParametersResult

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from std_msgs.msg import Bool, Float64MultiArray, String
from franka_dt_interfaces.msg import DigitalTwinCommand

from rclpy.clock import Clock
from rclpy.clock_type import ClockType


class CommandManager(Node):

    def __init__(self):
        super().__init__('command_manager')

        # Nomi dei sette giunti del braccio FR3
        self.joint_names = [
            'fr3_joint1',
            'fr3_joint2',
            'fr3_joint3',
            'fr3_joint4',
            'fr3_joint5',
            'fr3_joint6',
            'fr3_joint7'
        ]

        # Parametri per verificare il completamento dei comandi
        self.declare_parameter('position_tolerance', 0.01)
        self.declare_parameter('stable_duration', 0.5)
        self.declare_parameter('command_timeout', 10.0)
        self.declare_parameter('max_state_interval', 0.05)
        self.declare_parameter('trajectory_duration', 2.0)

        self.position_tolerance = float(
            self.get_parameter('position_tolerance').value
        )

        self.stable_duration = float(
            self.get_parameter('stable_duration').value
        )

        self.command_timeout = float(
            self.get_parameter('command_timeout').value
        )

        self.max_state_interval = float(
            self.get_parameter('max_state_interval').value
        )

        self.trajectory_duration = float(
            self.get_parameter('trajectory_duration').value
        )

        self.add_on_set_parameters_callback(
            self.parameters_callback
        )

        if not math.isfinite(self.position_tolerance) or self.position_tolerance <= 0:
            raise ValueError(
                'position_tolerance deve essere un numero finito e maggiore di zero'
            )

        if not math.isfinite(self.stable_duration) or self.stable_duration <= 0:
            raise ValueError(
                'stable_duration deve essere un numero finito e maggiore di zero'
            )

        if not math.isfinite(self.command_timeout) or self.command_timeout <= 0:
            raise ValueError(
                'command_timeout deve essere un numero finito e maggiore di zero'
            )

        if not math.isfinite(self.max_state_interval) or self.max_state_interval <= 0:
            raise ValueError(
                'max_state_interval deve essere un numero finito e maggiore di zero'
            )

        if (
            not math.isfinite(self.trajectory_duration)
            or self.trajectory_duration <= 0
        ):
            raise ValueError(
                'trajectory_duration deve essere finito e maggiore di zero'
            )

        # Stato del comando attualmente in esecuzione
        self.active_command = None
        self.active_command_id = None

        # Identificativo del goal Action associato al comando attivo
        self.active_goal_uuid = None
        self.active_goal_uuid_hex = None

        self.active_goal_handle = None
        self.goal_command_map = {}

        # Il goal è considerato accettato solo dopo la risposta positiva
        # dell'Action server e la ricezione dell'evento dal controller.
        self.active_goal_accepted = False
        self.active_physical_receipt_timestamp_ns = None
        self.accepted_command_published = False

        # Timestamp di ricezione dei goal da parte del controller fisico.
        # La chiave è il command_id, il valore è il timestamp ROS in nanosecondi.
        self.physical_goal_receipts = {}

        # Indica se il comando ha superato il timeout.
        self.command_timed_out = False

        # Timestamp originale di emissione da parte del Digital Twin
        self.command_emission_timestamp = None

        # Tempi di ricezione e avvio della gestione nel Command Manager
        self.command_start_time = None
        self.command_start_timestamp = None

        self.target_reached_time = None
        self.last_target_update_time = None
        self.target_stable_logged = False
        self.current_positions = {}
        self.last_joint_state_time = None

        # Publisher verso il controller di posizione
        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            '/fr3_position_controller/commands',
            10
        )

        # Client Action per inviare traiettorie al controller fisico FR3
        self.trajectory_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/fr3_arm_controller/follow_joint_trajectory'
        )

        # Subscriber ai comandi provenienti dal Digital Twin
        self.command_subscription = self.create_subscription(
            DigitalTwinCommand,
            '/digital_twin/command_positions',
            self.command_callback,
            10
        )

        # Subscriber alle posizioni effettive del robot
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Riceve dal controller fisico il timestamp di accettazione del goal.
        self.physical_goal_subscription = self.create_subscription(
            String,
            '/digital_twin/physical_goal_received',
            self.physical_goal_received_callback,
            10
        )

        # Publisher per comunicare l'esito dei comandi
        self.status_publisher = self.create_publisher(
            String,
            '/digital_twin/command_status',
            10
        )

        self.accepted_command_publisher = self.create_publisher(
            String,
            '/digital_twin/accepted_command',
            10,
        )

        # Heartbeat del lato fisico per il monitor ODTE.
        self.heartbeat_publisher = self.create_publisher(
            Bool,
            '/digital_twin/physical_heartbeat',
            10,
        )

        self.heartbeat_clock = Clock(
            clock_type=ClockType.STEADY_TIME
        )

        self.heartbeat_timer = self.create_timer(
            0.5,
            self.publish_heartbeat,
            clock=self.heartbeat_clock,
        )

        # Timer per controllare il timeout indipendentemente da /joint_states
        self.timeout_timer = self.create_timer(
            0.1,
            self.check_command_timeout
        )

        self.get_logger().info(
            'Command Manager avviato. '
            'In attesa di comandi su /digital_twin/command_positions.'
        )

    def parameters_callback(self, params):
        for param in params:

            if param.name == 'position_tolerance':

                value = float(param.value)

                if not math.isfinite(value) or value <= 0:
                    return SetParametersResult(
                        successful=False,
                        reason='position_tolerance deve essere finito e maggiore di zero'
                    )

                self.position_tolerance = value

                self.get_logger().info(
                    f'Nuova tolleranza impostata: {self.position_tolerance:.6f} rad'
                )

        return SetParametersResult(successful=True)

    def publish_heartbeat(self):
        """Pubblica un heartbeat solo se gli stati del robot sono recenti."""
        heartbeat = Bool()

        heartbeat.data = (
            self.last_joint_state_time is not None
            and time.monotonic() - self.last_joint_state_time < 0.2
        )

        self.heartbeat_publisher.publish(heartbeat)

    def publish_status(
        self,
        status,
        reason,
        command_id=None,
        emission_timestamp=None,
        completion_state_timestamp_ns=None,
        physical_goal_receipt_timestamp_ns=None
    ):
        """Pubblica lo stato del comando in formato JSON."""

        message = String()

        now_ros = self.get_clock().now().nanoseconds / 1e9
        now_monotonic = time.monotonic()

        if status != 'rejected':
            command_id = self.active_command_id
            emission_timestamp = self.command_emission_timestamp

        message_data = {
            'status': status,
            'reason': reason,
            'timestamp': now_ros,
            'command_id': command_id,
            'emission_timestamp': emission_timestamp,
        }

        if self.command_start_timestamp is not None and status != 'rejected':
            message_data['start_timestamp'] = self.command_start_timestamp

        if self.command_start_time is not None and status != 'rejected':
            message_data['elapsed_seconds'] = (
                now_monotonic - self.command_start_time
            )

        if completion_state_timestamp_ns is not None:
            message_data['completion_state_timestamp_ns'] = (
                completion_state_timestamp_ns
            )

        if physical_goal_receipt_timestamp_ns is not None:
            message_data['physical_goal_receipt_timestamp_ns'] = (
                physical_goal_receipt_timestamp_ns
            )

        message.data = json.dumps(message_data)
        self.status_publisher.publish(message)

    def log_final_errors(self):
        if not all(
            joint in self.current_positions
            for joint in self.joint_names
        ):
            self.get_logger().warning(
                'Impossibile calcolare gli errori finali: '
                'posizioni articolari non ancora disponibili.'
            )
            return

        """Registra gli errori articolari nell'ultimo aggiornamento."""
        errors = {
            joint: abs(
                self.active_command[joint]
                - self.current_positions[joint]
            )
            for joint in self.joint_names
        }

        max_error = max(errors.values())

        self.get_logger().info(
            f'Command ID: {self.active_command_id}'
        )
        self.get_logger().info(
            f'Tolleranza utilizzata: {self.position_tolerance:.6f} rad'
        )

        for joint, error in errors.items():
            self.get_logger().info(
                f'{joint}: errore = {error:.6f} rad'
            )

        self.get_logger().info(
            f'Errore massimo: {max_error:.6f} rad'
        )

    def finish_command(
        self,
        status,
        reason,
        completion_state_timestamp_ns=None
    ):
        """Conclude il comando attivo e pubblica il risultato."""
        if status == 'completed':

            # Tempo effettivo necessario al PT per raggiungere il target
            if self.target_reached_time is not None:
                movement_duration = (
                    self.target_reached_time - self.command_start_time
                )

                self.get_logger().info(
                    f'Durata effettiva del movimento PT: '
                    f'{movement_duration:.3f} s'
                )

                self.get_logger().info(
                    f'Durata verifica stabilità: '
                    f'{self.stable_duration:.3f} s'
                )

            self.log_final_errors()

        self.publish_status(
            status,
            reason,
            completion_state_timestamp_ns=completion_state_timestamp_ns
        )

        if status == 'completed':
            self.get_logger().info(
                'Comando completato: tutti i giunti hanno raggiunto '
                'e mantenuto le posizioni obiettivo.'
            )
        else:
            self.get_logger().error(
                f'Comando fallito: {reason}.'
            )

        # Libera lo stato per consentire un nuovo comando
        self.active_command = None
        self.active_command_id = None
        self.command_emission_timestamp = None
        self.command_start_time = None
        self.command_start_timestamp = None
        self.target_reached_time = None
        self.last_target_update_time = None

        self.active_goal_uuid = None
        self.active_goal_uuid_hex = None
        self.active_goal_handle = None
        self.active_goal_accepted = False
        self.active_physical_receipt_timestamp_ns = None
        self.accepted_command_published = False

        self.command_timed_out = False

    def check_command_timeout(self):
        """Controlla il timeout e richiede la cancellazione del goal."""

        if self.active_command is None:
            return

        # Evita di gestire ripetutamente lo stesso timeout.
        if self.command_timed_out:
            return

        elapsed = time.monotonic() - self.command_start_time

        if elapsed >= self.command_timeout:

            self.command_timed_out = True

            self.get_logger().error(
                f'Timeout del comando {self.active_command_id}. '
                'Richiesta di cancellazione del goal al controller.'
            )

            # Se il controller ha già accettato il goal,
            # possiamo richiederne immediatamente la cancellazione.
            if self.active_goal_handle is not None:

                try:
                    cancel_future = (
                        self.active_goal_handle.cancel_goal_async()
                    )

                    goal_uuid = self.active_goal_uuid_hex

                    cancel_future.add_done_callback(
                        lambda future: self.cancel_response_callback(
                            future,
                            goal_uuid
                        )
                    )

                    self.get_logger().info(
                        'Richiesta di cancellazione inviata al controller.'
                    )

                except Exception as e:
                    self.get_logger().error(
                        f'Errore durante la richiesta di cancellazione: {e}'
                    )

            else:
                # Il goal potrebbe essere ancora in attesa di accettazione.
                # La cancellazione verrà richiesta nella callback
                # quando riceveremo la risposta del controller.
                self.get_logger().warning(
                    'Goal non ancora accettato o risposta non ancora ricevuta. '
                    'La cancellazione verrà gestita alla ricezione della risposta.'
                )

    def cancel_response_callback(self, future, goal_uuid):
        """Gestisce la risposta del controller alla richiesta di cancellazione."""

        # Ignora eventuali risposte relative a comandi già conclusi.
        if goal_uuid != self.active_goal_uuid_hex:
            return

        try:
            cancel_response = future.result()

            if cancel_response.goals_canceling:
                self.get_logger().info(
                    f'Controller ha accettato la cancellazione '
                    f'del goal {goal_uuid}.'
                )
            else:
                self.get_logger().warning(
                    f'Controller non ha accettato la cancellazione '
                    f'del goal {goal_uuid}.'
                )

        except Exception as e:
            self.get_logger().error(
                f'Errore nella risposta alla cancellazione del goal: {e}'
            )

    def physical_goal_received_callback(self, msg):
        """Riceve e associa il timestamp di ricezione del goal fisico."""

        try:
            # Decodifica il messaggio JSON.
            event = json.loads(msg.data)

            goal_uuid = event['goal_uuid']
            receipt_timestamp_ns = int(event['receipt_timestamp_ns'])

            # Verifica la validità dei dati ricevuti.
            if not goal_uuid or receipt_timestamp_ns <= 0:
                self.get_logger().warning(
                    'Evento di ricezione goal non valido.'
                )
                return

            # Cerca il command_id associato all'UUID.
            command_id = self.goal_command_map.get(goal_uuid)

            if command_id is None:
                self.get_logger().warning(
                    f'Nessun comando associato al goal UUID {goal_uuid}.'
                )
                return

            # Evita di sovrascrivere un timestamp già acquisito.
            if command_id in self.physical_goal_receipts:
                self.get_logger().warning(
                    f'Timestamp di ricezione già registrato '
                    f'per il comando {command_id}.'
                )
                return

            # Conserva il timestamp originale del controller.
            self.physical_goal_receipts[command_id] = receipt_timestamp_ns

            if command_id == self.active_command_id:
                self.active_physical_receipt_timestamp_ns = (
                    receipt_timestamp_ns
                )
                self.publish_accepted_command_if_ready()

            self.get_logger().info(
                f'Ricezione fisica registrata per il comando {command_id}: '
                f'{receipt_timestamp_ns} ns.'
            )

        except (KeyError, ValueError, TypeError, json.JSONDecodeError) as e:
            self.get_logger().error(
                f'Errore nella decodifica dell evento physical_goal_received: {e}'
            )

    def publish_accepted_command_if_ready(self):
        """Pubblica il comando al Digital Model solo dopo l'accettazione PT."""
        if (
            self.active_command is None
            or not self.active_goal_accepted
            or self.active_physical_receipt_timestamp_ns is None
            or self.accepted_command_published
        ):
            return

        accepted_data = {
            'command_id': self.active_command_id,
            'target_positions': [
                self.active_command[joint]
                for joint in self.joint_names
            ],
            'emission_timestamp': self.command_emission_timestamp,
            'timestamp': self.command_start_timestamp,
            'physical_goal_receipt_timestamp_ns':
                self.active_physical_receipt_timestamp_ns,
        }

        accepted_msg = String()
        accepted_msg.data = json.dumps(accepted_data)
        self.accepted_command_publisher.publish(accepted_msg)

        # L'evento contiene i due timestamp necessari all'Equazione 7.
        self.publish_status(
            'goal_accepted',
            'goal_accepted',
            physical_goal_receipt_timestamp_ns=(
                self.active_physical_receipt_timestamp_ns
            ),
        )

        self.accepted_command_published = True

        self.get_logger().info(
            f'Comando {self.active_command_id} accettato dal controller; '
            'inoltrato al Digital Model.'
        )

    def joint_state_callback(self, msg):
        """Aggiorna le posizioni e verifica il completamento del comando."""
        # Verifica che nomi e posizioni abbiano la stessa lunghezza
        if len(msg.name) != len(msg.position):
            return

        # Associa ogni nome di giunto alla sua posizione
        positions = dict(zip(msg.name, msg.position))

        # Verifica che siano presenti tutti i giunti FR3
        if not all(
            joint in positions for joint in self.joint_names
        ):
            return

        # Verifica che le posizioni siano valori finiti
        if not all(
            math.isfinite(positions[joint])
            for joint in self.joint_names
        ):
            return

        # Salva le posizioni correnti
        self.current_positions = {
            joint: positions[joint]
            for joint in self.joint_names
        }

        self.last_joint_state_time = time.monotonic()

        # Se non c'è un comando attivo, non c'è nulla da verificare
        if self.active_command is None:
            return

        now = time.monotonic()

        # Controlla se è scaduto il tempo massimo.
        # La gestione della cancellazione è affidata al metodo dedicato.
        if now - self.command_start_time >= self.command_timeout:
            self.check_command_timeout()
            return

        # Verifica che ogni giunto sia sufficientemente vicino al bersaglio
        all_joints_reached = all(
            abs(
                self.current_positions[joint]
                - self.active_command[joint]
            ) <= self.position_tolerance
            for joint in self.joint_names
        )

        if all_joints_reached:

            # Primo aggiornamento entro la tolleranza:
            # inizia il conteggio della stabilità.
            if self.target_reached_time is None:
                self.target_reached_time = now
                self.last_target_update_time = now

            else:
                # Verifica l'intervallo dall'ultimo aggiornamento.
                state_interval = now - self.last_target_update_time

                if state_interval > self.max_state_interval:
                    # Aggiornamenti troppo distanti:
                    # si ricomincia il conteggio.
                    self.target_reached_time = now
                    self.last_target_update_time = now
                    self.target_stable_logged = False

                else:
                    # Aggiornamento regolare:
                    # aggiorna l'istante dell'ultima rilevazione.
                    self.last_target_update_time = now

                    # Verifica che la posizione sia stata mantenuta
                    # entro la tolleranza per tutta la durata prevista.
                    if (
                        now - self.target_reached_time >= self.stable_duration
                        and not self.target_stable_logged
                    ):

                        self.get_logger().info(
                            'Target raggiunto e mantenuto entro la tolleranza '
                            'per la durata richiesta. In attesa del risultato Action.'
                        )

                        self.target_stable_logged = True

        else:
            # Se anche un solo giunto esce dalla tolleranza,
            # si azzera il conteggio della stabilità.
            self.target_reached_time = None
            self.last_target_update_time = None
            self.target_stable_logged = False

    def goal_response_callback(self, future, goal_uuid):
        """Gestisce la risposta del controller alla richiesta di goal."""

        # Ignora eventuali risposte relative a comandi già conclusi.
        if goal_uuid != self.active_goal_uuid_hex:
            return

        try:
            goal_handle = future.result()

        except Exception as e:
            self.get_logger().error(
                f'Errore durante la richiesta del goal: {e}'
            )

            reason = (
                'timeout'
                if self.command_timed_out
                else 'goal_response_error'
            )

            self.finish_command('failed', reason)
            return

        # Il controller ha rifiutato il goal.
        if not goal_handle.accepted:
            self.get_logger().error(
                f'Goal {goal_uuid} rifiutato dal controller.'
            )
            reason = (
                'timeout'
                if self.command_timed_out
                else 'goal_rejected'
            )

            self.finish_command('failed', reason)
            return

        self.active_goal_handle = goal_handle

        self.get_logger().info(
            f'Goal {goal_uuid} accettato dal controller.'
        )

        # Se il timeout è già scattato, non iniziare normalmente
        # l'esecuzione: richiedi subito la cancellazione.
        if self.command_timed_out:

            self.get_logger().warning(
                'Il goal è stato accettato dopo il timeout. '
                'Richiedo immediatamente la cancellazione.'
            )

            try:
                cancel_future = (
                    self.active_goal_handle.cancel_goal_async()
                )

                cancel_future.add_done_callback(
                    lambda future: self.cancel_response_callback(
                        future,
                        goal_uuid
                    )
                )

            except Exception as e:
                self.get_logger().error(
                    f'Errore durante la cancellazione del goal: {e}'
                )

        else:
            self.active_goal_accepted = True
            self.publish_accepted_command_if_ready()

        # Attende il risultato finale dell'esecuzione.
        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            lambda future: self.action_result_callback(
                future,
                goal_uuid
            )
        )

    def action_result_callback(self, future, goal_uuid):
        """Gestisce il risultato finale restituito dal controller."""

        # Ignora risultati tardivi di comandi già conclusi.
        if goal_uuid != self.active_goal_uuid_hex:
            return

        try:
            action_result = future.result()

        except Exception as e:
            self.get_logger().error(
                f'Errore durante la ricezione del risultato Action: {e}'
            )

            reason = (
                'timeout'
                if self.command_timed_out
                else 'action_result_error'
            )

            self.finish_command('failed', reason)
            return

        result = action_result.result

        # Se il comando era scaduto, il risultato viene registrato
        # come timeout, anche se il controller ha completato
        # o cancellato il goal durante la gestione della richiesta.
        if self.command_timed_out:

            self.finish_command(
                'failed',
                'timeout'
            )
            return

        # Verifica sia lo stato ROS dell'Action sia il codice del controller.
        if (
            action_result.status == GoalStatus.STATUS_SUCCEEDED
            and result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        ):
            self.get_logger().info(
                f'Goal {goal_uuid} completato con successo.'
            )

            self.finish_command(
                'completed',
                'action_succeeded'
            )

        else:
            self.get_logger().error(
                'Esecuzione Action non completata correttamente. '
                f'Status ROS: {action_result.status}, '
                f'error_code: {result.error_code}, '
                f'error_string: {result.error_string}'
            )

            self.finish_command(
                'failed',
                f'action_failed_{result.error_code}'
            )

    def command_callback(self, msg):
        """Valida e inoltra un comando articolare al controller."""

        command_id = msg.command_id
        stamp = msg.emission_timestamp

        emission_timestamp = (
            stamp.sec + stamp.nanosec / 1e9
        )

        # Verifica l'identificativo del comando
        if not command_id:
            self.get_logger().error(
                'Comando rifiutato: command_id mancante.'
            )
            self.publish_status(
                'rejected',
                'invalid_command_id'
            )
            return

        # Verifica il timestamp originale di emissione
        if stamp.sec < 0 or stamp.nanosec >= 1_000_000_000:
            self.get_logger().error(
                'Comando rifiutato: timestamp non valido.'
            )
            self.publish_status(
                'rejected',
                'invalid_emission_timestamp',
                command_id,
                emission_timestamp
            )
            return

        if emission_timestamp <= 0:
            self.get_logger().error(
                'Comando rifiutato: timestamp di emissione non valido.'
            )
            self.publish_status(
                'rejected',
                'invalid_emission_timestamp',
                command_id,
                emission_timestamp
            )
            return

        # Estrae le posizioni desiderate
        target_positions = list(msg.target_positions)

        # Verifica che il vettore contenga sette posizioni
        if len(target_positions) != len(self.joint_names):
            self.get_logger().error(
                'Comando rifiutato: sono richieste '
                'esattamente sette posizioni articolari.'
            )
            self.publish_status(
                'rejected',
                'invalid_joint_count',
                command_id,
                emission_timestamp
            )
            return

        # Verifica che tutti i valori siano finiti
        if not all(math.isfinite(value) for value in target_positions):
            self.get_logger().error(
                'Comando rifiutato: il vettore contiene '
                'valori non finiti.'
            )
            self.publish_status(
                'rejected',
                'non_finite_position',
                command_id,
                emission_timestamp
            )
            return

        # Non consentire di sovrascrivere un comando in esecuzione
        if self.active_command is not None:
            self.get_logger().warning(
                'Nuovo comando rifiutato: è già in esecuzione '
                'un altro comando.'
            )
            self.publish_status(
                'rejected',
                'command_already_active',
                command_id,
                emission_timestamp
            )
            return

        # Salva le posizioni obiettivo nell'ordine dei giunti
        self.active_command = {
            joint: float(position)
            for joint, position in zip(
                self.joint_names,
                target_positions
            )
        }

        # Conserva l'identità originale del comando
        self.active_command_id = command_id
        self.command_emission_timestamp = emission_timestamp

        # Genera un UUID personalizzato per il goal Action
        self.active_goal_uuid = UUID(
            uuid=list(uuid.uuid4().bytes)
        )

        # Converte l'UUID nel formato esadecimale usato dal controller C++
        self.active_goal_uuid_hex = bytes(
            self.active_goal_uuid.uuid
        ).hex()

        self.goal_command_map[self.active_goal_uuid_hex] = command_id

        self.get_logger().info(
            f'Associazione comando-goal: '
            f'{command_id} -> {self.active_goal_uuid_hex}'
        )

        # Registra i tempi di ricezione e avvio nel Command Manager
        self.command_start_time = time.monotonic()
        self.command_start_timestamp = (
            self.get_clock().now().nanoseconds / 1e9
        )

        self.target_reached_time = None
        self.last_target_update_time = None
        self.target_stable_logged = False

        # Prepara il goal FollowJointTrajectory
        goal = FollowJointTrajectory.Goal()

        # Specifica i sette giunti interessati
        goal.trajectory.joint_names = self.joint_names

        # Crea il punto della traiettoria
        trajectory_point = JointTrajectoryPoint()

        # Imposta le posizioni obiettivo
        trajectory_point.positions = [
            self.active_command[joint]
            for joint in self.joint_names
        ]

        # Imposta il tempo entro cui raggiungere il target
        trajectory_point.time_from_start.sec = int(
            self.trajectory_duration
        )

        trajectory_point.time_from_start.nanosec = int(
            (self.trajectory_duration % 1) * 1_000_000_000
        )

        # Inserisce il punto nella traiettoria
        goal.trajectory.points = [trajectory_point]

        # Il Digital Model riceverà il comando soltanto quando il controller
        # avrà accettato il goal e avremo acquisito il timestamp PT'.

        # Verifica che il server Action del controller sia disponibile.
        if not self.trajectory_action_client.server_is_ready():

            self.get_logger().error(
                'Server Action /fr3_arm_controller/follow_joint_trajectory '
                'non disponibile.'
            )

            self.finish_command(
                'failed',
                'action_server_unavailable'
            )
            return

        # Salva l'identificativo del goal per correlare le callback.
        goal_uuid = self.active_goal_uuid_hex

        try:
            # Invia il goal in modalità asincrona.
            goal_future = self.trajectory_action_client.send_goal_async(
                goal,
                goal_uuid=self.active_goal_uuid
            )

            goal_future.add_done_callback(
                lambda future: self.goal_response_callback(
                    future,
                    goal_uuid
                )
            )

            self.publish_status(
                'submitted',
                'goal_submitted'
            )

            self.get_logger().info(
                f'Goal {goal_uuid} inviato al controller FR3.'
            )

        except Exception as e:

            self.get_logger().error(
                f'Impossibile inviare il goal Action: {e}'
            )

            self.finish_command(
                'failed',
                'goal_submission_error'
            )
            return

        self.get_logger().info(
            f'Timestamp emissione DT: {self.command_emission_timestamp:.9f}'
        )

        self.get_logger().info(
            f'Timestamp ricezione CM: {self.command_start_timestamp:.9f}'
        )

        for joint, position in self.active_command.items():
            self.get_logger().info(
                f'{joint}: posizione obiettivo = {position:.6f} rad'
            )


def main(args=None):

    rclpy.init(args=args)

    node = CommandManager()

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
