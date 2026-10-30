import json
import math
import time
import uuid

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from std_msgs.msg import Bool, Float64MultiArray, String


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

        self.position_tolerance = float(
            self.get_parameter('position_tolerance').value
        )

        self.stable_duration = float(
            self.get_parameter('stable_duration').value
        )

        self.command_timeout = float(
            self.get_parameter('command_timeout').value
        )

        # Stato del comando attualmente in esecuzione
        self.active_command = None
        self.active_command_id = None
        self.command_start_time = None
        self.command_start_timestamp = None
        self.target_reached_time = None
        self.current_positions = {}
        self.last_joint_state_time = None

        # Publisher verso il controller di posizione
        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            '/fr3_position_controller/commands',
            10
        )

        # Subscriber ai comandi provenienti dal Digital Twin
        self.command_subscription = self.create_subscription(
            Float64MultiArray,
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

        # Publisher per comunicare l'esito dei comandi
        self.status_publisher = self.create_publisher(
            String,
            '/digital_twin/command_status',
            10
        )

        # Heartbeat del lato fisico per il monitor ODTE.
        self.heartbeat_publisher = self.create_publisher(
            Bool,
            '/digital_twin/physical_heartbeat',
            10,
        )

        self.heartbeat_timer = self.create_timer(
            0.5,
            self.publish_heartbeat,
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

    def publish_heartbeat(self):
        """Pubblica un heartbeat solo se gli stati del robot sono recenti."""
        heartbeat = Bool()

        heartbeat.data = (
            self.last_joint_state_time is not None
            and time.monotonic() - self.last_joint_state_time < 0.2
        )

        self.heartbeat_publisher.publish(heartbeat)

    def publish_status(self, status, reason):
        """Pubblica lo stato del comando in formato JSON."""
        message = String()

        now_ros = self.get_clock().now().nanoseconds / 1e9
        now_monotonic = time.monotonic()

        # Un comando rifiutato non deve ereditare l'ID
        # e i tempi di un altro comando eventualmente attivo.
        if status == 'rejected':
            command_id = None
        else:
            command_id = self.active_command_id

        message_data = {
            'status': status,
            'reason': reason,
            'timestamp': now_ros,
            'command_id': command_id,
        }

        # I tempi di esecuzione sono pertinenti solo
        # al comando effettivamente gestito.
        if status != 'rejected':
            if self.command_start_timestamp is not None:
                message_data['start_timestamp'] = (
                    self.command_start_timestamp
                )

            if self.command_start_time is not None:
                message_data['elapsed_seconds'] = (
                    now_monotonic - self.command_start_time
                )

        message.data = json.dumps(message_data)
        self.status_publisher.publish(message)

    def finish_command(self, status, reason):
        """Conclude il comando attivo e pubblica il risultato."""
        self.publish_status(status, reason)

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
        self.command_start_time = None
        self.command_start_timestamp = None
        self.target_reached_time = None

    def check_command_timeout(self):
        """Controlla periodicamente se il comando attivo è scaduto."""
        if self.active_command is None:
            return

        elapsed = time.monotonic() - self.command_start_time

        if elapsed >= self.command_timeout:
            self.finish_command('failed', 'timeout')

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

        # Controlla se è scaduto il tempo massimo
        if now - self.command_start_time >= self.command_timeout:
            self.finish_command('failed', 'timeout')
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
            # Registra quando tutti i giunti raggiungono il bersaglio
            if self.target_reached_time is None:
                self.target_reached_time = now

            # Richiede che il bersaglio sia mantenuto per la durata prevista
            elif now - self.target_reached_time >= self.stable_duration:
                self.finish_command('completed', 'target_reached')
        else:
            # Se un giunto esce dalla tolleranza, ricomincia il conteggio
            self.target_reached_time = None

    def command_callback(self, msg):
        """Valida e inoltra un comando articolare al controller."""
        # Verifica che il vettore contenga sette posizioni
        if len(msg.data) != len(self.joint_names):
            self.get_logger().error(
                'Comando rifiutato: sono richieste '
                'esattamente sette posizioni articolari.'
            )
            self.publish_status('rejected', 'invalid_joint_count')
            return

        # Verifica che tutti i valori siano finiti
        if not all(math.isfinite(value) for value in msg.data):
            self.get_logger().error(
                'Comando rifiutato: il vettore contiene '
                'valori non finiti.'
            )
            self.publish_status('rejected', 'non_finite_position')
            return

        # Non consentire di sovrascrivere un comando in esecuzione
        if self.active_command is not None:
            self.get_logger().warning(
                'Nuovo comando rifiutato: è già in esecuzione '
                'un altro comando.'
            )
            self.publish_status('rejected', 'command_already_active')
            return

        # Salva le posizioni obiettivo nell'ordine dei giunti
        self.active_command = {
            joint: float(position)
            for joint, position in zip(self.joint_names, msg.data)
        }

        # Registra l'identità e i tempi di avvio del comando
        self.active_command_id = str(uuid.uuid4())
        self.command_start_time = time.monotonic()
        self.command_start_timestamp = (
            self.get_clock().now().nanoseconds / 1e9
        )
        self.target_reached_time = None

        # Pubblica il comando verso il controller
        command = Float64MultiArray()
        command.data = list(msg.data)
        self.command_publisher.publish(command)

        self.publish_status('executing', 'command_sent')

        self.get_logger().info(
            'Comando di posizione inviato al controller FR3.'
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
