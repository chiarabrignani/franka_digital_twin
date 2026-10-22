import json
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class ODTEMonitor(Node):

    def __init__(self):
        super().__init__('odte_monitor')

        # Parametri configurabili
        self.declare_parameter('expected_rate_hz', 100.0)
        self.declare_parameter('desired_timeliness_ms', 50.0)
        self.declare_parameter('window_seconds', 10.0)

        self.expected_rate = float(
            self.get_parameter('expected_rate_hz').value
        )

        self.desired_timeliness = float(
            self.get_parameter('desired_timeliness_ms').value
        ) / 1000.0

        self.window = float(
            self.get_parameter('window_seconds').value
        )

        if (
            self.expected_rate <= 0
            or self.desired_timeliness <= 0
            or self.window <= 0
        ):
            raise ValueError(
                'I parametri ODTE devono essere positivi.'
            )

        # (tempo di ricezione monotono, timeliness in secondi)
        self.observations = deque()

        self.start_time = time.monotonic()

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.publisher = self.create_publisher(
            String,
            '/digital_twin/odte',
            10
        )

        self.timer = self.create_timer(
            1.0,
            self.calculate_odte
        )

        self.get_logger().info(
            'ODTE Monitor avviato: percorso Physical -> Digital.'
        )

    def joint_state_callback(self, msg):
        # Tempo reale monotono all'ingresso del callback.
        reception_monotonic = time.monotonic()

        # Timestamp ROS al momento della ricezione.
        # Deve usare lo stesso riferimento temporale del messaggio.
        reception_ros = self.get_clock().now().nanoseconds / 1e9

        # Verifica che siano presenti le sette articolazioni FR3.
        required_joints = {
            f'fr3_joint{i}' for i in range(1, 8)
        }

        if not required_joints.issubset(set(msg.name)):
            return

        if len(msg.position) != len(msg.name):
            return

        positions = dict(zip(msg.name, msg.position))

        if not all(
            math.isfinite(positions[joint])
            for joint in required_joints
        ):
            return

        # Timestamp di generazione dell'aggiornamento.
        generation_ros = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec / 1e9
        )

        # Un timestamp nullo o nel futuro non è utilizzabile.
        if (
            generation_ros <= 0
            or generation_ros > reception_ros
        ):
            return

        # Tempo impiegato dal monitor per validare il messaggio.
        # È una stima del tempo di elaborazione del monitor,
        # non del tempo necessario ad aggiornare l'intero Digital Twin.
        execution_start = time.monotonic()

        # La validazione del messaggio è già stata effettuata.
        # Misuriamo il tempo trascorso durante questa elaborazione.
        execution_time = time.monotonic() - execution_start

        # Osservazione temporale Physical -> Digital:
        # ricezione - generazione + tempo di elaborazione.
        timeliness = (
            reception_ros
            - generation_ros
            + execution_time
        )

        self.observations.append(
            (reception_monotonic, timeliness)
        )

        self.remove_old_observations(reception_monotonic)

    def remove_old_observations(self, now):
        while (
            self.observations
            and now - self.observations[0][0] > self.window
        ):
            self.observations.popleft()

    def calculate_odte(self):
        now = time.monotonic()
        self.remove_old_observations(now)

        if len(self.observations) < 2:
            self.get_logger().warning(
                'ODTE non ancora calcolabile: '
                'servono almeno due aggiornamenti validi.'
            )
            return

        # Tempo effettivo di osservazione, limitato alla finestra.
        elapsed = min(
            self.window,
            now - self.start_time
        )

        if elapsed <= 0:
            return

        received_messages = len(self.observations)

        # Frequenza media degli aggiornamenti validi ricevuti.
        measured_rate = received_messages / elapsed

        # Reliability: rapporto tra frequenza misurata e attesa.
        reliability = min(
            1.0,
            measured_rate / self.expected_rate
        )

        # Timeliness: quota di osservazioni entro la soglia desiderata.
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

        # Per il percorso Physical -> Digital, A = 1 secondo il paper.
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
                'generation_to_reception_plus_monitor_processing'
        }

        message = String()
        message.data = json.dumps(metrics)
        self.publisher.publish(message)

        self.get_logger().info(
            f"T={timeliness:.3f} | "
            f"R={reliability:.3f} | "
            f"A={availability:.3f} | "
            f"ODTE={odte:.3f}"
        )


def main(args=None):
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
