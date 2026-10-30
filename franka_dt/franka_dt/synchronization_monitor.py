import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState


class SynchronizationMonitor(Node):

    def __init__(self):
        super().__init__('synchronization_monitor')

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

        # Istante di ricezione dell'ultimo messaggio valido
        self.last_receipt_time = None

        # Numero di messaggi validi ricevuti
        self.valid_messages = 0

        # Soglia massima di attesa tra aggiornamenti validi
        self.max_data_age = 2.0

        # Stato iniziale: ancora da determinare
        # True  = sincronizzato
        # False = non sincronizzato
        # None  = stato non ancora determinato
        self.is_synchronized = None

        # Sottoscrizione agli stati articolari
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Controllo periodico della freschezza dei dati
        self.timer = self.create_timer(
            1.0,
            self.check_synchronization
        )

        self.get_logger().info(
            'Synchronization Monitor avviato. '
            'In attesa di /joint_states...'
        )

    def joint_state_callback(self, msg):

        # Verifica che siano presenti tutti i sette giunti
        if not all(
            joint in msg.name for joint in self.joint_names
        ):
            return

        # Associa ogni giunto alla relativa posizione
        positions = dict(zip(msg.name, msg.position))

        # Verifica che siano disponibili tutte le posizioni
        if not all(
            joint in positions for joint in self.joint_names
        ):
            return

        # Registra l'istante di ricezione usando
        # l'orologio monotono del sistema
        self.last_receipt_time = time.monotonic()

        # Aggiorna il numero di messaggi validi
        self.valid_messages += 1

    def check_synchronization(self):

        # Caso 1: non è ancora arrivato alcun messaggio valido
        if self.last_receipt_time is None:

            if self.is_synchronized is not False:
                self.get_logger().warning(
                    'NON SINCRONIZZATO: '
                    'nessuno stato articolare valido ricevuto.'
                )

                self.is_synchronized = False

            return

        # Calcola da quanto tempo non riceviamo aggiornamenti
        data_age = time.monotonic() - self.last_receipt_time

        # Caso 2: gli ultimi dati sono troppo vecchi
        if data_age > self.max_data_age:

            # Avvisa soltanto quando cambia lo stato
            if self.is_synchronized is not False:
                self.get_logger().warning(
                    'NON SINCRONIZZATO: '
                    'aggiornamento dello stato non recente. '
                    f'Ultimo messaggio ricevuto '
                    f'{data_age:.2f} s fa.'
                )

                self.is_synchronized = False

        # Caso 3: gli aggiornamenti sono recenti
        else:

            # Segnala il ripristino soltanto se necessario
            if self.is_synchronized is not True:
                self.get_logger().info(
                    'SINCRONIZZATO: ricezione degli stati '
                    'articolari regolare. '
                    f'Ultimo aggiornamento: {data_age:.3f} s fa. '
                    f'Messaggi validi ricevuti: '
                    f'{self.valid_messages}.'
                )

                self.is_synchronized = True


def main(args=None):
    rclpy.init(args=args)

    node = SynchronizationMonitor()

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
