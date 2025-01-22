#!/usr/bin/env python3
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from haptic_control_interfaces.msg import SideCommand
try:
    # Changez cette ligne pour utiliser le nouveau package d'interfaces
    from haptic_control_interfaces.msg import HapticCommand
except ImportError as e:
    print(f"Error importing HapticCommand message: {e}")
    print("Make sure haptic_control_interfaces is properly built and sourced")
    raise
class HapticPublisher(Node):
    def __init__(self):
        super().__init__('haptic_publisher')
        
        # Créer le publisher
        self.publisher = self.create_publisher(
            HapticCommand,
            'haptic_commands',
            10)
            
        # Paramètres
        self.declare_parameter('default_intensity', 0.8)
        self.default_intensity = self.get_parameter('default_intensity').value
        
        # Ajout des constantes pour les groupes de moteurs
        self.RIGHT_MOTORS = [0, 1, 3]
        self.LEFT_MOTORS = [2,4,5]

        # Ajout du subscriber pour les commandes de côté
        self.side_subscription = self.create_subscription(
            SideCommand,
            'haptic_side',
            self.side_callback,
            10)
        
    def send_haptic_command(self, motor_id: int, waveform: int, activate: bool, intensity: float = None):
        """
        Envoie une commande à un moteur spécifique.
        
        Args:
            motor_id (int): ID du moteur (0-7)
            waveform (int): Pattern de vibration (0-123)
            activate (bool): Activer/désactiver le moteur
            intensity (float, optional): Intensité de la vibration (0.0-1.0)
        """
        msg = HapticCommand()
        msg.motor_id = motor_id
        msg.waveform = waveform
        msg.activate = activate
        msg.intensity = intensity if intensity is not None else self.default_intensity
        
        self.publisher.publish(msg)
        self.get_logger().info(
            f'Envoi commande: Motor {motor_id}, Waveform {waveform}, '
            f'Activate {activate}, Intensity {msg.intensity}'
        )

    def activate_motor(self, motor_id: int, waveform: int = 56, intensity: float = None):
        """Active un moteur avec les paramètres spécifiés."""
        self.send_haptic_command(motor_id, waveform, True, intensity)

    def deactivate_motor(self, motor_id: int):
        """Désactive un moteur spécifique."""
        self.send_haptic_command(motor_id, 0, False, 0.0)

    def activate_sequence(self, motor_sequence: list, delay: float = 0.1):
        """
        Active une séquence de moteurs.
        
        Args:
            motor_sequence (list): Liste des IDs de moteurs à activer en séquence
            delay (float): Délai entre chaque activation (en secondes)
        """
        def trigger_next(idx):
            if idx >= len(motor_sequence):
                return
            
            motor_id = motor_sequence[idx]
            self.activate_motor(motor_id)
            
            if idx < len(motor_sequence) - 1:
                self.create_timer(delay, lambda: trigger_next(idx + 1))
        
        trigger_next(0)

    def activate_motor_group(self, motors, waveform=56, intensity=None):
        """Active un groupe de moteurs avec les mêmes paramètres"""
        for motor_id in motors:
            self.activate_motor(motor_id, waveform, intensity)

    def deactivate_motor_group(self, motors):
        """Désactive un groupe de moteurs"""
        for motor_id in motors:
            self.deactivate_motor(motor_id)

    def side_callback(self, msg):
        """Callback pour les commandes de côté"""
        # Désactiver tous les moteurs d'abord
        self.deactivate_motor_group(self.RIGHT_MOTORS + self.LEFT_MOTORS)

    #Activer le groupe approprié
        if msg.side == SideCommand.RIGHT:
            self.activate_motor_group(self.RIGHT_MOTORS)
            self.get_logger().info('Activating RIGHT motors')
        else:
            self.activate_motor_group(self.LEFT_MOTORS)
            self.get_logger().info('Activating LEFT motors')    

def main(args=None):
    rclpy.init(args=args)
    haptic_publisher = HapticPublisher()
    
    try:
        rclpy.spin(haptic_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        haptic_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()