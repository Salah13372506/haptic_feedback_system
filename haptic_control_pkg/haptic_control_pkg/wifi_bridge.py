#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from haptic_control_interfaces.msg import HapticCommand
import socket
import json

class HapticWifiBridge(Node):
    def __init__(self):
        super().__init__('haptic_wifi_bridge')
        
        # Paramètres WiFi
        self.declare_parameter('arduino_ip', '192.168.4.1')
        self.declare_parameter('arduino_port', 8888)
        
        # Configuration socket
        self.arduino_ip = self.get_parameter('arduino_ip').value
        self.arduino_port = self.get_parameter('arduino_port').value
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Subscriber
        self.subscription = self.create_subscription(
            HapticCommand,
            'haptic_commands',
            self.command_callback,
            10)
            
        self.get_logger().info(f'WiFi Bridge initialized. Target: {self.arduino_ip}:{self.arduino_port}')
        
    def command_callback(self, msg):
        """Reçoit les commandes haptiques et les envoie via WiFi"""
        try:
            # Log la réception du message
            self.get_logger().info(f'Received command: Motor {msg.motor_id}, Waveform {msg.waveform}')
            
            # Convertir le message en format JSON pour l'Arduino
            command = {
                'motor': msg.motor_id,
                'waveform': msg.waveform,
                'activate': msg.activate,
                'intensity': msg.intensity
            }
            
            # Log avant envoi
            self.get_logger().info(f'Sending command: {command}')
            
            # Encoder et envoyer via UDP
            data = json.dumps(command).encode('utf-8')
            bytes_sent = self.sock.sendto(data, (self.arduino_ip, self.arduino_port))
            
            # Log après envoi
            self.get_logger().info(f'Sent {bytes_sent} bytes to Arduino')
            
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    bridge = HapticWifiBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.sock.close()
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()