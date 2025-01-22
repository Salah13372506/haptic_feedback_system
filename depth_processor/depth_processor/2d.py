
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from haptic_control_interfaces.msg import SideCommand

class SimpleDepthProjector(Node):
    def __init__(self):
        super().__init__('simple_depth_projector')
        self.bridge = CvBridge()
        self.depth_sub = self.create_subscription(Image, 'processed_depth', self.projection_callback, 1)
        self.projection_pub = self.create_publisher(Image, 'depth_projection_2d', 1)
        self.haptic_pub = self.create_publisher(SideCommand, 'haptic_side', 10)

    def create_projection(self, depth_image):
        output_height = 400
        output_width = depth_image.shape[1]
        projection = np.zeros((output_height, output_width), dtype=np.uint8)
        obstacle_threshold = 230
        
        for x in range(output_width):
            column = depth_image[:, x]
            obstacle_pixels = np.where(column <= obstacle_threshold)[0]
            
            if len(obstacle_pixels) > 0:
                for y in obstacle_pixels:
                    depth_value = column[y]
                    proj_y = int(((255 - depth_value) / 255.0) * (output_height - 1))
                    projection[proj_y, x] = 255
        
        kernel = np.ones((5, 5), np.uint8)
        projection = cv2.dilate(projection, kernel, iterations=1)
        projection = cv2.medianBlur(projection, 5)
        
        return projection

    def analyze_quadrants(self, projection):
        height, width = projection.shape
        mid_x = width // 2
        mid_y = height // 2
        
        # Compte les pixels blancs dans chaque quadrant
        near_left = np.sum(projection[mid_y:, :mid_x] == 255)
        near_right = np.sum(projection[mid_y:, mid_x:] == 255)
        far_left = np.sum(projection[:mid_y, :mid_x] == 255)
        far_right = np.sum(projection[:mid_y, mid_x:] == 255)
        
        # Priorité aux obstacles proches
        command = SideCommand()
        near_total = near_left + near_right
        if near_total > 0:  # S'il y a des obstacles proches
            # Envoie RIGHT quand il y a plus d'obstacles à droite
            command.side = SideCommand.RIGHT if near_right > near_left else SideCommand.LEFT
        else:  # Sinon, regarder les obstacles lointains
            command.side = SideCommand.RIGHT if far_right > far_left else SideCommand.LEFT

        # Publication si des obstacles sont détectés
        if near_total > 0 or (far_left + far_right) > 0:
            self.haptic_pub.publish(command)
            direction = "DROITE" if command.side == SideCommand.RIGHT else "GAUCHE"
            self.get_logger().info(f'Direction envoyée: {direction}')
            self.get_logger().info(f'Proche - Gauche: {near_left}, Droite: {near_right}')
            self.get_logger().info(f'Loin - Gauche: {far_left}, Droite: {far_right}')

        return command

    def add_visualization(self, projection):
        color_proj = cv2.cvtColor(projection, cv2.COLOR_GRAY2BGR)
        height, width = projection.shape
        mid_x = width // 2
        mid_y = height // 2
        
        # Lignes de séparation des quadrants
        cv2.line(color_proj, (mid_x, 0), (mid_x, height), (0, 0, 255), 2)  # Vertical
        cv2.line(color_proj, (0, mid_y), (width, mid_y), (255, 0, 0), 2)   # Horizontal
        
        return color_proj

    def projection_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            projection = self.create_projection(depth_image)
            visualization = self.add_visualization(projection)
            
            self.analyze_quadrants(projection)
            
            msg = self.bridge.cv2_to_imgmsg(visualization, encoding='bgr8')
            msg.header = msg.header
            self.projection_pub.publish(msg)
            
            cv2.imshow('2D Projection with Quadrants', visualization)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Projection error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    projector = SimpleDepthProjector()
    try:
        rclpy.spin(projector)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        projector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()