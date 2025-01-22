
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np

class DepthObstacleDetector(Node):
    def __init__(self):
        super().__init__('depth_obstacle_detector')
        self.bridge = CvBridge()
        self.depth_sub = self.create_subscription(Image, '/stereo/converted_depth', self.depth_callback, 1)
        self.processed_pub = self.create_publisher(Image, 'processed_depth', 1)
        self.quadrant_pub = self.create_publisher(Int32, 'best_quadrant', 1)

    def process_depth_image(self, depth_image):
        near_threshold = 0.5
        far_threshold = 4.0
        result = np.zeros_like(depth_image)
        result[(depth_image >= far_threshold) | (depth_image == 0)] = 255
        result[depth_image <= near_threshold] = 0
        
        mask_middle = (depth_image > near_threshold) & (depth_image < far_threshold)
        middle_pixels = depth_image[mask_middle]
        if middle_pixels.size > 0:
            normalized_middle = ((middle_pixels - near_threshold) /
                               (far_threshold - near_threshold) * 200 + 30)
            result[mask_middle] = normalized_middle

        result = result.astype(np.uint8)
        kernel = np.ones((5,5), np.uint8)
        eroded = cv2.erode(result, kernel, iterations=1)
        dilated = cv2.dilate(eroded, kernel, iterations=1)
        output = cv2.medianBlur(dilated, 3)

        # Division verticale en deux
        _, width = output.shape
        w_half = width // 2
        
        zones = [
            output[:, 0:w_half],      # Zone gauche (2)
            output[:, w_half:width],   # Zone droite (1)
        ]
        
        black_pixels = [int(np.sum(z == 0)) for z in zones]
        best_zone = 2 if black_pixels[0] < black_pixels[1] else 1

        msg = Int32()
        msg.data = int(best_zone)
        self.quadrant_pub.publish(msg)
        
        cv2.imshow('Processed Depth', output)
        cv2.waitKey(1)
        return output

    def depth_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            processed = self.process_depth_image(cv_image)
            processed_msg = self.bridge.cv2_to_imgmsg(processed, encoding='mono8')
            processed_msg.header = msg.header
            self.processed_pub.publish(processed_msg)
        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")

def main(args=None):
    rclpy.init(args=args)
    detector = DepthObstacleDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    
# #Dimensions: 720x1280
