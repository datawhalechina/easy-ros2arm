import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge

import os
import time

from dofbot_msgs.msg import Imageinfo

class PictureSubscriber(Node):
    
    def __init__(self):
        super().__init__('picture_subscriber')
        self.subscription = self.create_subscription(
            Imageinfo,
            'Camera_Images',
            self.listener_callback,
            5)
        self.subscription

    def listener_callback(self, msg):
        bridge = CvBridge()
        imgs = msg.image
        try:
            cv_image = bridge.imgmsg_to_cv2(imgs, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error('%s', e)
        
        # imgs_dirname = os.path.dirname(os.path.abspath(__file__))
        imgs_route = os.path.join("/root/dofbot_ws/src/dofbot_classification/", 'images/')
        if not os.path.exists(imgs_route):
            os.makedirs(imgs_route)
        cur_time = int(time.time())
        imgs_path = os.path.join(imgs_route, "image_{}.jpg".format(cur_time))
        # os.chdir(imgs_route)
        cv2.imwrite(imgs_path, cv_image)
        self.get_logger().info('{} have been saved!'.format(imgs_path))
    
def main(args=None):
    rclpy.init(args=args)
    picture_subscriber = PictureSubscriber()
    rclpy.spin(picture_subscriber)
    picture_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
