import os
import sys
import time
# ROS2的客户端库(python) rclpy
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
# image-----below----
# topic sensor_msgs/msg/Image to cv2
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
 
# import snoop
 
 
def cv2_imshow(win_name: str, img_mat) -> None:
    """
    显示接受到的图像数据
    :param win_name:
    :param img_mat:
    :return:
    """
    cv2.imshow(win_name, img_mat)
    # cv2.waitKey(0)  # delay – Delay in milliseconds. 0 is the special value that means “forever”.
    if cv2.waitKey(1) & 0xFF == ord('q'):  # 当你按下q键时，显示图片的窗口被关掉，并打印出I'm done，结束程序
        print("I'm done")
        sys.exit(0)
 
 
class ImageNode(Node):
    # Node constructor
    def __init__(self) -> None:
        super().__init__("image_node")
 
        self.print_count = 1
        self.img_count = 1
        self.cv_bridge = CvBridge()
 
        # get from topic /camera/image_raw [sensor_msgs/msg/Image]
        self._image_sub = self.create_subscription(
            Image,                   # topic_type : sensor_msgs/msg/Image
            "/camera/image_raw",     # topic_name :  /camera/image_raw
            self.image_cb,           # image_listener_callback
            qos_profile_sensor_data  # “queue size” is 10. Queue size is a required QoS
        )
 
    # @snoop
    def image_cb(self, msg: Image) -> None:
        """
        image listener callback
        :param msg:
        :return:
        """
        if self.print_count:
            print("msg.header :  {}".format(
                msg.header))  # msg.header: std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=23495, nanosec=888000000), frame_id='camera_link_optical')
            print("msg.encoding :  {}".format(msg.encoding))  # msg.encoding: bgr8
            # print("msg.data : \n {}".format(msg.data))
            print("msg.height :  {}".format(msg.height))  # msg.height: 480
            print("msg.width : {}".format(msg.width))  # msg.width: 640
            self.print_count = 0
 
        # record start time
        # start_time = time.perf_counter()
 
        # convert to cv image & predict
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        save_dir = "/home/parallels/test/src/mobile_robot_recognition/scripts/save"
        img_path = os.path.join(save_dir, str(self.img_count).zfill(4) + ".jpg")
        cv2.imwrite(filename=img_path, img=cv_image)
        print(img_path)
        self.img_count += 1
        time.sleep(2)  # 2.0s
 
        # end_time = time.perf_counter()
        # print(f"{end_time - start_time = }")
 
        # display
        # cv2_imshow("image_raw", cv_image)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = ImageNode()
    rclpy.spin(node)
    rclpy.shutdown()
    node.destroy_node()
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
