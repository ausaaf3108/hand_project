import rclpy
from rclpy.node import Node
from .hand_test import HandDetect
import cv2
from geometry_msgs.msg import Twist

class HandSend(Node):

    def __init__(self,name):
        super().__init__(name)
        self.publisher_ = self.create_publisher(Twist,"micro_ros_arduino_twist_subscriber",10)


    def hand_draw(self):
        self.hd = HandDetect()
        while True:
            self.frame=self.hd.vid_process()
            cv2.imshow('Webcam', self.frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            #self.get_logger().info(str(self.hd.state))
            msg = Twist()
            msg.linear.x = float(self.hd.state[0])
            msg.linear.y = float(self.hd.state[1])
            msg.linear.z = float(self.hd.state[2])
            msg.angular.x = float(self.hd.state[3])
            msg.angular.y = float(self.hd.state[4])
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            
        self.hd.cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    hand = HandSend("send_hand")
    hand.hand_draw()
    rclpy.spin(hand)
    rclpy.shutdown()

if __name__ == '__main__':
    main()