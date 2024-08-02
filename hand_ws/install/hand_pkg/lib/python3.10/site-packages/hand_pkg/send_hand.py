import rclpy
from rclpy.node import Node
from .hand_test import HandDetect
import cv2

class HandSend(Node):

    def __init__(self,name):
        super().__init__(name)

    def hand_draw(self):
        self.hd = HandDetect()
        while True:
            self.frame=self.hd.vid_process()
            cv2.imshow('Webcam', self.frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            self.get_logger().info(str(self.hd.state))
            
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