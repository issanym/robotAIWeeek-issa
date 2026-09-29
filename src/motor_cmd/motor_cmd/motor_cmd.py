import serial
import rclpy  
from geometry_msgs.msg import Twist
from rclpy.node import Node

class Motor_cmd(Node):
    def __int__(self):
        super().__init__('motor_cmd')
        self.vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=3)

    def cmd_callback(self, msg):
        if msg.linear.x > 0 and msg.angular.z == 0:
            val=1
        elif msg.linear.x < 0 and msg.angular.z == 0:
            val=0
        elif msg.linear.x == 0 and msg.angular.z > 0:
            val=5
        elif msg.linear.x == 0 and msg.angular.z < 0:
            val=4
        elif msg.linear.x > 0 and msg.angular.z < 0:
            val=2
        elif msg.linear.x > 0 and msg.angular.z > 0:
            val=3
        elif msg.linear.x > 0 and msg.angular.z < 0:
            val=6
        elif msg.linear.x > 0 and msg.angular.z > 0:
            val=7
        else :
            val='x'

        byte = f"{val}".encode("utf-8")
        self.cmd_send(byte)

    def cmd_send(self, code):
        self.ser.write(code)


def main(args=None):
    rclpy.init(args=args)
    node = Motor_cmd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
