from __future__ import print_function
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
import pygame._sdl2.controller

class Controller(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Initialize pygame and sdl controller and the joystick subsystem
        pygame.init()
        pygame.joystick.init()
        pygame._sdl2.controller.init()
        
        # Connect to the first available gamepad
        if pygame._sdl2.controller.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.controller = pygame._sdl2.controller.Controller.from_joystick(self.joystick)
            self.get_logger().info(f"Connection status: {self.controller.attached()}")
        else:
            self.get_logger().error("No gamepad detected!")
            self.controller = None
        
        # Video game style loop: Poll the hardware at a crisp 20Hz (every 0.05s)
        timer_period = 0.05 
        self.timer_ = self.create_timer(timer_period, self.pad_callback)

    def pad_callback(self):
        if not self.controller:
            return

        # 1. Pump internal event queue (Required by pygame to update hardware states)
        pygame.event.pump()
        
        # 2. Directly POLL the hardware states (Values range from -1.0 to 1.0 automatically)
        stick_x = self.controller.get_axis(pygame.CONTROLLER_AXIS_LEFTX)/32768.0
        stick_y = self.controller.get_axis(pygame.CONTROLLER_AXIS_LEFTY)/32768.0
        
        # left trigger is the deadman
        deadman_pressed = True if self.controller.get_axis(pygame.CONTROLLER_AXIS_TRIGGERLEFT)/32768.0 > 0.0 else False

        msg = Twist()
        if deadman_pressed:
            # 3. Apply Deadzone Filter (Pygame is highly sensitive)
            if abs(stick_y) > 0.1:
                # Pygame Joysticks are inverted: Up is negative, Down is positive
                msg.linear.x = -1.0 if stick_y > 0 else 1.0   
            if abs(stick_x) > 0.1:
                msg.angular.z = -0.8 if stick_x > 0 else 0.8

        # 4. Continuous publishing
        self.get_logger().info(f'Publishing: Linear={msg.linear.x}, Angular={msg.angular.z}')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    controller_node = Controller()

    rclpy.spin(controller_node)

    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()