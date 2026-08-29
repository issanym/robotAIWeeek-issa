import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        # Subscribe to the /scan topic published by sllidar_ros2
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.pub_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.linearX = 0.0
        self.angularZ = 0.0

    def listener_callback(self, msg):
        # Total points in this specific spin
        num_points = len(msg.ranges)
        msg_t = Twist()
        
        if num_points > 0:
            # Slice a 15-degree window to the left and 15-degree window to the right of 0 degrees
            # (Assuming 500 points total, 15 degrees is roughly 21 indices)
            front_left_sector = msg.ranges[0:63]
            front_right_sector = msg.ranges[-63:] # Wraps around the 360-degree mark
            left_up_sector = msg.ranges[63:125] 
            left_down_sector = msg.ranges[63:188]
            right_up_sector = msg.ranges[375:439]
            right_down_sector = msg.ranges[313:375]
            
            # Combine the front view
            front_cone = front_left_sector + front_right_sector
            left_cone = left_up_sector + left_down_sector
            right_cone = right_up_sector + right_down_sector
            
            # Clean out invalid 'inf' or '0.0' noise readings
            valid_ranges_front = [r for r in front_cone if msg.range_min < r < msg.range_max]
            valid_ranges_right = [r for r in right_cone if msg.range_min < r < msg.range_max]
            valid_ranges_leftt = [r for r in left_cone if msg.range_min < r < msg.range_max]
            
            if valid_ranges_front:
                obstacle_front = min(valid_ranges_front)
                if obstacle_front < 0.10:
                    msg_t.linear.x = 0.0
                    msg_t.angular.z = 0.8
                    self.get_logger().info(f'Closest threat ahead: {obstacle_front:.2f}m')
                    self.pub_vel.publish(msg_t)
                    return
                    
            if valid_ranges_right:
                obstacle_right = min(valid_ranges_front)
                if obstacle_right < 0.10:
                    msg_t.linear.x = 0.0
                    msg_t.angular.z = -0.8
                    self.get_logger().info(f'Closest threat right: {obstacle_right:.2f}m')
                    self.pub_vel.publish(msg_t)
                    return
                    
            if valid_ranges_leftt:
                obstacle_left = min(valid_ranges_front)
                if obstacle_left < 0.10:
                    msg_t.linear.x = 0.0
                    msg_t.angular.z = 0.8
                    self.get_logger().info(f'Closest threat left: {obstacle_left:.2f}m')
                    self.pub_vel.publish(msg_t)
                    return
                    
            msg_t.linear.x = 1.0
            msg_t.angular.z = 0.0
            self.get_logger().info(f'No Obstacle deteccted')
            self.pub_vel.publish(msg_t)
            

                

def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
