import rclpy
from rclpy.node import Node
from mavros_msgs.msg import Altitude
from geometry_msgs.msg import TwistStamped, Twist
import time

class Stabilizator(Node):
    def __init__(self, msg):
        super.__init__('stabilizator')

        # PID parameter
        self.dt = 0.1
        self.Kp_roll = 1.
        self.Ki_roll = 0.
        self.Kd_roll = 0.

        self.Kp_roll = 1.
        self.Ki_roll = 0.
        self.Kd_roll = 0.

        # hover setpoints
        self.setpoint_velx = 0.
        self.setpoint_vely = 0.

        # PID state variables
        self.prev_error_x = 0.
        self.prev_error_y = 0.
        self.e_int_x = 0.
        self.e_int_y = 0.

        self.current_velx = 0.
        self.current_vely = 0.

        # timer for hover control (10Hz)
        self.hover_timer = self.create_timer(self.dt, self.hover_control)
        
        # Subscriptions
        self.alt = self.create_subscription(
            Altitude,
            '/mavros/altitude',
            self.listenAltitude,
            5
        )
        self.velocity_local = self.create_subscription(
            TwistStamped,
            'mavros/local_position/velocity_body',
            self.listenVel,
            5
        )
        self.vel_out = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            10
        )
            
    def listenAltitude(self, msg):
        self.get_logger().info(f'alt : {msg.relative}')

    def listenVel(self, msg):
        self.get_logger().info(f'velX: {msg.twist.linear.x}\nvelY: {msg.twist.linear.y}')

    # PID controller for roll stabilization
    def holdX(self, current_speed, setpoint):
        error = current_speed - setpoint
        self.e_int_x = self.e_int_x + (self.prev_error_x + error)/2*self.dt
        e_dot = (error - self.prev_error_x) / self.dt
        speed_out = self.Kp_roll * error + self.Ki_roll * self.e_int_x + self.Kd_roll * e_dot
        self.prev_error_x = error
        return speed_out
    
    # PID controller for pitch stabilization
    def holdY(self, current_speed, setpoint):
        error = current_speed - setpoint
        self.e_int_y = self.e_int_y + (self.prev_error_y + error) / 2 * self.dt
        e_dot = (error - self.prev_error_y) / self.dt
        speed_out = self.Kp_pitch * error + self.Ki_pitch * self.e_int_y + self.Kd_pitch * e_dot
        self.prev_error_y = error
        return speed_out
    
    # control output to hold position
    def hoverControl(self):
        control_x = self.holdX(self.current_velx, self.setpoint_velx)
        control_y = self.holdY(self.current_vely, self.setpoint_vely)
        
        self.get_logger().info(f'control x = {control_x}\ncontrol y = {control_y}')

        cmd_msg = Twist()
        cmd_msg.linear.x = -control_x
        cmd_msg.linear.y = -control_y
        cmd_msg.linear.z = 0.
        self.vel_out(cmd_msg)

def main(args=None):
    rclpy.init(args = args)
    node = Stabilizator
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()