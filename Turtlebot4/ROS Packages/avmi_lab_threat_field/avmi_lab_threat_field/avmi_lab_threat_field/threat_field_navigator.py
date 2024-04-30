import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import math
from math import atan2

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from avmi_interfaces.srv import GetColorVals


## PARAMETERS

STEP_DIST = 0.35 # Constant radius for setting new goal point
INTERVAL_TIME = 7.5 # Time in seconds between camera measurements


## GLOBAL VARIABLES

# Robot state
x = 0.0
y = 0.0
theta = 0.0

# Goal
goal = Point()


## Function Definitions

def bound_angle(angle):
    # Normalize the angle to be between -pi and pi
    normalized_angle = math.atan2(math.sin(angle),math.cos(angle))
    return normalized_angle


def calc_direction(color_vals):
    # Find the highest sensor reading
    max_reading = max(color_vals)
    max_index = color_vals.index(max_reading)
    
    # Find the highest value between the two neighboring sensors
    left_neighbor = (max_index + 1) % len(color_vals)
    left_reading = color_vals[left_neighbor]

    right_neighbor = (max_index - 1) % len(color_vals)
    right_reading = color_vals[right_neighbor]

    # Calculate the interpolation factor based on the ratio of the readings`
    if left_reading > right_reading:
        max_neighbor_reading = left_reading
        interpolation_factor = max_reading / (max_reading + max_neighbor_reading)
    elif left_reading < right_reading:
        max_neighbor_reading = right_reading
        interpolation_factor = -max_reading / (max_reading + max_neighbor_reading)
    else:
        interpolation_factor = 0
    
    
    # Interpolate the direction in radians
    interpolated_direction = -math.pi/4 + max_index * (math.pi / 2) + interpolation_factor * (math.pi / 2)

    # Bound between 0 and 2 pi
    interpolated_direction = bound_angle(interpolated_direction)
    
    return interpolated_direction


def calc_goal(direction):

    global goal

    # Compute new X and Y for goal position
    x_new = x + STEP_DIST * math.cos(theta + direction)
    y_new = y + STEP_DIST * math.sin(theta + direction)

    # Goal point
    goal.x = x_new
    goal.y = y_new

    print("Direction = %f degrees" % direction) 


def calc_speed():

    speed = Twist()

    inc_x = goal.x - x
    inc_y = goal.y - y

    distance_to_goal = math.sqrt(inc_x**2 + inc_y**2)
    angle_to_goal = atan2(inc_y, inc_x)
    angular_error = bound_angle(angle_to_goal - theta)

    lin_speed = 0.25
    # kp = 1.0
    # lin_speed = distance_to_goal*kp + 0.05
    # if lin_speed > 0.5:
    #     lin_speed = 0.5

    if distance_to_goal < 0.1:
        speed.linear.x = 0.0
        speed.angular.z = 0.0
    else:
        if abs(angular_error) > 0.1:
            if angular_error > 0:
                speed.linear.x = 0.0
                speed.angular.z = 0.5
            else:
                speed.linear.x = 0.0
                speed.angular.z = -0.5
        else:
            speed.linear.x = lin_speed
            speed.angular.z = 0.0

    return speed


def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


class DrivingTest(Node):

    def __init__(self):
        super().__init__('driving_test')
        self.cb = None 
        self.timer_cb = MutuallyExclusiveCallbackGroup()
        # Set up client for requesting camera system color values
        self.cli = self.create_client(GetColorVals, 'get_color_vals')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = GetColorVals.Request()
        # Set up subscriber for odometry from robot
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.new_odom, qos_profile_sensor_data)
        # Set up publisher for commanded velocities
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel',1)
        # Set up a timer for polling camera system
        self.read_cameras_timer = self.create_timer(INTERVAL_TIME, self.read_cameras, callback_group=self.timer_cb)
        # Set up timer for motion control
        self.motion_control_timer = self.create_timer(0.1, self.motion_control)


    async def send_request(self):
        return await self.cli.call_async(self.req)
    

    def new_odom(self, msg):

        global x
        global y
        global theta

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion(rot_q.x, rot_q.y, rot_q.z, rot_q.w)


    async def read_cameras(self):

        # Request color values from camera system
        response = await self.send_request()
        print(
        'Color values received: cam1 = %f, cam2 = %f, cam3 = %f, cam4 = %f' % 
        (response.cam1, response.cam2, response.cam3, response.cam4))
        # Create array of color values from response
        color_vals = [response.cam1, response.cam2, response.cam3, response.cam4]
        # Compute the direction to travel
        direction = calc_direction(color_vals)
        # Compute the destination point
        calc_goal(direction)


    def motion_control(self):

        # Compute linear and angular speeds
        speed = calc_speed()
        # Send speed command to robot
        self.vel_pub.publish(speed)


def main():
    rclpy.init()

    driving_test = DrivingTest()

    # Uncomment this to keep node open
    rclpy.spin(driving_test)
    
    driving_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
