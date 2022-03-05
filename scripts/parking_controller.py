#!/usr/bin/env python

import rospy
import numpy as np
import time

from visual_servoing.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped

class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """

    Kp = 0.7
    Ki = 0
    Kd = 0

    def __init__(self):
        rospy.Subscriber("/relative_cone", ConeLocation, self.relative_cone_callback)

        DRIVE_TOPIC = rospy.get_param("~drive_topic") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error", ParkingError, queue_size=10)

        self.parking_distance = .75 # meters; try playing with this number!
        self.eps = 0.05
        self.relative_x = 0
        self.relative_y = 0

        self.previous_error = 0
        self.integral = 0
        self.previous_time = time.time()

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()
        
        def create_message(velocity, steering_angle):
            drive_cmd.header.stamp = rospy.Time.now()
            drive_cmd.header.frame_id = 'map'
            drive_cmd.drive.steering_angle = steering_angle
            drive_cmd.drive.steering_angle_velocity = 0
            drive_cmd.drive.speed = velocity
            drive_cmd.drive.acceleration = 0
            drive_cmd.drive.jerk = 0

        # if self.relative_x < self.parking_distance + self.eps:
        #     rospy.loginfo('stopping')
        #     create_message(0, 0)
        # else:
        #     # Cone to left of car path
        #     if self.relative_y > 0.05:
        #         rospy.loginfo('turning left')
        #         create_message(0.5, 0.34) # hard left
        #     # Cone to right of car path
        #     elif self.relative_y < 0.05:
        #         rospy.loginfo('turning right')
        #         create_message(0.5, -0.34) # hard right
        #     # Cone pretty much in front of car path
        #     else:
        #         rospy.loginfo('straight forwards')
        #         create_message(0.5, 0)
        if self.relative_x < self.parking_distance + self.eps:
            rospy.loginfo('stopping')
            create_message(0, 0)
        else:
            # Angle being misaligned
            error = self.relative_y
            output = self.pid_controller(error)
            if output > 0:
                angle = min(0.34, output)
            elif output <= 0:
                angle = max(-0.34, output)
            create_message(0.5, angle)
        
        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def pid_controller(self, error):
        curr_time = time.time()
        dt = curr_time - self.previous_time
        prop = error
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.Kp * prop + self.Ki * self.integral + self.Kd * derivative
        # Reset previous error/time values
        self.previous_error = error
        self.previous_time = curr_time
        return output

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        #################################

        # YOUR CODE HERE
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)

        #################################
        
        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
