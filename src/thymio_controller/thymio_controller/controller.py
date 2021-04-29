# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ROS2 example controller."""

from webots_ros2_core.webots_node import WebotsNode
from webots_ros2_msgs.srv import SetDifferentialWheelSpeed

import rclpy

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

import math

import os

class ExampleController(WebotsNode):

    def __init__(self, args):
        super().__init__('example_controller', args)
        self.jointTimer = self.create_timer(0.05, self.joint_callback)
        self.leftMotor = self.robot.getMotor('motor.left')
        self.rightMotor = self.robot.getMotor('motor.right')
        self.leftMotor.setPosition(float('inf'))
        self.rightMotor.setPosition(float('inf'))
        self.leftMotor.setVelocity(0)
        self.rightMotor.setVelocity(0)
        self.motorMaxSpeed = self.leftMotor.getMaxVelocity()
        self.motorService = self.create_service(SetDifferentialWheelSpeed,
                                                'motor', self.motor_callback)

        self.cmdVelSubscriber = self.create_subscription(Twist, 'cmd_vel',
                                                         self.cmdVel_callback,
                                                         10)

        # the supervisor node, which we use to get absolute position for now
        self.robot_node = self.robot.getSelf()
        self.translation_field = self.robot_node.getField("translation")
        self.rotation_field = self.robot_node.getField("rotation")

        # tf broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)


    def joint_callback(self):
        trans = self.translation_field.getSFVec3f()
        #print("MY_ROBOT is at position: %g %g %g" % (trans[0], trans[1], trans[2]))
        
        rot = self.rotation_field.getSFRotation()
        #print("MY_ROBOT is at orientation %g %g %g %g" % (rot[0], rot[1], rot[2], rot[3]))

        now = self.get_clock().now().to_msg()
        odom_tf = TransformStamped()
        base_link_tf = TransformStamped()
        
        base_link_tf.transform.translation.x = trans[0]
        base_link_tf.transform.translation.y = trans[1]
        base_link_tf.transform.translation.z = 0.0 # ignore z 

        # http://www.cse.unsw.edu.au/~cs9018/vrml98/quat.html
        base_link_tf.transform.rotation.x = rot[0] * math.sin(rot[3] / 2)
        base_link_tf.transform.rotation.y = rot[1] * math.sin(rot[3] / 2)
        base_link_tf.transform.rotation.z = rot[2] * math.sin(rot[3] / 2)
        base_link_tf.transform.rotation.w = math.cos(rot[3] / 2)

        # create static transform +90 around X and +180 around Z.

        # w0 = base_link_tf.transform.rotation.w
        # x0 = base_link_tf.transform.rotation.x
        # y0 = base_link_tf.transform.rotation.y
        # z0 = base_link_tf.transform.rotation.z

        # w1 = 0.0
        # x1 = 0.0
        # y1 = 0.0
        # z1 = 1.0

        # w2 = -x1*x0 - y1*y0 - z1*z0 + w1*w0
        # x2 = x1*w0 + y1*z0 - z1*y0 + w1*x0
        # y2 = -x1*z0 + y1*w0 + z1*x0 + w1*y0
        # z2 = x1*y0 - y1*x0 + z1*w0 + w1*z0

        # base_link_tf.transform.rotation.x = x2
        # base_link_tf.transform.rotation.y = y2
        # base_link_tf.transform.rotation.z = z2
        # base_link_tf.transform.rotation.w = w2

        base_link_tf.header.frame_id = "odom"
        base_link_tf.child_frame_id = "base_link"
        base_link_tf.header.stamp = now

        self.tf_broadcaster.sendTransform(base_link_tf)

        odom_tf.transform.translation.x = 0.0;
        odom_tf.transform.translation.y = 0.0;
        odom_tf.transform.translation.z = 0.0;

        odom_tf.transform.rotation.x = 0.0
        odom_tf.transform.rotation.y = 0.0
        odom_tf.transform.rotation.z = 0.0
        odom_tf.transform.rotation.w = 1.0

        odom_tf.header.frame_id = "map"
        odom_tf.child_frame_id = "odom"
        odom_tf.header.stamp = now
        self.tf_broadcaster.sendTransform(odom_tf)

    def motor_callback(self, request, response):
        self.leftMotor.setVelocity(request.left_speed)
        self.rightMotor.setVelocity(request.right_speed)
        return response

    def cmdVel_callback(self, msg):
        wheelGap = 1  # in meter
        wheelRadius = 0.21  # in meter
        leftSpeed = ((2.0 * msg.linear.x - msg.angular.z * wheelGap) /
                     (2.0 * wheelRadius))
        rightSpeed = ((2.0 * msg.linear.x + msg.angular.z * wheelGap) /
                      (2.0 * wheelRadius))
        leftSpeed = min(self.motorMaxSpeed, max(-self.motorMaxSpeed,
                                                leftSpeed))
        rightSpeed = min(self.motorMaxSpeed, max(-self.motorMaxSpeed,
                                                 rightSpeed))
        self.leftMotor.setVelocity(leftSpeed)
        self.rightMotor.setVelocity(rightSpeed)


def main(args=None):
    print('WEBOTS_ROBOT_NAME:')
    print(os.environ['WEBOTS_ROBOT_NAME'])
    rclpy.init(args=args)

    exampleController = ExampleController(args=args)

    rclpy.spin(exampleController)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
