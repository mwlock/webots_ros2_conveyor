# BSD 3-Clause License

# Copyright (c) 2023, Matthew Lock

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""ROS2 Conveyor Belt Driver"""

import rclpy
from geometry_msgs.msg import Twist

SPEED = 0.5

class ConveyorBeltDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__robot_name = self.__robot.getName()
        self.__timestep = int(self.__robot.getBasicTimeStep())

        # Get belt_motor
        self.__belt_motor = self.__robot.getDevice('belt_motor')
        self.__belt_motor.setPosition(float('inf'))
        self.__belt_motor.setVelocity(SPEED)

        # ROS interface
        rclpy.init(args=None)
        self.__node = rclpy.create_node(f"{self.__robot.getName()}_driver")
        self.__node.get_logger().info(f"Conveyor Belt Driver for {self.__robot.getName()} initialized")

        # State
        self.__target_twist = Twist()
        self.__target_twist.linear.x = SPEED

        # Belt Motor Subscriber
        self.__node.create_subscription(Twist, f"{self.__robot_name}/cmd_vel", self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist: Twist):
        self.__target_twist = twist
        self.__belt_motor.setVelocity(self.__target_twist.linear.x)        

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)