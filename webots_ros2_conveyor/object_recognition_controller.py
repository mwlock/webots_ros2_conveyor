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

"""ROS2 Object Recognition Controller"""


import rclpy
from sensor_msgs.msg import PointCloud2

SPEED = 0.5

class ObjectRecognitionDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__robot_name = self.__robot.getName()
        self.__timestep = int(self.__robot.getBasicTimeStep())

        # Get camera and range finder devices
        self.__camera = self.__robot.getDevice('camera')
        self.__range_finder = self.__robot.getDevice('range-finder')

        # Enable camera and range finder
        self.__camera.enable(self.__timestep)
        self.__camera.recognitionEnable(self.__timestep)
        self.__range_finder.enable(self.__timestep)

        # ROS interface
        rclpy.init(args=None)
        self.__node = rclpy.create_node(f"{self.__robot.getName()}_driver")
        self.__node.get_logger().info(f"Recognition Driver for {self.__robot.getName()} initialized")

        # Create publishers
        self.__rgbd_publisher = self.__node.create_publisher(PointCloud2, f"{self.__robot_name}/camera/rgbd_image", 1)

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        number_of_objects = self.__camera.getRecognitionNumberOfObjects()
        objects = self.__camera.getRecognitionObjects()

        for object in objects:
            print(object.getModel())