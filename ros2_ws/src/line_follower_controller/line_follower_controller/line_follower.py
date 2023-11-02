'''
Test script for basic line-following
'''

#!/usr/bin/env python3

import rclpy
import sys
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from irobot_create_msgs.msg import WheelVels
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.msg import Parameter

import socket
import time

robotNamespace = 'Maeve'
CREATE_IP = '192.168.0.100' #we define the IP as static in the router configuration, for convenience
CREATE_PORT = 8883 #this value is set through the iRobot's internal web application and can't be different

class TCP_Server():
    def __init__(self, IP, PORT):
        try:
            self._client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        except:
            print('Failed to create socket')
            sys.exit()
        
        print('socket created')

        self._client.connect((IP, PORT))
        print('Socket connected to ' + CREATE_IP)
    
    def read(self):
        try:
            msg = self._client.recv(1024)
        except socket.error:
            print ('failed to read data')
        return msg
    
    def close(self):
        self._client.detach()
        self._client.close()

class ReaderNode(Node):
    def __init__(self):
        super().__init__('reader_node')
        self._tcp = TCP_Server(CREATE_IP, CREATE_PORT)
        #self.create_timer(0.05, self.timer_callback)
        self._readCt = 0
        self._currentDir = 1 #0=left, 1=straight, 2 = right

        self._wheels_publisher = self.create_publisher(Twist, robotNamespace + '/cmd_vel', 10) 
        self._wheels = Twist()
        self._linear = Vector3() #start by trying linear speed of 0.01
        self._angular = Vector3() #negative angular is a right turn when driving forward. start by trying +/- 0.4

        self._client = self.create_client(SetParameters, robotNamespace + '/motion_control/set_parameters')
        print('node established')

    def set_params(self, mode:str):
        request = SetParameters.Request()
        param = Parameter()
        param.name = "safety_override"
        param.value.type = ParameterType.PARAMETER_STRING
        param.value.string_value = mode
        request.parameters.append(param)

        print('waiting for service to override safeties')

        self._client.wait_for_service()
        print('request sent')
        self._future = self._client.call_async(request)

    def send_speed(self, linear_speed:float, angular_speed:float):
        self._linear.x = linear_speed
        self._linear.y = 0.0
        self._linear.z = 0.0
        self._angular.x = 0.0
        self._angular.y = 0.0
        self._angular.z = angular_speed
        self._wheels.linear = self._linear
        self._wheels.angular = self._angular
        self._wheels_publisher.publish(self._wheels)

        
    def timer_callback(self):
        self.get_data()

    def get_data(self):
        print('getting data' + str(self._readCt))
        self._readCt += 1
        #our microcontroller is programmed to perform averaging of positional values to reduce noise
        #we should not need to perform further averaging here.
        #the value we want is the last in the array

        #value: 0 = robot too far right, turn left, 255 =  robot too far left, turn right, 127, 128 = perfectly in middle
        data = self._tcp.read()
        c = len(data)
        if c == 0:
            return
        v = data[c -1]
        print (v)
        diff = v - 127
        angvel = float(diff) / 127.0 #note that very small numbers are interpreted by the iRobot as too small / it won't turn (essentially it goes straight)
        if (v < 127):
            
            self.send_speed(-0.1, angvel)
            self._currentDir = 2
            print('Right')
            
        else: #elif (v > 127):
            self.send_speed(-0.1, angvel)
            self._currentDir = 0
            print('Left')
            
        # else:
        #     self.send_speed(-0.1, 0.0)
        #     self._currentDir = 1
        #     print('Straight')

    def shutdown(self):
        self._tcp.close()
        print ('closed TCP')
        self.send_speed(0.0, 0.0)
        print ('stopped robot')
        self.set_params('none')
        print('reset safety overrides')


def main(args = None):
    rclpy.init(args=args)
    node = ReaderNode()
    node.set_params('full')
    print('safeties overidden')
    try:
        while(True):
            node.get_data()
            time.sleep(0.025)
        #rclpy.spin(node)
    except KeyboardInterrupt:
        print('Caught keyboard interrupt')
    finally:
        #stop robot and reset safety overrides, close TCP
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        print('Done')

if __name__ == '__main__':
    main()