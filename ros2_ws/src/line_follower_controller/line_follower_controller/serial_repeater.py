'''
This is a simple script to just relay the serial data that the iRobot is getting from the microcontroller (USB serial),
sending over wifi,
and being processed on this computer
'''

#!/usr/bin/env python3

import rclpy
import sys
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import socket
import time

robotNamespace = 'Maeve'
CREATE_IP = '192.168.0.100'
CREATE_PORT = 8883

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
            msg = self._client.recv(1)
        except socket.error:
            print ('failed to read data')
        return msg
    
    def close(self):
        self._client.close()

class ReaderNode(Node):
    def __init__(self):
        super().__init__('reader_node')
        self._tcp = TCP_Server(CREATE_IP, CREATE_PORT)
        self.create_timer(0.05, self.timer_callback)
        
    def timer_callback(self):
        self.get_data()

    def get_data(self):
        print('getting data')
        data = self._tcp.read()
        for d in data:
            print(d)
        


def main(args = None):
    rclpy.init(args=args)
    node = ReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Caught keyboard interrupt')
    finally:
        print('Done')
        #node.destroy_node
        rclpy.shutdown()

if __name__ == '__main__':
    main()