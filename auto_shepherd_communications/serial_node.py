'''
serial_node.py

Basic Serial device bridge node with generic message reading and 
publishing, along with subscribing and writing

'''

import serial
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class SerialCommNode(Node):
    def __init__(self):
        super.__init__('Serial_Communication_Node')
        
        # list of parameters used by the serial node
        parameters=[
            ('serial_device_path', '/dev/ttyS0'),
            ('baudrate', 115200),
            ('serial_timeout', 1.0),
            ('serial_topic', 1),
            ('publishing_rate', 1.0),
        ]

        self.publishers = self.create_publisher(
            String, 
            self.get_param_float('serial_topic'),
            10 # queue size
            )
        self.timers = self.create_timer(
            self.get_param_float('publishing_rate'), 
            self.timer_callback
            )
        
        # setup serial connection
        self.serial_device = serial.Serial(
            port=self.get_param_str('serial_device_path'),
            baudrate=self.get_param_int('baudrate'),
            timeout=self.get_param_float('serial_timeout')
        )
        self.data_recv = '' # data that is recieved from the serial device 
        self.data_send = '' # data that is sent to the serial device 

    def get_param_float(self, name):
        try:
            return float(self.get_parameter(name).get_parameter_value().double_value)
        except:
            pass
    def get_param_str(self, name):
        try:
            return self.get_parameter(name).get_parameter_value().string_value
        except:
            pass
    def get_param_int(self, name):
        try:
            return self.get_parameter(name).get_parameter_value().integer_value
        except:
            pass
        
    def timer_callback(self):
        msg = String()
        msg.data = self.get_param