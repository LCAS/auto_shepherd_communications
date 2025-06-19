'''
serial_node.py

Basic Serial device bridge node with generic message reading and 
publishing, along with subscribing and writing

'''

import serial
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import NavSatFix, Imu

class SerialCommNode(Node):
    def __init__(self):
        super().__init__('Serial_Communication_Node')
        
        # list of parameters used by the serial node
        parameters=[
            ('serial_device_path', '/dev/ttyUSB1'),
            ('baudrate', 115200),
            ('serial_timeout', 1.0),
            ('serial_topic', 1),
            ('publishing_rate', 1.0),
        ]

        self.declare_parameter('serial_device_path', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('serial_timeout', 1.0)
        self.declare_parameter('publishing_rate', 1.0)
        self.declare_parameter('serial_topic', 1)
        rate = self.get_parameter('publishing_rate').get_parameter_value().double_value
        # Subscriptions
        self.create_subscription(PoseStamped, '/goal_pose', self.send_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.send_callback, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.send_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.send_callback, 10)        
        # Read parameter values
        device_path = self.get_parameter('serial_device_path').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        timeout = self.get_parameter('serial_timeout').get_parameter_value().double_value

        # Use them to configure the serial port
        self.serial_device = serial.Serial(
            port=device_path,
            baudrate=baudrate,
            timeout=timeout
        )
        self.data_recv = '' # data that is recieved from the serial device 
        self.data_send = '' # data that is sent to the serial device 
        self.START_MARKER = b'\xAA\x55'
        self.Emptypacket = b'\xAA\x55\x00\x00\x00\x00'
        self.packet_to_send = b''
        self.LENGTH_FIELD_SIZE = 4
        self.packet_to_send = b''
        self.MSG_TYPE_IDS = MSG_TYPE_IDS = {
            'PoseStamped': b'\x00',
            'Twist': b'\x01',
            'NavSatFix': b'\x02',
            'Imu': b'\x03',
        }
        # === Message type ID map ===
        self.ID_TO_INFO = {
            b'\x00': ('/goal_pose3', PoseStamped),
            b'\x01': ('/cmd_vel3', Twist),
            b'\x02': ('/gps/fix3', NavSatFix),
            b'\x03': ('/imu/data3', Imu),
        }

        self.my_publishers = {}
        for type_id, (topic, msg_class) in self.ID_TO_INFO.items():
            #print(type_id)
            self.my_publishers[type_id] = self.create_publisher(msg_class, topic, 10)
        
        self.RecieverLoop()

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
    
    def read_exact(self, size):
        data = b''
        while len(data) < size:
            chunk = self.serial_device.read(size - len(data))
            data += chunk
        return data
    
    def sync_to_start_marker(self):
        while True:
            byte = self.read_exact(1)
            #print(f"byte1 = {byte}")
            if byte == self.START_MARKER[:1]:
                second = self.read_exact(1)
                #print(f"byte2 = {second}")
                if second == self.START_MARKER[1:]:
                    return
    
    def send_callback(self,msg):
        msg_type = msg.__class__.__name__
        type_id = self.MSG_TYPE_IDS.get(msg_type)
        type_id_bytes=type_id.to_bytes(1, 'big') 
        # Fully serialize the ROS 2 message to bytes
        serialized = serialize_message(msg)

        # Add length prefix for framing
        length_bytes = len(serialized).to_bytes(self.LENGTH_FIELD_SIZE, byteorder='big')
        self.packet_to_send = self.START_MARKER + length_bytes + type_id_bytes + serialized

    def RecieverLoop(self):
        while True:
            self.sync_to_start_marker()
            length_bytes = self.read_exact(4)
            length = int.from_bytes(length_bytes, byteorder='big') 
            #print(f"length: {length} {length_bytes}")
            if length != 0:
                type_id = self.read_exact(1)
                #print(f"typeID: {type_id}")
                msg_class = self.ID_TO_INFO.get(type_id)
                if not msg_class:
                    print(f"Unknown type_id: {type_id}")
                else:
                    payload_length = int.from_bytes(length_bytes, 'big')
                    serialized_data = self.read_exact(payload_length)
                    #print(f"Data: {serialized_data}")
                    #print(f"msg class - {msg_class}")
                    msg = deserialize_message(serialized_data, msg_class[1])
                    #print(msg)
                    #print(f"typeID - {type_id}")
                    self.my_publishers[type_id].publish(msg)
            if self.packet_to_send == b'':
                self.packet_to_send = self.Emptypacket
            self.serial_device.write(self.packet_to_send)
            #print(f"sending = {self.packet_to_send}")
            self.packet_to_send = b''

def main(args=None):
    rclpy.init(args=args)
    node = SerialCommNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()