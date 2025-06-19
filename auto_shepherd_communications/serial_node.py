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
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import NavSatFix, Imu

class SerialCommNode(Node):
    def __init__(self):
        super().__init__('Serial_Communication_Node')
        
        # list of parameters used by the serial node

        self.declare_parameter('serial_device_path', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('serial_timeout', 1.0)
        self.declare_parameter('publishing_rate', 0.1)
        self.declare_parameter('serial_topic', 1)
        rate = self.get_parameter('publishing_rate').get_parameter_value().double_value
        # Subscriptions
        self.create_subscription(PoseStamped, '/goal_pose', self.send_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.send_callback, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.send_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.send_callback, 10)

        self.timer = self.create_timer(1.0 / rate, self.timer_callback)
        
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
        self.MSG_TYPE_IDS = MSG_TYPE_IDS = {
            'PoseStamped': b'\x00',
            'Twist': b'\x01',
            'NavSatFix': b'\x02',
            'Imu': b'\x03',
        }
        # === Message type ID map ===
        self.ID_TO_INFO = {
            b'\x00': ('/goal_pose2', PoseStamped),
            b'\x01': ('/cmd_vel2', Twist),
            b'\x02': ('/gps/fix2', NavSatFix),
            b'\x03': ('/imu/data2', Imu),
        }
        self.my_publishers = {}
        for type_id, (topic, msg_class) in self.ID_TO_INFO.items():
            self.my_publishers[type_id] = self.create_publisher(msg_class, topic, 10)
    
    def read_exact(self, size):
        data = b''
        while len(data) < size:
            chunk = self.serial_device.read(size - len(data))
            if not chunk:
                raise RuntimeError("Serial read timeout or disconnection.")
            data += chunk
        return data
    
    def send_callback(self, msg):
        msg_type = msg.__class__.__name__
        type_id = self.MSG_TYPE_IDS.get(msg_type)
        # Fully serialize the ROS 2 message to bytes
        serialized = serialize_message(msg)

        msg = deserialize_message(serialized, PoseStamped)
        print(msg)
        #serializedbytes = serialized.tobytes(serialized, byteorder='big')
        # Add length prefix for framing
        length_bytes = len(serialized).to_bytes(self.LENGTH_FIELD_SIZE, byteorder='big')
        #print(f"length: {length_bytes}")
        #print(f"type_id_bytes: {type_id}")

        my_string = "Hello"
        my_bytes = my_string.encode('utf-8')  # default encoding is UTF-8
        print(f"Serialized = {serialized}")
        self.packet_to_send = self.START_MARKER + length_bytes + type_id + serialized
            
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
    
    def sync_to_start_marker(self):
        #TODO add timeout to this
        while True:
            byte = self.read_exact(1)
            if byte == self.START_MARKER[:1]:
                second = self.read_exact(1)
                if second == self.START_MARKER[1:]:
                    return

    def timer_callback(self):
        if self.packet_to_send == b'':
            self.packet_to_send = self.Emptypacket
        self.serial_device.write(self.packet_to_send)
        print(f"sending = {self.packet_to_send}")
        self.packet_to_send = b''
        

def main(args=None):
    rclpy.init(args=args)
    node = SerialCommNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()