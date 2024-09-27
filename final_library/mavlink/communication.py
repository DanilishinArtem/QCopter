import socket
from machine import Pin, I2C
from mpu9250 import MPU9250
import network
from msgs import (
    MAVLink_heartbeat_message,
    MAVLink_attitude_message,
)
from decoder import decoder



i2c = I2C(0, scl=Pin(8), sda=Pin(7))
from filter import Fusion

class Communication:
    def __init__(self, essid: str, password: str):
        self.essid = essid
        self.password = password
        self.MAV_TYPE = 2
        self.MAV_AUTOPILOT = 1
        self.MAV_MODE = 0
        self.MAV_STATE = 0
        self.MAVLINK_VERSION = 1
        self.CRC_EXTRA_HEARTBEAT = 50
        self.CRC_EXTRA_COMMAND_LONG = 20
        self.CRC_EXTRA_COMMAND_ACK = 143
        self.create_AP()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.msg = None
        self.dec = decoder()
        self.imu = MPU9250(i2c)
        self.filter = Fusion()

    def create_AP(self):
        ap = network.WLAN(network.AP_IF)
        ap.active(True)
        ap.config(essid=self.essid, password=self.password, authmode=network.AUTH_WPA2_PSK)
        while not ap.active():
            pass
        print('Access point is ready')
        print('Configuration of the network: {}'.format(ap.ifconfig()))

    def create_heartbeat(self, system_id, component_id):
        self.msg = MAVLink_heartbeat_message(type=self.MAV_TYPE, autopilot=self.MAV_AUTOPILOT, base_mode=self.MAV_MODE, custom_mode=self.MAV_MODE, system_status=self.MAV_STATE, mavlink_version=self.MAVLINK_VERSION)

    def send_heartbeat(self, udp_ip="192.168.4.255", udp_port=14550):
        heartbeat_message = self.msg.pack()
        try:
            self.sock.sendto(heartbeat_message, (udp_ip, udp_port))
        except:
            pass
        # print("Heartbeat message sent:", heartbeat_message)
        
    def receive_message(self):
        data, addr = self.sock.recvfrom(1024)
        msg_ = ''
        for byte in data:
            msg = self.dec.parse_char(bytes([byte]))
            if msg:
                if msg.get_type() == "MANUAL_CONTROL":
                    # print(f"x: {msg.x}, y: {msg.y}, z: {msg.z}, r: {msg.r}")
                    pass

    def sendAccelGyroMag(self, udp_ip="192.168.4.255", udp_port=14550):
        self.filter.update(self.imu.accel.xyz, self.imu.gyro.xyz, self.imu.mag.xyz)
        roll, pitch, heading = map(lambda x: x / 180 * 3.1415, [self.filter.roll, self.filter.pitch, self.filter.heading])
        ryp = MAVLink_attitude_message(0, roll, pitch, heading, 0, 0, 0).pack()
        try:
            self.sock.sendto(ryp, (udp_ip, udp_port))
        except:
            pass


