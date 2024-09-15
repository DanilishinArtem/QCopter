import socket
import struct
import network
import time
from lightMavlink import MAVLink, MAVLink_heartbeat_message


class Communication:
    def __init__(self, essid: str, password: str, mav_type: int, mav_autopilot: int, mav_mode: int, mav_state: int, version: int):
        self.MAV_TYPE = mav_type  # Quadrotor
        self.MAV_AUTOPILOT = mav_autopilot  # Generic autopilot
        self.MAV_MODE = mav_mode  # Preflight
        self.MAV_STATE = mav_state  # Standby
        self.MAVLINK_VERSION = version  # MAVLink 1.0
        self.CRC_EXTRA_HEARTBEAT = 50  # CRC_EXTRA for heartbeat message
        self.CRC_EXTRA_COMMAND_LONG = 20  # CRC_EXTRA for COMMAND_LONG message
        self.CRC_EXTRA_COMMAND_ACK = 143
        self.create_AP(essid=essid, password=password)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.mav = MAVLink(srcSystem=255, srcComponent=0, use_native=False)

    def create_AP(self, essid: str, password: str):
        ap = network.WLAN(network.AP_IF)
        ap.active(True)
        ap.config(essid=essid, password=password, authmode=network.AUTH_WPA2_PSK)
        while not ap.active():
            pass
        print('Access point is ready')
        print('Configuration of the network: {}'.format(ap.ifconfig()))

    def create_heartbeat(self):
        return MAVLink_heartbeat_message(self.MAV_TYPE, self.MAV_AUTOPILOT, self.MAV_MODE, 0, self.MAV_STATE, self.MAVLINK_VERSION)

    def send_heartbeat(self, udp_ip="192.168.4.255", udp_port=14550):
        heartbeat_message = self.create_heartbeat()
        # Send the heartbeat message via UDP
        self.sock.sendto(heartbeat_message, (udp_ip, udp_port))
        print("Heartbeat message sent:", heartbeat_message)
        
    def receive_message(self):
        data, addr = self.sock.recvfrom(1024)  # Получение данных из сокета
        print(f"Received message from {addr}: {data}")


def main():
    # Настраиваем параметры для создания точки доступа и отправки heartbeat сообщений
    essid = "eps32"  # Имя точки доступа
    password = "7779777119"  # Пароль для подключения к точке доступа
    mav_type = 2  # Тип MAV (2 - квадрокоптер)
    mav_autopilot = 1  # Автопилот (1 - generic)
    mav_mode = 0  # Режим MAV (0 - preflight)
    mav_state = 0  # Состояние MAV (0 - standby)
    mavlink_version = 1  # Версия MAVLink
    comm = Communication(essid, password, mav_type, mav_autopilot, mav_mode, mav_state, mavlink_version)
    while True:
        comm.send_heartbeat()
        time.sleep(1)
        try:
            comm.receive_message()
        except OSError:
            pass

if __name__ == "__main__":
    main()