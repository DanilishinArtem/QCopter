import socket
import struct
import time
import network



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

    def create_AP(self, essid: str, password: str):
        ap = network.WLAN(network.AP_IF)
        ap.active(True)
        ap.config(essid=essid, password=password, authmode=network.AUTH_WPA2_PSK)
        while not ap.active():
            pass
        print('Access point is ready')
        print('Configuration of the network: {}'.format(ap.ifconfig()))

    def crc_accumulate(self, data, crcAccum):
        tmp = data ^ (crcAccum & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crcAccum = ((crcAccum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
        return crcAccum

    def crc_calculate(self, buffer):
        crc = 0xFFFF  # Initial value
        for byte in buffer:
            crc = self.crc_accumulate(byte, crc)
        return crc

    def create_heartbeat(self, system_id, component_id):
        # Heartbeat payload
        payload = struct.pack('<IBBBBB',
            0,  # custom_mode
            self.MAV_TYPE,  # mav_type
            self.MAV_AUTOPILOT,  # autopilot
            self.MAV_MODE,  # base_mode
            self.MAV_STATE,  # system_status
            self.MAVLINK_VERSION  # mavlink_version
        )
        # MAVLink header
        header = struct.pack('<BBBBBB',
            0xFE,  # Start byte (MAVLink 1.0)
            len(payload),  # Payload length
            0,  # Sequence number (can be incremented each message)
            system_id,  # System ID
            component_id,  # Component ID
            0  # Message ID for heartbeat
        )
        message = header + payload
        crc = self.crc_calculate(message[1:])
        crc = self.crc_accumulate(self.CRC_EXTRA_HEARTBEAT, crc)
        full_message = message + struct.pack('<H', crc)
        return full_message

    def send_heartbeat(self, udp_ip="192.168.4.255", udp_port=14550):
        system_id = 1
        component_id = 1
        # Create and serialize the heartbeat message
        heartbeat_message = self.create_heartbeat(system_id, component_id)
        # Send the heartbeat message via UDP
        self.sock.sendto(heartbeat_message, (udp_ip, udp_port))
        print("Heartbeat message sent:", heartbeat_message)

    def send_command_ack(self, command, result, udp_ip="192.168.4.255", udp_port=14550):
        system_id = 1
        component_id = 1
        payload = struct.pack('<HHBB',
            command,
            result,
            system_id,
            component_id
        )
        header = struct.pack('<BBBBBB',
            0xFE,
            len(payload),
            0,
            system_id,
            component_id,
            77  # COMMAND_ACK message ID
        )
        message = header + payload
        crc = self.crc_calculate(message[1:])
        crc = self.crc_accumulate(self.CRC_EXTRA_COMMAND_ACK, crc)
        full_message = message + struct.pack('<H', crc)
        self.sock.sendto(full_message, (udp_ip, udp_port))


    def send_command(self, command, param1, param2, param3, param4, param5, param6, param7, udp_ip="192.168.4.255", udp_port=14550):
        system_id = 1
        component_id = 1
        payload = struct.pack('<HHfffffff',
            command,
            0,  # Command result (status) - 0 means no result
            param1,
            param2,
            param3,
            param4,
            param5,
            param6,
            param7
        )
        header = struct.pack('<BBBBBB',
            0xFE,      # Start byte
            len(payload),  # Payload length
            0,           # Sequence number
            system_id,   # System ID
            component_id, # Component ID
            76            # COMMAND_LONG message ID
        )
        message = header + payload
        crc = self.crc_calculate(message[1:])
        crc = self.crc_accumulate(self.CRC_EXTRA_COMMAND_LONG, crc)
        full_message = message + struct.pack('<H', crc)
        self.sock.sendto(full_message, (udp_ip, udp_port))
        print("Command sent:", full_message)



if __name__ == "__main__":
    com = Communication('eps32', '7779777119', 2, 0, 0, 3, 3)
    com.send_heartbeat()