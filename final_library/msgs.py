import struct

MAVLINK_MSG_ID_HEARTBEAT = 0
MAVLINK_MSG_ID_COMMAND_ACK = 77
MAVLINK_MSG_ID_COMMAND_LONG = 76
PROTOCOL_MARKER_V1 = 0xFE


class MAVLink_header(object):
    """MAVLink message header"""

    def __init__(self, msgId: int, incompat_flags: int = 0, compat_flags: int = 0, mlen: int = 0, seq: int = 0, srcSystem: int = 1, srcComponent: int = 1) -> None:
        self.mlen = mlen
        self.seq = seq
        self.srcSystem = srcSystem
        self.srcComponent = srcComponent
        self.msgId = msgId
        self.incompat_flags = incompat_flags
        self.compat_flags = compat_flags

    def pack(self) -> bytes:
        fields = [self.mlen, self.seq, self.srcSystem, self.srcComponent, self.msgId]
        for field in fields:
            if not (0 <= field <= 255):
                raise ValueError(f"Field value {field} is out of range (0-255)")
        return struct.pack("<BBBBBB", PROTOCOL_MARKER_V1, self.mlen, self.seq, self.srcSystem, self.srcComponent, self.msgId)


class x25crc(object):
    """CRC-16/MCRF4XX - based on checksum.h from mavlink library"""

    def __init__(self, buf = None) -> None:
        self.crc = 0xFFFF
        if buf is not None:
            self.accumulate(buf)

    def accumulate(self, buf) -> None:
        """add in some more bytes (it also accepts python2 strings)"""
        if type(buf) is str:
            buf = bytearray(buf)

        accum = self.crc
        for b in buf:
            tmp = b ^ (accum & 0xFF)
            tmp = (tmp ^ (tmp << 4)) & 0xFF
            accum = (accum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        self.crc = accum


class MAVLink_message(object):
    """base MAVLink message class"""

    id = 0
    msgname = ""
    fieldnames = []
    ordered_fieldnames = []
    fieldtypes = []
    fielddisplays_by_name = {}
    fieldenums_by_name = {}
    fieldunits_by_name = {}
    native_format = bytearray(b"")
    orders = []
    lengths = []
    array_lengths = []
    crc_extra = 0
    unpacker = None # struct.pack("")
    instance_field = None
    instance_offset = -1

    def __init__(self, msgId: int, name: str) -> None:
        self._header = MAVLink_header(msgId)
        self._payload = None
        self._msgbuf = bytearray(b"")
        self._crc = None
        self._fieldnames = []
        self._type = name
        self._signed = False
        self._link_id = None
        self._instances = None
        self._instance_field = None

    def format_attr(self, field: str):
        """override field getter"""
        raw_attr = getattr(self, field)
        if isinstance(raw_attr, bytes):
            return raw_attr.decode(errors="backslashreplace").rstrip("\x00")
        return raw_attr

    def get_msgbuf(self) -> bytearray:
        return self._msgbuf

    def get_header(self) -> MAVLink_header:
        return self._header

    def get_payload(self):
        return self._payload

    def get_crc(self):
        return self._crc

    def get_fieldnames(self):
        return self._fieldnames

    def get_type(self) -> str:
        return self._type

    def get_msgId(self) -> int:
        return self._header.msgId

    def get_srcSystem(self) -> int:
        return self._header.srcSystem

    def get_srcComponent(self) -> int:
        return self._header.srcComponent

    def get_seq(self) -> int:
        return self._header.seq

    def get_signed(self) -> bool:
        return self._signed

    def get_link_id(self):
        return self._link_id

    def __str__(self) -> str:
        ret = "%s {" % self._type
        for a in self._fieldnames:
            v = self.format_attr(a)
            ret += "%s : %s, " % (a, v)
        ret = ret[0:-2] + "}"
        return ret

    def __ne__(self, other: object) -> bool:
        return not self.__eq__(other)

    def __eq__(self, other: object) -> bool:
        if other is None:
            return False

        if not isinstance(other, MAVLink_message):
            return False

        if self.get_type() != other.get_type():
            return False

        if self.get_crc() != other.get_crc():
            return False

        if self.get_seq() != other.get_seq():
            return False

        if self.get_srcSystem() != other.get_srcSystem():
            return False

        if self.get_srcComponent() != other.get_srcComponent():
            return False

        for a in self._fieldnames:
            if self.format_attr(a) != other.format_attr(a):
                return False

        return True

    def to_dict(self):
        d = {}
        d["mavpackettype"] = self._type
        for a in self._fieldnames:
            d[a] = self.format_attr(a)
        return d

    def _pack(self, crc_extra: int, payload: bytes, force_mavlink1: bool = False) -> bytes:
        plen = len(payload)
        nullbyte = 0
        while plen > 1 and payload[plen - 1] == nullbyte:
            plen -= 1
        self._payload = payload[:plen]
        incompat_flags = 0
        self._header = MAVLink_header(
            self._header.msgId,
            incompat_flags=incompat_flags,
            compat_flags=0,
            mlen=len(self._payload),
            seq=self._header.seq,
            srcSystem=self._header.srcSystem,
            srcComponent=self._header.srcComponent,
        )
        self._msgbuf = self._header.pack()
        self._msgbuf += self._payload
        crc = x25crc(self._msgbuf[1:])
        if True:
            # we are using CRC extra
            crc.accumulate(struct.pack("B", crc_extra))
        self._crc = crc.crc
        self._msgbuf += struct.pack("<H", self._crc)
        return bytes(self._msgbuf)

    def __getitem__(self, key: str) -> str:
        """support indexing, allowing for multi-instance sensors in one message"""
        if self._instances is None:
            raise IndexError()
        if key not in self._instances:
            raise IndexError()
        return self._instances[key]


class MAVLink_heartbeat_message(MAVLink_message):
    id = MAVLINK_MSG_ID_HEARTBEAT
    msgname = "HEARTBEAT"
    fieldnames = ["type", "autopilot", "base_mode", "custom_mode", "system_status", "mavlink_version"]
    ordered_fieldnames = ["custom_mode", "type", "autopilot", "base_mode", "system_status", "mavlink_version"]
    fieldtypes = ["uint8_t", "uint8_t", "uint8_t", "uint32_t", "uint8_t", "uint8_t"]
    fielddisplays_by_name = {"base_mode": "bitmask"}
    fieldenums_by_name = {"type": "MAV_TYPE", "autopilot": "MAV_AUTOPILOT", "base_mode": "MAV_MODE_FLAG", "system_status": "MAV_STATE"}
    fieldunits_by_name = {}
    native_format = bytearray(b"<IBBBBB")
    orders = [1, 2, 3, 0, 4, 5]
    lengths = [1, 1, 1, 1, 1, 1]
    array_lengths = [0, 0, 0, 0, 0, 0]
    crc_extra = 50
    instance_field = None
    instance_offset = -1

    def __init__(self, type: int, autopilot: int, base_mode: int, custom_mode: int, system_status: int, mavlink_version: int):
        MAVLink_message.__init__(self, MAVLink_heartbeat_message.id, MAVLink_heartbeat_message.msgname)
        self._fieldnames = MAVLink_heartbeat_message.fieldnames
        self._instance_field = None
        self._instance_offset = -1
        self.type = type
        self.autopilot = autopilot
        self.base_mode = base_mode
        self.custom_mode = custom_mode
        self.system_status = system_status
        self.mavlink_version = mavlink_version

    def get_message(self):
        return self._pack(self.crc_extra, struct.pack("<IBBBBB", self.custom_mode, self.type, self.autopilot, self.base_mode, self.system_status, self.mavlink_version), force_mavlink1=False)


class MAVLink_command_ack_message(MAVLink_message):
    id = MAVLINK_MSG_ID_COMMAND_ACK
    msgname = "COMMAND_ACK"
    fieldnames = ["command", "result"]
    ordered_fieldnames = ["command", "result"]
    fieldtypes = ["uint16_t", "uint8_t"]
    fielddisplays_by_name = {}
    fieldenums_by_name = {"command": "MAV_CMD", "result": "MAV_RESULT"}
    fieldunits_by_name = {}
    native_format = bytearray(b"<HB")
    orders = [0, 1]
    lengths = [1, 1]
    array_lengths = [0, 0]
    crc_extra = 143
    unpacker = struct
    instance_field = None
    instance_offset = -1

    def __init__(self, command: int, result: int):
        MAVLink_message.__init__(self, MAVLink_command_ack_message.id, MAVLink_command_ack_message.msgname)
        self._fieldnames = MAVLink_command_ack_message.fieldnames
        self._instance_field = MAVLink_command_ack_message.instance_field
        self._instance_offset = MAVLink_command_ack_message.instance_offset
        self.command = command
        self.result = result

    def pack(self, force_mavlink1: bool = False) -> bytes:
        return self._pack(self.crc_extra, self.unpacker.pack("<HB", self.command, self.result), force_mavlink1=force_mavlink1)


class MAVLink_command_long_message(MAVLink_message):
    id = MAVLINK_MSG_ID_COMMAND_LONG
    msgname = "COMMAND_LONG"
    fieldnames = ["target_system", "target_component", "command", "confirmation", "param1", "param2", "param3", "param4", "param5", "param6", "param7"]
    ordered_fieldnames = ["param1", "param2", "param3", "param4", "param5", "param6", "param7", "command", "target_system", "target_component", "confirmation"]
    fieldtypes = ["uint8_t", "uint8_t", "uint16_t", "uint8_t", "float", "float", "float", "float", "float", "float", "float"]
    fielddisplays_by_name = {}
    fieldenums_by_name = {"command": "MAV_CMD"}
    fieldunits_by_name = {}
    native_format = bytearray(b"<fffffffHBBB")
    orders = [8, 9, 7, 10, 0, 1, 2, 3, 4, 5, 6]
    lengths = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
    array_lengths = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    crc_extra = 152
    unpacker = struct
    instance_field = None
    instance_offset = -1

    def __init__(self, target_system: int, target_component: int, command: int, confirmation: int, param1: float, param2: float, param3: float, param4: float, param5: float, param6: float, param7: float):
        MAVLink_message.__init__(self, MAVLink_command_long_message.id, MAVLink_command_long_message.msgname)
        self._fieldnames = MAVLink_command_long_message.fieldnames
        self._instance_field = MAVLink_command_long_message.instance_field
        self._instance_offset = MAVLink_command_long_message.instance_offset
        self.target_system = target_system
        self.target_component = target_component
        self.command = command
        self.confirmation = confirmation
        self.param1 = param1
        self.param2 = param2
        self.param3 = param3
        self.param4 = param4
        self.param5 = param5
        self.param6 = param6
        self.param7 = param7

    def pack(self, force_mavlink1: bool = False) -> bytes:
        return self._pack(self.crc_extra, self.unpacker.pack("<fffffffHBBB", self.param1, self.param2, self.param3, self.param4, self.param5, self.param6, self.param7, self.command, self.target_system, self.target_component, self.confirmation), force_mavlink1=force_mavlink1)
