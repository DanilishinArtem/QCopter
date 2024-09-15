from msgs import MAVLink_heartbeat_message, MAVLink_command_ack_message, MAVLink_command_long_message, x25crc, MAVLink_header
import struct

# mavlink message ids --------------------->
MAVLINK_MSG_ID_BAD_DATA = -1
MAVLINK_MSG_ID_UNKNOWN = -2
MAVLINK_MSG_ID_SYS_STATUS = 1
MAVLINK_MSG_ID_SYSTEM_TIME = 2
MAVLINK_MSG_ID_PING = 4
MAVLINK_MSG_ID_COMMAND_LONG = 76
MAVLINK_MSG_ID_COMMAND_ACK = 77
MAVLINK_MSG_ID_HEARTBEAT = 0

# mavlink message map --------------------->
mavlink_map = {
   MAVLINK_MSG_ID_COMMAND_LONG: MAVLink_command_long_message,
    MAVLINK_MSG_ID_COMMAND_ACK: MAVLink_command_ack_message,
   MAVLINK_MSG_ID_HEARTBEAT: MAVLink_heartbeat_message,
}


HEADER_LEN_V1 = 6
MAVLINK_SIGNATURE_BLOCK_LEN = 13


class decoder:
    def __init__(self):
        self.buf = bytearray()
        self.buf_index = 0
        self.total_bytes_received = 0
        self.total_packets_received = 0
        self.expected_length = HEADER_LEN_V1 + 2
        self.unpacker = struct

    def buf_len(self) -> int:
        return len(self.buf) - self.buf_index

    def parse_char(self, c):
        self.buf.extend(c)
        self.total_bytes_received += len(c)
        m = self.__parse_char_legacy()
        if m is not None:
            self.total_packets_received += 1
        else:
            if self.buf_len() == 0 and self.buf_index != 0:
                self.buf = bytearray()
                self.buf_index = 0
        return m

    def __parse_char_legacy(self):
        incompat_flags = 0
        header_len = HEADER_LEN_V1
        m = None
        self.have_prefix_error = False
        if self.buf_len() >= 3:
            sbuf = self.buf[self.buf_index : 3 + self.buf_index]
            (magic, self.expected_length, incompat_flags) = self.unpacker.unpack("BBB",sbuf)
            self.expected_length += header_len + 2
        if self.expected_length >= (header_len + 2) and self.buf_len() >= self.expected_length:
            mbuf = self.buf[self.buf_index : self.buf_index + self.expected_length]
            self.buf_index += self.expected_length
            self.expected_length = header_len + 2
            m = self.decode(mbuf)
            return m
        return None

    def decode(self, msgbuf):
        headerlen = 6
        try:
            magic, mlen, seq, srcSystem, srcComponent, msgId = self.unpacker.unpack("<cBBBBB", msgbuf[:headerlen])
            incompat_flags = 0
            compat_flags = 0
        except (ValueError, OSError) as emsg:
            raise ValueError("Unable to unpack MAVLink header: %s" % emsg)
        mapkey = msgId
        signature_len = 0

        if mlen != len(msgbuf) - (headerlen + 2 + signature_len):
            raise ValueError("invalid MAVLink message length. Got %u expected %u, msgId=%u headerlen=%u" % (len(msgbuf) - (headerlen + 2 + signature_len), mlen, msgId, headerlen))

        if mapkey not in mavlink_map:
            ValueError('unknown messege')

        msgtype = mavlink_map[mapkey]
        # return msgtype
        order_map = msgtype.orders
        len_map = msgtype.lengths
        crc_extra = msgtype.crc_extra

        # decode the checksum
        try:
            (crc,) = self.unpacker.unpack("<H", msgbuf[-(2 + signature_len) :][:2])
        except (ValueError, OSError) as emsg:
            raise ValueError("Unable to unpack MAVLink CRC: %s" % emsg)
        crcbuf = msgbuf[1 : -(2 + signature_len)]
        if True:
            # using CRC extra
            crcbuf.append(crc_extra)
        crc2 = x25crc(crcbuf)
        if crc != crc2.crc:
            raise ValueError("invalid MAVLink CRC in msgID %u 0x%04x should be 0x%04x" % (msgId, crc, crc2.crc))

        sig_ok = False

        csize = struct.calcsize(msgtype.native_format.decode('utf-8'))
        mbuf = msgbuf[headerlen : -(2 + signature_len)]
        if len(mbuf) < csize:
            # zero pad to give right size
            mbuf.extend([0] * (csize - len(mbuf)))
        if len(mbuf) < csize:
            raise ValueError("Bad message of type %s length %u needs %s" % (msgtype, len(mbuf), csize))
        mbuf = mbuf[:csize]
        try:
            t = msgtype.unpacker.unpack(msgtype.native_format.decode('utf-8'),mbuf)
        except (ValueError, OSError) as emsg:
            raise ValueError("Unable to unpack MAVLink payload type=%s payloadLength=%u: %s" % (msgtype, len(mbuf), emsg))

        tlist = list(t)
        # handle sorted fields
        if True:
            if sum(len_map) == len(len_map):
                # message has no arrays in it
                for i in range(0, len(tlist)):
                    tlist[i] = t[order_map[i]]
            else:
                # message has some arrays
                tlist = []
                for i in range(0, len(order_map)):
                    order = order_map[i]
                    L = len_map[order]
                    tip = sum(len_map[:order])
                    field = t[tip]
                    if L == 1 or isinstance(field, bytes):
                        tlist.append(field)
                    else:
                        tlist.append(list(t[tip : (tip + L)]))

        # terminate any strings
        for i, elem in enumerate(tlist):
            if isinstance(elem, bytes):
                tlist[i] = elem.rstrip(b"\x00")
        try:
            m = msgtype(*tlist)  # type: ignore
        except Exception as emsg:
            raise ValueError("Unable to instantiate MAVLink message of type %s : %s" % (msgtype, emsg))
        m._signed = sig_ok
        if m._signed:
            m._link_id = msgbuf[-13]
        m._msgbuf = msgbuf
        m._payload = msgbuf[6 : -(2 + signature_len)]
        m._crc = crc
        m._header = MAVLink_header(msgId, incompat_flags, compat_flags, mlen, seq, srcSystem, srcComponent)
        return m