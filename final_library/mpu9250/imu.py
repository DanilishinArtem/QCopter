from utime import sleep_ms
from machine import I2C
from vector3d import Vector3d


class MPUException(OSError):
    pass


def bytes_toint(msb, lsb):
    if not msb & 0x80:
        return msb << 8 | lsb  # +ve
    return -(((msb ^ 255) << 8) | (lsb ^ 255) + 1)


class MPU6050(object):
    _I2Cerror = "I2C failure when communicating with IMU"
    _mpu_addr = (104, 105)
    _chip_id = 104

    def __init__(self, side_str, device_addr=None, transposition=(0, 1, 2), scaling=(1, 1, 1)):

        self._accel = Vector3d(transposition, scaling, self._accel_callback)
        self._gyro = Vector3d(transposition, scaling, self._gyro_callback)
        self.buf1 = bytearray(1)
        self.buf2 = bytearray(2)
        self.buf3 = bytearray(3)
        self.buf6 = bytearray(6)

        sleep_ms(200)
        if isinstance(side_str, str):
            self._mpu_i2c = I2C(side_str)
        elif hasattr(side_str, "readfrom"):
            self._mpu_i2c = side_str
        else:
            raise ValueError("Invalid I2C instance")

        if device_addr is None:
            devices = set(self._mpu_i2c.scan())
            mpus = devices.intersection(set(self._mpu_addr))
            number_of_mpus = len(mpus)
            if number_of_mpus == 0:
                raise MPUException("No MPU's detected")
            elif number_of_mpus == 1:
                self.mpu_addr = mpus.pop()
            else:
                raise ValueError("Two MPU's detected: must specify a device address")
        else:
            if device_addr not in (0, 1):
                raise ValueError("Device address must be 0 or 1")
            self.mpu_addr = self._mpu_addr[device_addr]

        self.chip_id
        self.wake()
        self.passthrough = True
        self.accel_range = 0
        self.gyro_range = 0

    # read from device
    def _read(
        self, buf, memaddr, addr
    ):
        self._mpu_i2c.readfrom_mem_into(addr, memaddr, buf)

    # write to device
    def _write(self, data, memaddr, addr):
        self.buf1[0] = data
        self._mpu_i2c.writeto_mem(addr, memaddr, self.buf1)

    # wake
    def wake(self):
        try:
            self._write(0x01, 0x6B, self.mpu_addr)  # Use best clock source
        except OSError:
            raise MPUException(self._I2Cerror)
        return "awake"

    # mode
    def sleep(self):
        try:
            self._write(0x40, 0x6B, self.mpu_addr)
        except OSError:
            raise MPUException(self._I2Cerror)
        return "asleep"

    # chip_id
    @property
    def chip_id(self):
        try:
            self._read(self.buf1, 0x75, self.mpu_addr)
        except OSError:
            raise MPUException(self._I2Cerror)
        chip_id = int(self.buf1[0])
        if chip_id != self._chip_id:
            print(f"Unexpected chip ID: 0x{chip_id:02x}. Possible clone chip?")
        return chip_id

    @property
    def sensors(self):
        return self._accel, self._gyro

    # get temperature
    @property
    def temperature(self):
        try:
            self._read(self.buf2, 0x41, self.mpu_addr)
        except OSError:
            raise MPUException(self._I2Cerror)
        return bytes_toint(self.buf2[0], self.buf2[1]) / 340 + 35  # I think

    # passthrough
    @property
    def passthrough(self):
        try:
            self._read(self.buf1, 0x37, self.mpu_addr)
            return self.buf1[0] & 0x02 > 0
        except OSError:
            raise MPUException(self._I2Cerror)

    @passthrough.setter
    def passthrough(self, mode):
        if type(mode) is bool:
            val = 2 if mode else 0
            try:
                self._write(val, 0x37, self.mpu_addr)  # I think this is right.
                self._write(0x00, 0x6A, self.mpu_addr)
            except OSError:
                raise MPUException(self._I2Cerror)
        else:
            raise ValueError("pass either True or False")

    # sample rate. Not sure why you'd ever want to reduce this from the default.
    @property
    def sample_rate(self):
        """
        Get sample rate as per Register Map document section 4.4
        SAMPLE_RATE= Internal_Sample_Rate / (1 + rate)
        default rate is zero i.e. sample at internal rate.
        """
        try:
            self._read(self.buf1, 0x19, self.mpu_addr)
            return self.buf1[0]
        except OSError:
            raise MPUException(self._I2Cerror)

    @sample_rate.setter
    def sample_rate(self, rate):
        """
        Set sample rate as per Register Map document section 4.4
        """
        if rate < 0 or rate > 255:
            raise ValueError("Rate must be in range 0-255")
        try:
            self._write(rate, 0x19, self.mpu_addr)
        except OSError:
            raise MPUException(self._I2Cerror)

    # Low pass filters. Using the filter_range property of the MPU9250 is
    # harmless but gyro_filter_range is preferred and offers an extra setting.
    @property
    def filter_range(self):
        """
        Returns the gyro and temperature sensor low pass filter cutoff frequency
        Pass:               0   1   2   3   4   5   6
        Cutoff (Hz):        250 184 92  41  20  10  5
        Sample rate (KHz):  8   1   1   1   1   1   1
        """
        try:
            self._read(self.buf1, 0x1A, self.mpu_addr)
            res = self.buf1[0] & 7
        except OSError:
            raise MPUException(self._I2Cerror)
        return res

    @filter_range.setter
    def filter_range(self, filt):
        """
        Sets the gyro and temperature sensor low pass filter cutoff frequency
        Pass:               0   1   2   3   4   5   6
        Cutoff (Hz):        250 184 92  41  20  10  5
        Sample rate (KHz):  8   1   1   1   1   1   1
        """
        # set range
        if filt in range(7):
            try:
                self._write(filt, 0x1A, self.mpu_addr)
            except OSError:
                raise MPUException(self._I2Cerror)
        else:
            raise ValueError("Filter coefficient must be between 0 and 6")

    # accelerometer range
    @property
    def accel_range(self):
        try:
            self._read(self.buf1, 0x1C, self.mpu_addr)
            ari = self.buf1[0] // 8
        except OSError:
            raise MPUException(self._I2Cerror)
        return ari

    @accel_range.setter
    def accel_range(self, accel_range):
        """
        Set accelerometer range
        Pass:               0   1   2   3
        for range +/-:      2   4   8   16  g
        """
        ar_bytes = (0x00, 0x08, 0x10, 0x18)
        if accel_range in range(len(ar_bytes)):
            try:
                self._write(ar_bytes[accel_range], 0x1C, self.mpu_addr)
            except OSError:
                raise MPUException(self._I2Cerror)
        else:
            raise ValueError("accel_range can only be 0, 1, 2 or 3")

    # gyroscope range
    @property
    def gyro_range(self):
        try:
            self._read(self.buf1, 0x1B, self.mpu_addr)
            gri = self.buf1[0] // 8
        except OSError:
            raise MPUException(self._I2Cerror)
        return gri

    @gyro_range.setter
    def gyro_range(self, gyro_range):
        """
        Set gyroscope range
        Pass:               0   1   2    3
        for range +/-:      250 500 1000 2000  degrees/second
        """
        gr_bytes = (0x00, 0x08, 0x10, 0x18)
        if gyro_range in range(len(gr_bytes)):
            try:
                self._write(
                    gr_bytes[gyro_range], 0x1B, self.mpu_addr
                )  # Sets fchoice = b11 which enables filter
            except OSError:
                raise MPUException(self._I2Cerror)
        else:
            raise ValueError("gyro_range can only be 0, 1, 2 or 3")

    # Accelerometer
    @property
    def accel(self):
        return self._accel

    def _accel_callback(self):
        try:
            self._read(self.buf6, 0x3B, self.mpu_addr)
        except OSError:
            raise MPUException(self._I2Cerror)
        self._accel._ivector[0] = bytes_toint(self.buf6[0], self.buf6[1])
        self._accel._ivector[1] = bytes_toint(self.buf6[2], self.buf6[3])
        self._accel._ivector[2] = bytes_toint(self.buf6[4], self.buf6[5])
        scale = (16384, 8192, 4096, 2048)
        self._accel._vector[0] = self._accel._ivector[0] / scale[self.accel_range]
        self._accel._vector[1] = self._accel._ivector[1] / scale[self.accel_range]
        self._accel._vector[2] = self._accel._ivector[2] / scale[self.accel_range]

    def get_accel_irq(self):
        self._read(self.buf6, 0x3B, self.mpu_addr)
        self._accel._ivector[0] = bytes_toint(self.buf6[0], self.buf6[1])
        self._accel._ivector[1] = bytes_toint(self.buf6[2], self.buf6[3])
        self._accel._ivector[2] = bytes_toint(self.buf6[4], self.buf6[5])

    # Gyro
    @property
    def gyro(self):
        return self._gyro

    def _gyro_callback(self):
        try:
            self._read(self.buf6, 0x43, self.mpu_addr)
        except OSError:
            raise MPUException(self._I2Cerror)
        self._gyro._ivector[0] = bytes_toint(self.buf6[0], self.buf6[1])
        self._gyro._ivector[1] = bytes_toint(self.buf6[2], self.buf6[3])
        self._gyro._ivector[2] = bytes_toint(self.buf6[4], self.buf6[5])
        scale = (131, 65.5, 32.8, 16.4)
        self._gyro._vector[0] = self._gyro._ivector[0] / scale[self.gyro_range]
        self._gyro._vector[1] = self._gyro._ivector[1] / scale[self.gyro_range]
        self._gyro._vector[2] = self._gyro._ivector[2] / scale[self.gyro_range]

    def get_gyro_irq(self):
        self._read(self.buf6, 0x43, self.mpu_addr)
        self._gyro._ivector[0] = bytes_toint(self.buf6[0], self.buf6[1])
        self._gyro._ivector[1] = bytes_toint(self.buf6[2], self.buf6[3])
        self._gyro._ivector[2] = bytes_toint(self.buf6[4], self.buf6[5])