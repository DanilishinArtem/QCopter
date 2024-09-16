from machine import I2C

class MPU9250:
    def __init__(self, i2c, address=0x68):
        self.i2c = i2c
        self.address = address
        self.i2c.writeto_mem(self.address, 0x6B, b'\x00')  # Активируем MPU9250

    @property
    def acceleration(self):
        data = self.i2c.readfrom_mem(self.address, 0x3B, 6)
        x = int.from_bytes(data[0:2], 'big', signed=True) / 16384.0
        y = int.from_bytes(data[2:4], 'big', signed=True) / 16384.0
        z = int.from_bytes(data[4:6], 'big', signed=True) / 16384.0
        return (x, y, z)

    @property
    def gyro(self):
        data = self.i2c.readfrom_mem(self.address, 0x43, 6)
        x = int.from_bytes(data[0:2], 'big', signed=True) / 131.0
        y = int.from_bytes(data[2:4], 'big', signed=True) / 131.0
        z = int.from_bytes(data[4:6], 'big', signed=True) / 131.0
        return (x, y, z)

    @property
    def magnetic(self):
        self.i2c.writeto_mem(self.address, 0x37, b'\x02')  # Активируем магнитометр
        data = self.i2c.readfrom_mem(0x0C, 0x03, 6)  # Магнитометр на 0x0C
        x = int.from_bytes(data[0:2], 'big', signed=True) * 0.15
        y = int.from_bytes(data[2:4], 'big', signed=True) * 0.15
        z = int.from_bytes(data[4:6], 'big', signed=True) * 0.15
        return (x, y, z)
