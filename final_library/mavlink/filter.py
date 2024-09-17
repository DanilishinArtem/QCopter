import math
import time

class SimpleMovingAverage:
    def __init__(self, window_size=10):
        self.window_size = window_size
        self.data = []

    def add(self, value):
        self.data.append(value)
        if len(self.data) > self.window_size:
            self.data.pop(0)
        return sum(self.data) / len(self.data)

class ComplementaryFilter:
    def __init__(self, alpha=0.95):
        self.alpha = alpha
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_time = time.time()
        self.init_roll = None
        self.init_pitch = None
        self.init_yaw = None
        self.gyro_roll_filter = SimpleMovingAverage()
        self.gyro_pitch_filter = SimpleMovingAverage()
        self.gyro_yaw_filter = SimpleMovingAverage()

    def wrap_to_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def update(self, accel, gyro, mag):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Вычисление углов на основе акселерометра
        accel_roll = math.atan2(accel.y, accel.z)
        accel_pitch = math.atan2(-accel.x, math.sqrt(accel.y**2 + accel.z**2))

        # Вычисление изменения углов на основе гироскопа
        gyro_roll = self.gyro_roll_filter.add(gyro.x * dt)
        gyro_pitch = self.gyro_pitch_filter.add(gyro.y * dt)
        gyro_yaw = self.gyro_yaw_filter.add(gyro.z * dt)

        self.roll = self.alpha * (self.roll + gyro_roll) + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * (self.pitch + gyro_pitch) + (1 - self.alpha) * accel_pitch

        # Рассчет yaw на основе магнитометра
        mag_x = mag.x * math.cos(self.pitch) + mag.y * math.sin(self.roll) * math.sin(self.pitch) + mag.z * math.cos(self.roll) * math.sin(self.pitch)
        mag_y = mag.y * math.cos(self.roll) - mag.z * math.sin(self.roll)
        self.yaw = math.atan2(mag_y, mag_x)

        # Угловые скорости
        rollspeed = gyro.x  # уже в радианах/сек
        pitchspeed = gyro.y  # уже в радианах/сек
        yawspeed = gyro.z  # уже в радианах/сек

        # Инициализация углов (зафиксировать стартовые значения)
        if self.init_roll is None and self.init_pitch is None and self.init_yaw is None:
            self.init_roll = self.roll
            self.init_pitch = self.pitch
            self.init_yaw = self.yaw

        # Приведение углов к диапазону [-pi, pi] относительно инициализации
        self.roll = self.wrap_to_pi(self.roll - self.init_roll)
        self.pitch = self.wrap_to_pi(self.pitch - self.init_pitch)
        self.yaw = self.wrap_to_pi(self.yaw - self.init_yaw)

        return self.roll, self.pitch, self.yaw, rollspeed, pitchspeed, yawspeed, self.last_time
