import socket
import struct
import time
import network
from communication import Communication



def main():
    essid = "eps32"  # Имя точки доступа
    password = "7779777119"  # Пароль для подключения к точке доступа
    comm = Communication(essid, password)
    comm.create_heartbeat(1,1)
    while True:
        # size = gc.mem_free()
        gc.collect()
        # print('mem free = {}'.format(gc.mem_free() - size))
        # print('free mem = {}'.format(gc.mem_free()))
        comm.send_heartbeat()
        # time.sleep(0.1)
        # При необходимости можно получать сообщения от QGroundControl
        try:
            comm.receive_message()  # Получаем и обрабатываем ответные сообщения
        except OSError:
            pass  # Если сообщений нет, продолжаем цикл
        comm.sendAccelGyroMag()
        # time.sleep(0.1)
        
if __name__ == "__main__":
    main()

