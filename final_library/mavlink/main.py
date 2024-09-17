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
        gc.collect()
        comm.send_heartbeat()
        # time.sleep(0.1)
        try:
            comm.receive_message()
        except OSError:
            pass
        comm.sendAccelGyroMag()
        # time.sleep(0.1)
        
if __name__ == "__main__":
    main()

