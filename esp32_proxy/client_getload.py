from MangDang.mini_pupper.ESP32Interface import ESP32Interface
import time

esp32 = ESP32Interface()

while True:
    print(esp32.servos_get_load())
    time.sleep(1 / 20)  # 20 Hz
