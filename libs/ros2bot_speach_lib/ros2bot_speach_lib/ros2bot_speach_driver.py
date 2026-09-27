#!/usr/bin/env python3
# coding: utf-8

import time
import serial

# v0.0.1
class Ros2botSpeachDriver(object):

    def __init__(self, com="/dev/r2bspeach"):
        # com="/dev/ttyUSB0"
        self.ser = serial.Serial(com, 115200)
        self.__rx_buffer = bytearray()
        if self.ser.isOpen():
            print("[INFO] speach serial comm opened, baudrate=115200")
        else:
            print("[ERROR] speach serial comm open failed")

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

    def close(self):
        ser = getattr(self, "ser", None)
        if ser is not None and ser.is_open:
            ser.close()
            print("[INFO] speach serial comm closed")

    def void_write(self, void_data):
        value = int(void_data)
        if not 0 <= value <= 999:
            raise ValueError("void_data must be between 0 and 999")
        void_data1 = value // 100 + 48
        void_data2 = value % 100 // 10 + 48
        void_data3 = value % 10 + 48
        cmd = [0x24, 0x41, void_data1, void_data2, void_data3, 0x23]
        #print(cmd)
        self.ser.write(cmd)
        time.sleep(0.005)
        self.ser.reset_input_buffer()

    def speech_read(self):
        count = self.ser.in_waiting
        if count:
            self.__rx_buffer.extend(self.ser.read(count))

        while self.__rx_buffer:
            start = self.__rx_buffer.find(b"$A")
            if start < 0:
                self.__rx_buffer[:] = self.__rx_buffer[-1:] if self.__rx_buffer[-1:] == b"$" else b""
                return 999
            if start > 0:
                del self.__rx_buffer[:start]
            if len(self.__rx_buffer) < 6:
                return 999

            frame = self.__rx_buffer[:6]
            if frame[5:] != b"#" or not frame[2:5].isdigit():
                del self.__rx_buffer[:2]
                continue

            del self.__rx_buffer[:6]
            return int(frame[2:5])

        return 999
