#!/usr/bin/env python3

import os
import time
import threading
import cv2 as cv
import pyzbar.pyzbar as pyzbar
import RPi.GPIO as GPIO

from ros2bot_master_lib import Ros2botMasterDriver

class Ros2botWifi(object):
    def __init__(self, driver, debug=False):
        self._key1_pin = 17
        self._led_pin = 18
        self._key1_pressed = False
        self._config_mode = False
        self._count = 0
        self._ssid = ''
        self._passwd = ''
        self._ip = 'x.x.x.x'
        self._key_scan_state = 0  
        self._led_count = 0
        self._driver = driver 
        self._debug = debug
        self.init_gpio()

    def init_gpio(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self._key1_pin, GPIO.IN)
        GPIO.setup(self._led_pin, GPIO.OUT, initial=GPIO.LOW)     

    # get local IP address
    def get_ip_address(self):
        ip = os.popen(
            "/sbin/ifconfig eth0 | grep 'inet' | awk '{print $2}'").read()
        ip = ip[0: ip.find('\n')]
        if (ip == ''):
            ip = os.popen(
                "/sbin/ifconfig wlan0 | grep 'inet' | awk '{print $2}'").read()
            ip = ip[0: ip.find('\n')]
            if(ip == ''):
                ip = 'x.x.x.x'
        if len(ip) > 15:
            ip = 'x.x.x.x'
        return ip      

    # scan button
    def key_scan(self):
        print("[INFO] key thread started")
        while True:
            if GPIO.input(self._key1_pin) == GPIO.LOW:
                time.sleep(0.05)
                if GPIO.input(self._key1_pin) == GPIO.LOW:
                    if self._key1_pressed == False:
                        self._key1_pressed = True
                        self._count = 0
                        if self._debug:
                            print("[INFO] key pressed start")
                    else:
                        self._count += 1
                        # long press K1 to enter network mode
                        if self._count == 40: 
                            self._driver.set_beep(50)
                            self._config_mode = not self._config_mode
                            if self._debug:
                                print("[INFO] key pressed long event")
                else:
                    self._count = 0
                    self._key1_pressed = False
            else:
                self._count = 0
                self._key1_pressed = False
                time.sleep(0.1)      

    # read the current mode, enter the camera to recognize the TWO-DIMENSIONAL
    # code return True, otherwise return False
    def read_mode(self):
        return self._config_mode
    
    # display LED WiFi connection status, connected to the light, not connected to the light off
    def led_show_state(self):
        self._ip = self.get_ip_address()
        if self._ip == 'x.x.x.x':
            self.wifi_led_off()
        else:
            self.wifi_led_on()
        return self._ip    
    
    # turn on WiFi indicator
    def wifi_led_on(self):
        GPIO.output(self._led_pin, GPIO.HIGH)

    # turn off WiFi indicator
    def wifi_led_off(self):
        GPIO.output(self._led_pin, GPIO.LOW)  

    # cleanup GPIO 
    def destroy(self):
        GPIO.cleanup() 

    # define and parse the TWO-DIMENSIONAL code interface
    def decodeDisplay(self, image):
        barcodes = pyzbar.decode(image)
        ssid = ''
        passwd = ''
        for barcode in barcodes:
            (x, y, w, h) = barcode.rect
            cv.rectangle(image, (x, y), (x + w, y + h), (225, 225, 225), 2)
            bc_data = barcode.data.decode("utf-8")
            bc_type = barcode.type
            text = "{} ({})".format(bc_data, bc_type)
            cv.putText(image, text, (x, y - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            a = bc_data.find('SSID')
            b = bc_data.find('|')
            SSID = bc_data[6:b-1]
            PASSWD = bc_data[b+2:-1]
            if self._debug:
                print(f'[INFO] SSID: {ssid}')
                print(f'[INFO] PASSWD: {passwd}')
                print(f'[INFO] Found {bc_type} barcode: {bc_data}')
        return image, ssid, passwd      

    # connect camera to wifi and return IP address and image
    def connect(self, frame):
        try:
            self._led_count += 1

            if self._led_count == 5:
                self._driver.set_beep(30)
                self.wifi_led_on()
            elif self._led_count >= 10:
                self.wifi_led_off()
                self._led_count = 0

            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            img, self._ssid, self._passwd = self.decodeDisplay(gray)

            if (self._ssid != '' and self._passwd != ''):
                print(f'[INFO] connecting to WiFi network {self._ssid}...')
                self._driver.set_beep(1000)
                passwd = "jetson"
                cmd_scan = "sudo iw dev wlan0 scan | grep " + self._ssid
                os.system('echo %s | sudo -S %s' % (passwd, cmd_scan))
                cmd = "sudo nmcli dev wifi connect \'" + self._ssid + "\' password \'" + self._passwd + "\'"
                os.system('echo %s | sudo -S %s' % (passwd, cmd))
                
                self._ssid = ''
                self._passwd = ''
                self._config_mode = False

                time.sleep(.2)

                for ip in range(5):
                    self._ip = self.get_ip_address()
                    if self._ip != "x.x.x.x":
                        for i in range(3):
                            self._driver.set_beep(100)
                            time.sleep(.2)
                        if self._debug:
                            print(f'[INFO] WIFI connected, IP={self._ip}')
                        return self._ip, img
                    time.sleep(1)                
                print("[ERROR] WIFI connect failed")
        except:
            img = frame
        return "x.x.x.x", img
    
    def init_camera(self):
        camera = cv.VideoCapture(0)
        if cv.__version__ == '3':
            camera.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'XVID'))
        else:
            camera.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))    

    def process_frames(self):
        gfps = 0
        calc_avg = False
        start_time = time.time()
        camera = self.init_camera()

        while camera.isOpened():
            if calc_avg:
                result, frame = camera.read()
                gfps += 1
                fps = gfps / (time.time() - start_time)
            else:
                start = time.time()
                result, frame = camera.read()
                end = time.time()
                fps = 1 / (end - start)
            label = f'FPS: {fps}'

            cv.putText(frame, label, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 0), 1)
            cv.imshow("img", frame)

            if self.read_mode():
                ip, img = self.connect(frame)
                if ip != 'x.x.x.x':
                    print("IP:" + ip)
                    break    

            action = cv.waitKey(10) & 0xff
            if action == ord('q') or action == 27:
                break                                    

def main():
    driver = Ros2botMasterDriver()
    wifi = Ros2botWifi(driver)
    try:
        wifi.process_frames()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
