#!/usr/bin/env python3

import os
import sys
import struct
import inspect
import ctypes
import socket
import time
import threading
import cv2 as cv

#from gevent import pywsgi
from flask import Flask, render_template, Response
from ros2bot_master_lib import Ros2botMasterDriver
from ros2bot_camera import Ros2botCamera
from ros2bot_wifi import Ros2botWifi

# constants
AKM_DEFAULT_ANGLE = 100
AKM_LIMIT_ANGLE = 45
AKM_PWMSERVO_ID = 1

# globals
_driver = Ros2botMasterDriver()
_type = 1
_camera = Ros2botCamera()
_wifi = Ros2botWifi(_driver)
_ip_addr = "x.x.x.x"
_tcp_ip = _ip_addr
_wifi_state = False
_init = False
_mode = 'home'
_socket = None
_speed_ctrl_xy = 100
_speed_ctrl_z = 100
_motor_speed = [0, 0, 0, 0]
_stabilize_state = 0
_akm_def_angle = 100
_tcp_except_count = 0
_debug = False

def return_bot_version(tcp):
    T_TYPE = _type
    T_FUNC = 0x01
    T_LEN = 0x04
    version = int(_driver.get_version() * 10)
    if version < 0: version = 0
    checknum = (6 + version) % 256
    data = "$%02x%02x%02x%02x%02x#" % (T_TYPE, T_FUNC, T_LEN, version, checknum)
    tcp.send(data.encode(encoding="utf-8"))
    if _debug:
        print("[INFO] driver version:", version / 10.0)
        print("[INFO] TCP send:", data)

def return_battery_voltage(tcp):
    T_TYPE = _type
    T_FUNC = 0x02
    T_LEN = 0x04
    vol = int(_driver.get_battery_voltage() * 10) % 256
    if vol < 0: vol = 0
    checknum = (T_TYPE + T_FUNC + T_LEN + vol) % 256
    data = "$%02x%02x%02x%02x%02x#" % (T_TYPE, T_FUNC, T_LEN, vol, checknum)
    tcp.send(data.encode(encoding="utf-8"))
    if _debug:
        print("[INFO] voltage:", vol / 10.0)
        print("[INFO] TCP send:", data)
    return vol / 10.0

def return_speed(tcp, speed_xy, speed_z):
    T_TYPE = _type
    T_FUNC = 0x16
    T_LEN = 0x06
    checknum = (T_TYPE + T_FUNC + T_LEN + int(speed_xy) + int(speed_z)) % 256
    data = "$%02x%02x%02x%02x%02x%02x#" % (T_TYPE, T_FUNC, T_LEN, int(speed_xy), int(speed_z), checknum)
    tcp.send(data.encode(encoding="utf-8"))
    if _debug:
        print("[INFO] speed:", speed_xy, speed_z)
        print("[INFO] TCP send:", data)

def return_stabilize(tcp, state):
    T_TYPE = _type
    T_FUNC = 0x17
    T_LEN = 0x04
    checknum = (T_TYPE + T_FUNC + T_LEN + int(state)) % 256
    data = "$%02x%02x%02x%02x%02x#" % (T_TYPE, T_FUNC, T_LEN, int(state), checknum)
    tcp.send(data.encode(encoding="utf-8"))
    if _debug:
        print("[INFO] stabilize:", int(state))
        print("[INFO] TCP send:", data)

def return_current_speed(tcp):
    T_TYPE = _type
    T_FUNC = 0x22
    T_LEN = 0x0E
    speed = _driver.get_motion_data()
    num_x = int(speed[0]*100)
    num_y = int(speed[1]*100)
    num_z = int(speed[2]*20)
    speed_x = num_x.to_bytes(2, byteorder='little', signed=True)
    speed_y = num_y.to_bytes(2, byteorder='little', signed=True)
    speed_z = num_z.to_bytes(2, byteorder='little', signed=True)
    checknum = (T_TYPE + T_FUNC + T_LEN + speed_x[0] + speed_x[1] + speed_y[0] + speed_y[1] + speed_z[0] + speed_z[1]) % 256
    data = "$%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x#" % \
        (T_TYPE, T_FUNC, T_LEN, speed_x[0], speed_x[1], speed_y[0], speed_y[1], speed_z[0], speed_z[1], checknum)
    tcp.send(data.encode(encoding="utf-8"))
    if _debug:
        print("[INFO] current_speed:", num_x, num_y, num_z)
        print("[INFO] TCP send:", data)

def return_ackerman_angle(tcp, id, angle):
    T_TYPE = _type
    T_FUNC = 0x50
    T_LEN = 0x06
    checknum = (T_TYPE + T_FUNC + T_LEN + int(id) + int(angle)) % 256
    data = "$%02x%02x%02x%02x%02x%02x#" % (T_TYPE, T_FUNC, T_LEN, int(id), int(angle), checknum)
    tcp.send(data.encode(encoding="utf-8"))
    if _debug:
        print("[INFO] ackerman_angle:", int(angle))
        print("[INFO] TCP send:", data)

def return_arm_angle(tcp):
    T_TYPE = _type
    T_FUNC = 0x14
    T_LEN = 0x1A

    if T_TYPE != _driver.BOTTYPE_X3_PLUS:
        return
    
    angle = _driver.get_uart_servo_angle_array()    
    if angle[1] != -1: angle[1] = 180 - angle[1]
    if angle[2] != -1: angle[2] = 180 - angle[2]
    if angle[3] != -1: angle[3] = 180 - angle[3]

    angle_s1 = bytearray(struct.pack('h', int(angle[0])))
    angle_s2 = bytearray(struct.pack('h', int(angle[1])))
    angle_s3 = bytearray(struct.pack('h', int(angle[2])))
    angle_s4 = bytearray(struct.pack('h', int(angle[3])))
    angle_s5 = bytearray(struct.pack('h', int(angle[4])))
    angle_s6 = bytearray(struct.pack('h', int(angle[5])))

    checknum = (T_TYPE + T_FUNC + T_LEN + angle_s1[0] + angle_s1[1] + angle_s2[0] + angle_s2[1] + angle_s3[0] + angle_s3[1]) % 256
    data = "$%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x#" % \
            (T_TYPE, T_FUNC, T_LEN, \
            angle_s1[0], angle_s1[1], angle_s2[0], angle_s2[1], angle_s3[0], angle_s3[1], \
            angle_s4[0], angle_s4[1], angle_s5[0], angle_s5[1], angle_s6[0], angle_s6[1], \
            checknum)
    tcp.send(data.encode(encoding="utf-8"))
    if _debug:
        print("[INFO] return angle_s1-s6:", angle)
        print("[INFO] TCP send:", data)

def return_arm_offset_state(tcp, id, state):
    T_TYPE = _type
    T_FUNC = 0x40
    T_LEN = 0x06
    checknum = (T_TYPE + T_FUNC + T_LEN + int(id) + int(state)) % 256
    data = "$%02x%02x%02x%02x%02x%02x#" % (T_TYPE, T_FUNC, T_LEN, int(id), int(state), checknum)
    tcp.send(data.encode(encoding="utf-8"))
    if _debug:
        print("[INFO] return arm offset state:", id, state)
        print("[INFO] TCP send:", data)    

def my_map(x, in_min, in_max, out_min, out_max):
    return (out_max - out_min) * (x - in_min) / (in_max - in_min) + out_min

def parse_data(tcp, data):
    global _mode
    global _driver
    global _camera
    global _type
    global _akm_def_angle
    global _motor_speed
    global _speed_ctrl_xy
    global _speed_ctrl_z
    global _stabilize_state

    data_size = len(data)

    if data_size < 8:
        print(f'[ERROR] data length is too short ({data_size})')  
        return  
    if int(data[5:7], 16) != data_size-8:
        print(f'[ERROR] The data length error {int(data[5:7], 16)}, {data_size-8}')
        return
    
    checknum = 0
    num_checknum = int(data[data_size-3:data_size-1], 16)

    for i in range(0, data_size-4, 2):
        checknum = (int(data[1+i:3+i], 16) + checknum) % 256
        if _debug:
            print(f'[INFO] checksum: {i}, {int(data[1+i:3+i], 16)}, {checknum}')
    if checknum != num_checknum:
        print("[ERROR] num_checknum error", checknum, num_checknum)
        print("[ERROR] checksum error cmd:0x%02x, calnum:%d, recvnum:%d" % (int(data[3:5], 16), checknum, num_checknum))
        return 

    num_type = int(data[1:3], 16)

    if num_type <= 0 or num_type > 5:
        print("[ERROR] number type error")
        return
    else:
        if _type != num_type:
            _type = num_type
            _driver.set_bot_type(_type)      

    cmd = data[3:5]

    if cmd == "0F":  
        func = int(data[7:9])
        if _debug:
            print(f'[INFO] command func={func}')
        _mode = 'home'
        if func == 0:
            return_battery_voltage(tcp)
        elif func == 1:
            return_speed(tcp, _speed_ctrl_xy, _speed_ctrl_z)
            return_stabilize(tcp, _stabilize_state)
            _mode = 'standard'
        elif func == 2:
            return_current_speed(tcp)
            _mode = 'mecanum'             

    elif cmd == "01":  
        if _debug:
            print("[INFO] get version")
        return_bot_version(tcp)

    elif cmd == "02":  
        if _debug:
            print("[INFO] get voltage")
        return_battery_voltage(tcp)

    elif cmd == "10":  
        num_x = int(data[7:9], 16)
        num_y = int(data[9:11], 16)
        if num_x > 127:
            num_x = num_x - 256
        if num_y > 127:
            num_y = num_y - 256
        speed_x = num_y / 100.0
        speed_y = -num_x / 100.0
        if speed_x == 0 and speed_y == 0:
            _driver.set_bot_run(0, _stabilize_state)
        else:    
            if _type == _driver.BOTTYPE_R2:
                speed_y = my_map(speed_y, -1, 1, AKM_LIMIT_ANGLE/1000.0, AKM_LIMIT_ANGLE/-1000.0)
                # speed_z = my_map(speed_y, -1, 1, 3.0, -3.0)
                _driver.set_bot_motion(speed_x*1.8, speed_y, 0)
            else:
                _driver.set_bot_motion(speed_x, speed_y, 0)
        if _debug:
            print("[INFO] speed_x:%.2f, speed_y:%.2f" % (speed_x, speed_y))      

    elif cmd == "11":
        num_id = int(data[7:9], 16)
        num_angle = int(data[9:11], 16)
        if _debug:
            print("[INFO] pwm servo id:%d, angle:%d" % (num_id, num_angle))
        if _type == _driver.BOTTYPE_R2:
            angle_mini = (AKM_DEFAULT_ANGLE - AKM_LIMIT_ANGLE)
            angle_max = (AKM_DEFAULT_ANGLE + AKM_LIMIT_ANGLE)
            if num_angle < angle_mini:
                num_angle = angle_mini
            elif num_angle > angle_max:
                num_angle = angle_max
            _driver.set_pwm_servo(num_id, num_angle)
        else:
            _driver.set_pwm_servo(num_id, num_angle)  

    elif cmd == "12":
        num_id = int(data[7:9], 16)
        num_angle_l = int(data[9:11], 16)
        num_angle_h = int(data[11:13], 16)
        uart_servo_angle = num_angle_h * 256 + num_angle_l
        if _debug:
            print("[INFO] uart servo id:%d, angle:%d" % (num_id, uart_servo_angle))
        if 1 < num_id < 5:
            uart_servo_angle = 180 - uart_servo_angle
        _driver.set_uart_servo_angle(num_id, uart_servo_angle)  

    elif cmd == "13":
        num_state = int(data[7:9], 16)
        num_delay = int(data[9:11], 16)
        if _debug:
            print("[INFO] beep:%d, delay:%d" % (num_state, num_delay))
        delay_ms = 0
        if num_state > 0:
            if num_delay == 255:
                delay_ms = 1
            else:
                delay_ms = num_delay * 10
        _driver.set_beep(delay_ms)    

    elif cmd == "14":
        num_id = int(data[7:9], 16)
        if _debug:
            print("[INFO] read angle:%d" % num_id)
        if num_id == 6:
            return_arm_angle(tcp)    

    elif cmd == "15":
        num_dir = int(data[7:9], 16)
        if _debug:
            print("[INFO] btn ctl:%d" % num_dir)
        speed = 0
        if _type == _driver.BOTTYPE_R2:
            if num_dir == 0:
                _driver.set_bot_run(0, _stabilize_state)
            elif num_dir == 1 or num_dir == 2:
                speed = int(_speed_ctrl_xy*1.8)
                _driver.set_bot_run(num_dir, speed)
            else:
                speed = int(_speed_ctrl_z*1.8)
                _driver.set_bot_run(num_dir, speed, _stabilize_state)
        else:    
            if num_dir == 0:
                _driver.set_bot_run(0, _stabilize_state)
            elif num_dir == 5 or num_dir == 6:
                speed = _speed_ctrl_z
                _driver.set_bot_run(num_dir, speed)
            else:
                speed = _speed_ctrl_xy
                _driver.set_bot_run(num_dir, speed, _stabilize_state)
        if _debug:
            print("[INFO] robot speed:%.2f" % speed)   

    elif cmd == '16':
        num_speed_xy = int(data[7:9], 16)
        num_speed_z = int(data[9:11], 16)
        if _debug:
            print("[INFO] speed ctl:%d, %d" % (num_speed_xy, num_speed_z))
        _speed_ctrl_xy = num_speed_xy
        _speed_ctrl_z = num_speed_z
        if _speed_ctrl_xy > 100:
            _speed_ctrl_xy = 100
        if _speed_ctrl_xy < 0:
            _speed_ctrl_xy = 0
        if _speed_ctrl_z > 100:
            _speed_ctrl_z = 100
        if _speed_ctrl_z < 0:
            _speed_ctrl_z = 0     

    elif cmd == '17':
        num_stab = int(data[7:9], 16)
        if _debug:
            print("[INFO] robot stabilize:%d" % num_stab)
        if num_stab > 0:
            _stabilize_state = 1
        else:
            _stabilize_state = 0 

    elif cmd == '20':
        num_id = int(data[7:9], 16)
        num_speed = int(data[9:11], 16)
        if num_speed > 127:
            num_speed = num_speed - 256
        if _debug:
            print("[INFO] mecanum wheel ctrl:%d, %d" % (num_id, num_speed))
        if num_id >= 0 and num_id <= 4:
            if num_speed > 100:
                num_speed = 100
            if num_speed < -100:
                num_speed = -100
            if num_id == 0:
                _motor_speed[0] = 0
                _motor_speed[1] = 0
                _motor_speed[2] = 0
                _motor_speed[3] = 0
            else:
                _motor_speed[num_id-1] = num_speed
            _driver.set_motor(_motor_speed[0], _motor_speed[1], _motor_speed[2], _motor_speed[3]) 

    elif cmd == '21':
        num_speed_m1 = int(data[7:9], 16)
        num_speed_m2 = int(data[9:11], 16)
        num_speed_m3 = int(data[11:13], 16)
        num_speed_m4 = int(data[13:15], 16)
        if num_speed_m1 > 127:
            num_speed_m1 = num_speed_m1 - 256
        if num_speed_m2 > 127:
            num_speed_m2 = num_speed_m2 - 256
        if num_speed_m3 > 127:
            num_speed_m3 = num_speed_m3 - 256
        if num_speed_m4 > 127:
            num_speed_m4 = num_speed_m4 - 256
        if _debug:
            print("[INFO] mecanum wheel update:%d, %d, %d, %d" % (num_speed_m1, num_speed_m2, num_speed_m3, num_speed_m4))
        _motor_speed[0] = num_speed_m1
        _motor_speed[1] = num_speed_m2
        _motor_speed[2] = num_speed_m3
        _motor_speed[3] = num_speed_m4
        _driver.set_motor(_motor_speed[0], _motor_speed[1], _motor_speed[2], _motor_speed[3])  

    elif cmd == "30":
        num_id = int(data[7:9], 16)
        num_r = int(data[9:11], 16)
        num_g = int(data[11:13], 16)
        num_b = int(data[13:15], 16)
        if _debug:
            print("[INFO] lamp:%d, r:%d, g:%d, b:%d" % (num_id, num_r, num_g, num_b))
        _driver.set_colorful_lamps(num_id, num_r, num_g, num_b)

    elif cmd == "31":
        num_effect = int(data[7:9], 16)
        num_speed = int(data[9:11], 16)
        if _debug:
            print("[INFO] effect:%d, speed:%d" % (num_effect, num_speed))
        _driver.set_colorful_effect(num_effect, num_speed, 255)

    elif cmd == "32":
        num_color = int(data[7:9], 16)
        if _debug:
            print("[INFO] breath color:%d" % num_color)
        if num_color == 0:
            _driver.set_colorful_effect(0, 255, 255)
        else:
            _driver.set_colorful_effect(3, 255, num_color - 1)

    elif cmd == '40':
        num_cali = int(data[7:9], 16)
        if _debug:
            print("[INFO] arm offset:%d" % num_cali)
        if num_cali == 1:
            for i in range(6):
                id = int(i+1)
                state = _driver.set_uart_servo_offset(id)
                return_arm_offset_state(tcp, id, state)

    elif cmd == '41':
        num_verify = int(data[7:9], 16)
        if _debug:
            print("[INFO] arm up:%d" % num_verify)
        if num_verify == 1:
            _driver.set_uart_servo_torque(True)
            time.sleep(.01)
            angle_array = [90, 90, 90, 90, 90, 180]
            _driver.set_uart_servo_angle_array(angle_array)
            time.sleep(.5)

    elif cmd == '42':
        num_verify = int(data[7:9], 16)
        if _debug:
            print("[INFO] arm torque:%d" % num_verify)
        if num_verify == 0:
            _driver.set_uart_servo_torque(False)
        else:
            _driver.set_uart_servo_torque(True)

    elif cmd == '50':
        num_id = int(data[7:9], 16)
        if _debug:
            print("[INFO] akm angle read:%d" % num_id)
        if num_id == AKM_PWMSERVO_ID:
            g_akm_def_angle = _driver.get_akm_default_angle()
            return_ackerman_angle(tcp, AKM_PWMSERVO_ID, g_akm_def_angle)

    elif cmd == '51':
        num_id = int(data[7:9], 16)
        num_angle = int(data[9:11], 16)
        if _debug:
            print("[INFO] akm change angle:%d, %d" % (num_id, num_angle))
        if 60 <= num_angle <= 120 and num_id == AKM_PWMSERVO_ID:
            g_akm_def_angle = num_angle
            _driver.set_akm_default_angle(num_angle)

    elif cmd == '52':
        num_verify = int(data[7:9], 16)
        if _debug:
            print("[INFO] akm save angle:%d, %d" % (num_verify, g_akm_def_angle))
        if num_verify == 1:
            _driver.set_akm_default_angle(g_akm_def_angle, True)
            time.sleep(.1)

    elif cmd == '53':
        num_id = int(data[7:9], 16)
        num_angle = int(data[9:11], 16)
        if num_angle > 127:
            num_angle = num_angle - 256
        if _debug:
            print("[INFO] akm angle:%d, %d" % (num_id, num_angle))
        if -45 <= num_angle <= 45 and num_id == AKM_PWMSERVO_ID:
            _driver.set_akm_steering_angle(num_angle)

def start_tcp_server(ip, port):
    global _init
    global _tcp_except_count
    global _socket
    global _mode
    _init = True
    if _debug:
        print(f'[INFO] TCP server started (IP:{ip}, Port:{port})')
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(None)
    sock.bind((ip, port))
    sock.listen(1)    
    while True:
        if _debug:
            print("[INFO] waiting for the client to connect")
        tcp_state = 0
        _tcp_except_count = 0
        _socket, address = sock.accept()
        if _debug:
            print(f'[INFO] connected, client IP:{address}')
        tcp_state = 1
        while True:
            try:
                tcp_state = 2
                cmd = _socket.recv(1024).decode(encoding="utf-8")
                if not cmd:
                    break
                tcp_state = 3
                if _debug:
                    print(f'[INFO] [-]cmd:{cmd}, len:{len(cmd)}')
                tcp_state = 4
                index1 = cmd.rfind("$")
                index2 = cmd.rfind("#")
                if index1 < 0 or index2 <= index1:
                    continue
                tcp_state = 5
                # parse_data(g_socket, cmd[index1:index2 + 1])
                g_tcp_except_count = 0
            except:
                if tcp_state == 2:
                    _tcp_except_count += 1
                    if _tcp_except_count >= 10:
                        _tcp_except_count = 0
                        break
                else:
                    print(f'[ERROR] socket exception {tcp_state}')
                continue
            parse_data(_socket, cmd[index1:index2 + 1])
        if _debug:
            print("[INFO] socket disconnected")
        _socket.close()
        _mode = 'home'    

# initialize TCP socket
def init_tcp_socket():
    global _init
    global _tcp_ip
    if _init: return
    while True:
        ip = _wifi.get_ip_address()
        if ip == "x.x.x.x":
            _tcp_ip = ip
            if _debug:
                print("[INFO] get ip address fail!")
            time.sleep(0.5)
            continue
        if ip != "x.x.x.x":
            _tcp_ip = ip
            if _debug:
                print("[INFO] TCP service IP=", ip)
            break
    task_tcp = threading.Thread(target=start_tcp_server, name="task_tcp", args=(ip, 6000))
    task_tcp.setDaemon(True)
    task_tcp.start()
    if _debug:
        print('[INFO] initialized TCP socket')

def mode_handle():
    global _mode
    global _camera
    if _debug:
        print("[INFO] mode handle")
    gfps = 0
    start_time = time.time()
    while True:
        if _mode == 'standard':
            success, frame = _camera.get_frame()
            gfps += 1
            fps = gfps / (time.time() - start_time)
            label = f'FPS: {fps}'
            if not success:
                _camera.clear()
                gfps = 0
                start_time = time.time()
                if _debug:
                    print("[INFO] camera is reconnecting")
                _camera.reconnect()
                time.sleep(0.5)
                continue
            cv.putText(frame, label, (10, 25), cv.FONT_HERSHEY_TRIPLEX, 0.8, (0, 200, 0), 1)
            ret, img_encode = cv.imencode('.jpg', frame)
            if ret:
                img_encode = img_encode.tobytes()
                yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + img_encode + b'\r\n')
        else:
            time.sleep(.1)   

@app.route('/')
def index():
    return render_template('index.html')


@app.route('/video_feed')
def video_feed():
    if _debug:
        print("[INFO] video feed")
    return Response(mode_handle(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/init')
def init():
    init_tcp_socket()
    return render_template('init.html')   

def thread_mecanum():
    global _mode
    global _socket
    while True:
        if _mode == 'mecanum':
            return_current_speed(_socket)
            time.sleep(0.35)
        else:
            time.sleep(1)          

def main(debug=False):
    global _driver
    global _debug

    _debug = debug

    task_mecanum = threading.Thread(target=thread_mecanum, name="task_mecanum")
    task_mecanum.setDaemon(True)
    task_mecanum.start()

    init_tcp_socket()
    time.sleep(0.1)

    for i in range(3):
        _driver.set_beep(60)
        time.sleep(0.2)

    print(f'[INFO] ros2bot driver version: {_driver.get_version()}')
    print(f'[INFO] Waiting for connect to the app')

    try:
        server = pywsgi.WSGIServer(('0.0.0.0', 6500), app)
        server.serve_forever()
    except KeyboardInterrupt:
        _driver.set_bot_motion(0, 0, 0)
        _driver.set_beep(0)
        if _debug:
            print("[INFO] ros2bot driver deleted")
        del _driver    

if __name__ == '__main__':
    main()