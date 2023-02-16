#!/usr/bin/env python3

import time
import cv2 as cv

class Ros2botCamera(object):
    
    def __init__(self, camera_id=0, width=640, height=480, debug=False):
        self._camera_id = camera_id
        self._width = width
        self._height = height
        self._state = False
        self._camera = None
        self._debug = debug
        self._state = self.init_camera()

    def __del__(self):
        if self._debug:
            print("[INFO] camera released")
        self._camera.release()
        self._state = False        

    def config_camera(self):
        cvver = cv.__version__
        if cvver[0]=='3':
            self._camera.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'XVID'))
        else:
            self._camera.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        
        # self._camera.set(cv.CAP_PROP_BRIGHTNESS, 30)  # -64 - 64  0.0
        # self._camera.set(cv.CAP_PROP_CONTRAST, 50)  # -64 - 64  2.0
        # self._camera.set(cv.CAP_PROP_EXPOSURE, 156)  # 1.0 - 5000  156.0

        self._camera.set(cv.CAP_PROP_FRAME_WIDTH, self._width)  # 640
        self._camera.set(cv.CAP_PROP_FRAME_HEIGHT, self._height)  # 480        

    def init_camera(self):
        self._camera = cv.VideoCapture(self._camera_id)
        result = self._camera.isOpened()
        if not result:
            print("[ERROR] camera initialization failed")
            return False
        self.config_camera()  
        if self._debug:
            print(f'[INFO] camera {self._camera_id} initialized')    
        return True
    
    # check if camera is enabled
    def is_opened(self):
        return self._camera.isOpened()
    
    # release camera
    def clear(self):
        self._camera.release()  

    # reconnect camera
    def reconnect(self):  
        self._camera = cv.VideoCapture(self._camera_id)
        result, _ = self._camera.read()
        if not result:
            self._camera_id = (self._camera_id + 1) % 2
            self._camera = cv.VideoCapture(self._camera_id)
            result, _ = self._camera.read()
            if not result:
                self.__state = False
                print("[ERROR] camera reconnect failed")
                return False
        if not self._state:
            if self._debug:
                print(f'[INFO] camera {self._camera_id} reconnect succeeded')
            self.__state = True
            self.config_camera()
        return True  

    # get camera frame
    def get_frame(self):
        result, image = self._camera.read()
        if not result:
            return result, bytes({1})
        return result, image       

    # get camera as JPG imagethe JPG image
    def get_frame_jpg(self, label="", color=(0, 255, 0)):
        result, image = self._camera.read()
        if not result:
            return result, bytes({1})
        if label != "":
            # params: image, added text, top left coordinate, font, font size, color, font size  
            cv.putText(image, label, (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        result, jpeg = cv.imencode('.jpg', image)
        return result, jpeg.tobytes()           

def main():
    gfps = 0
    calc_avg = False
    start_time = time.time()
    camera = Ros2botCamera()

    while camera.is_opened():
        if calc_avg:
            ret, frame = camera.get_frame()
            gfps += 1
            fps = gfps / (time.time() - start_time)
        else:
            start = time.time()
            ret, frame = camera.get_frame()
            end = time.time()
            fps = 1 / (end - start)
        label = f'FPS: {fps}'
        cv.putText(frame, label, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 0), 1)
        cv.imshow('frame', frame)
        k = cv.waitKey(1) & 0xFF
        if k == 27 or k == ord('q'):
            break
    del camera
    cv.destroyAllWindows()        

if __name__ == '__main__':
    main()