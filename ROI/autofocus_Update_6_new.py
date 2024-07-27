import serial
import time
import cv2
from time import sleep
from picamera import PiCamera
import numpy as np
import io
import os
from datetime import datetime

class Autofocus:
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600, timeout=1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.camera = PiCamera()
        self.x = 0
        self.y = 0
        self.z = 0
        sleep(1)

    def movexclock(self, distance):
        command = "xclk,{}".format(distance)
        self.ser.write(command.encode("utf-8"))
        self.x += distance

    def movexanticlock(self, distance):
        command = "xcclk,{}".format(distance)
        self.ser.write(command.encode("utf-8"))
        self.x -= distance

    def movey(self, steps):
        command = "ycclk,{}".format(steps)
        self.ser.write(command.encode("utf-8"))
        self.y += steps
        sleep(1)
    def moveyc(self, steps):
        command = "yclk,{}".format(steps)
        self.ser.write(command.encode("utf-8"))
        self.y -= steps
        sleep(1)

    def movezclock(self, distance):
        command = "zclk,{}".format(distance)
        self.ser.write(command.encode("utf-8"))
        self.z -= distance

    def movezanticlock(self, distance):
        command = "zcclk,{}".format(distance)
        self.ser.write(command.encode("utf-8"))
        self.z += distance

    def variance(self, image):
        bg = cv2.GaussianBlur(image, (11, 11), 0)
        v = cv2.Laplacian(bg, cv2.CV_64F).var()
        return v
    def roixy_4x(self,x,y):
            self.movexclock(x)
            sleep(3)
            self.moveyc(y)
    def roixy(self,x,y):
            self.movexclock(x)
            sleep(8)
            self.moveyc(y)
          
    def init_pos(self,xi,yi):
            self.movexanticlock(xi)
            sleep(3)
            self.movey(yi)

    def auto(self, obj_value):
        z_positions = []
        variances = []
        max_variance = 0
        max_z = self.z

        self.camera.start_preview()
        self.ser.reset_input_buffer()
        sleep(1)

        self.camera.resolution = (320, 240)
        self.camera.framerate = 24
        
        if (obj_value ==4):
                step_size=50
        elif(obj_value==10):
                step_size=20
        elif(obj_value==40):
                step_size=5
                

        for i in range(21):
            stream = io.BytesIO()
            self.camera.capture(stream, format='jpeg')
            stream.seek(0)
            image = np.frombuffer(stream.getvalue(), dtype=np.uint8)
            image = cv2.imdecode(image, cv2.IMREAD_COLOR)

            current_variance = self.variance(image)
            variances.append(current_variance)
            z_positions.append(self.z)

            if current_variance > max_variance:
                max_variance = current_variance
                max_z = self.z

            if i < 20 and current_variance >= max_variance:  # Move motor only if variance keeps improving
                self.movezclock(step_size)
                sleep(1)
            else:
                break  # Stop if variance does not improve

        # Adjust to the position with the maximum variance
        adjust_steps = self.z - max_z
        if adjust_steps > 0:
            self.movezanticlock(adjust_steps)
        else:
            self.movezclock(abs(adjust_steps))

        # Capture the final focused image at high resolution



















        self.camera.resolution = (1920, 1088)
        sleep(2)  # Allow time for the camera to adjust
        stream = io.BytesIO()
        self.camera.capture(stream, format='jpeg')
        stream.seek(0)
        high_res_image = np.frombuffer(stream.getvalue(), dtype=np.uint8)
        high_res_image = cv2.imdecode(high_res_image, cv2.IMREAD_COLOR)

        # Create directories for different objectives and save image with date and time
        base_dir = "/home/pi/Downloads/autoscan"
        objective_dir = os.path.join(base_dir, f"{obj_value}X")
        os.makedirs(objective_dir, exist_ok=True)
        current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        image_path = os.path.join(objective_dir, f"focusedimage_{current_time}.jpg")
        cv2.imwrite(image_path, high_res_image)
        
        sleep(2)
        print(variances)
        print(z_positions)
        print(max_z)
        print(max_variance)

if __name__ == "__main__":
    af = Autofocus()
    
    ##init###
    #####xcclk=350
    #####ycclk=150
    print("Intialization Starting Please Wait......")
    xi=350
    yi=200
    af.init_pos(xi,yi)
    sleep(5)
    af.movezclock(20000)
    
    print("Intialization Stops for x,y z in Process")
    print("Please enter the Objective value 4 ,10 or 40")
    in_obj = int(input())
    
    if in_obj == 4:
        af.movezanticlock(3500)
        ## goto roi
        sleep(5)
        x=345
        y=175
        
        af.roixy_4x(x,y)
        sleep(2)
    elif in_obj == 10:
        af.movezanticlock(13852)
        sleep(16)
        x=345
        y=175
        
        af.roixy(x,y)
        sleep(4)
     
    ## 12_july 40X acode added
    elif in_obj == 40:
            af.movezanticlock(16650) # delay was given for 10X 
            sleep(22) # delay of ack_from Motor
    af.auto(in_obj)

