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

    def auto(self, obj_value):
        z_positions = []
        variances = []
        max_variance = 0
        second_max_variance = 0
        max_z = self.z
        second_max_z = self.z

        self.camera.start_preview()
        self.ser.reset_input_buffer()
        sleep(1)

        self.camera.resolution = (320, 240)
        self.camera.framerate = 24
        
        if (obj_value == 4):
                step_size = 50
        elif (obj_value == 10):
                step_size = 20
        elif (obj_value == 40):
                step_size = 5

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
                second_max_variance = max_variance
                second_max_z = max_z
                max_variance = current_variance
                max_z = self.z
            elif current_variance > second_max_variance:
                second_max_variance = current_variance
                second_max_z = self.z

            if i < 20 and current_variance >= max_variance:  # Move motor only if variance keeps improving
                self.movezclock(step_size)
                sleep(1)
            else:
                break  # Stop if variance does not improve

        # Capture the first image at maximum variance position
        adjust_steps = self.z - max_z
        if adjust_steps > 0:
            self.movezanticlock(adjust_steps)
        else:
            self.movezclock(abs(adjust_steps))
        sleep(2)
        self.camera.resolution = (1920, 1080)
        stream = io.BytesIO()
        self.camera.capture(stream, format='jpeg')
        stream.seek(0)
        image_max_var = np.frombuffer(stream.getvalue(), dtype=np.uint8)
        image_max_var = cv2.imdecode(image_max_var, cv2.IMREAD_COLOR)

        # Capture the second image at second highest variance position
        adjust_steps = self.z - second_max_z
        if adjust_steps > 0:
            self.movezanticlock(adjust_steps)
        else:
            self.movezclock(abs(adjust_steps))
        sleep(2)
        self.camera.resolution = (1920, 1080)
        stream = io.BytesIO()
        self.camera.capture(stream, format='jpeg')
        stream.seek(0)
        image_second_var = np.frombuffer(stream.getvalue(), dtype=np.uint8)
        image_second_var = cv2.imdecode(image_second_var, cv2.IMREAD_COLOR)

        # Capture the third image at the stopping position
        sleep(2)
        self.camera.resolution = (1920, 1080)
        stream = io.BytesIO()
        self.camera.capture(stream, format='jpeg')
        stream.seek(0)
        image_third_var = np.frombuffer(stream.getvalue(), dtype=np.uint8)
        image_third_var = cv2.imdecode(image_third_var, cv2.IMREAD_COLOR)

        # Perform focus stacking
        images = [image_max_var, image_second_var, image_third_var]
        stacked_image = self.focus_stacking(images)

        # Save the final focused image
        base_dir = "/home/pi/Downloads/autoscan"
        objective_dir = os.path.join(base_dir, f"{obj_value}X")
        os.makedirs(objective_dir, exist_ok=True)
        current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        image_path = os.path.join(objective_dir, f"focusedimage_{current_time}.jpg")
        cv2.imwrite(image_path, stacked_image)
        
        sleep(2)
        print(variances)
        print(z_positions)
        print(max_z)
        print(max_variance)

    def focus_stacking(self, images):
        # Convert images to grayscale
        gray_images = [cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) for img in images]

        # Calculate the sharpness (Laplacian variance) of each pixel
        sharpness = np.stack([cv2.Laplacian(img, cv2.CV_64F) for img in gray_images], axis=-1)
        max_sharpness_index = np.argmax(sharpness, axis=-1)

        # Create an empty image for the focus-stacked result
        stacked_image = np.zeros_like(images[0])

        # For each pixel, choose the value from the image with the highest sharpness
        for i in range(stacked_image.shape[0]):
            for j in range(stacked_image.shape[1]):
                stacked_image[i, j] = images[max_sharpness_index[i, j]][i, j]

        return stacked_image

if __name__ == "__main__":
    af = Autofocus()
    
    af.movezclock(20000)
    
    print("Please enter the Objective value 4, 10, or 40")
    in_obj = int(input())
    
    if in_obj == 4:
        af.movezanticlock(3500)
    elif in_obj == 10:
        af.movezanticlock(13750)
        sleep(17)
    elif in_obj == 40:
        af.movezanticlock(16650)
        sleep(22)
    af.auto(in_obj)

