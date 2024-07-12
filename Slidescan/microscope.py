import serial
import cv2 
from time import sleep
from datetime import datetime
import subprocess
import logging
from picamera import PiCamera
from PIL import Image
import io
import numpy as np
import os

class Microscope:
    def __init__(self, port='/dev/ttyUSB0', baud_rate=9600, timeout=1):
        self.board = serial.Serial(port, baud_rate, timeout=timeout)
        self.camera = PiCamera()
        self.x = 0
        self.y = 0
        self.z = 0
        self.scan_count = 0

    def movexclock(self, distance):
        self.board.write("xclk,{}".format(distance).encode())
        self._wait_for_completion()
        self.x += distance

    def movexanticlock(self, distance):
        self.board.write("xcclk,{}".format(distance).encode())
        self._wait_for_completion()
        self.x -= distance

    def movey(self, steps):
        self.board.write("ycclk,{}".format(steps).encode())
        self._wait_for_completion()
        self.y += steps

    def movezclock(self, distance):
        self.board.write("zclk,{}".format(distance).encode())
        self._wait_for_completion()
        self.z -= distance

    def movezanticlock(self, distance):
        self.board.write("zcclk,{}".format(distance).encode())
        self._wait_for_completion()
        self.z += distance

    def _wait_for_completion(self):
        while True:
            data = self.board.readline()
            if data == b'Done\r\n':
                break
        self.board.reset_input_buffer()

    def calculate_variance(self, image_bytes):
        image = Image.open(image_bytes)
        image = image.convert("L")
        image_array = np.array(image)
        variance = np.var(image_array)
        return variance

    def variance(self, image):
        bg = cv2.GaussianBlur(image, (11, 11), 0)
        v = cv2.Laplacian(bg, cv2.CV_64F).var()
        return v

    def autofocus(self):
        logging.info("[Autofocus] Starting AutoFocus")
        var = []
        ze = []
        
        self.board.write("init".encode())
        self._wait_for_completion()
        
        self.board.write("zcclk,{}".format(13500).encode())
        self.z += 14500
        self._wait_for_completion()
        
        for i in range(25):
            image = self.capture_image()
            var.append(self.variance(image))
            
            self.board.write("zcclk,{}".format(50).encode())
            self.z += 50
            self._wait_for_completion()
            ze.append((i + 1) * 50)
        
        var = np.array(var)
        l = np.argmax(var)
        
        self.board.write("zclk,{}".format(1290 - l * 50).encode())
        self.z -= 1290 - l * 50
        self._wait_for_completion()
    
    
    ###### 10_July_2024_Start########
    
    def auto(self):
        obj_value=4
        z_positions = []
        variances = []
        max_variance = 0
        max_z = self.z

        self.camera.start_preview()
        self.board.reset_input_buffer()
        sleep(1)

        self.camera.resolution = (320, 240)
        self.camera.framerate = 24
        
        if (obj_value ==4):
                step_size=50
        else:
                step_size=20
                

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

    
    
    
    ###############10July_2024_End############
    

    def capture_image(self):
        image = np.empty((240 * 320 * 3), dtype=np.uint8)
        self.camera.capture(image, "bgr")
        return image.reshape((240, 320, 3))

    def scan(self):
        cur_time = datetime.now()
        dir_path = "/home/pi/Downloads/Images_Scan/scan_{}/".format(cur_time.strftime("%Y%m%d_%H%M"))
        subprocess.run(["mkdir", dir_path])
        logging.info("[Scan] First Autofocus")
        #10_july_24#
        self.auto()
        
        #self.autofocus()

        for i in range(10):
            for j in range(15):
                if j == 7:
                    ####10_july_24##
                    self.auto()
                    #self.autofocus()

                if i % 2 == 0:
                    self.capture_and_save_image(dir_path, i, j)
                    self.movexclock(15)
                else:
                    self.capture_and_save_image(dir_path, i, 14 - j)
                    self.movexanticlock(15)

            self.movey(12)
            logging.info("[Scan] Changing y Pos")
        #######10_July_24############
        #self.board.write("init".encode())
        #self._wait_for_completion()
        self.stitch_images(dir_path, dir_path + 'complete_slide.jpg', 10, 15)

    def capture_and_save_image(self, dir_path, i, j):
        self.camera.capture("{}/imagerow{},{}.jpg".format(dir_path, i, j))
        self.scan_count += 1

    def stitch_images(self, image_paths, output_path, rows, cols):
        images = []
        max_width = 0
        max_height = 0

        for i in range(rows):
            for j in range(cols):
                image = Image.open(image_paths + "imagerow{},{}.jpg".format(i, j))
                images.append(image)
                max_width = max(max_width, image.width)
                max_height = max(max_height, image.height)

        stitched_width = max_width * cols
        stitched_height = max_height * rows
        stitched_image = Image.new('RGB', (stitched_width, stitched_height), (255, 255, 255))

        for i, image in enumerate(images):
            row = i // cols
            col = i % cols
            x = col * max_width
            y = row * max_height
            stitched_image.paste(image, (x, y))

        stitched_image.save(output_path)

if __name__ == "__main__":
    microscope = Microscope()
    # Example usage:
    microscope.scan()
    # microscope.autofocus()

