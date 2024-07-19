import mmap
import struct
import cv2
import numpy as np
class IPCData:
    def __init__(self, img_size=(480, 640)):
        self.img_size = img_size
        self.shm_img = mmap.mmap(-1, img_size[0] * img_size[1] * 3, tagname="Local\\ImgAbove")
        self.shm_img_below = mmap.mmap(-1, img_size[0] * img_size[1] * 3, tagname="Local\\ImgBelow")
        #self.shm_data = mmap.mmap(-1, struct.calcsize('ff'), tagname="Local\\Data")
        self.shm_map = mmap.mmap(-1, 1024, tagname="Local\\TriggerVision")  
        self.shm_status = mmap.mmap(-1, 1024, tagname="Local\\VisionStatus")  

        self.robot_position_size = 50 * 6 * struct.calcsize('f') # 50 ros pos
        self.shm_rospos = mmap.mmap(-1, self.robot_position_size, tagname="Local\\PosPytoCPP")
        self.shm_coord = None


    def send_frame(self, img_np, float1=-1, float2=-1):
        # print('send_frame')
        img_bgr = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)  
        _, img_encoded = cv2.imencode('.jpg', img_bgr)
        self.shm_img.seek(0)
        self.shm_img.write(img_encoded.tobytes())



    def send_robot_positions(self, positions):
        # print('send_pos')

        if len(positions) > 50:
            raise ValueError("Too many robot positions to store in shared memory.")
        
        self.shm_rospos.seek(0)
        for pos in positions:
            if len(pos) != 6:
                raise ValueError("Each robot position must contain exactly 6 floats.")
            self.shm_rospos.write(struct.pack('6f', *pos))
        
        # Fill the remaining space with -99999.0f
        remaining_positions = 50 - len(positions)
        if remaining_positions > 0:
            filler = [-99999.0] * 6
            for _ in range(remaining_positions):
                self.shm_rospos.write(struct.pack('6f', *filler))
        self.shm_rospos.close()
    def send_frame_below(self, img_np, float1=-1, float2=-1):
        # print('send_frame_bl')

        img_bgr = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)  
        _, img_encoded = cv2.imencode('.jpg', img_bgr)
        self.shm_img_below.seek(0)
        self.shm_img_below.write(img_encoded.tobytes())

    def send_status(self, status_key, status_value):
        # print('send_status')

        if not isinstance(status_key, int) or status_key < 0:
            raise ValueError("Status key must be a non-negative integer")
        status_key_encoded = struct.pack('I', status_key)
        status_value_encoded = str(status_value).encode('utf-8')
        self.shm_status.seek(0)
        self.shm_status.write(status_key_encoded.ljust(4, b'\x00'))  # 4 bytes for the key (unsigned int)
        self.shm_status.write(status_value_encoded.ljust(252, b'\x00'))  # 252 bytes for the value

    def get_data(self):
        trigger_list = list()
        self.shm_map.seek(0)
        data = self.shm_map.read(1024)       

        data_dict = {}
        num_pairs = len(data) // (40 + 5)  # 32 byte key + 4 byte int
        
        get_str = data[0:128].decode('utf-8').rstrip('\x00')            
    
        trigger_str = get_str.split(";")
  
        for idx, value in enumerate(trigger_str):
            try: 
                trigger = int(value.split(":")[-1])
                trigger_list.append(trigger)
            except: 
                pass
        # print(f'trigger:{trigger_list}')

        #clean data 
        # send [0, 0, 0..., 0] -0> shm_map
        # method: reference from send_3Ddata
        return trigger_list

    def send_3Ddata(self, key, rownb, data):
        """
        Send 3D data to shared memory.        
        :param key: Key to identify the data
        :param rownb: Number of rows in the data
        :param data: A 2D list of float values 1024*3072
        """
        if not isinstance(key, int) or key < 0:
            raise ValueError("Key must be a non-negative integer")

        if not isinstance(rownb, int) or rownb < 0 or rownb > 3072:
            raise ValueError("Row number must be between 0 and 3072")
        total_size = 4 + 4 + 1024 * rownb * struct.calcsize('f')
        # Ensure key and rownb are written first
        self.shm_coord = mmap.mmap(-1, total_size, tagname="Local\\Coor3DMesh")
        self.shm_coord.seek(0)
        self.shm_coord.write(struct.pack('I', key))  # Write key as unsigned int
        self.shm_coord.write(struct.pack('I', rownb))  # Write rownb as unsigned int
        
        # Write the data rows
        for row in data:
            if len(row) != 1024:
                raise ValueError("Each row must contain exactly 1024 floats.")
            self.shm_coord.write(struct.pack(f'{len(row)}f', *row))
        
        # Fill the remaining space with -99999.0 if data is less than 3072 rows
        remaining_rows = 3072 - len(data)
        if remaining_rows > 0:
            filler = [-99999.0] * 1024
            for _ in range(remaining_rows):
                self.shm_coord.write(struct.pack('1024f', *filler))

    def send_3Ddata_close(self):
        self.shm_coord.close()

    def close(self):
        """
        Closes the shared memory objects.
        """
        self.shm_img.close()
        self.shm_img_below.close()
        self.shm_map.close()
