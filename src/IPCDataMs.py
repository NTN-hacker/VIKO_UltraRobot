import mmap
import struct
import cv2
import numpy as np
import json
import sys, time
class IPCData:
    def __init__(self, img_size=(480, 640)):
        self.img_size = img_size
        self.shm_img = mmap.mmap(-1, img_size[0] * img_size[1] * 3, tagname="Local\\ImgAbove")
        self.shm_img_below = mmap.mmap(-1, img_size[0] * img_size[1] * 3, tagname="Local\\ImgBelow")
        #self.shm_data = mmap.mmap(-1, struct.calcsize('ff'), tagname="Local\\Data")
        self.shm_map = mmap.mmap(-1, 1024, tagname="Local\\TriggerVision")  
        self.shm_status = mmap.mmap(-1, 1024, tagname="Local\\VisionStatus")  

        self.robot_position_size = None
        self.shm_rospos = None
        self.shm_coord = None

        


    def send_frame(self, img_np, float1=-1, float2=-1):
        # print('send_frame')
        img_bgr = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)  
        _, img_encoded = cv2.imencode('.jpg', img_bgr)
        self.shm_img.seek(0)
        self.shm_img.write(img_encoded.tobytes())

    def send_chart(self, keyid, valContent: list):
        """valContent: list[[float, str],]"""

        self.shm_chart = mmap.mmap(-1, 8*260+4, tagname="Local\\ChartInspection")
        if not isinstance(keyid, int) or keyid < 0:
            raise ValueError("Status key must be a non-negative integer")
        self.shm_chart.seek(0)
        keyidEndcode = struct.pack('I', keyid)
        self.shm_chart.write(keyidEndcode.ljust(4,b'\x00'))
        id = 1
        for chartVal, chartContent in valContent:
            if (id >8): break
            id += 1
            chartValEncode = struct.pack("f",chartVal)
            ChartcontentEncode = str(chartContent).encode('utf-8')
            self.shm_chart.write(chartValEncode.ljust(4,b'\x00'))
            self.shm_chart.write(ChartcontentEncode.ljust(256,b'\x00'))
        for _ in range(id, 9):
            chartValEncode = struct.pack("f", 0.0)
            ChartcontentEncode = b'\x00' * 256
            self.shm_chart.write(chartValEncode.ljust(4, b'\x00'))
            self.shm_chart.write(ChartcontentEncode.ljust(256, b'\x00'))

        self.shm_chart.close()

    def send_robot_positions(self, key, robnb, positions):
        robnb +=1
        self.robot_position_size = robnb * 6 * struct.calcsize('f') # 50 ros pos
        self.shm_rospos = mmap.mmap(-1, self.robot_position_size, tagname="Local\\Robpos")
        # Ensure positions are valid
        id = 0
        for pos in positions:
            id+=1
            if type(pos) == None:
                print('nontype',pos, '-', id)
                break
            if len(pos) != 6:
                print(f"Each robot position must contain exactly 6 floats. Invalid position: {pos}")
        
        self.shm_rospos.seek(0)  # Clean the data
        
        # Write key and robnb
        self.shm_rospos.write(struct.pack('I', key))  # Write key as unsigned int
        self.shm_rospos.write(struct.pack('I', robnb))  # Write robnb as unsigned int
        
        # Write all positions
        for pos in positions:            
            self.shm_rospos.write(struct.pack('6f', *pos))  # Write each position as 6 floats
        print ("ROBOT COOR are sended to IPC")

    def send_frame_below(self, keyid, img_np, float1=-1, float2=-1):
        # print('send_frame_bl')
        if not isinstance(keyid, int) or keyid < 0:
            raise ValueError("Status key must be a non-negative integer")
        status_key_encoded = struct.pack('I', keyid)

        img_bgr = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)  
        _, img_encoded = cv2.imencode('.jpg', img_bgr)

        self.shm_img_below.seek(0)
        self.shm_img_below.write(status_key_encoded.ljust(4, b'\x00'))  # 4 bytes for the key (unsigned int)
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

        # if not isinstance(rownb, int) or rownb < 0 or rownb > 3072:
        #     raise ValueError("Row number must be between 0 and 3072")
        total_size = 4 + 4 + 1024 * rownb * struct.calcsize('f')
        # Ensure key and rownb are written first
        i = 0
        while i<10:
            i+=1
            try:
                self.shm_coord = mmap.mmap(-1, total_size, tagname="Local\\Coor3DMesh")
                self.shm_coord.seek(0)
                self.shm_coord.write(struct.pack('I', key))  # Write key as unsigned int
                self.shm_coord.write(struct.pack('I', rownb))  # Write rownb as unsigned int
                print ("sended 3Ddata to IPC: ", i)
                break
            except:
                print ('error in send_3Ddata: ', i)
                continue
            
        # Write the data rows
        for row in data:
            # print(len(row))
            try:
                if len(row) != 1024:
                    raise ValueError("Each row must contain exactly 1024 floats.")
                self.shm_coord.write(struct.pack(f'{len(row)}f', *row))
                print("Done")
            except:
                print ("ERROR while sending 3Ddata to IPC: ", i)
                return False
        return True
        # # Fill the remaining space with -99999.0 if data is less than 3072 rows
        # remaining_rows = rownb - len(data)
        # if remaining_rows > 0:
        #     filler = [-99999.0] * 1024
        #     for _ in range(remaining_rows):
        #         self.shm_coord.write(struct.pack('1024f', *filler))

    def send_3Ddata_close(self):
        self.shm_coord.close()

    def sendLIDARcs(tlength: int):
        # Create a memory-mapped file with 4 bytes size
        shm_lidarcs = mmap.mmap(-1, 4, tagname="Local\\LIDAR_LASER_START")  
        
        # Encode the integer tlength into 4 bytes
        tlength_encoded = struct.pack('I', tlength)
        
        # Write the encoded integer to the memory-mapped file
        shm_lidarcs.seek(0)
        shm_lidarcs.write(tlength_encoded) 
        
        if tlength:
            sys.stdout.write("sended tlength to c#")
            sys.stdout.flush()
            sys.stdout.write('\r')
        else:
            print("stop sending tlength")

    def get_lidar_data():
        map_name = "Local\\LIDAR_LAEER_RESULT"  # Tên IPC của bộ nhớ ánh xạ
        map_size = 1024 * 2016 * 8 * 10  # Kích thước bộ nhớ ánh xạ, điều chỉnh cho phù hợp với dữ liệu thực tế

        while True:
            with mmap.mmap(-1, map_size, tagname=map_name) as mm:
                data_bytes = mm[:].rstrip(b'\x00')  # Bỏ các byte \x00 dư thừa
                if data_bytes:
                    lidar_data = json.loads(data_bytes.decode('utf-8'))                    
                    # Làm sạch nội dung của bộ nhớ ánh xạ
                    try:
                        mm.seek(0)  # Di chuyển con trỏ về đầu bộ nhớ
                        mm.write(b'\x00' * map_size)  # Ghi các byte mặc định vào bộ nhớ
                    except Exception as e:
                        print(f"Error cleaning memory-mapped object: {e}")
                    return True,lidar_data
                else:
                    # In "loading..." trên cùng một dòng
                    sys.stdout.write("loading...")
                    sys.stdout.flush()
                    time.sleep(1)  # Đợi 1 giây trước khi kiểm tra lại
                    sys.stdout.write('\r')  # Di chuyển con trỏ về đầu dòng
                    return False,None





    def close(self):
        """
        Closes the shared memory objects.
        """
        self.shm_img.close()
        self.shm_img_below.close()
        self.shm_map.close()
        self.shm_rospos.close()
