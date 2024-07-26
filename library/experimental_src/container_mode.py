import ctypes as ct
import math as m
import time
import cv2
import numpy as np
import pyllt as llt
from enum import Enum
from datetime import datetime
from scipy.ndimage import median_filter
from config import config as CFG
from matplotlib import pyplot as plt

class Laser():
    def __init__(self) -> None:        
        self.hllt = llt.create_llt_device(llt.TInterfaceType.INTF_TYPE_ETHERNET)
        self.IDLE_TIME = CFG.IDLE_TIME
        self.scanner_type = ct.c_int(0)
        self.container_size = CFG.CONTAINER_SIZE

        # Init profile buffer and timestamp info
        self.available_resolutions = (ct.c_uint * 4)()
        self.available_interfaces = (ct.c_uint * 6)()
        self.lost_profiles = ct.c_int(0) 
        self.exposure_time = CFG.EXPOSURE_TIME

        # Null pointer if data not necessary
        self.null_ptr_short = ct.POINTER(ct.c_ushort)()
        self.null_ptr_int = ct.POINTER(ct.c_uint)()

        # Variable to store data after scan
        self.data_scan = []
        self.num_profile = 0
    
    def connect(self):
        # Get available interfaces
        ret = llt.get_device_interfaces_fast(self.hllt, self.available_interfaces, len(self.available_interfaces))
        if ret < 1:
            raise ValueError("Error getting interfaces : " + str(ret))

        ret = llt.set_device_interface(self.hllt, self.available_interfaces[0], 0)
        if ret < 1:
            raise ValueError("Error setting device interface: " + str(ret))
        
        # Connect
        ret = llt.connect(self.hllt)
        if ret < 1:
            raise ConnectionError("Error connect: " + str(ret))

        # Get available resolutions
        ret = llt.get_resolutions(self.hllt, self.available_resolutions, len(self.available_resolutions))
        if ret < 1:
            raise ValueError("Error getting resolutions : " + str(ret))

        # Set max. resolution
        self.resolution = self.available_resolutions[0]
        
        ret = llt.set_resolution(self.hllt, self.resolution)
        if ret < 1:
            raise ValueError("Error getting resolutions : " + str(ret))

        # Declare measuring data arrays
        self.profile_buffer = (ct.c_ubyte * (self.resolution * 2 * self.container_size))()
        self.X_value = (ct.c_double * self.resolution)()
        self.Z_value = (ct.c_double * (self.resolution * self.container_size))()
        self.intensities = (ct.c_ushort * self.resolution)()

        # Equidistant ranges
        x = np.linspace(0, self.resolution, self.resolution)
        # print('X value 2: ', x)
        y = np.linspace(0, self.container_size, self.container_size)
        self.X, self.Y = np.meshgrid(x, y)

        # Scanner type
        ret = llt.get_llt_type(self.hllt, ct.byref(self.scanner_type))
        if ret < 1:
            raise ValueError("Error scanner type: " + str(ret))

        # Set container to profile config
        ret = llt.set_profile_config(self.hllt, llt.TProfileConfig.CONTAINER)
        if ret < 1:
            raise ValueError("Error setting profile config: " + str(ret))

        # Set trigger
        ret = llt.set_feature(self.hllt, llt.FEATURE_FUNCTION_TRIGGER, llt.TRIG_INTERNAL)
        if ret < 1:
            raise ValueError("Error setting trigger: " + str(ret))

        # Set exposure time
        ret = llt.set_feature(self.hllt, llt.FEATURE_FUNCTION_EXPOSURE_TIME, self.exposure_time)
        if ret < 1:
            raise ValueError("Error setting exposure time: " + str(ret))

        # Set idle time
        ret = llt.set_feature(self.hllt, llt.FEATURE_FUNCTION_IDLE_TIME, self.IDLE_TIME)
        if ret < 1:
            raise ValueError("Error idle time: " + str(ret))
        
        # Set rearrangement (z only)
        rec_log2 = 1.0 / m.log(2.0)
        container_resolution = m.floor((m.log(self.resolution) * rec_log2) + 0.5)
        ret = llt.set_feature(self.hllt, llt.FEATURE_FUNCTION_PROFILE_REARRANGEMENT, llt.CONTAINER_DATA_Z | llt.CONTAINER_STRIPE_1 | (container_resolution << 12))
        if ret < 1:
            raise ValueError("Error setting rearrangement: " + str(ret))

        # Set container size
        ret = llt.set_profile_container_size(self.hllt, 0, self.container_size)
        if ret < 1:
            raise ValueError("Error setting profile container size: " + str(ret))

    def startScan(self):       
        # Start transmission
        print("Start transfer!")
        ret = llt.transfer_profiles(self.hllt, llt.TTransferProfileType.NORMAL_CONTAINER_MODE, 1)
        if ret < 1:
            raise ValueError("Error starting transfer profiles: " + str(ret))
        print(f"Time transfer: {CFG.TIME_SCAN}s")
        print("Transfering..")

        # Wait for container to fill
        time.sleep(CFG.SLEEP) 
        print("Finish transfer!")

        # Start get profile
        print("Start get profile!")
        ret = llt.get_actual_profile(self.hllt, self.profile_buffer, len(self.profile_buffer), llt.TProfileConfig.CONTAINER, ct.byref(self.lost_profiles))
        if ret != len(self.profile_buffer):
            print("Error get profile buffer data: " + str(ret))
        print("Finish get profile!")         
        print("Finish scanning!")

        # Stop transmission
        ret = llt.transfer_profiles(self.hllt, llt.TTransferProfileType.NORMAL_CONTAINER_MODE, 0)
        if ret < 1:
            raise ValueError("Error stopping transfer profiles: " + str(ret))        

        # Get Z value from buffer
        # Convert buffer to big-endian ushort values and reshape them to 2D array
        self.Z = np.frombuffer(self.profile_buffer, dtype='>H').reshape((self.container_size, self.resolution))

        print("Z")
        print(self.Z)
        print(f"Lines: {len(self.Z)}")

        # Process data
        # Remove lines contains value 0
        Z_remove_0_overflow = self.Z[~np.any((self.Z == 0), axis=1)]
        # Invert value of Z, flip each column in the up/down direction
        Z_inverted = np.flipud(Z_remove_0_overflow)  
        # This filter replaces each point with the median of its neighborhood, effectively removing outliers
        Z_processed = median_filter(Z_inverted, size=3)

        print("\nZ_processed")
        print(Z_processed)
        print(f"Lines: {len(Z_processed)}")

        # Draw figure
        x = np.linspace(0, self.resolution, self.resolution)
        y = np.linspace(0, len(Z_processed), len(Z_processed))
        X, Y = np.meshgrid(x, y)

        fig = plt.figure()
        fig.subplots_adjust(wspace=0.3)
        plt.pcolormesh(X, Y, Z_processed)
        plt.colorbar()
        fig.canvas.draw()
        fig_np = np.array(fig.canvas.renderer.buffer_rgba())        

        # Export to .TXT and .PNG file
        current_time = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        fig.savefig(f'laser/figure_{current_time}_{CFG.CONTAINER_SIZE}l-{CFG.EXPOSURE_TIME}e-{CFG.IDLE_TIME}i.png', dpi=300, bbox_inches='tight')
        # fig.savefig(f'result_temp.png', dpi=300, bbox_inches='tight')
        np.savetxt(f'laser/datascan-python_{current_time}_{CFG.CONTAINER_SIZE}l-{CFG.EXPOSURE_TIME}e-{CFG.IDLE_TIME}i_post-process.txt', self.Z, fmt='%d')
        np.savetxt(f'laser/datascan-python_{current_time}_{CFG.CONTAINER_SIZE}l-{CFG.EXPOSURE_TIME}e-{CFG.IDLE_TIME}i_pre-process.txt', Z_processed, fmt='%d')


        # Start convert profile to double value
        # print("Start convert profile to value!")
        # ret = llt.convert_profile_2_values(self.hllt, self.profile_buffer, self.resolution, llt.TProfileConfig.PROFILE, self.scanner_type, 0, 1, self.null_ptr_short,
        #                                    self.intensities, self.null_ptr_short, self.X_value, self.Z_value, self.null_ptr_int, self.null_ptr_int)
        # if ret & llt.CONVERT_X is 0 or ret & llt.CONVERT_Z is 0 or ret & llt.CONVERT_MAXIMUM is 0:
        #     raise ValueError("Error converting data: " + str(ret))
        # print("Finish convert profile to value!")

        # self.Z = np.ctypeslib.as_array(self.Z_value)
        # self.Z_value_double = self.Z.reshape(self.container_size, self.resolution)

        # Ship data to software to draw 3D
        self.num_profile = len(Z_processed)
        self.data_scan = Z_processed

        # Gray scale image
        min_val = np.min(Z_processed)
        max_val = np.max(Z_processed)

        # Normalized Z_processed to get value in range [0,1]
        normalized_Z = (Z_processed - min_val) / (max_val - min_val)

        # Convert to gray scale (same RGB)
        # gray_image = np.stack((normalized_Z, normalized_Z, normalized_Z), axis=-1)

        # gray_image = cv2.resize(gray_image, (640, 640), cv2.INTER_CUBIC)
        # image_rgb = (gray_image * 255).astype(np.uint8)  
        # image_rgb = cv2.cvtColor(image_rgb, cv2.COLOR_BGR2RGB) 
        gray_image = np.stack((normalized_Z), axis=-1)

        gray_image = cv2.resize(gray_image, (640, 640), cv2.INTER_CUBIC)
        image_rgb = (gray_image * 255).astype(np.uint8) 
        image_rgb = cv2.cvtColor(image_rgb, cv2.COLOR_BGR2RGB) 

        grad_x, grad_y, _ = np.gradient(image_rgb)

        slope = np.pi/2. - np.arctan(np.sqrt(grad_x**2 + grad_y**2))
        aspect = np.arctan2(-grad_y, grad_x)

        azimuth = CFG.AZIMUTH  
        altitude = CFG.ALTITUDE  

        azimuth_rad = np.radians(azimuth)
        altitude_rad = np.radians(altitude)

        shaded = np.sin(altitude_rad) * np.sin(slope) + \
                np.cos(altitude_rad) * np.cos(slope) * np.cos(azimuth_rad - np.pi/2. - aspect)

        shaded = (shaded - shaded.min()) / (shaded.max() - shaded.min())
        
        img_shaded = (shaded * 255).astype(np.uint8)
        image_rgb_shaded = cv2.cvtColor(img_shaded, cv2.COLOR_BGR2RGB) 

        print('shape ', image_rgb_shaded.shape)
        return image_rgb_shaded

    def get_data_scan(self):
        return self.num_profile, self.data_scan

    def disconnect(self):
        # Disconnect
        ret = llt.disconnect(self.hllt)
        if ret < 1:
            raise ConnectionAbortedError("Error while disconnect: " + str(ret))

        ret = llt.del_device(self.hllt)
        if ret < 1:
            raise ConnectionAbortedError("Error while delete: " + str(ret))

    def process_data() -> np.array:
        file_path = 'E:\\AutoRoboticInspection\\VIKO_UltraRobot\\laser\\datascan.txt'
        data = np.loadtxt(file_path)
        for row in data:
            first_non_zero = next((item for item in row if item != 0), 0)
            row[row == 0] = first_non_zero
        np.save('E:\\AutoRoboticInspection\\VIKO_UltraRobot\\laser\\datascan_processed.npy', data)
        
if __name__ == '__main__':
    laser = Laser(100, 5000)
    print(laser.container_size)
    print(laser.exposure_time)
