import ctypes as ct
import time
import cv2
import numpy as np
import pyllt as llt
from datetime import datetime
from scipy.ndimage import median_filter
from config import config as CFG
from matplotlib import pyplot as plt
import threading

global count
count = 0

class Laser:
    def __init__(self) -> None:
        self.hllt = llt.create_llt_device(llt.TInterfaceType.INTF_TYPE_ETHERNET)
        self.IDLE_TIME = CFG.IDLE_TIME
        self.scanner_type = ct.c_int(0)
        self.exposure_time = CFG.EXPOSURE_TIME
        self.null_ptr_short = ct.POINTER(ct.c_ushort)()
        self.null_ptr_int = ct.POINTER(ct.c_uint)()
        self.available_resolutions = (ct.c_uint * 4)()
        self.available_interfaces = (ct.c_uint * 6)()
        self.lost_profiles = ct.c_int()
        self.profile_buffer = None
        self.resolution = None
        self.data_width = 8
        self.start_data = 0

    def profile_callback(self, data, size, user_data):
        if user_data == 1:
            ct.memmove(self.profile_buffer, data, size)
            self.event.set()

    def connect(self):
        ret = llt.get_device_interfaces_fast(self.hllt, self.available_interfaces, len(self.available_interfaces))
        if ret < 1:
            raise ValueError(f"Error getting interfaces: {ret}")

        ret = llt.set_device_interface(self.hllt, self.available_interfaces[0], 0)
        if ret < 1:
            raise ValueError(f"Error setting device interface: {ret}")

        ret = llt.connect(self.hllt)
        if ret < 1:
            raise ConnectionError(f"Error connect: {ret}")

        ret = llt.get_resolutions(self.hllt, self.available_resolutions, len(self.available_resolutions))
        if ret < 1:
            raise ValueError(f"Error getting resolutions: {ret}")

        self.resolution = self.available_resolutions[0]

        ret = llt.set_resolution(self.hllt, 1024)
        print('Resolution ',ret)
        if ret < 1:
            raise ValueError(f"Error setting resolution: {ret} - The requested resolution is not supported ")

        self.profile_buffer = (ct.c_ubyte * (self.resolution * self.data_width))()
        self.X_value = (ct.c_double * self.resolution)()
        self.Z_value = (ct.c_double * self.resolution)()
        self.intensities = (ct.c_ushort * self.resolution)()

        ret = llt.get_llt_type(self.hllt, ct.byref(self.scanner_type))
        if ret < 1:
            raise ValueError(f"Error scanner type: {ret}")
        


        ret = llt.set_profile_config(self.hllt, llt.TProfileConfig.PARTIAL_PROFILE)
        if ret < 1:
            raise ValueError(f"Error setting profile config: {ret}")

        ret = llt.set_feature(self.hllt, llt.FEATURE_FUNCTION_TRIGGER, llt.TRIG_INTERNAL)
        if ret < 1:
            raise ValueError(f"Error setting trigger: {ret}")

        ret = llt.set_feature(self.hllt, llt.FEATURE_FUNCTION_EXPOSURE_TIME, self.exposure_time)
        if ret < 1:
            raise ValueError(f"Error setting exposure time: {ret}")

        ret = llt.set_feature(self.hllt, llt.FEATURE_FUNCTION_IDLE_TIME, self.IDLE_TIME)
        if ret < 1:
            raise ValueError(f"Error setting idle time: {ret}")

        self.partial_profile_struct = llt.TPartialProfile(0, self.start_data, self.resolution, self.data_width)
        ret = llt.set_partial_profile(self.hllt, ct.byref(self.partial_profile_struct))
        if ret < 1:
            raise ValueError(f"Error setting partial profile: {ret}")

        self.get_profile_cb = llt.buffer_cb_func(self.profile_callback)
        self.event = threading.Event()

        ret = llt.register_callback(self.hllt, llt.TCallbackType.C_DECL, self.get_profile_cb, 1)
        if ret < 1:
            raise ValueError(f"Error setting callback: {ret}")

    def start_scan(self):
        print("Start transfer!")
        ret = llt.transfer_profiles(self.hllt, llt.TTransferProfileType.NORMAL_TRANSFER, 1)
        if ret < 1:
            raise ValueError(f"Error starting transfer profiles: {ret}")
        print("Transfering...")

        # Open file to save profiles
        file = open("profile_data.txt", "w")

        def data_gen():
            while True:
                global count
                count+= 1
                print('Count scan ', count)
                self.event.wait()
                fret = llt.convert_part_profile_2_values(
                    self.hllt, self.profile_buffer, ct.byref(self.partial_profile_struct), self.scanner_type, 0, 1,
                    self.null_ptr_short, self.null_ptr_short, self.null_ptr_short, self.X_value, self.Z_value, self.null_ptr_int, self.null_ptr_int
                )
                if fret & llt.CONVERT_X == 0 or fret & llt.CONVERT_Z == 0:
                    raise ValueError("Error converting data: " + str(fret))

                # Save profile data to file
                for i in range(self.resolution):
                    file.write(f"{self.X_value[i]}\t{self.Z_value[i]}\n")

                file.write("\n")  # Separate profiles by an empty line

                self.event.clear()

        data_gen_thread = threading.Thread(target=data_gen)
        data_gen_thread.start()

        time.sleep(CFG.SLEEP)
        print("Finish transfer!")

        ret = llt.transfer_profiles(self.hllt, llt.TTransferProfileType.NORMAL_TRANSFER, 0)
        if ret < 1:
            raise ValueError(f"Error stopping transfer profiles: {ret}")

        data_gen_thread.join()
        file.close()
        print("Finish scanning!")

        result_image = cv2.imread('data/AI/Record_2024-05-07/20240507-104051354444-acA2440-35uc_22481496.png', cv2.IMREAD_COLOR)

        return result_image

    def disconnect(self):
        ret = llt.disconnect(self.hllt)
        if ret < 1:
            raise ConnectionAbortedError(f"Error while disconnecting: {ret}")

        ret = llt.del_device(self.hllt)
        if ret < 1:
            raise ConnectionAbortedError(f"Error while deleting: {ret}")

if __name__ == '__main__':
    laser = Laser()
    laser.connect()
    laser.start_scan()
    laser.disconnect()
