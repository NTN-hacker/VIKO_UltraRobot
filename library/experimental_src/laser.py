import ctypes as ct
import math as m
import time
import cv2
import numpy as np
import pyllt as llt
from datetime import datetime
from scipy.ndimage import median_filter
from config import config as CFG
from matplotlib import pyplot as plt


class Laser:
    def __init__(self) -> None:
        self.hllt = llt.create_llt_device(llt.TInterfaceType.INTF_TYPE_ETHERNET)
        self.IDLE_TIME = CFG.IDLE_TIME
        self.scanner_type = ct.c_int(0)
        self.container_size = CFG.CONTAINER_SIZE
        self.available_resolutions = (ct.c_uint * 4)()
        self.available_interfaces = (ct.c_uint * 6)()
        self.lost_profiles = ct.c_int(0)
        self.exposure_time = CFG.EXPOSURE_TIME
        self.null_ptr_short = ct.POINTER(ct.c_ushort)()
        self.null_ptr_int = ct.POINTER(ct.c_uint)()

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
        ret = llt.set_resolution(self.hllt, self.resolution)
        if ret < 1:
            raise ValueError(f"Error setting resolution: {ret}")

        self.profile_buffer = (ct.c_ubyte * (self.resolution * 2 * self.container_size))()
        self.X_value = (ct.c_double * self.resolution)()
        self.Z_value = (ct.c_double * (self.resolution * self.container_size))()
        self.intensities = (ct.c_ushort * self.resolution)()

        ret = llt.get_llt_type(self.hllt, ct.byref(self.scanner_type))
        if ret < 1:
            raise ValueError(f"Error scanner type: {ret}")

        ret = llt.set_profile_config(self.hllt, llt.TProfileConfig.CONTAINER)
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

        rec_log2 = 1.0 / m.log(2.0)
        container_resolution = m.floor((m.log(self.resolution) * rec_log2) + 0.5)
        ret = llt.set_feature(
            self.hllt, 
            llt.FEATURE_FUNCTION_PROFILE_REARRANGEMENT,
            llt.CONTAINER_DATA_Z | llt.CONTAINER_STRIPE_1 | (container_resolution << 12)
        )
        if ret < 1:
            raise ValueError(f"Error setting rearrangement: {ret}")

        ret = llt.set_profile_container_size(self.hllt, 0, self.container_size)
        if ret < 1:
            raise ValueError(f"Error setting profile container size: {ret}")

    def start_scan(self):
        print("Start transfer!")
        ret = llt.transfer_profiles(self.hllt, llt.TTransferProfileType.NORMAL_CONTAINER_MODE, 1)
        if ret < 1:
            raise ValueError(f"Error starting transfer profiles: {ret}")
        print(f"Time transfer: {CFG.TIME_SCAN}s")
        print("Transfering...")

        time.sleep(CFG.SLEEP)
        print("Finish transfer!")

        print("Start get profile!")
        ret = llt.get_actual_profile(self.hllt, self.profile_buffer, len(self.profile_buffer), llt.TProfileConfig.CONTAINER, ct.byref(self.lost_profiles))
        if ret != len(self.profile_buffer):
            print(f"Error getting profile buffer data: {ret}")
        print("Finish get profile!")
        print("Finish scanning!")

        ret = llt.transfer_profiles(self.hllt, llt.TTransferProfileType.NORMAL_CONTAINER_MODE, 0)
        if ret < 1:
            raise ValueError(f"Error stopping transfer profiles: {ret}")

        return self.process_data()

    def process_data(self):
        self.Z = np.frombuffer(self.profile_buffer, dtype='>H').reshape((self.container_size, self.resolution))
        print("Z")
        print(self.Z)
        print(f"Lines: {len(self.Z)}")

        Z_filtered = self.Z[~np.any((self.Z == 0) | (self.Z > 36000), axis=1)]
        Z_inverted = np.flipud(Z_filtered)
        Z_processed = median_filter(Z_inverted, size=3)

        print("\nZ_processed")
        print(Z_processed)
        print(f"Lines: {len(Z_processed)}")

        x = np.linspace(0, self.resolution, self.resolution)
        y = np.linspace(0, len(Z_processed), len(Z_processed))
        X, Y = np.meshgrid(x, y)

        fig = plt.figure()
        fig.subplots_adjust(wspace=0.3)
        plt.pcolormesh(X, Y, Z_processed)
        plt.colorbar()
        fig.canvas.draw()
        fig_np = np.array(fig.canvas.renderer.buffer_rgba())

        current_time = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

        return self.convert_to_grayscale_image(Z_processed, current_time)

    def convert_to_grayscale_image(self, Z_processed, current_time):
        min_val = np.min(Z_processed)
        max_val = np.max(Z_processed)

        normalized_Z = (Z_processed - min_val) / (max_val - min_val)
        gray_image = np.stack((normalized_Z), axis=-1)
        gray_image = cv2.resize(gray_image, (640, 640), cv2.INTER_CUBIC)
        image_rgb = (gray_image * 255).astype(np.uint8)
        image_rgb = cv2.cvtColor(image_rgb, cv2.COLOR_BGR2RGB)

        cv2.imwrite(f'laser/depthmap/datascan-python_{current_time}.png', image_rgb)
        cv2.imwrite(f'laser/depthmap/datascan-python.png', image_rgb)

        return self.apply_shading(image_rgb, current_time)

    def apply_shading(self, image_rgb, current_time):
        grad_x, grad_y, _ = np.gradient(image_rgb)
        slope = np.pi / 2. - np.arctan(np.sqrt(grad_x ** 2 + grad_y ** 2))
        aspect = np.arctan2(-grad_y, grad_x)

        azimuth = CFG.AZIMUTH
        altitude = CFG.ALTITUDE

        azimuth_rad = np.radians(azimuth)
        altitude_rad = np.radians(altitude)

        shaded = (np.sin(altitude_rad) * np.sin(slope) +
                  np.cos(altitude_rad) * np.cos(slope) * np.cos(azimuth_rad - np.pi / 2. - aspect))

        shaded = (shaded - shaded.min()) / (shaded.max() - shaded.min())
        img_shaded = (shaded * 255).astype(np.uint8)
        image_rgb_shaded = cv2.cvtColor(img_shaded, cv2.COLOR_BGR2RGB)

        print('shape ', image_rgb_shaded.shape)
        cv2.imwrite(f'laser/shaded/datascan-python_{current_time}.png', image_rgb_shaded)

        return image_rgb_shaded

    def get_data_scan(self):
        return self.num_profile, self.data_scan

    def disconnect(self):
        ret = llt.disconnect(self.hllt)
        if ret < 1:
            raise ConnectionAbortedError(f"Error while disconnecting: {ret}")

        ret = llt.del_device(self.hllt)
        if ret < 1:
            raise ConnectionAbortedError(f"Error while deleting: {ret}")

    @staticmethod
    def process_data_file(file_path: str) -> np.array:
        data = np.loadtxt(file_path)
        for row in data:
            first_non_zero = next((item for item in row if item != 0), 0)
            row[row == 0] = first_non_zero
        np.save(file_path.replace('.txt', '_processed.npy'), data)
        return data


if __name__ == '__main__':
    laser = Laser()
    laser.connect()
    laser.start_scan()
    laser.disconnect()
