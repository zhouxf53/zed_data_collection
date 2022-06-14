import sys
import time
import pyzed.sl as sl
import math
import threading


class TimestampHandler:
    def __init__(self):
        self.t_imu = sl.Timestamp()
        self.t_baro = sl.Timestamp()
        self.t_mag = sl.Timestamp()

    ##
    # check if the new timestamp is higher than the reference one, and if yes, save the current as reference
    def is_new(self, sensor):
        if (isinstance(sensor, sl.IMUData)):
            new_ = (sensor.timestamp.get_nanoseconds() > self.t_imu.get_nanoseconds())
            if new_:
                self.t_imu = sensor.timestamp
            return new_


def printSensorParameters(sensor_parameters):
    if sensor_parameters.is_available:
        print("*****************************")
        print("Sensor type: " + str(sensor_parameters.sensor_type))
        print("Max rate: " + str(sensor_parameters.sampling_rate) + " " + str(sl.SENSORS_UNIT.HERTZ))
        print("Range: " + str(sensor_parameters.sensor_range) + " " + str(sensor_parameters.sensor_unit))
        print("Resolution: " + str(sensor_parameters.resolution) + " " + str(sensor_parameters.sensor_unit))
        if not math.isnan(sensor_parameters.noise_density):
            print("Noise Density: " + str(sensor_parameters.noise_density) + " " + str(
                sensor_parameters.sensor_unit) + "/√Hz")
        if not math.isnan(sensor_parameters.random_walk):
            print("Random Walk: " + str(sensor_parameters.random_walk) + " " + str(
                sensor_parameters.sensor_unit) + "/s/√Hz")


def cap_image():
    prev_img = 0
    while True:
        if zed.grab(sl.RuntimeParameters()) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, sl.VIEW.LEFT, resolution=resolution)
            ts_img = zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_milliseconds()
            tmp = int(ts_img) - int(prev_img)
            print("%7.3f" % tmp)
            prev_img = ts_img


def cap_imu():
    prev = 0
    while True:
        if zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT) == sl.ERROR_CODE.SUCCESS:
            ts = zed.get_timestamp(sl.TIME_REFERENCE.CURRENT).get_milliseconds()
            # print("xxx ", int(ts) - int(prev))
            tmp = int(ts) - int(prev)
            print("%7.1f" % tmp)
            prev = ts
            if ts_handler.is_new(sensors_data.get_imu_data()):
                linear_acceleration = sensors_data.get_imu_data().get_linear_acceleration()
                angular_velocity = sensors_data.get_imu_data().get_angular_velocity()
                ts = zed.get_timestamp(sl.TIME_REFERENCE.CURRENT).get_milliseconds()
                tmp = int(ts) - int(prev)
                print("%7.1f" % tmp)
                prev = ts


if __name__ == '__main__':
    global thread_list
    thread_list = []
    ts_handler = TimestampHandler()

    init = sl.InitParameters(camera_resolution=sl.RESOLUTION.HD720,
                             depth_mode=sl.DEPTH_MODE.ULTRA,
                             coordinate_units=sl.UNIT.MILLIMETER,
                             coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP,
                             depth_minimum_distance=0.20)
    zed = sl.Camera()
    status = zed.open(init)

    sensors_data = sl.SensorsData()

    info = zed.get_camera_information()
    printSensorParameters(info.sensors_configuration.accelerometer_parameters)  # accelerometer configuration
    printSensorParameters(info.sensors_configuration.gyroscope_parameters)  # gyroscope configuration

    # Grab new frames and retrieve sensors data

    image = sl.Mat()
    resolution = sl.Resolution()
    resolution.width = 720 / 2
    resolution.height = 404 / 2

    thread_list.append(threading.Thread(target=cap_image))
    thread_list.append(threading.Thread(target=cap_imu))

    thread_list[0].start()
    thread_list[1].start()
