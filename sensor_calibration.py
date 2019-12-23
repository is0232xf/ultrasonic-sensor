# -*- coding: utf-8 -*-
"""
Created on Mon Jan  7 19:51:45 2019

@author: FujiiChang
"""
import time
import threading
import transrate_g_to_ms2 as trans_accel
import mpu6050_threading as mpu
from statistics import mean, variance
from threading import Timer
# import gyrosensor_kalmanfilternsor as G_Kalman

sensor_threrad = threading.Thread(target=mpu.run_mpu6050, args=())
sensor_threrad.daemon = True
sensor_threrad.start()

class RepeatedTimer(Timer):
    def __init__(self, interval, function, args=[], kwargs={}):
        Timer.__init__(self, interval, self.run, args, kwargs)
        self.thread = None
        self.function = function
        self.time = []

    def run(self):
        self.thread = Timer(self.interval, self.run)
        self.thread.start()
        self.function(*self.args, **self.kwargs)

    def cancel(self):
        if self.thread is not None:
            self.thread.cancel()
            self.thread.join()
            del self.thread

class Sensor():
    def __init__(self):
        self.gx = [0]
        self.gy = [0]
        self.gz = [0]
        self.ax = [0]
        self.ay = [0]
        self.az = [0]

    def append_sensor_data(self, sensor_data):
        self.gx.append(sensor_data[0])
        self.gy.append(sensor_data[1])
        self.gz.append(sensor_data[2])
        self.ax.append(trans_accel.trans_g_ms2(sensor_data[3]))
        self.ay.append(trans_accel.trans_g_ms2(sensor_data[4]))
        self.az.append(trans_accel.trans_g_ms2(sensor_data[5]))
    
IMU = Sensor()
         
def culc_angle(angle_rate):
    angle = sum(angle_rate)
    return angle

def get_sensor_data():
    gyro_x, gyro_y, gyro_z = mpu.get_gyro_data()
    accl_x, accl_y, accl_z = mpu.get_accl_data()
    sensor_data = [gyro_x, gyro_y, gyro_z, accl_x, accl_y, accl_z]
    IMU.append_sensor_data(sensor_data)
    
# Output data to screen
def sensors_calibration():
    sensor_thread = RepeatedTimer(0.085, get_sensor_data)
    sensor_thread.start()
    time.sleep(30)
    sensor_thread.cancel()
    mean_gyro_x = mean(IMU.gx)
    mean_gyro_y = mean(IMU.gy)
    mean_gyro_z = mean(IMU.gz)
    mean_accel_x = mean(IMU.ax)
    mean_accel_y = mean(IMU.ay)
    mean_accel_z = mean(IMU.az)

    var_gyro_x = variance(IMU.gx)
    var_gyro_y = variance(IMU.gy)
    var_gyro_z = variance(IMU.gz)
    var_accel_x = variance(IMU.ax)
    var_accel_y = variance(IMU.ay)
    var_accel_z = variance(IMU.az)

    print("----------------------------------------")
    print("avg gyro x error: ", mean_gyro_x)
    print("avg gyro y error: ", mean_gyro_y)
    print("avg gyro z error: ", mean_gyro_z)

    print("variance gyro x error: ", var_gyro_x)
    print("variance gyro y error: ", var_gyro_y)
    print("variance gyro z error: ", var_gyro_z)

    print("avg accel x error: ", mean_accel_x)
    print("avg accel y error: ", mean_accel_y)
    print("avg accel z error: ", mean_accel_z)

    print("variance accel x error: ", var_accel_x)
    print("variance accel y error: ", var_accel_y)
    print("variance accel z error: ", var_accel_z)
    print("----------------------------------------")
    return mean_gyro_x, mean_gyro_y, mean_gyro_z, mean_accel_x, mean_accel_y, mean_accel_z
