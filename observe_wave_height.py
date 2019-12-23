# -*- coding: utf-8 -*-
"""
Created on Wed Jun 26 15:05:56 2019

@author: FujiiChang
"""
import csv
import datetime
import signal
import time
import dht11
import threading
import RPi.GPIO as GPIO
import mpu6050_threading as sensor
import numpy as np
import sensor_calibration as sense_calib
from threading import Timer
sensor_threrad = threading.Thread(target=sensor.run_mpu6050, args=())
sensor_threrad.daemon = True
sensor_threrad.start()

accl = []
sonic = []
timestamp = []
   
# TRIGとECHOのGPIO番号   
TRIG_PIN = 14
ECHO_PIN = 15
instance = dht11.DHT11(pin=18)

# 気温24[℃]の場合�E音速[cm/s]

GPIO.setwarnings(False)
# ピン番号をGPIOで持E��E
GPIO.setmode(GPIO.BCM)
# TRIG_PINを�E劁E ECHO_PINを�E劁E
GPIO.setup(TRIG_PIN,GPIO.OUT)
GPIO.setup(ECHO_PIN,GPIO.IN)

# get date time object
detail = datetime.datetime.now()
date = detail.strftime("%Y_%m_%d_%H_%M_%S")
# open csv file
file = open('./csv/'+ date +'.csv', 'a', newline='')
csvWriter = csv.writer(file)
csvWriter.writerow(['timestamp', 'ax', 'ay', 'az', 'distance'])

print("start gyro/accel calibration")
gyro_x_offset = 0 
gyro_y_offset = 0
gyro_z_offset = 0
accel_x_offset = 0
accel_y_offset = 0 
accel_z_offset = 0
gyro_x_offset, gyro_y_offset, gyro_z_offset, accel_x_offset, accel_y_offset, accel_z_offset = sense_calib.sensors_calibration()
offset = np.array([accel_x_offset, accel_y_offset, accel_z_offset])
print("finish calibration")

# HIGH or LOWの時計測
def pulseIn(PIN, start=1, end=0):
    if start==0: end = 1
    t_start = 0
    t_end = 0
    # ECHO_PINがHIGHである時間を計測
    while GPIO.input(PIN) == end:
        t_start = time.time()
        
    while GPIO.input(PIN) == start:
        t_end = time.time()
    return t_end - t_start

# 距離計測
def calc_distance(TRIG_PIN, ECHO_PIN, v=34000): 
    # TRIGピンめE.3[s]だけLOW
    GPIO.output(TRIG_PIN, GPIO.LOW)
    time.sleep(0.01)
    # TRIGピンめE.00001[s]だけ�E劁E趁E��波発封E        
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)
    # HIGHの時間計測
    t = pulseIn(ECHO_PIN)
    # 距離[cm] = 音速[cm/s] * 時間[s]/2
    distance = v * t/2
    return distance
 
# 距離計測(TRIGピン番号, ECHO_PIN番号, 計測回数, 音速[cm/s])

def accl_data(signal, frame):
    temp = sensor.get_temp_data()#instance.read()
    v = 33150 + 60*temp
    accl_x, accl_y, accl_z = sensor.get_accl_data()-offset*0.1
    distance = calc_distance(TRIG_PIN, ECHO_PIN, v)
    detail = datetime.datetime.now()
    date = detail.strftime("%Y_%m_%d_%H_%M_%S")
    accl.append(np.array([accl_x, accl_y, accl_z]))
    sonic.append(distance)
    timestamp.append(date)

def accl_signal():
    signal.signal(signal.SIGALRM, accl_data)
    signal.setitimer(signal.ITIMER_REAL, 1, 0.1)

def kill_signal_process(arg1, args2):
    pass

if __name__=='__main__': 
    try:  
        print("start")
        accl_signal()
        time.sleep(60)
        print("Done.")
        GPIO.cleanup()
        signal.signal(signal.SIGALRM, kill_signal_process)
        print(accl)
        for i in range(len(accl)):
            data = [timestamp[i], accl[i][0], accl[i][1], accl[i][2], sonic[i]]
            csvWriter.writerow(data)

    except KeyboardInterrupt:
        GPIO.cleanup()
        signal.signal(signal.SIGALRM, kill_signal_process)
        for i in range(len(accl)):
            csvWriter.writerow([timestamp[i], accl[i][0], accl[i][1], accl[i][2], sonic[i]])
       
