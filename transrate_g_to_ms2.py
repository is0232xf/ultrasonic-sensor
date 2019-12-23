# -*- coding: utf-8 -*-
"""
Created on Thu Apr  25 12:13:08 2019

@author: FujiiChang
"""

# 加速度センサの重力加速度をメートル法に変換する関数
def trans_g_ms2(accel):
	g = 9.80665
	accel = accel * g
	return accel 
