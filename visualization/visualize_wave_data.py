# -*- coding: utf-8 -*-
"""
Created on Mon Dec 23 11:40:49 2019

@author: FujiiChang
"""

import matplotlib.pyplot as plt
import numpy as np

# Data for plotting
data = np.genfromtxt('../csv/2019_12_20_12_55_52.csv', delimiter=',',
                  names=True, dtype=None, encoding='utf-8')
distance = []
num = []
for i in range(len(data)):
    value = data[i][4]
    if value > 100:
        value = 52 # average of distance data
    distance.append(value-52)

for n in range(len(distance)):
    num.append(n)

fig, ax = plt.subplots()

ax.plot(num, distance)

ax.set(xlabel='time (ms)', ylabel='height (cm)',
       title='Wave height')
ax.grid()

fig.savefig("test.png")
plt.show()
