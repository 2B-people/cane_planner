#!/usr/bin/python
# -*- coding:utf-8 -*-

import math
import numpy as np
import matplotlib.pyplot as plt

def statTransit(state1,input,n):
    state2 = np.array([0.,0.,0.])
    
    yaw_new = state1[2] + input[2];
    state2[0] = state1[0] + math.cos(yaw_new) * input[0] - math.sin(yaw_new) * input[1];
    state2[1] = state1[1] + math.sin(yaw_new) * input[0] - math.pow(-1, n) * math.cos(yaw_new) * input[1];
    state2[2] = yaw_new 
    return state2


# six step input set
input_list = np.array([[40.,10.,0.],[40.,10.,0.2],[40.,10.,0.],[40.,10.,0.],[40.,10.,-0.2],[40.,10.,0.]])

start_state = np.array([0.,10.,0.])
temp_state = start_state
x_list = list()
y_list = list()
x_list.append(temp_state[0])
y_list.append(temp_state[1])
n = 0
for nm in input_list:
    print("--------input and state")
    print(nm)
    print(temp_state)
    temp_state = statTransit(temp_state,nm,n)
    x_list.append(temp_state[0])
    y_list.append(temp_state[1])
    n = n+1

#设置坐标轴刻度
my_x_ticks = np.arange(0, 200, 40)
my_y_ticks = np.arange(0, 60, 10)
plt.xticks(my_x_ticks)
plt.yticks(my_y_ticks)

plt.plot(x_list,y_list,'o')
plt.show()
