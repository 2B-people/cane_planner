# -*- coding: utf-8 -*-

import numpy as np
import csv
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

position = []
quaterntions = []
timestamp = []
tx_index = 5

file = open('vio.csv',newline='')
csvreader = csv.reader(file)
for row in csvreader:
        # print(row)
        string = "x:"+row[1]+' y:' + row[2]
        print(string)
        float(row[1])
