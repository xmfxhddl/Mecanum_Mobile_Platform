#! /usr/bin/env python

import pickle
from matplotlib import pyplot as plt
import numpy as np

with open('보내준 theta_data.pickle의 경로를 입력하시오', 'rb') as th:
    theta_list = pickle.load(th, encoding='latin1')
    

count = len(theta_list)

for i in range(0, count):
    print(theta_list[i])

