#!/usr/bin/python3
import math
import numpy as np
import matplotlib.pyplot as plt
from random import uniform

eq1_current = lambda x: (0.4629629629629629 * x) - 1.574074074074074
eq2_current = lambda x: (12.49999999999999 * x) - 48.99999999999996
eq3_current = lambda x: (749.0 * x) - 2995.0

def fake_mos(x):
    if(x <= 3.4):
        return 0
    elif(x <= 3.94):
        return eq1_current(x)
    elif(x <= 4.0):
        return eq2_current(x)
    else:
        return eq3_current(x)

v_gs = np.linspace(0.0,5.0,10000)
v_gs_q = np.linspace(0.0,5.0,100)

noiseV = 2/1000
noiseV = noiseV/2

plt.plot(v_gs,[fake_mos(x) for x in v_gs])
# plt.plot(v_gs,[fake_mos(x) for x in v_gs])
plt.errorbar(v_gs_q,[fake_mos(x) for x in v_gs_q],noiseV)
plt.show()