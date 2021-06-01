#!/usr/bin/python3
import math
import numpy as np
import matplotlib.pyplot as plt

eq1_current = lambda x: (2.5923060142355E-15) * math.exp(8.1407449923113 * x)
eq2_current = lambda x: 734.04065811719 * math.pow(x,2) - 5824.45578150388 * x + 11554.22728209
fake_mos = lambda x: eq1_current(x) if x < 3.96739 else eq2_current(x)

v_gs = np.linspace(3.4,4.0,1000)
fake_mos_data = [fake_mos(x) for x in v_gs]
plt.plot(v_gs,fake_mos_data)
plt.show()