from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
import numpy as np

def func(x, a, b, c):
    return a * np.exp(-b * x) + c

xdata = []
ydata = []
while(True):
    n = float(input())
    if(n < 0):
        break
    xdata.append(n)

for i in range(len(xdata)):
    ydata.append(float(input()))

xdata = np.array(xdata)
xdata -= xdata[0]
plt.plot(xdata, ydata, 'b-')
popt, pcov = curve_fit(func, xdata, ydata)
# popt数组中，三个值分别是待求参数a,b,c
y2 = [func(i, popt[0], popt[1], popt[2]) for i in xdata]
plt.plot(xdata, y2, 'r--')
print(popt)
print("a = %f" % (-popt[0]*popt[1]))
plt.show()
