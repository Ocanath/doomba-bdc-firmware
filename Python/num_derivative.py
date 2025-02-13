import numpy as np
import matplotlib.pyplot as plt


t = np.linspace(0,2,1000)
sig = t**3


def num_derivative(x, y):
	dydx = y*0
	for i, xi in enumerate(x):	#for i = 1:len(x)
		if(i >= 1):	#throw out the first value because you have nothing to reference. in matlab, you need to change this 1 to a 2 because of indexing at 1
			dx = x[i] - x[i-1]
			dy = y[i] - y[i-1]
			dydx[i] = dy/dx
	return dydx



fix,ax = plt.subplots()
ax.plot(t,sig,label="x^3")
ax.plot(t, 3*t**2,label="3x^2")
ax.plot(t, num_derivative(t,sig),label="numerical first derivative of x^3")
handles, labels = ax.get_legend_handles_labels()
ax.legend(handles, labels)
plt.show()