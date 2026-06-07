import numpy as np
import matplotlib.pyplot as plt



print((2**31 / (np.pi*2**12))/4)

"""
	Function which performs modulo on an input, centered at +/-half the specified range.
	If total_range is set to 2*pi, this function is equal to output = np.atan2(np.sin(input),np.cos(input))
"""
def wrap_symm(input, total_range):
	half_range = total_range/2
	return np.mod(input+half_range, total_range) - half_range


t = np.linspace(0,10,1000)
q1 = wrap_symm(t,np.pi*2)	#same as above
q2 = (q1/np.pi)*2**14
q3 = wrap_symm(q2, np.pi*2**12)

fig,ax = plt.subplots()
ax.plot(t,q2)
# ax2 = ax.twinx()
ax.plot(t, q3)


# plt.show()