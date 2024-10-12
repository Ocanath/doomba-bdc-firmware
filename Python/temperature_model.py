import numpy as np
import matplotlib.pyplot as plt



t = np.linspace(0,100,100000)


Current = 1.0
Resistance = 1.2
CopperCoefficientofThermalResistivity = 0.00393 #percent resistance increase per degree celsius
Rth = 0.025	#degrees C per watt
CopperSpecificHeatCapacity = 0.385	#Joules per (gram degree c)
MassOfWindings = 0.5	#grams
Cth = CopperSpecificHeatCapacity*MassOfWindings	#joules per degree c


Temp = t*0
Pout = t*0 - 1000
TemperatureCurrent = 10
Temp[0] = TemperatureCurrent
TemperatureAmbient = 25
Pout[0] = (TemperatureCurrent - TemperatureAmbient)*Rth
for i in range(1,len(t)):
	dt = t[i] - t[i-1]
	dQin = (Current**2)*Resistance
	dQout = (TemperatureCurrent - TemperatureAmbient)*Rth
	dTemp = (dQin - dQout)/Cth
	TemperatureCurrent += dt*dTemp
	Temp[i] = TemperatureCurrent 
	Pout[i] = dQout

Power = Current**2*Resistance
AnalyticalTemperature = -((-(Power + Rth*TemperatureAmbient)/Rth) + (-Temp[0] + (Power + Rth*TemperatureAmbient)/Rth)*np.e**(-(Rth*t)/Cth))
GPTCheck = TemperatureAmbient + Power/Rth + (Temp[0] - (TemperatureAmbient + Power/Rth))*np.e**((-Rth*t)/Cth)


SteadyStateTemperature = TemperatureAmbient + Power/Rth	#wow simple

print("Num:", Temp[len(Temp)-1])
print("Analytical:", AnalyticalTemperature[len(AnalyticalTemperature)-1])
print("SteadyState:", SteadyStateTemperature)

print("Num:", Temp[0])
print("Analytical:", AnalyticalTemperature[0])


fig,ax = plt.subplots()
ax.plot(t,Temp)
ax.plot(t, AnalyticalTemperature)
ax.plot(t, GPTCheck)
# ax2 = ax.twinx()
# ax2.plot(t, Pout, color='r')
plt.show()