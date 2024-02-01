import numpy as np


"""
    This function takes as input:
        RPM: The RPM of the motor
        radix_of_2pi: the number of bits which is multiplied by pi to obtain the fixedpoint representation of pi.
          For example, if you're using sin_12b, that means this number is 12. If you are using sin_14b, this number is 14.
        bits_per_container: size of the signed integer you're using to hold the number. 16,32,64, whatever.


        Returns number of MINUTES that will elaspe if the motor runs at that speed before overflow, when starting at q=0 and running for long time
"""
def get_overflow_minutes(RPM, radix_of_pi, bits_per_container):

    numticks = 2**(bits_per_container-1)-1
    revolutions_per_tick = 1/( 2**radix_of_pi*2*np.pi )
    seconds_per_revolution = 1/(RPM/60)

    revolutions_per_overflow = numticks * revolutions_per_tick #note that the last partial revolution will occur before the overflow event, so it won't be a valid revolution, but time of overflow will be accurate
    seconds_per_overflow = seconds_per_revolution * revolutions_per_overflow - seconds_per_revolution/2   #starting at q=0 means exactly one half revolution to record before full revolutions will start getting added, so this must be reflected in the total time. 
    minutes_per_second = 1/60
    minutes_per_overflow = seconds_per_overflow*minutes_per_second

    return minutes_per_overflow




minutes = get_overflow_minutes(320*12, 14, 32)
print("Minutes until overflow of MC4114 at 12V max speed, 14bit angle representation", minutes)


minutes = get_overflow_minutes(50e3, 12, 32)
print("Minutes until overflow of vishan at 8V, max speed, 12bit angle representation", minutes)

minutes = get_overflow_minutes(50e3, 14, 32)
print("Minutes until overflow of vishan at 8V, max speed, 14bit angle representation", minutes)
