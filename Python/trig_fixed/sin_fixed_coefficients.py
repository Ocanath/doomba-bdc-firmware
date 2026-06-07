import numpy as np


HALF_PI_12B = np.int32(np.round(np.pi/2*2**12))
THREE_BY_TWO_PI_12B = np.int32(np.round(np.pi*3/2*2**12))
PI_12B = np.int32(np.round(np.pi*2**12))
TWO_PI_12B = np.int32(np.round(np.pi*2*2**12))

print("#define HALF_PI_12B", HALF_PI_12B)
print("#define THREE_BY_TWO_PI_12B", THREE_BY_TWO_PI_12B)
print("#define PI_12B", PI_12B)
print("#define TWO_PI_12B", TWO_PI_12B)

print("\r\n")

HALF_PI_14B = np.int32(np.round(np.pi/2*2**14))
THREE_BY_TWO_PI_14B = np.int32(np.round(np.pi*3/2*2**14))
PI_14B = np.int32(np.round(np.pi*2**14))
TWO_PI_14B = np.int32(np.round(np.pi*2*2**14))

print("#define HALF_PI_14B", HALF_PI_14B)
print("#define THREE_BY_TWO_PI_14B", THREE_BY_TWO_PI_14B)
print("#define PI_14B", PI_14B)
print("#define TWO_PI_14B", TWO_PI_14B)

print("\r\n")