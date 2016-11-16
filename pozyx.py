#https://pgi-jcns.fz-juelich.de/portal/pages/using-c-from-python.html
# to make so files https://docs.python.org/3/extending/building.html
import ctypes

_pozyx = ctypes.CDLL('pozyx.so')
_pozyx.somewriteablefunction.argtypes = (cytes.c_int, ctypes.POINTER(ctypes.c_int))

def somewriteablefunction(someints):
	global _pozyx
	num_someints = len(someints)
	array_type = ctypes.c_int * num_someints
	return int(_pozyx.somewritablefunction(ctypes.c_int(num_someints), array_type(*someints)))