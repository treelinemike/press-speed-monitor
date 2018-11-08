import struct

# convert two-byte two's complement numbers with a given min/max range
# e.g. ST accelerometer output
# into the appropriate signed floating point values
def imuConvert( data, range):

	# convert data out of bytes
	data = bytearray(data)
	data.extend(b'\x00\x00')
	data = struct.unpack('<L',data)
	data = data[0]
	
	# deal with negative numbers
	if ((data & (1 << 15)) != 0):
		imuVal = (~(data-1) & ((1 << 16)-1))
		imuVal = imuVal*(-1*range/(((1 << 16)/2)))
		
	# non-negative numbers
	else:
		imuVal = data*(range/(((1 << 16)/2)-1))
		
	return imuVal