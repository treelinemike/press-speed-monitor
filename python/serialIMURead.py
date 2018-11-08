import sys
import serial
import struct
from imuConvert import imuConvert

# serial transmission byte definitions
DLE_byte = 0x10
STX_byte = 0x02
ETX_byte = 0x03

# global scope variables
byteBuffer = bytearray()   # list to hold raw bytes read from serial line
firstRun = True			   # flag indicating first message processed (for starting time t)
microTime = 0              # time reported by the microcontroller
t = 0                      # time since first packet read (i.e. time for plotting signals)
dispCount = 0              # counter for displaying data on screen
dispEvery = 10             # display data once every xx messages

# open the serial port
ser = serial.Serial(port='COM4',baudrate=115200,bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,timeout=0.0005)

# capture serial data forever
while ( True ):
	# read one byte from the serial line
	# this value can be increased slightly, but larger reads result in significant lag
	data = ser.read(1)
	dataLen = len(data)
	
	# add these bytes to the END of the serial bytes vector
	if( dataLen > 0 ):
		byteBuffer.extend(bytearray(data))

		# look for the start of a message
		while( (len(byteBuffer) > 1) and (byteBuffer[0] != DLE_byte) and (byteBuffer[1] != STX_byte) ):
			if ((byteBuffer[0] == DLE_byte) and (byteBuffer[1] == DLE_byte)):
				byteBuffer.pop(0)
				byteBuffer.pop(0)
			else:
				byteBuffer.pop(0)
		
		# if there are at least two bytes left in the buffer, we should be at the start of a message
		if( len(byteBuffer) > 2 ):
		
			# message search variables that will be used later
			# within the scope of this if() statement
			msgFound = False
			fwdIdx = 2
						
			# look for the end of a message
			# TODO: this needs to be optimized!
			while ( (msgFound == False) and (fwdIdx < len(byteBuffer)) ):
				
				# is our fwdIdx marker at an ETX_byte?
				if ( byteBuffer[fwdIdx] == ETX_byte ):

					# if so, count the number of (DLE_byte) characters
					# that immediately preceed it
					atPrevDLE = True
					numPrevDLE = 0
					revIdx = fwdIdx - 1
					while ( atPrevDLE and (revIdx > 0) ):
						if (byteBuffer[revIdx] == DLE_byte):
							numPrevDLE += 1
							revIdx -= 1
						else:
							atPrevDLE = False
					
					# if we found an odd number of previous DLEs, this is a message
					if ( (numPrevDLE > 0) and ((numPrevDLE % 2) != 0) ):
						msgFound = True

				# increment counter to move on to next byte
				fwdIdx += 1			 
																 
			# if we found a message, extract it into its own structure
			# message will start at byteBuffer[0] and end at byteBuffer[fwdIdx-1]
			# because we incremented fwdIdx at the end of the while() loop above
			if ( msgFound == True ):
				
				# new bytearray to contain this message 
				newMsg = bytearray()
				
				# copy the message found into its own bytearray
				copyIdx = 0
				while ( copyIdx < fwdIdx ):
					# removed stuffed DLE_byte characters
					if ( (len(byteBuffer) > 1) and (byteBuffer[0] == DLE_byte) and (byteBuffer[1] == DLE_byte) ):
						byteBuffer.pop(0)
						copyIdx += 1
				
					# we should always have a byte left in the buffer here
					# so this assertion should never be raised
					assert (len(byteBuffer) >= 0), ("Big Error: 0 serial bytes", len(byteBuffer))
					
					# add current character to the new message
					newMsg.append(byteBuffer.pop(0))
					copyIdx += 1;

				# save only messages of the proper length
				# TODO: figure out why message is wrong length (wrong type? read error?)
				if (len(newMsg) != 23):
					newMsg.clear()		

				# if message is correct length, process it
				# TODO: check the packet type, and the CRC... for now we go on faith that the correct length packet is right...
				else:

					# extract microcontroller time
					# TODO: this is pathetically inelegant
					microTimeBytes = newMsg[3:5]
					microTimeBytes.extend(b'\x00\x00')
					newMicroTime = struct.unpack('<L',microTimeBytes)
					newMicroTime = newMicroTime[0]
					
					# update elapsed time for plotting signals
					if( firstRun == True ):
						microTime = newMicroTime
						firstRun = False
					else:
						if (newMicroTime > microTime):
							t += (newMicroTime - microTime)*(0.000016)
							microTime = newMicroTime
						else:
							t += ((65535-microTime) + newMicroTime + 1)*(0.000016)
							microTime = newMicroTime
				
					# finally, process the accelerometer and gyro signals per ST axes
					Gy = imuConvert(newMsg[8:10], 1000.0)
					Gx = -1*imuConvert(newMsg[10:12], 1000.0)
					Gz = imuConvert(newMsg[12:14], 1000.0)
					Ay = imuConvert(newMsg[14:16], 2.0)
					Ax = -1*imuConvert(newMsg[16:18], 2.0)
					Az = imuConvert(newMsg[18:20], 2.0)
					
					# write IMU data to screen, every so often...
					# TODO: this is a bit of a hack, clean up the timing
					if ( dispCount < dispEvery ):
						dispCount += 1						
					else:
						sys.stdout.write("%+8.2f   %+8.2f   %+8.2f   %+5.2f   %+5.2f   %+5.2f\r" % (Gx, Gy, Gz, Ax, Ay, Az))
						dispCount = 0

# close the serial port
# TODO: is it bad that we never get here? neither python nor windows seem to care...
ser.close()