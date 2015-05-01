#!/usr/bin/env python

from struct import*

import serial
import time
import array
import math

#COM Port Open
def COM_Open():

	global SER

	# SER=serial.Serial('/dev/ttyUSB0', 115200)
	# SER.open()
	# SER.write("S8\r")
	# SER.write("O\r")

def RS485_Open():

	global SER

	#SER=serial.Serial('/dev/ttyUSB0', 115200)
	# SER=serial.Serial('/dev/ttyUSB0', 1382400)
	#SER.open()

#Com Port Close
def COM_Close():

	global SER

	# SER.write("C\r")
	# SER.close()

def RS485_Close():

	global SER

	#SER.write("C\r")
	# SER.close()

#Binary data to Hex data and decide number of byte
def num2hex(value,byte=1):
	if value < 0:
		value = 0xFFFFFF + 1 + value
	return hex(value).upper()[2:][-2:].rjust(2**byte,"0")

#SEED CAN data send
def SEED_CAN_Snd(SndData):

	global SER

	CAN_Transmit = ''.join(map(str,SndData)) + '\r'
	# SER.write(CAN_Transmit)

	print CAN_Transmit

#SEED 485 data send
def SEED_485_Snd(SndData):

	global SER

	# SER.write(''.join(map(chr,SndData)))

	print "send: %s" % ''.join("{:02X}".format(ord(c)) for c in ''.join(map(chr,SndData)))

#SEED 485 data read
def SEED_485_Read():

	global SER

	#RcvData = SER.read(34)

	# RcvData = ''.join("{:02X}".format(ord(c)) for c in SER.read(77))

	# print "Receive Data \t:%s" % RcvData

#SEED Command Function
'''
id_num	:ID data
d3		:SEED Command
d4~d5	:Parameter
'''
def SEED_SCM(id_num,d3,d4,d5,d6,d7,d8):

	SndData = 12*[0]

	SndData[0] ='t'

	if d3 == 0x53:
		SndData[1] = num2hex(0x00,0)
	else:
		SndData[1] = num2hex(0x03,0)

	SndData[2] = num2hex(id_num)
	SndData[3] = num2hex(8,0)
	SndData[4] = num2hex(0xF0 + id_num)
	SndData[5] = num2hex(0x00)
	SndData[6] = num2hex(d3)
	SndData[7] = num2hex(d4)
	SndData[8] = num2hex(d5)
	SndData[9] = num2hex(d6)
	SndData[10] = num2hex(d7)
	SndData[11] = num2hex(d8)

	SEED_CAN_Snd(SndData)

#Run SEED Script
def SEED_SGo(id_num,s_num):
	if s_num > 0x00 and s_num < 0x0F :
		SEED_SCM(id_num,0x5F,id_num,s_num,0x00,0x00,0x00)

#absolute Go (Servo mode)
def SEED_TMove_Servo(id_num,time,pos):
	data = 5*[0]

	if pos >= 0xFFFFFF/2:
		pos = 0xFFFFFF/2 - 1
	elif pos <= -0xFFFFFF/2 + 1:
		pos = -0xFFFFFF/2-1

	data[0] = time >> 8
	data[1] = time
	data[2] = pos >> 16
	data[3] = pos >> 8
	data[4] = pos

	print data
	SEED_SCM(id_num,0x64,data[0],data[1],data[2],data[3],data[4])

#Set Parameter Command(0x20, 0x25, 0x24, 0x67, 0x68)
def Set_Command(id_num,cmd,d0,d1,d2,d3,d4,d5,d6,d7,d8,d9,d10,d11,d12,d13,d14,d15,d16,d17,d18,d19,d20,d21,d22,d23,d24,d25,d26,d27,d28,d29,d30,d31,d32,d33,d34,d35):

	bCheckSum = 0
	bCount = 0
	bLength = 77		#number of data +1

	data= bLength*[0]

	data[0] = 0xFA
	data[1] = 0xAF
	data[2] = 0xF0 + id_num

	data[3] = cmd

	#time
	data[4] = d0 >> 8
	data[5] = d0
	#CAN1 Actuator
	data[6] = d1 >> 8
	data[7] = d1
	data[8] = d2 >> 8
	data[9] = d2
	data[10] = d3 >> 8
	data[11] = d3
	data[12] = d4 >> 8
	data[13] = d4
	data[14] = d5 >> 8
	data[15] = d5
	data[16] = d6 >> 8
	data[17] = d6
	data[18] = d7 >> 8
	data[19] = d7
	data[20] = d8 >> 8
	data[21] = d8
	data[22] = d9 >> 8
	data[23] = d9
	data[24] = d10 >> 8
	data[25] = d10
	data[26] = d11 >> 8
	data[27] = d11
	data[28] = d12 >> 8
	data[29] = d12
	#CAN1 Sensor
	data[30] = d13 >> 8
	data[31] = d13
	data[32] = d14 >> 8
	data[33] = d14
	data[34] = d15 >> 8
	data[35] = d15
	data[36] = d16 >> 8
	data[37] = d16
	#CAN2
	data[38] = d17 >> 8
	data[39] = d17
	data[40] = d18 >> 8
	data[41] = d18
	data[42] = d19 >> 8
	data[43] = d19
	data[44] = d20 >> 8
	data[45] = d20
	data[46] = d21 >> 8
	data[47] = d21
	data[48] = d22 >> 8
	data[49] = d22
	data[50] = d23 >> 8
	data[51] = d23
	data[52] = d24 >> 8
	data[53] = d24
	data[54] = d25 >> 8
	data[55] = d25
	data[56] = d26 >> 8
	data[57] = d26
	data[58] = d27 >> 8
	data[59] = d27
	data[60] = d28 >> 8
	data[61] = d28
	#CAN2 Sensor
	data[62] = d29 >> 8
	data[63] = d29
	data[64] = d30 >> 8
	data[65] = d30
	data[66] = d31 >> 8
	data[67] = d31
	data[68] = d32 >> 8
	data[69] = d32
	#IMU
	data[70] = d33 >> 8
	data[71] = d33
	data[72] = d34 >> 8
	data[73] = d34
	data[74] = d35 >> 8
	data[75] = d35

	#num to hex
	for i in range(0,bLength-1):
		data[i] = int(num2hex(data[i]),16)

	#check sum
	for bCount in range(2,bLength-1,1):
		bCheckSum += int(num2hex(data[bCount]),16)

	data[bLength - 1] =  int(num2hex(~bCheckSum),16)

	SEED_485_Snd(data)

	#print data as hex
	for i in range(0,bLength):
		data[i] = num2hex(data[i])

	#print ''.join(data)

#Get Information Command(0x42, 0x43, 0x45)
def Get_Command(id_num,cmd):
	bCheckSum = 0
	bCount = 0
	bLength = 77		#number of data +1

	data= bLength*[0]

	data[0] = 0xFA
	data[1] = 0xAF
	data[2] = 0xF0 + id_num

	data[3] = cmd

	for i in range(4,bLength-1):
		data[i] = 0

	#check sum
	for bCount in range(2,bLength-1,1):
		bCheckSum += int(num2hex(data[bCount]),16)

	data[bLength - 1] =  int(num2hex(~bCheckSum),16)

	SEED_485_Snd(data)

#Motor ON / OFF Command(0x50)
def Servo_Command(id_num,d0):
	bCheckSum = 0
	bCount = 0
	bLength = 77		#number of data +1

	data= bLength*[0]

	data[0] = 0xFA
	data[1] = 0xAF
	data[2] = 0xF0 + id_num

	data[3] = 0x50

	for i in range(4,bLength-1):
		data[i] = d0

	#check sum
	for bCount in range(2,bLength-1,1):
		bCheckSum += int(num2hex(data[bCount]),16)

	data[bLength - 1] =  int(num2hex(~bCheckSum),16)

	SEED_485_Snd(data)


#Snd 485
def RS485_Snd_test():

	bCheckSum = 0
	bCount = 0
	bLength = 2		#number of data +1

	data= bLength*[0]

	data[0] = 0xFA
	data[1] = 0xFF

	SEED_485_Snd(data)

#----------- main -----------------#
if __name__ == "__main__":

	global SER

	i=0
# Trajectory Parameter
	f=0.5						#[Hz]
	omega = 2*math.pi*f
	A=100						#*0.01[mm]

	move_time = 20		#[ms]

# Port Open
	RS485_Open()

# Servo ON
	Servo_Command(1,1)

# Buffer Clear
        #SER.flushInput()		#flush input buffer
	#SER.flushOutput()	#flush output buffer

	time.sleep(5)

# Move to Oringinal Position
	Set_Command(1,0x68,2000,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0)

	time.sleep(5)

	#Get_Command(1,0x42)
	#SEED_485_Read()

	# while 1:
        for j in range(100):

# Move Command
		y=int(A*math.sin(omega*i))
		Set_Command(1,0x68,move_time,y*5,y/2,y/2,y,y,y*5,y,y*10,y/2,y/2,y,y,y,y,y,y, y*5,y,y,y,y,y*5,y,y*5,y,y,y,y,y,y,y,y,y,y,y)

			#y=100
			#Set_Command(1,0x68,20,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y)
			#time.sleep(1)
			#y=-100
			#Set_Command(1,0x68,20,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y,y)
			#time.sleep(1)

# Receive present position
		SEED_485_Read()
		time.sleep(move_time*0.001)
		i += move_time*0.001


	RS485_Close()
