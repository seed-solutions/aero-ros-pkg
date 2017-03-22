
#!/usr/bin/env python

from struct import*

import serial
import time
import array
import sys
import random

class SeedCommand:

	def __init__(self,dev="",baud=460800):
		self.ser=serial.Serial(dev, baud, timeout=0.1)

	#COM Port Open
	def COM_Open(self):

		self.ser.write("S8\r")
		self.ser.write("O\r")

	#Com Port Close
	def COM_Close(self):
		#self.ser.write("C\r")
		self.ser.close()

	#Binary data to Hex String data and decide number of byte
	def num2str(self,value,byte=1):
		if value < 0:
			value = 0xFFFFFF + 1 + value
			#return str(hex(value).upper()[3:][-2:].rjust(2**byte,"F"))
		return str(hex(value).upper()[2:][-2:].rjust(2**byte,"0"))

	#Binary data to Hex data and decide number of byte
	def num2hex(self,value,byte=1):
		if value < 0:
			value = 0xFFFFFF + 1 + value
		return hex(value).upper()[2:][-2:].rjust(2**byte,"0")

	#SEED CAN data send
	def SEED_CAN_Snd(self,SndData):

		CAN_Transmit = ''.join(SndData) + '\r'
		self.ser.write(CAN_Transmit)

		print "Send Data \t:%s" % CAN_Transmit

	#SEED CAN data read
	def SEED_CAN_Read(self):

		ret_str = ''

		while(self.ser.inWaiting() < 1):
			pass

		while(self.ser.read(1) != 't'):
			pass

		ret_str = 't'

		# Get Data Length 
		ret = self.ser.read(4)
		data_len = int(ret[-1],16)
		ret_str += ret
	
		# Get All data
		ret = self.ser.read(data_len*2)
		ret_str += ret

		self.ser.flushInput()

		print "Receive Data \t:%s" % ret_str

		return ret_str

	#SEED UART data send
	def SEED_UART_Snd(self,SndData):

		self.ser.write(''.join(map(chr,SndData)))

		#print "Send Data \t:%s" % ''.join("{:02X}".format(ord(c)) for c in ''.join(map(chr,SndData)))
		#print ''.join("{:02X}".format(ord(c)) for c in ''.join(map(chr,SndData)))

	#SEED UART data read
	def SEED_UART_Read(self,number):

		while(self.ser.inWaiting() < number):
			pass

		RcvData = ''.join("{:02X}".format(ord(c)) for c in self.ser.read(number))

		if (len(RcvData) == number*2):
			self.pre_RcvData = RcvData

			print "Receive Data \t:%s" % RcvData
	
			return RcvData
		else :
			print "Read Error"
			return self.pre_RcvData

		#print "Busy: %s, IO:%s" % (RcvData[9],RcvData[11])

		#print "Receive Data \t:%s" % RcvData
	

	#####################################################################################################

	#####################################################################################################
	########################## Dynamixel Function  ##################################
	#####################################################################################################

	#Aero Data Read
	def SEED_DX_Read(self):

		while(self.ser.inWaiting() < 4):
			pass

		headder1 = ord(self.ser.read(1))

		if (headder1 == 0xFF):
			pass
		else:
			print "Read Error. Headder is %d" % headder1
			self.ser.flushInput()
			self.ser.flushOutput()
			return None

		headder2 = ord(self.ser.read(1))
		id_data = ord(self.ser.read(1))
		data_length = ord(self.ser.read(1))

		timeout = time.time() + 1

		while(self.ser.inWaiting() < data_length):
			if(time.time() > timeout):
				print "Read Error. data_length is %d" % data_length
				self.ser.flushInput()
				self.ser.flushOutput()
				return None
			else:
				pass

		RcvData = self.num2str(headder1) + self.num2str(headder2) + self.num2hex(id_data) +  self.num2hex(data_length)\
				+ ''.join("{:02X}".format(ord(c)) for c in self.ser.read(data_length))

		#print "Receive Data \t:%s" % RcvData

		return RcvData

	def SEED_DX_Get(self,ID):
		bCheckSum = 0
		bCount = 0
		bLength = 8

		RS_TX= bLength*[0]

		RS_TX[0] = 0xFF
		RS_TX[1] = 0xFF
		RS_TX[2] = ID
		RS_TX[3] = 0x04		# Data Length
		RS_TX[4] = 0x02		# Instruction
		RS_TX[5] = 0x24		# Address
		RS_TX[6] = 0x08		# Count

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(RS_TX[bCount]),16)
		RS_TX[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		#num to hex
		for i in range(0,bLength-1):
			RS_TX[i] = int(self.num2hex(RS_TX[i]),16)

		self.SEED_UART_Snd(RS_TX)

	def SEED_DX_Get_POS(self,ID):
		bCheckSum = 0
		bCount = 0
		bLength = 8

		RS_TX= bLength*[0]

		RS_TX[0] = 0xFF
		RS_TX[1] = 0xFF
		RS_TX[2] = ID
		RS_TX[3] = 0x04
		RS_TX[4] = 0x02
		RS_TX[5] = 0x24
		RS_TX[6] = 0x02

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(RS_TX[bCount]),16)
		RS_TX[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		#num to hex
		for i in range(0,bLength-1):
			RS_TX[i] = int(self.num2hex(RS_TX[i]),16)

		self.SEED_UART_Snd(RS_TX)

	def SEED_DX_SRV(self,ID,SW):
		bCheckSum = 0
		bCount = 0
		bLength = 9

		RS_TX= bLength*[0]

		RS_TX[0] = 0xFF
		RS_TX[1] = 0xFF
		RS_TX[2] = ID
		RS_TX[3] = 0x05
		RS_TX[4] = 0x03
		RS_TX[5] = 0x18
		RS_TX[6] = SW
		RS_TX[7] = SW

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(RS_TX[bCount]),16)
		RS_TX[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		#num to hex
		for i in range(0,bLength-1):
			RS_TX[i] = int(self.num2hex(RS_TX[i]),16)

		self.SEED_UART_Snd(RS_TX)

	def SEED_DX_TURN(self,ID,SPD):

		bCheckSum = 0
		bCount = 0
		bLength = 9

		SndData = bLength*[0]

		SndData[0] = 0xFF
		SndData[1] = 0xFF
		SndData[2] = ID
		SndData[3] = 0x05	#LENGTH
		SndData[4] = 0x03	#Instruction
		SndData[5] = 0x20	#CMD

		SndData[6] = SPD
		SndData[7] = SPD >> 8

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(SndData[bCount]),16)

		SndData[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		#num to hex
		for i in range(0,bLength-1):
			SndData[i] = int(self.num2hex(SndData[i]),16)

		self.SEED_UART_Snd(SndData)

	def SEED_DX_MOV(self,ID,SPD,TRG):

		bCheckSum = 0
		bCount = 0
		bLength = 11

		SndData = bLength*[0]

		SndData[0] = 0xFF
		SndData[1] = 0xFF
		SndData[2] = ID
		SndData[3] = 0x07	#LENGTH
		SndData[4] = 0x03	#Instruction
		SndData[5] = 0x1E	#CMD
		SndData[6] = TRG
		SndData[7] = TRG >> 8
		SndData[8] = SPD
		SndData[9] = SPD >> 8

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(SndData[bCount]),16)

		SndData[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		#num to hex
		for i in range(0,bLength-1):
			SndData[i] = int(self.num2hex(SndData[i]),16)

		self.SEED_UART_Snd(SndData)

	def SEED_DX_MODE(self,ID,CW_Limit,CCW_Limit):
		bCheckSum = 0
		bCount = 0
		bLength = 11

		SndData = bLength*[0]

		SndData[0] = 0xFF
		SndData[1] = 0xFF
		SndData[2] = ID
		SndData[3] = 0x07	#LENGTH
		SndData[4] = 0x03	#Instruction
		SndData[5] = 0x06	#CMD
		SndData[6] = CW_Limit
		SndData[7] = CW_Limit >> 8
		SndData[8] = CCW_Limit
		SndData[9] = CCW_Limit >> 8

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(SndData[bCount]),16)

		SndData[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		#num to hex
		for i in range(0,bLength-1):
			SndData[i] = int(self.num2hex(SndData[i]),16)

		self.SEED_UART_Snd(SndData)

	def SEED_DX_PING(self):
		bCheckSum = 0
		bCount = 0
		bLength = 6

		SndData = bLength*[0]

		SndData[0] = 0xFF
		SndData[1] = 0xFF
		SndData[2] = 0xFE
		SndData[3] = 0x02	#LENGTH
		SndData[4] = 0x01	#Instruction

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(SndData[bCount]),16)

		SndData[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		#num to hex
		for i in range(0,bLength-1):
			SndData[i] = int(self.num2hex(SndData[i]),16)

		self.SEED_UART_Snd(SndData)

		rec = self.SEED_DX_Read()		
	
		if (rec == None):
			print "False"
			return None
		else:
			print "ID No. -> %d" % int(rec[4]+rec[5],16)
			return int(rec[4]+rec[5],16)
