import libm2k
from math import *
from time import sleep
dev = libm2k.m2kOpen()
print("calibrating...")
#dev.calibrate()
print("done")
analog_out = dev.getAnalogOut()

def setup():
	# power up the digital part
	ps = dev.getPowerSupply()
	ps.pushChannel(0,4)

	io = dev.getDigital()
	for pin in [0,1,2,3,4,5,6,7,8,12,13,14,15]:
		io.setDirection(pin, libm2k.DIO_OUTPUT)

	analog_out.setSampleRate(0, 750)
	analog_out.enableChannel(0, True)

def analog_write(channel, value):
	if value >= 4.99:
		value = 4.99
	
	analog_out.push(channel, [value]*10)

def reset():
	io = dev.getDigital()
	io.setValueRaw(8, 0)
	sleep(0.01)
	io.setValueRaw(8, 1)
	sleep(0.01)

# pin 0..8 = data 0..8
# pin 8 = /RESET
# pin 12..15 = FD0..3 (flip flop select)

def write(byte_id, data_u8):
	io = dev.getDigital()
	for i in range(8):
		io.setValueRaw(i, data_u8 & (1<<i) != 0)
	#sleep(0.1)
	io.setValueRaw(12+byte_id, 1)
	#sleep(0.1)
	io.setValueRaw(12+byte_id, 0)


setup()
reset()

def set_target_value(idx, value):
	addr = idx % 8
	mux_id = int(idx / 8)

	analog_write(0, value)
	#sleep(0.01)
	write(3, addr | ( (0x7 & ~(1<<mux_id)) << 3))
	#sleep(0.01)
	write(3, 7 << 3)


#write(3, 0)
#analog_write(0, 3)
#sleep(0.2)
#write(3, 0x38)
#sleep(0.2)
#analog_write(0, 3)

print("setting target values")
for i in range(17):
	print("%d" % i)
	#set_target_value(i, i / 16 * 4 + 0.5)
	set_target_value(i, sin(2*3.1415 *2* i / 17)*2 + 2.5 )

print()



#for i in range(17):
#	bitid = i%8
#	byteid = int(i/8)
#	write(byteid, (1<<(bitid+1))-1)
#	sleep(0.2)
#	#write(byteid, 0)

write(0,0xff)
write(1,0xff)
write(2,1)
sleep(0.5)

for i in range(17):
	set_target_value(i, 0.7 )

for i in range(17):
	set_target_value(i, 4.3 )

for i in range(17):
	set_target_value(i, i / 16 * 4 + 0.5)

for j in range(10):
	for i in range(17):
		set_target_value(i, sin(2*3.1415 *2* i / 17 + 3.1415*2*j/10)*2 + 2.5 )

sleep(0.5)
write(0,0)
write(1,0)
write(2,0)

adc=dev.getAnalogIn()
while True:
	v1s = []
	v2s = []

	for i in range(4):
		#print(i)
		write(3, (i<<6) | (7<<3))
		#sleep(0.1)
		v1s += [adc.getVoltage(0)]
		v2s += [adc.getVoltage(1)]
		#sleep(0.01)

	vs = v1s+v2s
	print("%s" % ["%6.3f"%vs[i] for i in [2,3,0,1,5,6,7,4]])
