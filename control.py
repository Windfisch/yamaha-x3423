import libm2k
from time import sleep
dev = libm2k.m2kOpen()

def setup():
	# power up the digital part
	ps = dev.getPowerSupply()
	ps.pushChannel(0,4)

	io = dev.getDigital()
	for pin in [0,1,2,3,4,5,6,7,8,12,13,14,15]:
		io.setDirection(pin, libm2k.DIO_OUTPUT)


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
	sleep(0.01)
	io.setValueRaw(12+byte_id, 1)
	sleep(0.01)
	io.setValueRaw(12+byte_id, 0)


setup()
reset()
write(3, 42)

