print("importing")

import matplotlib.image as i
import sys
import math

print("starting")

number = int(sys.argv[1])
infile = sys.argv[2]
outfile = open(sys.argv[3], "wb")

img = i.imread(infile)

if img.shape[1] != 128 or img.shape[0] != 64:
	print("Error: image must be 128x64")
	exit(1)

img = img.transpose()[0].transpose() > 0.5 # extract red channel
height = img.shape[0]

print("converting")
sysex = bytearray([0xF0, 0x00, 0x13, 0x37, number])

for byte in range(int(math.ceil(128*64/7))):
	result = 0
	for bit in range(7):
		bit_total = 7*byte + bit

		y_small = bit_total % 8
		x = (bit_total // 8) % 128
		y_big = bit_total // 8 // 128
		y = y_big * 8 + y_small

		if y >= height: y = height - 1

		if img[y][x]:
			result = result | (1 << bit)
	sysex.append(result)

sysex.append(0xF7)
print("done")
outfile.write(sysex)
