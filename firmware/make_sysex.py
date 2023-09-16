print("importing")

import matplotlib.image as i
import sys
import math
import mido
import time

print("starting")

number = int(sys.argv[1])
infile = sys.argv[2]
outfile = open(sys.argv[3], "wb")

img = i.imread(infile)

if img.shape[1] != 128 or img.shape[0] != 64:
	print("Error: image must be 128x64")
	exit(1)

img = img.transpose()[0].transpose() > 0.5 # extract red channel

print("converting")
def img2sysex(img, number):
	sysex = bytearray([0xF0, 0x00, 0x13, 0x37, number])
	height = img.shape[0]

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

	return sysex
print("done")
sysex = img2sysex(img, number)
outfile.write(sysex)


output_name = [x for x in mido.get_output_names() if "Faderboard" in x][0]
midi_out = mido.open_output(output_name)

i = 0
while True:

	i = (i+1) % 56
	print("x")
	for number in range(9):


		img[1:63, 1:15] = False
		img[2:62, [2,13]] = True
		img[[2,61], 2:14] = True
		img[(59-i):60, 4:12] = True

		sysex = img2sysex(img, number)
		message = mido.Message.from_bytes(sysex)
		midi_out.send(message)
		time.sleep(0.01)
