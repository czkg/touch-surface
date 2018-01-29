import os
import sys
import glob
import numpy as np

def r(x, y):
	if x == y:
		return 0
	else:
		return 1

def msd(presented, transcribed):
	width = len(presented)
	height = len(transcribed)
	a = range(width)
	b = range(height)
	mat = np.zeros((height, width))
	mat[0] = a
	mat[:, 0] = b
	for i in range(height)[1:]:
		for j in range(width)[1:]:
			mat[i, j] = min(mat[i - 1, j] + 1, mat[i, j - 1] + 1, mat[i - 1, j - 1] + r(transcribed[i], presented[j]))

	return mat[height - 1, width - 1]

def wpm(length, time):
	wpm = float(length - 1) / float(time) * float(60) / float(5)
	return wpm

# def cer(presented, transcribed):
# 	msd = msd(presented, transcribed)
# 	return float(msd) / float(len(presented)) * float(100)

if not len(sys.argv) == 3:
	print "invalid arguments!"
	sys.exit(1)

name = str(sys.argv[1]) #user name
layout = str(sys.argv[2]) #keyboard size
path = name + "/" + layout + "/"
error_sum = 0.0
length = []
time = []
for filename in glob.glob(os.path.join(path, '*.txt')):
	with open(filename) as f:
		lines = [line.rstrip('\n').rstrip('\r') for line in f]
		presented = ""
		transcribed = ""
		for line in lines:
			content = line.split()
			val = content[0]
			if val == "space":
				val = " "
			#transcribed
			if not val == "back" and not val == "enter" and not val == "shift":
				transcribed += val
			#presented
			if val == "back":
				presented = presented[:-1]
			if val == "back" or val == "enter" or val == "shift":
				val = ""
			presented += val
		time_s = lines[0].split()[1]
		time_e = lines[-1].split()[1]
		time_line = int(time_e) - int(time_s)
		length_line = len(presented)
		# add to array
		length.append(length_line)
		time.append(time_line)

		#calsulate cer
		error_sum += msd(presented, transcribed)


wpm = wpm(sum(length), sum(time))
cer = float(error_sum) / float(sum(length)) * 100
print wpm
print cer



