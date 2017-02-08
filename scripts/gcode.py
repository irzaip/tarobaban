import re

fil = open('test.gcode')
com = fil.readlines()

def Find(pat,txt):
	ret = []
	match = re.search(pat,txt)
	if match: ret = match.group()
	return ret

for x in com:

	#G0-3 Command
	G = Find(r'{G0+|G1+|G2+|G3+|}',x)
	if G <> []:
		X = Find(r'X\S+',x)
		if X == []: X='Empty' 
		else: X = X[1:]
		

		Y = Find(r'Y\S+',x)
		if Y == []: Y='Empty' 
		else: Y = Y[1:]
		

		Z = Find(r'Z\S+',x)
		if Z == []: Z='Empty' 
		else: Z = Z[1:]

		print "X:",X," Y:",Y," Z:",Z
	else:
		print x



