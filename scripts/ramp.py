import numpy as np



def sin_step_ramp(currentPosition,targetPosition,div=1):
	''' buat STEP dari currentPosition sampai dengan targetPosition
	dengan bantuan fungsi sinus. berguna membuat ramp yang gradasi
	seperti kurva sinus. 
	div dipakai sebagai resolusi pembagian,
	semakin besar div, semakin kecil jarak step
	return sebagai array
	'''
	arr = []
	r = targetPosition - currentPosition
	angles = np.array( (range(190)) [0::1*div]) - 90
	m = ( np.sin( angles * np.pi / 180. ) + 1 ) /2

	for mi in np.nditer(m):
		pos = currentPosition + mi*r
		arr.append(pos)
		#print "pos: ", pos 
	return arr


