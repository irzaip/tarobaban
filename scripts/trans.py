# demo code for transformation 
# by irza
import robot as rb

def trns(x,y,z):
	#define here the transform translation and rotation
	a = rb.transl(0, 700,0)
	b = rb.trotz(rb.deg2rad(90))

	#asumsi titik + [1]
	pt = [[x],[y],[z],[1]]
	cc = a + (b * pt)
	result = rb.transl(cc)
	x,y,z=result

	#kembalikan kebentuk kord
	x=x[0,0]
	y=y[0,0]
	z=z[0,0]
	return x,y,z


x=12
y=44
z=2


x,y,z= trns(x,y,z)
print x,y,z