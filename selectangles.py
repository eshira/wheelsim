import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from matplotlib.lines import Line2D
import math

"""Select the angles for the wheels of a 3 wheeled robot.
   Shows the intersection of the normals.
   When the three coincide perfectly, there is no slip and that is the instantaneous center of curvature, or ICC
   Otherwise, slippage would occur on some wheels and the model is not as useful (probably worse the further away the intersection points get)
"""

fig, ax = plt.subplots()
plt.subplots_adjust(left=0.1, bottom=0.3)
fig.suptitle('3 Independently Steered and Driven Wheels', fontsize=20)
fig.canvas.set_window_title('Robot Drive Simulation')
plt.xlim(-2, 2)
plt.ylim(-2, 2)
plt.gca().set_aspect('equal', adjustable='box')

a = 1.0
r = a/math.sqrt(3)
errorrad = .25 # Radius of error. The (todo: define strictly the way this works)

# Draw the robot frame (equilateral triangle, side length a)
points = [[0,r], [a/2,r-a*math.sqrt(3)/2], [-a/2,r-a*math.sqrt(3)/2]]
polygon = plt.Polygon(points,fc='None')
plt.gca().add_patch(polygon)
sz=20;
dotcolor = "black"

winit = [0,0,0]

wheelcolors = [("darkmagenta","plum"),("blue","skyblue"),("green","lightgreen")]

w = [plt.axes([0.15, 0.05, 0.4, 0.03], axisbg=wheelcolors[0][1]), \
	plt.axes([0.15, 0.1, 0.4, 0.03], axisbg=wheelcolors[1][1]), \
	plt.axes([0.15,0.15,0.4,0.03], axisbg=wheelcolors[2][1])]

wsliders = [Slider(w[0], 'Wheel C', -90, 90, valfmt='%0.0f', valinit=winit[0],color=wheelcolors[0][0]), \
			Slider(w[1], 'Wheel B', -90, 90, valfmt='%0.0f', valinit=winit[1],color=wheelcolors[1][0]), \
			Slider(w[2], 'Wheel A', -90, 90, valfmt='%0.0f', valinit=winit[2],color=wheelcolors[2][0]) ]

wd = .8*a/2 # wheel radius
nd = 15 # Length to draw the norms

# Draw the wheels
wheels = [ Line2D( [ points[i][0]+math.cos(math.radians(winit[i]+i*60))*wd/2,points[i][0]-math.cos(math.radians(winit[i]+i*60))*wd/2 ],\
				   [ points[i][1]+math.sin(math.radians(winit[i]+i*60))*wd/2,points[i][1]-math.sin(math.radians(winit[i]+i*60))*wd/2 ],\
				   linewidth=5,color=wheelcolors[i][0]) for i in range(3) ]

norms = [ Line2D( [ points[i][0]+math.cos(math.radians(winit[i]+90+i*60))*nd/2,points[i][0]-math.cos(math.radians(winit[i]+90+i*60))*nd/2 ],\
				   [ points[i][1]+math.sin(math.radians(winit[i]+90+i*60))*nd/2,points[i][1]-math.sin(math.radians(winit[i]+90+i*60))*nd/2 ],\
				   linewidth=1,color=wheelcolors[i][1]) for i in range(3) ]

for wheel in wheels:
	ax.add_line(wheel)

for norm in norms:
	ax.add_line(norm)

soln = []
dots = []
coeffs = np.zeros([3,3])

for i in range(3):
	x1,x2 = norms[i].get_xdata()
	y1,y2 = norms[i].get_ydata()
	coeffs[i][0] = y1-y2
	coeffs[i][1] = x2-x1
	coeffs[i][2] = y1*x2-y2*x1

# Create the necessary matrices to put problem in the form Ax = B for each pair of wheel normal lines
for i in range(3):
	A = np.array([coeffs[i,:2],coeffs[(i+1)%3,:2]]) # All pairs--select the A matrices
	B = np.array([coeffs[i,2],coeffs[(i+1)%3,2]])[:,np.newaxis]
	soln.append(np.linalg.lstsq(A,B)[0]) # Solve
	dots.append(ax.scatter(soln[i][0],soln[i][1],s=sz,color=dotcolor))

def update(i,val):
	global dots
	wheels[i].set_xdata( [ points[i][0]+math.cos(math.radians(val+i*60))*wd/2,points[i][0]-math.cos(math.radians(val+i*60))*wd/2 ] )
	wheels[i].set_ydata( [ points[i][1]+math.sin(math.radians(val+i*60))*wd/2,points[i][1]-math.sin(math.radians(val+i*60))*wd/2 ] )
	norms[i].set_xdata( [ points[i][0]+math.cos(math.radians(val+90+i*60))*nd/2,points[i][0]-math.cos(math.radians(val+90+i*60))*nd/2 ] )
	norms[i].set_ydata( [ points[i][1]+math.sin(math.radians(val+90+i*60))*nd/2,points[i][1]-math.sin(math.radians(val+90+i*60))*nd/2 ] )

	soln = []
	coeffs = np.zeros([3,3])

	for i in range(3):
		dots[i].remove()
		x1,x2 = norms[i].get_xdata()
		y1,y2 = norms[i].get_ydata()
		coeffs[i][0] = y1-y2
		coeffs[i][1] = x2-x1
		coeffs[i][2] = y1*x2-y2*x1
	dots = []
	for i in range(3):
		A = np.array([coeffs[i,:2],coeffs[(i+1)%3,:2]]) # All pairs--select the A matrices
		B = np.array([coeffs[i,2],coeffs[(i+1)%3,2]])[:,np.newaxis]
		soln.append(np.linalg.lstsq(A,B)[0])
		dots.append(ax.scatter(soln[i][0],soln[i][1],s=sz,color=dotcolor))
	fig.canvas.draw()

for i in range(3):
	wsliders[i].on_changed( (lambda i: lambda val: update(i,val))(i))
plt.show()
