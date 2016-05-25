import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import math

"""
Tool to explore 3 wheeled robot modes of driving by selecting instantaneous center of curvature (ICC)
Select the ICC with the mouse (click to place)
"""

fig, ax = plt.subplots()
fig.canvas.set_window_title('Robot Drive Simulation')
pltsize = 5
plt.xlim(-pltsize, pltsize)
plt.ylim(-pltsize, pltsize)
plt.gca().set_aspect('equal', adjustable='box')

# This circle is the input. The user clicks to place it.
circle = plt.Circle((0,0), radius=.03, color='red')
plt.gca().add_patch(circle)

ICC = plt.Circle((0,0), radius=0, fc="none")
plt.gca().add_patch(ICC)


def angle_wrt_x(A,B):
    """Return the angle between B-A and the positive x-axis.
    Values go from 0 to pi in the upper half-plane, and from 
    0 to -pi in the lower half-plane.
    Snippet from
    http://stackoverflow.com/questions/13543977/python-does-a-module-exist-which-already-find-an-angle-and-the-distance-between
    """
    ax, ay = A
    bx, by = B
    return math.atan2(by-ay, bx-ax)

def on_click(event):
	"""adapted from http://matplotlib.org/examples/pylab_examples/coords_demo.html"""
	global circle
	# get the x and y coords, flip y from top to bottom
	x, y = event.x, event.y
	if event.button == 1:
		if event.inaxes is ax:
			circle.center = (event.xdata,event.ydata)
		else:
			clicklocation = [event.x, event.y]
			figsize = fig.get_size_inches()*fig.dpi
			circle.center = ( 1000*(clicklocation[0] - figsize[0]/2.)/figsize[0], 1000*(clicklocation[1] - figsize[1]/2.)/figsize[1])
		update()
		fig.canvas.draw()		
		
plt.connect('button_press_event', on_click)

wheelcolors = [("darkmagenta","plum"),("blue","skyblue"),("green","lightgreen")]

# Frame of robot
# Long term goal: make it possible to load in structures from text files and pre-generate some useful/interesting ones
a = 1.0 # side length
r = a/math.sqrt(3)
wd = .8*a/2 # wheel radius
pvt = .2*a # pivot arm distance
bd = 20 # Draw boundary length very long so it fills the page
# Draw the robot frame as a polygon (equilateral triangle, side length a)
points = [[0,r], [-a/2,r-a*math.sqrt(3)/2],[a/2,r-a*math.sqrt(3)/2]]
polygon = plt.Polygon(points,fc='None')
plt.gca().add_patch(polygon)

# Normals to the wheels
norms = [ Line2D( [ points[i][0],circle.center[0]],\
				   [ points[i][1],circle.center[1] ],\
				   linewidth=2,color=wheelcolors[i][1]) for i in range(3) ]
# Add them
for norm in norms:
	ax.add_line(norm)

# Across the matching boundary, the wheel will have to switch position drastically
boundaries = [ Line2D( [ points[i][0]-bd*math.cos(math.pi-i*math.radians(60)),points[i][0]+bd*math.cos(math.pi-i*math.radians(60))],\
				   	   [ points[i][1]-bd*math.sin(math.pi-i*math.radians(60)),points[i][1]+bd*math.sin(math.pi-i*math.radians(60)) ],\
				   		linewidth=1,color="red") for i in range(3) ]
for bound in boundaries:
	ax.add_line(bound)

# Compute angles between norms and the x positive axis
angles = [ angle_wrt_x((points[i][0],points[i][1]),(circle.center[0],circle.center[1]))for i in range(3)]

# Create the wheel drawings as Line2Ds
wheels = [ Line2D( [ points[i][0]-pvt*math.cos(angles[i])+math.cos(angles[i] + math.pi/2 )*wd/2,points[i][0]-pvt*math.cos(angles[i])-math.cos(angles[i] + math.pi/2 )*wd/2 ],\
				   [ points[i][1]-pvt*math.sin(angles[i])+math.sin(angles[i] + math.pi/2 )*wd/2,points[i][1]-pvt*math.sin(angles[i])-math.sin(angles[i] + math.pi/2 )*wd/2 ],\
				   linewidth=5,color=wheelcolors[i][0]) for i in range(3) ]
for wheel in wheels:
	ax.add_line(wheel)

# Create the pivot arms (the pivot points are at the main frame triangle vertices)
arms = [ Line2D( [ points[i][0],points[i][0]-pvt*math.cos(angles[i])],\
				 [ points[i][1],points[i][1]-pvt*math.sin(angles[i])],\
				linewidth=1,color="black") for i in range(3) ]
for arm in arms:
	ax.add_line(arm)

# Update the drawing (called by the on_click method)
def update():
	for i in range(3):
		norms[i].set_xdata( [ points[i][0],circle.center[0]] )
		norms[i].set_ydata( [ points[i][1],circle.center[1]] )
		angles[i] = angle_wrt_x((points[i][0],points[i][1]),(circle.center[0],circle.center[1]))
		#Check and flip pivoting arms angle when wheel-frame intersections would occur
		if i == 0:
			if ( math.sin(angles[i] >= 0 )):
				angles[i] = (angles[i]-math.pi)
		else:
			if ( math.sin(angles[i]+pow(-1,i)*(2.0/3.0)*math.pi) >= 0 ):
				angles[i] = (angles[i]-math.pi)
	
		arms[i].set_xdata([points[i][0],points[i][0]-pvt*math.cos(angles[i])])
		arms[i].set_ydata([points[i][1],points[i][1]-pvt*math.sin(angles[i])])
		wheels[i].set_xdata([points[i][0]+pvt*math.cos(angles[i]-math.pi)+math.cos(angles[i] + math.pi/2 )*wd/2,points[i][0]+pvt*math.cos(angles[i]-math.pi)-math.cos(angles[i] + math.pi/2 )*wd/2 ])
		wheels[i].set_ydata([points[i][1]+pvt*math.sin(angles[i]-math.pi)+math.sin(angles[i] + math.pi/2 )*wd/2,points[i][1]+pvt*math.sin(angles[i]-math.pi)-math.sin(angles[i] + math.pi/2 )*wd/2 ])
		ICC.center = circle.center
		ICC.radius = math.hypot(circle.center[0], circle.center[1]) # Draws to (0,0 which is on robot frame )
	fig.canvas.draw()

plt.show()
