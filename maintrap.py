import Tkinter as tk
import t2
import trapzone
import shapely
import math

if __name__ == '__main__':
	
	# Define a TrapZone and some obstacles to test with
	# Be sure to enter them in counterclockwise order for the triangulation
	zone = [	
		(100,400),
		(150,250),
		(50,100),
		(400,100),
		(400,250),
		(250,400),
		(250,250)
	]
	
	xs = [math.floor(x[0]) for x in zone]
	ys = [math.floor(x[1]) for x in zone]
	
	obstacles = [
		[
			(275,125),
			(350,125),
			(350,180),
			(275,180)
		]
	]
	
	zone = shapely.geometry.Polygon(zone)
	obstacles = [shapely.geometry.Polygon(o) for o in obstacles]
	
	# Obtain input from the user and create instance of TrapZone
	elements = input('Enter number of traps: ')
	response = raw_input('Would you like to shrink the zone? [y/n]: ')
	zone1 = zone
	if response == "y":
		erode = True
		bufAmt = input('How much buffer?: ')
		zone1 = zone.buffer(-1*abs(bufAmt))
	tz = trapzone.TrapZone(zone1,obstacles,elements)
	
	# Store the corners of the buffered square
	bounds1 = zone1.bounds
	rect1 = shapely.geometry.Polygon(
		[
			(bounds1[0],bounds1[1]),
			(bounds1[0],bounds1[3]),
			(bounds1[2],bounds1[3]),
			(bounds1[2],bounds1[1])
		]
	)
	
	# Obtain the locations of the traps
	allPoints = tz.genPoints()
	traps = allPoints[0]
	outerTraps = allPoints[1]
	
	# Prepare the window that will display the map
	root = tk.Tk()
	canvas = tk.Canvas(root, width=500, height=500, highlightthickness=0,bg="white")
	canvas.pack()
	
	# Display the points in the window
	t2.draw_poly(rect1,canvas,'black')
	t2.draw_poly(zone,canvas,'red')
	t2.draw_poly(zone1,canvas,'yellow')
	trapzone.drawPoints(canvas,traps,3,'green')
	trapzone.drawPoints(canvas,outerTraps,3,'white')
	
	root.mainloop()