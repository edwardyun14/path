import Tkinter as tk
import t2
import trapzone
import shapely
import math

if __name__ == '__main__':
	
	option = input("(1) Use existing map\n(2) Manually input map\n(3) Use default map (Which is map #3)\nChoose option: ")
	
	zone = []
	obstacles = []
	if option==1:
		map = input("Which map? (1, 2, or 3): ")
		if map==1:
			zone = [
				(0,0),
				(200,0),
				(200,125),
				(0,125)
			]
			obstacles = [
				[
					(35,45),
					(55,45),
					(55,60),
					(35,60)
				], 
				[
					(30,95),
					(40,95),
					(40,105),
					(30,105)
				],
				[
					(100,85),
					(140,85),
					(140,115),
					(100,115)
				],
				[
					(150,30),
					(160,30),
					(160,40),
					(150,40)
				]
				
			]
		if map==2:
			zone = [
				(0,0),
				(200,0),
				(200,125),
				(0,125)
			]
			obstacles = [
				[
					(10,65),
					(20,65),
					(20,75),
					(10,75)
				], 
				[
					(30,30),
					(50,30),
					(50,50),
					(30,50)
				],
				[
					(80,55),
					(105,55),
					(105,80),
					(80,80)
				],
				[
					(145,90),
					(160,90),
					(160,115),
					(145,115)
				],
				[
					(140,30),
					(160,30),
					(160,50),
					(140,50)
				]
			]	
		if map==3:
			zone = [
				(0,0),
				(150,0),
				(150,125),
				(0,125)
			]
			obstacles = [
				[
					(15,25),
					(35,25),
					(35,45),
					(15,45)
				], 
				[
					(40,90),
					(50,90),
					(50,100),
					(40,100)
				],
				[
					(80,50),
					(90,50),
					(90,60),
					(80,60)
				],
				[
					(105,75),
					(125,75),
					(125,95),
					(105,95)
				],
				[
					(105,15),
					(145,15),
					(145,35),
					(105,35)
				]
			]
	
	if option==2:
		print("INPUT FLY-ZONE")
		num = input("How many points?: ")
		for i in range(1,num+1):
			tuple = input("Point "+str(i)+" (x,y): ")
			zone = zone + [tuple]
		
		new = input("New obstacle? (y/n): ")
		while new=="y":
			obstacle = []
			num = input("How many points?: ")
			for i in range(1,num+1):
				tuple = input("Point "+str(i)+" (x,y): ")
				obstacle = obstacle + [tuple]
			obstacles = obstacles + obstacle
			new = input("New obstacle? (y/n): ")
			
			
	if option==3:
		zone = [
			[
				(0,0),
				(150,0),
				(150,125),
				(0,125)
			]
		]
		obstacles = [
			[
				(15,25),
				(35,25),
				(35,45),
				(15,45)
			], 
			[
				(40,90),
				(50,90),
				(50,100),
				(40,100)
			],
			[
				(80,50),
				(90,50),
				(90,60),
				(80,60)
			],
			[
				(105,75),
				(125,75),
				(125,95),
				(105,95)
			],
			[
				(105,15),
				(145,15),
				(145,35),
				(105,35)
			]
		]
	
	zone = shapely.geometry.Polygon(zone)
	obstacles = [shapely.geometry.Polygon(o) for o in obstacles]
	obstacles1 = []
	
	# Obtain input from the user and create instance of TrapZone
	radius = input('Enter radius of traps: ')
	response = raw_input('Would you like to shrink the zone? [y/n]: ')
	zone1 = zone
	if response == "y":
		erode = True
		bufAmt = input('How much buffer?: ')
		zone1 = zone.buffer(-1*abs(bufAmt))
		for obstacle in obstacles:
			obstacles1 = obstacles1 + [obstacle.buffer(abs(bufAmt))]
	tz = trapzone.TrapZone(zone1,obstacles1,radius)
	
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
	canvas = tk.Canvas(root, width=220, height=220, highlightthickness=0,bg="white")
	canvas.pack()
	
	# Display the points in the window
	t2.draw_poly(rect1,canvas,'black')
	t2.draw_poly(zone,canvas,'red')
	t2.draw_poly(zone1,canvas,'yellow')
	for obstacle in obstacles1:
		t2.draw_poly(obstacle,canvas,'black')
	for obstacle in obstacles:
		t2.draw_poly(obstacle,canvas,'blue')
	trapzone.drawPoints(canvas,traps,3,'green')
	trapzone.drawPoints(canvas,outerTraps,3,'white')
	
	root.mainloop()