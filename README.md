#### Installation
Install libgeos-dev if you don't have it (native geo library)
`sudo apt-get install libgeos-dev`
Also install shapely (Python wrapper providing geometric functions)
`python -m pip install shapely`
or whatever python package manager you use

#### Usage
```
obstacles = [
	[
		(225, 175),
		(265, 225),
		(225, 275)
	]
]
grid = OccupancyGrid(obstacles)
start = (.5, .5)  # (x, y)
waypoint = (475.5, 475.5, 50)  # (x, y, r)
print(grid.find_path(start, waypoint))
```

usage subject to change
