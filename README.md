#### Usage
```
obstacles = [
	[
		(225, 175),
		(265, 225),
		(225, 275)
	]
]
grid = OccupancyGrid(
	[Polygon([Vertex(p[0], [1]) for p in points]) for points in obstacles]
)
print(grid.find_path((.5, .5), (475.5, 475.5)), canvas)
```

usage subject to change
