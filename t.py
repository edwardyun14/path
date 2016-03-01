import path
import tkinter as tk
from shapely.geometry import Point

def main():
    root = tk.Tk()
    canvas = tk.Canvas(root, width=500, height=500,
                      highlightthickness=0, bg="white")
    canvas.pack()
    obstacles = [
        [
            (320, 200),
            (280, 230),
            (220, 230),
            (180, 200),
            (220, 170),
            (280, 170)
        ],
        [
            (225, 175),
            (265, 225),
            (225, 275)
        ],
        [
            (125, 125),
            (225, 125),
            (225, 175),
            (125, 175)
        ],
        [
            (375, 25),
            (475, 25),
            (475, 100),
            (425, 160),
            (375, 100)
        ]
    ]
    grid = path.OccupancyGrid(obstacles)
    end = (475.5, 475.5, 50)
    waypoints = [
        (475.5, 475.5, 50),
        (100, 300, 50),
        (300, 150, 50)
    ]
    cur_pos = (.5, .5)
    draw_grid(grid, canvas)
    for w in waypoints:
        final_path = grid.find_path(cur_pos, w)
        cur_pos = final_path[-1]
        draw_path(final_path, canvas)
        draw_poly(Point(w[0], w[1]) \
                  .buffer(w[2]), canvas, "brown", outline=True)
        """
        opt_path = {}
        print(grid._optimize_path(final_path, 0, opt_path))
        print(opt_path)
        s = 0
        true_path = []
        while s != len(final_path)-1:
            true_path.append(final_path[s])
            s = opt_path[s]
        true_path.append(final_path[s])
        draw_path(true_path, canvas)
        """
    root.mainloop()

def draw_path(path, canvas):
    for i in range(len(path)-1):
        x, y = path[i]
        x2, y2 = path[i+1]
        canvas.create_line(x, y, x2, y2, fill="purple")

def draw_grid(grid, canvas):
    for obstacle in grid.obstacles:
        draw_poly(obstacle, canvas, "blue")
    for col in grid.nodes:
        for node in col:
            if not node.occupied:
                draw_poly(node.poly, canvas, "green", outline=True)
    for col in grid.nodes:
        for node in col:
            if node.occupied:
                draw_poly(node.poly, canvas, "red", outline=True)

def draw_poly(poly, canvas, color, outline=False):
    if outline:
        vertices = list(poly.exterior.coords)
        for i in range(len(vertices)-1):
            cur = vertices[i]
            nxt = vertices[i+1]
            canvas.create_line(cur[0], cur[1], nxt[0], nxt[1], fill=color)
    else:
        flat_verts = []
        for v in list(poly.exterior.coords):
            flat_verts += [v[0], v[1]]
        canvas.create_polygon(*flat_verts, fill=color)

if __name__ == '__main__':
    main()
