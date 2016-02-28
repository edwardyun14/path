from path import *
import tkinter as tk


def draw_grid(grid, canvas):
    for obstacle in grid.obstacles:
        draw_polygon(obstacle, canvas, "blue")
    for col in grid.nodes:
        for node in col:
            if not node.occupied:
                draw_node(node, canvas)
    for col in grid.nodes:
        for node in col:
            if node.occupied:
                draw_node(node, canvas)

def draw_polygon(poly, canvas, fill):
    flat_verts = []
    for v in poly.vertices:
        flat_verts += [v.x, v.y]
    canvas.create_polygon(*flat_verts, fill=fill)

def draw_node(node, canvas):
    fill = "red" if node.occupied else "green"
    draw_rect(node.rect, canvas, fill)

def draw_rect(rect, canvas, fill):
    canvas.create_line(rect.x, rect.y, rect.x2, rect.y, fill=fill)
    canvas.create_line(rect.x, rect.y, rect.x, rect.y2, fill=fill)
    canvas.create_line(rect.x2, rect.y2, rect.x2, rect.y, fill=fill)
    canvas.create_line(rect.x2, rect.y2, rect.x2, rect.y2, fill=fill)

def draw_path(path, canvas):
    for i in range(len(path)-1):
        x, y = path[i]
        x2, y2 = path[i+1]
        canvas.create_line(x, y, x2, y2, fill="purple")

if __name__ == "__main__":
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
    print(find_path(obstacles, (.5, .5), (475.5, 475.5)))

    # testing using GUI
    root = tk.Tk()
    canvas = tk.Canvas(root, width=500, height=500,
                       highlightthickness=0, bg="black")
    canvas.pack()
    grid = OccupancyGrid(
        [Polygon([Vertex(p[0], p[1]) for p in points]) for points in obstacles]
    )
    draw_grid(grid, canvas)
    draw_path(find_path(obstacles, (.5, .5), (475.4, 475.5)), canvas)
    root.mainloop()
