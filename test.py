import tkinter as tk
import collections

class Rect(object):
    def __init__(self, x, y, w, h):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.x1 = x
        self.y1 = y
        self.x2 = x + w
        self.y2 = y + h

    def draw(self, canvas, fill):
        canvas.create_line(self.x, self.y, self.x2, self.y, fill=fill)
        canvas.create_line(self.x, self.y, self.x, self.y2, fill=fill)
        canvas.create_line(self.x2, self.y2, self.x2, self.y, fill=fill)
        canvas.create_line(self.x2, self.y2, self.x, self.y2, fill=fill)

    def intersectRect(self, other):
        return self.x < (other.x + other.w) and \
               (self.x + self.w) > other.x and \
               self.y < (other.y + other.h) and \
               (self.y + self.y) > other.y

    def contains(self, x, y):
        return x > self.x1 and x < self.x2 and \
                y > self.y1 and y < self.y2

    def __str__(self):
        return "Rect({}, {}, {}, {})".format(self.x, self.y,
                                             self.w, self.h)

    def __repr__(self):
        return str(self)

class OccupancyNode(object):

    def __init__(self, rect):
        self.rect = rect
        self.occupied = False

    def draw(self, canvas):
        fill = "red" if self.occupied else "green"
        self.rect.draw(canvas, fill)

class OccupancyGrid(object):

    def __init__(self):
        self.nodes = []
        for i in range(10):
            self.nodes.append([])
            for j in range(10):
                self.nodes[i].append(OccupancyNode(Rect(i*50, j*50, 50, 50)))
        self.obstacles = [
            Rect(75, 75, 100, 150),
            Rect(275, 75, 200, 200)
        ]
        self._calculate()

    def draw(self, canvas):
        for col in self.nodes:
            for node in col:
                if not node.occupied:
                    node.draw(canvas)
        for col in self.nodes:
            for node in col:
                if node.occupied:
                    node.draw(canvas)
        for obstacle in self.obstacles:
            obstacle.draw(canvas, "blue")

    def draw_path(self, path, canvas):
        for i, j in path:
            node = self.nodes[i][j]
            canvas.create_rectangle(node.rect.x1,
                                    node.rect.y1,
                                    node.rect.x2,
                                    node.rect.y2,
                                    fill="purple")

    def _calculate(self):
        for col in self.nodes:
            for node in col:
                for obstacle in self.obstacles:
                    if node.rect.intersectRect(obstacle):
                        node.occupied = True

    def _adjacent_nodes(self, node_coords):
        i, j = node_coords
        tmp = [
            (i+1, j),
            (i-1, j),
            (i, j+1),
            (i, j-1)
        ]
        res = []
        for t in tmp:
            if self._in_bounds(t) and \
               not self.nodes[t[0]][t[1]].occupied:
                res.append(t)
        return res

    def _in_bounds(self, coords):
        i, j = coords
        return i >= 0 and j >= 0 and \
                i < len(self.nodes) and j < len(self.nodes[0])

    def find_path(self, start, end):
        start_coord = None
        end_coord = None
        for i, col in enumerate(self.nodes):
            for j, node in enumerate(col):
                if node.rect.contains(start[0], start[1]):
                    start_coord = (i, j)
                if node.rect.contains(end[0], end[1]):
                    end_coord = (i, j)
        if start_coord is None or end_coord is None or \
            self.nodes[start_coord[0]][start_coord[1]].occupied or \
            self.nodes[end_coord[0]][end_coord[1]].occupied:
            raise Exception("can't find path")
        visited = {start_coord}
        q = collections.deque()
        q.append(start_coord)
        paths = {start_coord: [start_coord]}
        while len(q) > 0:
            cur = q.popleft()
            if cur == end_coord:
                return paths[cur]
            for n in self._adjacent_nodes(cur):
                if n not in visited:
                    paths[n] = paths[cur] + [n]
                    q.append(n)
                    visited.add(n)
        raise Exception("can't find path")

if __name__ == "__main__":
    root = tk.Tk()
    canvas = tk.Canvas(root, width=500, height=500,
                       highlightthickness=0, bg="black")
    canvas.pack()
    grid = OccupancyGrid()
    grid.draw(canvas)
    path = grid.find_path((1, 1), (475, 475))
    print(path)
    grid.draw_path(path, canvas)

    root.mainloop()
