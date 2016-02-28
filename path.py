import collections

"""
:param shapes: list of list of tuples (x, y) which
    signify the points of each vertex, with each list of tuples
    being its own shape
:param start: tuple (x, y) which is the starting point
:param dest: tuple (x, y) which is the destination point
:return: list of tuples (x, y) signifying waypoints which can be
    traversed in straight lines until destination
"""
def find_path(shapes, start, dest):
    obstacles = [Polygon([Vertex(p[0], p[1]) for p in points]) for points in shapes]
    grid = OccupancyGrid(obstacles)
    path = grid.find_path(start, dest)
    return [grid.nodes[x][y].center() for x,y in path]

class Circle(object):
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius

class Vertex(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def left_normal(self):
        return Vertex(self.y, -1 * self.x)

    def dot(self, other):
        return self.y * other.x + self.x * other.y

    def __repr__(self):
        return "Vertex({}, {})".format(self.x, self.y)

class Polygon(object):
    def __init__(self, vertices):
        self.vertices = vertices

    def norm(self):
        norms = []
        for i in range(len(self.vertices)-1):
            cur = Vertex(
                self.vertices[i+1].x - self.vertices[i].x,
                self.vertices[i+1].y - self.vertices[i].y
            ).left_normal()
            norms.append(cur)
        norms.append(Vertex(
            self.vertices[0].x - self.vertices[-1].x,
            self.vertices[0].y - self.vertices[-1].y
        ).left_normal())
        return norms

    def intersects(self, other):
        norms1 = self.norm()
        norms2 = other.norm()
        vecs1 = self._prepare_vector()
        vecs2 = other._prepare_vector()
        seperated = False
        for n in norms1:
            result_box1 = self._min_max(vecs1, n)
            result_box2 = self._min_max(vecs2, n)
            seperated = result_box1['max_proj'] < result_box2['min_proj'] or result_box2['max_proj'] < result_box1['min_proj']
            if seperated:
                break
        if not seperated:
            for i in range(1, len(norms2)):
                result_P1 = self._min_max(vecs1, norms2[i])
                result_P2 = self._min_max(vecs2, norms2[i])
                seperated = result_P1['max_proj'] < result_P2['min_proj'] or result_P2['max_proj'] < result_P1['min_proj']
                if seperated:
                    break
        return not seperated


    def _prepare_vector(self):
        vecs_box = []
        for v in self.vertices:
            vecs_box.append(v)
        return vecs_box

    def _min_max(self, vecs_box, axis):
        min_proj_box = vecs_box[0].dot(axis)
        min_dot_box = 0
        max_proj_box = vecs_box[0].dot(axis)
        max_dot_box = 0
        for i in range(1, len(vecs_box)):
            cur = vecs_box[i].dot(axis)
            if (min_proj_box > cur):
                min_proj_box = cur
                min_dot_box = i
            if (cur > max_proj_box):
                max_proj_box = cur
                max_dot_box = i
        return {
            'min_proj': min_proj_box,
            'max_proj': max_proj_box,
            'min_index': min_dot_box,
            'max_index': max_dot_box
        }


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

    def center(self):
        x = (self.rect.x1+self.rect.x2)/2
        y = (self.rect.y1+self.rect.y2)/2
        return x, y

    def intersects(self, poly):
        p1 = Polygon([
            Vertex(self.rect.x1, self.rect.y1),
            Vertex(self.rect.x1, self.rect.y2),
            Vertex(self.rect.x2, self.rect.y2),
            Vertex(self.rect.x2, self.rect.y1),
        ])
        return p1.intersects(poly)

class OccupancyGrid(object):

    """
    :param obstacles: list of Polygons to avoid
    """
    def __init__(self, obstacles):
        self.nodes = []
        for i in range(25):
            self.nodes.append([])
            for j in range(25):
                self.nodes[i].append(OccupancyNode(Rect(i*20, j*20, 20, 20)))

        self.obstacles = obstacles
        self._calculate()

    def _calculate(self):
        for col in self.nodes:
            for node in col:
                for obstacle in self.obstacles:
                    if node.intersects(obstacle):
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
