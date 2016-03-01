import collections
from shapely.geometry import Polygon, Point

class OccupancyNode(object):
    def __init__(self, x, y, w, h):
        self.poly = Polygon([
            (x, y),
            (x+w, y),
            (x+w, y+h),
            (x, y+h)
        ])
        self.occupied = False
        self.center = ((2*x+w)/2, (2*y+h)/2)

class OccupancyGrid(object):

    """
    :param obstacles: list of shapes to avoid, each shape being its
        list of vertices
    """
    def __init__(self, obstacles):
        self.nodes = []
        for i in range(25):
            self.nodes.append([])
            for j in range(25):
                self.nodes[i].append(OccupancyNode(i*20, j*20, 20, 20))
        self.obstacles = [Polygon(o) for o in obstacles]
        for col in self.nodes:
            for node in col:
                for obstacle in self.obstacles:
                    if node.poly.intersects(obstacle):
                        node.occupied = True

    def _adjacent_nodes(self, node_coords):
        i, j = node_coords
        tmp = [
            (i+1, j),
            (i-1, j),
            (i, j-1),
            (i, j+1)
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

    """
    :param start: tuple (x, y) denoting starting coordinates
    :param end_waypoint: tuple (x, y, r) denoting a circular shaped waypoint
    :return: array of tuples [(x, y)...] which denotes points to be taken
        in a straight light in sequential order to get to destination
    :throws: Exception("can't find path") if there's not possible pathing
        in current resolution, etc.
    """
    def find_path(self, start, end_waypoint):
        start_coord = None
        end_coords = set([])
        end_point = Point(end_waypoint[0], end_waypoint[1]) \
                .buffer(end_waypoint[2])
        for i, col in enumerate(self.nodes):
            for j, node in enumerate(col):
                if node.poly.contains(Point(start[0], start[1])):
                    start_coord = (i, j)
                if end_point.contains(node.poly):
                    end_coords.add((i, j))
        if start_coord is None or len(end_coords) == 0 or \
           self.nodes[start_coord[0]][start_coord[1]].occupied:
            raise Exception("can't find path")
        visited = {start_coord}
        q = collections.deque()
        q.append(start_coord)
        paths = {start_coord: [start_coord]}
        while len(q) > 0:
            cur = q.popleft()
            if cur in end_coords:
                # return paths[cur]
                path = [self.nodes[x][y].center for x,y in paths[cur]]
                if start != path[0]:
                    path = [start] + path
                return path
            for n in self._adjacent_nodes(cur):
                if n not in visited:
                    paths[n] = paths[cur] + [n]
                    q.append(n)
                    visited.add(n)
        print("hi")
        raise Exception("can't find path")

