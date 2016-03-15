import collections
from shapely.geometry import Polygon, Point, LineString

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
        for i in range(50):
            self.nodes.append([])
            for j in range(50):
                self.nodes[i].append(OccupancyNode(i*10, j*10, 10, 10))
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
                if node.poly.intersects(Point(start[0], start[1])):
                    start_coord = (i, j)
                if end_point.intersects(node.poly):
                    end_coords.add((i, j))
        if start_coord is None or len(end_coords) == 0 or \
           self.nodes[start_coord[0]][start_coord[1]].occupied:
            raise Exception("start or end is occupied")
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
                # RETURN AS SOON AS HIT VALID END STATE
                return self._post_process_path(path)
                # return self._post_process_path(path)
            for n in self._adjacent_nodes(cur):
                if n not in visited:
                    paths[n] = paths[cur] + [n]
                    q.append(n)
                    visited.add(n)
        print("hi")
        raise Exception("can't find path")

    def _post_process_path(self, path):
        opt_steps = {}
        self._optimize_path(path, 0, opt_steps)
        s = 0
        opt_path = []
        while s != len(path)-1:
            opt_path.append(path[s])
            s = opt_steps[s]
        opt_path.append(path[s])
        return opt_path

    def _get_path_options(self, path):
        path_options = {}
        for i in range(len(path)):
            path_options[i] = []
            for j in range(i+1, len(path)):
                ls = LineString([path[i], path[j]])
                no_intersections = True
                for col in self.nodes:
                    for n in col:
                        if n.occupied and ls.intersects(n.poly):
                            no_intersections = False
                if no_intersections:
                    path_options[i].append(j)
        return path_options

    def _optimize_path(self, path, cur, opt_steps, path_options=None, mem = None):
        if path_options is None:
            path_options = self._get_path_options(path)
        if mem is None:
            mem = {}
        if cur in mem:
            return mem[cur]
        if cur == len(path)-1:
            return 0
        opts = path_options[cur]
        min_path_dst = 2000000000
        opt_nxt = cur+1
        for opt in path_options[cur]:
            dst = LineString([path[cur], path[opt]]).length
            rest = self._optimize_path(path, opt, opt_steps, path_options, mem)
            res = dst + rest
            if res < min_path_dst:
                min_path_dst = res
                opt_nxt = opt
        mem[cur] = min_path_dst
        opt_steps[cur] = opt_nxt
        return min_path_dst

