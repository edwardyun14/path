from shapely.geometry import Polygon, Point, LineString
import heapq
from math import *

class PriorityQueue(object):
    def __init__(self):
        self._items = []
        self._index = 0

    def push(self, item, priority):
        heapq.heappush(self._items, (priority, self._index, item))
        self._index += 1

    def pop(self):
        return heapq.heappop(self._items)[2]

    def empty(self):
        return len(self._items) == 0


class VisibilityNode(object):

    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r
        self.circle = Point(x, y).buffer(r)


class VisibilityGraph(object):

    def __init__(self, obstacles, uav_radius, fly_zone): 
        self.uav_radius = uav_radius
        self.obstacles = [Polygon(o) for o in obstacles]
        self.fly_zone = Polygon(fly_zone)
        self.nodes = []
        for obstacle in obstacles:
            for i in range(len(obstacle)):
                j = obstacle[(i+1) % len(obstacle)]
                k = obstacle[(i-1) % len(obstacle)]
                cur = obstacle[i]
                r1 = atan2(j[1]-cur[1], j[0]-cur[0])
                r2 = atan2(k[1]-cur[1], k[0]-cur[0])
                if abs(r1-r2) > pi:
                    if r1 < 0:
                        r1 += 2*pi
                    elif r2 < 0:
                        r2 += 2*pi
                r = (r1+r2)/2
                y = sin(r)
                x = cos(r)
                x, y = -x*uav_radius, -y*uav_radius
                # mult by 1.5 since it's slightly larger than sqrt 2 (~1.414)
                node = VisibilityNode(cur[0]+x*1.5, cur[1]+y*1.5, uav_radius)
                noIntersection = True
                for p in self.obstacles:
                    if p.intersects(node.circle):
                        noIntersection = False
                if not self.fly_zone.contains(node.circle):
                    noIntersection = False
                if noIntersection:
                    self.nodes.append(node)

    def find_path(self, start, waypt):
        end_coords = [(waypt[0], waypt[1])]
        for i in range(8):
            r = i * pi / 4
            x, y = cos(r), sin(r)
            c = (waypt[0]+waypt[2]*x-x*self.uav_radius, waypt[1]+waypt[2]*y-y*self.uav_radius)
            #print(c, x, y)
            end_coords.append(c)
        visited = {start}
        q = PriorityQueue()
        q.push(start, 0)
        paths = {start: [start]}
        while not q.empty():
            cur = q.pop()
            if cur in end_coords:
                return paths[cur]
            for coord in self.get_reachable(cur, end_coords):
                if coord not in visited:
                    visited.add(coord)
                    paths[coord] = paths[cur]+[coord]
                    score = self.cost_of_path(paths[coord])
                    q.push(coord, score)
                elif self.cost_of_path(paths[coord]) > self.cost_of_path(paths[cur]+[coord]):
                    paths[coord] = paths[cur]+[coord]
                    q.push(coord, score)
        raise Exception("can't find path")

    def cost_of_path(self, path):
        cost = 0
        for i in range(len(path)-1):
            c1 = path[i]
            c2 = path[i+1]
            cost += ( (c1[0]-c2[0])**2 + (c1[1]-c2[1])**2 )**.5
        return cost

    def _add_pairs(self, t1, t2):
        return (t1[0]+t2[0], t1[1]+t2[1])

    # take coordinate and extra searchable coords, get reachable node coords
    def get_reachable(self, coord, extra_coords):
        res = []
        for c in [(node.x, node.y) for node in self.nodes] + extra_coords:
            if c == coord:
                continue

            dirvec = (c[0]-coord[0], c[1]-coord[1])
            norm = (dirvec[0]**2 + dirvec[1]**2)**.5
            scl = self.uav_radius / norm
            norvecs = [(v[0]*scl, v[1]*scl) for v in [(-dirvec[1], dirvec[0]), (dirvec[1], -dirvec[0])]]
            corners = [
                (c[0]+norvecs[0][0], c[1]+norvecs[0][1]),
                (c[0]+norvecs[1][0], c[1]+norvecs[1][1]),
                (coord[0]+norvecs[0][0], coord[1]+norvecs[0][1]),
                (coord[0]+norvecs[1][0], coord[1]+norvecs[1][1])
            ]
            path_poly = Polygon(corners)

            # ls = LineString([coord, c])
            canReach = True
            for o in self.obstacles:
                # if ls.intersects(o):
                    # canReach = False
                if path_poly.intersects(o):
                    canReach = False
            #if not self.fly_zone.contains(ls):
                #canReach = False
            if canReach:
                res.append(c)
        return res
