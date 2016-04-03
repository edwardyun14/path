import visgraph

def format(obstacles, uav_radius, fly_zone, start, end_waypt):
    graph = visgraph.VisibilityGraph(obstacles, uav_radius, fly_zone)
    path = graph.find_path(start, end_waypt)
    with open('waypts.qgc', 'w') as f:
        f.write("QGC WPL 120\n")
        i = 0
        for pt in path:
            i += 1
            cmd = 22 if i == 1 else 21 if i == len(path) else 16  # takeoff, land, nav
            p1 = .261799390000000021 if cmd == 22 else 25 if cmd == 21 else 0
            p2 = 3  # TODO if cmd == 16, then it should have acceptance radius which we must decide
            f.write("{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n".format(
                str(i),  # should be the index
                1 if i == 1 else 0,  # ?? I believe this is that starting point?
                3,  # copied from qgc example
                cmd,  # command, more info https://pixhawk.ethz.ch/mavlink under MAV_CMD
                p1,  # param1
                p2,  # param2
                0,  # param3
                0,  # param4
                pt[1],  # x coord
                pt[0],  # y coord
                25,  # z coord
                1  # autocontinue
            ))

if __name__ == '__main__':
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
    fly_zone = [
        (10, 10),
        (10, 490),
        (490, 490),
        (490, 10)
    ]
    format(obstacles, 5, fly_zone, (30, 30), (450, 450, 20))

