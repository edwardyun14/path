
import json
import visgraph

##def createVertexFromPoint(pt):
##    return path.Vertex(pt[0], pt[1])

#reads in the map described in the file (as generated by the map generation script)
#creates a path from it
def createPathFromFile(filename):
    mapobj=None
    with open(filename, "r") as infile:
        dec=json.JSONDecoder()
        mapobj=dec.decode(infile.read())
    if mapobj:
        return createPathFromMap(mapobj)
    else:
        return None

#creates a path from the given map object (the parsed contents of the map generation script)
def createPathFromMap(mapobj):
    obstacles=[]
    waypoints_l=[] 
    start_points=[] 
    vehicle_size=5 #TODO
    HEIGHT_CONSTANT=3*vehicle_size
    for name, vals in mapobj.items():
        if vals["type"] == "waypoint": #TODO import map gen constants [Future Team]
            center=vals["center"]
            rad=vals["radius"]
            vi=vals["vehicle_i"]

            if vi<len(waypoints_l):
                waypoints_l[vi].append(((center[0], center[1], rad), vals["index"]))
            else:
                while(vi>=len(waypoints_l)):
                    #if this waypoint belongs to a vehicle with index past the end of the list
                    #we need to expand the list to be that long
                    waypoints_l.append([])
                waypoints_l[vi].append(((center[0], center[1], rad), vals["index"]))
            #each element in the waypoints list is a tuple, first element is the waypoint tuple
            #second is the index of the waypoint
            
        elif vals["type"]=="zone":
            if not vals["flight_legal"]:
                #is no fly zone, add as such
                #no longer need to turn into polygon here
                #obstacles.append(Polygon([createVertexFromPoint(p) for p in vals["bounds"]]))

                obstacles.append(vals["bounds"])
            else:
                #TODO
                #this is the "fly zone" for the map, so the area "outside" of it needs to be turned
                #into a no fly zone
                pass
        elif vals["type"] == "start_point":
            start_points.append(((vals["point"][0],vals["point"][1]), vals["vehicle_i"]))
            #much like the waypoint code, each value in this list is a tuple
            #with form (start_point, vehicle_index)
            #so the tuple ((4,1), 2) indicates that the start point for vehicle 2 is (4,1)
            
            
            #below code is used if the vehicle paths need to be found in order of vehicle index
##            while vals["vehicle_i"]>=len(start_points):
##                #expand start point list to be long enough to contain this vehicle's start point
##                start_points.append(None)
##            start_points[vals["vehicle_i"]]=vals["point"]
            

    #sort the waypoints list by index
    for wp in waypoints_l:
        wp.sort(key=lambda w: w[1]) #sort all the waypoint
    #print(start_points)
    paths=[]
    graph = visgraph.VisibilityGraph(obstacles, vehicle_size)
    for start_point, vi in start_points:
        #add the start point to the path                            
        path_list=[start_point]
        #ignore the waypoint index now, its not used
        for wp, _ in waypoints_l[vi]:
            #print(path_list)
            #print(waypoints_l[vi])
            sp=path_list[-1] #our starting point is the most recent position we were in
            path_list+=graph.find_path(sp, wp)
        paths.append(transform_path_3d(path_list, HEIGHT_CONSTANT*(vi+1)))
    return paths #currently will double include middle points, because find_path adds the start point

#changes a path in (x,y) to be in (x,y,z) where z=height
#returns the new path
def transform_path_3d(path, height):
    npath=[]
    for x, y in path:
        npath.append((x,y,height))

    return npath
