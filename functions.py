import numpy as np  
import math
from edge import Edge

def get_data(filename):
    data = np.load(filename, allow_pickle=True).item()
    return data

def interpolate_vectors(p1, p2, V1, V2, xy, dim):
    ## find line connecting p1-p2
    # parallel to xx
    if p1[1] == p2[1]:
        # obviously dim == 1
        p = np.array([xy, p1[1]])

    # vertical line
    elif p1[0] == p2[0]:
        # obviously dim == 2
        p = np.array([p1[0], xy])

    # normal line
    else:
        m = (p2[1] - p1[1]) / (p2[0] - p1[0])
        b = p1[1] - m * p1[0]
        if dim == 1:
            p = np.array([xy, int(m * xy + b)])
        elif dim == 2:
            p = np.array([int((xy - b) / m), xy])

    ## find p-p1 and p1-p2 distance
    # p-p1
    d1 = np.linalg.norm(p - p1)
    
    # p1-p2
    d2 = np.linalg.norm(p1 - p2)

    ## interpolation
    # coefficient
    l = d1 / d2

    # interpolate
    V = l * V1 + (1 - l) * V2

    return p, V

def flats(canvas, vertices, vcoloros):
    updated_canvas = canvas

    ## save edges
    edges = [Edge() for _ in range(3)]
    for i in range(3):
        if i == 2:
            edges[i] = Edge(i, np.array([vertices[(i + 1) % 3, :], vertices[i, :]]))
        else:
            edges[i] = Edge(i, np.array([vertices[i, :], vertices[(i + 1) % 3, :]]))

    ## find overall min and max
    y_min = min([edge.y_min[1] for edge in edges])
    y_max = max([edge.y_max[1] for edge in edges])

    active_edges = []
    horizontal = False

    ## find active edges for y == ymin
    for edge in edges:
        # found edge
        if edge.y_min[1] == y_min:
            # active edge is horizontal
            if edge.y_min[1] == edge.y_max[1]:
                horizontal = True
            else:
                edge.active = True
                active_edges.append(edge)

    ## if edge with ymin is horizontal
    # whole line is drawed
    if horizontal:
        pass

    ## find border points
    # a border point is: xk, yk, mk, on which edge
    border_points = [[active_edges[0].y_min[0], active_edges[0].m, active_edges[0].ordinal], \
                     [active_edges[1].y_min[0], active_edges[1].m, active_edges[1].ordinal]]

    ### scanlines
    for y in range(y_min, y_max):
        border_points = sorted(border_points, key=lambda x: x[0])

        ## scan x between border points
        for x in range(math.floor(border_points[0][0] + 0.5), \
                       math.floor(border_points[1][0] + 0.5)):
            # draw pixel
            # reverse due to how it is rendered by imshow later
            updated_canvas[y, x, :] = [0, 0, 0]

        ## refresh active edges
        # add any new ones
        for edge in edges:
            if edge.y_min[1] == y + 1:
                edge.active = True
                active_edges.append(edge)

        # remove any old ones
        for i, active_edge in enumerate(active_edges):
            if active_edge.y_max[1] == y:
                active_edges[i].active = False
                del active_edges[i]
                break

        ## refresh border points
        # remove old points
        temp = []
        for point in border_points:
            if edges[point[2]].active:
                temp.append(point)
        border_points = temp

        # add 1 / m to previous points
        for point in border_points:
            point[0] = point[0] + 1 / point[1]

        # add points of edges with ykmin == y + 1
        for active_edge in active_edges:
            if active_edge.vertices[0, 1] == y + 1:
                border_points.append([active_edge.vertices[0, 0], active_edge.m, active_edge.ordinal])
            elif active_edge.vertices[1, 1] == y + 1:
                border_points.append([active_edge.vertices[1, 0], active_edge.m, active_edge.ordinal])


    return updated_canvas
