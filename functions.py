import numpy as np  
import math
from edge import Edge

M = N = 512

def get_data(filename):
    data = np.load(filename, allow_pickle=True).item()
    return data

def interpolate_vectors(p1, p2, V1, V2, xy, dim):
    if dim == 1:
        if p1[0] == p2[0]:
            return V1
        l = (xy - p1[0]) / (p2[0] - p1[0])
    else:
        if p1[1] == p2[1]:
            return V1
        l = (xy - p1[1]) / (p2[1] - p1[1])

    # interpolate
    V = (1 - l) * V1 + l * V2
    V = np.clip(V, 0, 1)

    return V

def flats(canvas, vertices, vcolors):
    updatedcanvas = canvas

    # color that will be used
    flat_color = np.mean(vcolors, axis = 0)

    # one point
    if all(vertices[0] == vertices[1]) and all(vertices[1] == vertices[2]):
        updatedcanvas[vertices[0, 1], vertices[0, 0]] = flat_color
        return updatedcanvas

    # save edges
    edges = [Edge() for _ in range(3)]
    edges[0] = Edge(np.array([vertices[0], \
                                 vertices[1]]))
    edges[1] = Edge(np.array([vertices[1], \
                                 vertices[2]]))
    edges[2] = Edge(np.array([vertices[2], \
                                 vertices[0]]))

    y_min = min([edge.y_min[1] for edge in edges])
    y_max = max([edge.y_max for edge in edges])

    # active edges
    actives = 0
    for edge in edges:
        if edge.y_min[1] == y_min:
            edge.active = True
            actives = actives + 1
    border_points = []

    # plain lane
    # hold twice the only actual edge
    if actives == 3:
        for edge in edges:
            if edge.m == float('-inf'):
                edge.active = False
                actives = actives - 1

    # triangle with first edge parallel to x'x
    # find edge with m = 0
    # initialize next as border
    # initialize other edges as actives
    # start from y_min + 1
    if actives == 3:
        for i, edge in enumerate(edges):
            if edge.m == 0:
                # draw first line
                for x in range(edge.vertices[0][0], edge.vertices[1][0] + 1):
                    canvas[y_min, x] = flat_color
                actives = actives - 1
                edge.active = False
            else:
                # border points: xk, m, edge
                border_points.append([edge.y_min[0] + 1 / edge.m, edge.m, i])
        y_min = y_min + 1

    # normal cases
    if len(border_points) == 0:
        for i, edge in enumerate(edges):
            if edge.active:
                border_points.append([edge.y_min[0], edge.m, i])

    for y in range(y_min, y_max + 1):
        # sort points
        border_points = sorted(border_points, key=lambda x: x[0])

        # draw between points
        for x in range(math.floor(border_points[0][0] + 0.5), \
                       math.floor(border_points[1][0] + 0.5) + 1):
            canvas[y, x] = flat_color

        if y == y_max:
            break

        # move previous points
        for point in border_points:
            point[0] = point[0] + 1 / point[1]

        # add new edge
        # and its point
        for i, edge in enumerate(edges):
            if edge.y_min[1] == y + 1:
                edge.active = True
                actives = actives + 1
                border_points.append([edge.y_min[0], edge.m, i])

        # remove if three
        # and remove its border point
        if actives == 3:
            # only one edge case for m = 0 on last edge
            if border_points[-1][1] == 0:
                del border_points[-1]
                continue
            for i, edge in enumerate(edges):
                if edge.y_max == y + 1:
                    if border_points[0][2] == i:
                        del border_points[0]
                    else:
                        del border_points[1]
                    edge.active = False
                    actives = actives - 1
                    break


    return updatedcanvas

def Gourauds(canvas, vertices, vcolors):
    updatedcanvas = canvas

    if all(vertices[0] == vertices[1]) and all(vertices[1] == vertices[2]):
        updatedcanvas[vertices[0, 1], vertices[0, 0]] = np.mean(vcolors, axis=0)
        return updatedcanvas

    edges = [Edge() for _ in range(3)]
    edges[0] = Edge(np.array([vertices[0], \
                                 vertices[1]]))
    edges[1] = Edge(np.array([vertices[1], \
                                 vertices[2]]))
    edges[2] = Edge(np.array([vertices[2], \
                                 vertices[0]]))

    y_min = min([edge.y_min[1] for edge in edges])
    y_max = max([edge.y_max for edge in edges])

    actives = 0
    for edge in edges:
        if edge.y_min[1] == y_min:
            edge.active = True
            actives = actives + 1
    border_points = []

    if actives == 3:
        for edge in edges:
            if edge.m == float('-inf'):
                edge.active = False
                actives = actives - 1

    if actives == 3:
        for i, edge in enumerate(edges):
            if edge.m == 0:
                for x in range(edge.vertices[0][0], edge.vertices[1][0] + 1):
                    canvas[y_min, x] = interpolate_vectors(edge.vertices[0], edge.vertices[1],\
                                                           vcolors[i], vcolors[(i + 1) % 3], \
                                                           x, 1)
                actives = actives - 1
                edge.active = False
            else:
                border_points.append([edge.y_min[0] + 1 / edge.m, edge.m, i])
        y_min = y_min + 1

    if len(border_points) == 0:
        for i, edge in enumerate(edges):
            if edge.active:
                border_points.append([edge.y_min[0], edge.m, i])

    for y in range(y_min, y_max + 1):
        border_points = sorted(border_points, key=lambda x: x[0])

        # find color in border scanline points
        color_A = interpolate_vectors(edges[border_points[0][2]].vertices[0], \
                                      edges[border_points[0][2]].vertices[1], \
                                      vcolors[border_points[0][2]], \
                                      vcolors[(border_points[0][2] + 1) % 3], \
                                      y, 2)
        color_B = interpolate_vectors(edges[border_points[1][2]].vertices[0], \
                                      edges[border_points[1][2]].vertices[1], \
                                      vcolors[border_points[1][2]], \
                                      vcolors[(border_points[1][2] + 1) % 3], \
                                      y, 2)

        for x in range(math.floor(border_points[0][0] + 0.5), \
                       math.floor(border_points[1][0] + 0.5) + 1):
            # find color in scanline
            canvas[y, x] = interpolate_vectors(np.array([math.floor(border_points[0][0] + 0.5), y]), \
                                               np.array([math.floor(border_points[1][0] + 0.5) + 1, y]), \
                                               color_A, color_B, x, 1)

        if y == y_max:
            break

        for point in border_points:
            point[0] = point[0] + 1 / point[1]

        for i, edge in enumerate(edges):
            if edge.y_min[1] == y + 1:
                edge.active = True
                actives = actives + 1
                border_points.append([edge.y_min[0], edge.m, i])

        if actives == 3:
            ## only one edge case for m = 0 on last edge
            if border_points[-1][1] == 0:
                del border_points[-1]
                continue
            for i, edge in enumerate(edges):
                if edge.y_max == y + 1:
                    if border_points[0][2] == i:
                        del border_points[0]
                    else:
                        del border_points[1]
                    edge.active = False
                    actives = actives - 1
                    break


    return updatedcanvas

def render(verts2d, faces, vcolors, depth, shade_t):
    # calculate depth of each triagle
    triangles_depth = np.array(np.mean(depth[faces], axis = 1))

    # sort faces triangles depth
    indices = np.flip(np.argsort(triangles_depth))
    triangles_depth = triangles_depth[indices]
    faces = faces[indices]

    img = np.ones((M, N, 3))

    for face in faces:
        if shade_t == 'gouraud':
            img = Gourauds(img, verts2d[face], vcolors[face])
        elif shade_t == 'flat':
            img = flats(img, verts2d[face], vcolors[face])
        else:
            print("Not compatible shading method")
            break

    return img