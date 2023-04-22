import numpy as np

class Edge:
    def __init__(self, ordinal=None, vertices=None):
        if ordinal is not None and vertices is not None:
            self.ordinal = ordinal
            self.vertices = vertices
            if vertices[0, 1] < vertices[1, 1]:
                self.y_min = vertices[0, :]
                self.y_max = vertices[1, :]
            else:
                self.y_min = vertices[1, :]
                self.y_max = vertices[0, :]
            if self.y_min[0] != self.y_max[0]:
                self.m = (vertices[0, 1] - vertices[1, 1]) / (vertices[0, 0] - vertices[1, 0])
            else:
                self.m = float('inf')
            self.active = False
        else:
            self.ordinal = None
            self.vertices = None
            self.y_min = None
            self.y_max = None
            self.m = None
            self.active = None
    
    def __str__(self):
        return f"Edge ordinal: {self.ordinal}, vertices: {self.vertices}, y_min: {self.y_min}, y_max: {self.y_max}, m: {self.m}"


