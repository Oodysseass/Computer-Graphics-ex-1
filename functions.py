import numpy as np  

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
            p = np.array([xy, m * xy + b])
        elif dim == 2:
            p = np.array([(xy - b) / m, xy])

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

    return V

