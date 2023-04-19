import numpy as np
from functions import get_data, flats

# canvas dimensions
M = N = 512

# get data
data = get_data('h1.npy')
verts2d = np.array(data['verts2d'])
vcolors = np.array(data['vcolors'])
faces = np.array(data['faces'])
depth = np.array(data['depth'])


