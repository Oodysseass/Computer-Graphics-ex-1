import numpy as np
from functions import get_data, render
from matplotlib import pyplot as plt

# get data
data = get_data('h1.npy')
verts2d = np.array(data['verts2d'])
vcolors = np.array(data['vcolors'])
faces = np.array(data['faces'])
depth = np.array(data['depth'])

img = render(verts2d, faces, vcolors, depth, 'flat')

plt.imsave('flat.pdf', img)
plt.imshow(img)
plt.show()

