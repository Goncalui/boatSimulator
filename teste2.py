import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Criação dos pontos para o barco
z = np.array([0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])
y = np.array([2, 1, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
x = np.array([0, 1, 2, 0, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

# Criação da figura e do eixo 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Desenho do barco
ax.plot(x, y, z, c='k')
ax.plot(x, y, z + 1, c='k')
ax.plot(x + 2, y, z, c='k')
ax.plot(x + 2, y, z + 1, c='k')

# Customização da visualização
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Barco em 3D')

plt.show()