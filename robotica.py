# Remover linha abaixo se não for executar no Jupyter Notebook
%matplotlib notebook

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

print('Separar valores decimais com ponto')
print('Sistema para rotação de eixos fixos XYZ, ou euler ZYX')

dx = float(input('Deslocamento da origem do quadro B no eixo x: '))
dy = float(input('Deslocamento da origem do quadro B no eixo y: '))
dz = float(input('Deslocamento da origem do quadro B no eixo z: '))
angX = np.radians(float(input('Rotação em X: ')))
angY = np.radians(float(input('Rotação em Y: ')))
angZ = np.radians(float(input('Rotação em Z: ')))
xB = float(input('Posição Pb no eixo x: '))
yB = float(input('Posição Pb no eixo y: '))
zB = float(input('Posição Pb no eixo z: '))

r11 = np.cos(angZ) * np.cos(angY)
r12 = (np.cos(angZ) * np.sin(angY) * np.sin(angX)) - (np.sin(angZ) * np.cos(angX))
r13 = (np.cos(angZ) * np.sin(angY) * np.cos(angX)) + (np.sin(angZ) * np.sin(angX))
r21 = np.sin(angZ) * np.cos(angY)
r22 = (np.sin(angZ) * np.sin(angY) * np.sin(angX)) + (np.cos(angZ) * np.cos(angX))
r23 = (np.sin(angZ) * np.sin(angY) * np.cos(angX)) - (np.cos(angZ) * np.sin(angX))
r31 = -(np.sin(angY))
r32 = np.cos(angY) * np.sin(angX)
r33 = np.cos(angY) * np.cos(angX)

rot = np.matrix([[r11, r12, r13],
                 [r21, r22, r23],
                 [r31, r32, r33]])

pABorg = np.matrix([[dx], [dy], [dz]])
pB = np.matrix([[xB], [yB], [zB]])
pA = (rot * pB) + pABorg

print('pB:\n{}\nrot:\n{}\npA:\n{}'.format(pB, rot, pA))

x = np.matrix([0, 0, 0, 0, dx, dx, dx, dx, 0])
y = np.matrix([0, 0, 0, 0, dy, dy, dy, dy, 0])
z = np.matrix([0, 0, 0, 0, dz, dz, dz, dz, 0])

u = np.matrix([1, 0, 0, dx, rot[0,0], rot[0,1], rot[0,2], (rot * pB)[0,0], pA[0,0]])
v = np.matrix([0, 1, 0, dy, rot[1,0], rot[1,1], rot[1,2], (rot * pB)[1,0], pA[1,0]])
w = np.matrix([0, 0, 1, dz, rot[2,0], rot[2,1], rot[2,2], (rot * pB)[2,0], pA[2,0]])

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.quiver(x, y, z, u, v, w, arrow_length_ratio=0.1)
ax.set_xlim(pA.min(), pA.max())
ax.set_ylim(pA.min(), pA.max())
ax.set_zlim(pA.min(), pA.max())
ax.set_xlabel('eixo x')
ax.set_ylabel('eixo y')
ax.set_zlabel('eixo z')
plt.show()