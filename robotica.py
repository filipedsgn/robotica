# Remover linha abaixo se não for executar no Jupyter Notebook
# %matplotlib notebook

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

print('Separar valores decimais com ponto')
print('Sistema para rotação de eixos fixos XYZ, ou euler ZYX')

class Frame(object):

    def __init__(self, dados):

        global x, y, z, u, v, w

        self.angX, self.angY, self.angZ, self.dx, self.dy, self.dz, self.xB, self.yB, self.zB = dados
        self.angX = np.radians(self.angX)
        self.angY = np.radians(self.angY)
        self.angZ = np.radians(self.angZ)
        self.r11 = np.cos(self.angZ) * np.cos(self.angY)
        self.r12 = (np.cos(self.angZ) * np.sin(self.angY) * np.sin(self.angX)) - (np.sin(self.angZ) * np.cos(self.angX))
        self.r13 = (np.cos(self.angZ) * np.sin(self.angY) * np.cos(self.angX)) + (np.sin(self.angZ) * np.sin(self.angX))
        self.r21 = np.sin(self.angZ) * np.cos(self.angY)
        self.r22 = (np.sin(self.angZ) * np.sin(self.angY) * np.sin(self.angX)) + (np.cos(self.angZ) * np.cos(self.angX))
        self.r23 = (np.sin(self.angZ) * np.sin(self.angY) * np.cos(self.angX)) - (np.cos(self.angZ) * np.sin(self.angX))
        self.r31 = -(np.sin(self.angY))
        self.r32 = np.cos(self.angY) * np.sin(self.angX)
        self.r33 = np.cos(self.angY) * np.cos(self.angX)

        self.rot = np.matrix([[self.r11, self.r12, self.r13],
                              [self.r21, self.r22, self.r23],
                              [self.r31, self.r32, self.r33]])

        self.aPborg = np.matrix([[self.dx], [self.dy], [self.dz]])
        self.pB = np.matrix([[self.xB], [self.yB], [self.zB]])
        self.pA = (self.rot * self.pB) + self.aPborg

        x = np.append(x, [[self.dx, self.dx, self.dx, self.dx, 0, 0]], axis=1)
        y = np.append(y, [[self.dy, self.dy, self.dy, self.dy, 0, 0]], axis=1)
        z = np.append(z, [[self.dz, self.dz, self.dz, self.dz, 0, 0]], axis=1)
        u = np.append(u, [[self.rot[0,0], self.rot[0,1], self.rot[0,2],
                           (self.rot * self.pB)[0,0], self.pA[0,0], self.aPborg[0,0]]], axis=1)
        v = np.append(v, [[self.rot[1,0], self.rot[1,1], self.rot[1,2],
                           (self.rot * self.pB)[1,0], self.pA[1,0], self.aPborg[0,0]]], axis=1)
        w = np.append(w, [[self.rot[2,0], self.rot[2,1], self.rot[2,2],
                           (self.rot * self.pB)[2,0], self.pA[2,0], self.aPborg[0,0]]], axis=1)
        self.info()

    def info(self):
        print('x({}°) y({}°) z({}°)'.format(np.degrees(self.angX), np.degrees(self.angY), np.degrees(self.angZ)))
        print('dx({}), dy({}), dz({})'.format(self.dx, self.dy, self.dz))
        print('Matriz de rotação:\n{}\n'.format(self.rot))
        print('pB: {}'.format(self.pB.T))
        print('pA: {}'.format(self.pA.T))

x = np.matrix([0, 0, 0])
y = np.matrix([0, 0, 0])
z = np.matrix([0, 0, 0])
u = np.matrix([1, 0, 0])
v = np.matrix([0, 1, 0])
w = np.matrix([0, 0, 1])

frames = []
quant = int(input('quantidade de quadros: '))

for i in range(1, (quant + 1)):
    print('Sistema{}, insira os dados separados por espaço: '.format(i))
    print('AngX, AngY, AngZ, Trans. X, Trans. Y, Trans. Z, bP(x), bP(y), bP(z)')
    frames.append(Frame(list(map(float, input().split()))))

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.quiver(x, y, z, u, v, w, arrow_length_ratio=0.1)
ax.set_xlabel('eixo x')
ax.set_ylabel('eixo y')
ax.set_zlabel('eixo z')
plt.show()