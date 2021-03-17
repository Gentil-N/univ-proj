import numpy as np
from numpy import *
from numpy.linalg import *
import scipy, scipy.interpolate

from OpenGL.GL import *
from OpenGL.GLU import *

import game

class ParticlesWind(game.Element):
    def __init__(self, x, y, g, nparticles):
        self.xmin, self.width, self.nx = x
        self.ymin, self.height, self.ny = y
        self.g = g
        self.tmin, self.tmax, self.nparticles, self.vmax = 1, 2, nparticles, 1000
        self._nparticles = 1000
        self.potential = zeros((self.ny + 1, self.nx + 1))
        finished = False
        while not finished:
            p = copy(self.potential)
            p[0, :] = 0
            p[-1, :] = 0
            p[:, 0] = 0
            p[:, -1] = 0
            for (gx, gy, gv) in self.g:
                p[int(round(self.ny * (gy - self.ymin) / self.height)), 
                  int(round(self.nx * (gx - self.xmin) / self.width))] = gv
            p = p + 0.1 * self.laplacian(p)
            finished = np.max(abs(p - self.potential)) < 0.1 / (self.nx * self.ny)
            self.potential = p
        gx = linspace(self.xmin, self.xmin + self.width, self.nx + 1)
        gy = linspace(self.ymin, self.ymin + self.height, self.ny + 1)
        self.interpolation = scipy.interpolate.RectBivariateSpline(gx, gy, self.potential.T)
        
        self.density = zeros((self.ny + 1, self.nx + 1))
        self.particles = random.uniform((self.xmin, self.ymin, self.tmin), 
                                        (self.xmin + self.width, self.ymin + self.height, self.tmax), 
                                        (self._nparticles, 3))
        ppp = copy(self.particles)
        n = 0.0
        for k in range(1000):
            a = array((self.interpolation.ev(self.particles[:, 0], self.particles[:, 1], dx = 1), 
                       self.interpolation.ev(self.particles[:, 0], self.particles[:, 1], dy = 1),
                       -ones(self._nparticles))).T
            pp = self.particles + 0.01 * a
            t = ((pp[:, 0] <= self.xmin) + (pp[:, 0] >= self.xmin + self.width) +
                 (pp[:, 1] <= self.ymin) + (pp[:, 1] >= self.ymin + self.height) +
                 (pp[:, 2] <= 0))
            for (gx, gy, gv) in self.g:
                #if gv > 0:
                    t += ((abs(self.particles[:, 0] - gx) < self.width / self.nx) * 
                          (abs(self.particles[:, 1] - gy) < self.height / self.ny))
            self.density[np.round(self.ny * (self.particles[t, 1] - self.ymin) / self.height).astype(int), 
                         np.round(self.nx * (self.particles[t, 0] - self.xmin) / self.width).astype(int)] += norm(
                self.particles[t, :2] - ppp[t, :2], axis = 1)
            n += norm(self.particles[t, :2] - ppp[t, :2], axis = 1).sum()
            pp[t] = random.uniform((self.xmin, self.ymin, self.tmin), 
                                   (self.xmin + self.width, self.ymin + self.height, self.tmax), 
                                   (len(pp[t]), 3))
            ppp[t] = pp[t]
            self.particles = pp
    
        gx = linspace(self.xmin, self.xmin + self.width, self.nx + 1)
        gy = linspace(self.ymin, self.ymin + self.height, self.ny + 1)
        gx = array(range(0, self.nx + 1), float)
        gy = array(range(0, self.ny + 1), float)
        mx, my = meshgrid(gx, gy)
        self.density -= 0 * 0.2 * n / ((self.nx + 1) * (self.ny + 1))
#        for k in range(5):
#            self.density += 0.1 * self.laplacian(self.density)
#        for (gx, gy, gv) in self.g:
#            if gv > 0:
#                x = (gx - self.xmin) * self.nx / self.width
#                y = (gy - self.ymin) * self.ny / self.width
#                self.density[(mx - x) ** 2 + (my - y) ** 2 <= 1] = 0
        self.mesh = array((mx.flatten(), my.flatten())).T
        self.density[self.density < 0] = 0
        self.cumdensity = self.density.cumsum() / self.density.sum()
        
        self.particles = random.uniform((self.xmin, self.ymin, self.tmin), 
                                        (self.xmin + self.width, self.ymin + self.height, self.tmax), 
                                        (self.nparticles, 3))
    
    def __call__(self, x, y):
        return array([self.interpolation(x, y, dx = 1)[0, 0], self.interpolation(x, y, dy = 1)[0, 0]])
    
    def evolve(self, dt):
        t = ((self.particles[:, 0] <= self.xmin) + (self.particles[:, 0] >= self.xmin + self.width) +
             (self.particles[:, 1] <= self.ymin) + (self.particles[:, 1] >= self.ymin + self.height) +
             (self.particles[:, 2] <= 0))
        for (gx, gy, gv) in self.g:
            if gv < 0:
                t += ((abs(self.particles[:, 0] - gx) < self.width / self.nx) *
                      (abs(self.particles[:, 1] - gy) < self.height / self.ny))
        for k in range(self.nparticles):
            if t[k]:
                self.particles[k] = self.generate()
        a = array((self.interpolation.ev(self.particles[:, 0], self.particles[:, 1], dx = 1), 
                   self.interpolation.ev(self.particles[:, 0], self.particles[:, 1], dy = 1),
                   ones(self.nparticles))).T
        n1 = norm(a, axis = 1)
        n2 = n1.copy()
        n1[n1 > self.vmax] = self.vmax
        n = n1 / (n2 + 1e-6)
        a[:, 0] *= n
        a[:, 1] *= n
        self.particles -= 0.01 * a
        return
        
        for k in range(self.nparticles):
            if self.particles[k, 2] <= 0:
                self.particles[k] = self.generate()
            a1, a2 = self.evalGrad(self.particles[k, 0], self.particles[k, 1])
            self.particles[k, 0] -= dt * a1
            self.particles[k, 1] -= dt * a2
            self.particles[k, 2] -= dt
#            if self.particles[k, 0] < self.xmin or self.particles[k, 0] > self.xmin + self.width or \
#               self.particles[k, 1] < self.ymin or self.particles[k, 1] > self.ymin + self.height or \
#               self.particles[k, 2] <= 0:# or self.interpolation(x, y)[0, 0] < 0:
            if self.invalid(self.particles[k]):
                self.particles[k] = self.generate()
            
    def draw(self):
        glColor3d(1.0, 1.0, 1.0)
        glPointSize(1)
        p = array(self.particles[:, :2], double)
        glEnableClientState(GL_VERTEX_ARRAY)
        glVertexPointer(2, GL_DOUBLE, 0, p)
        glDrawArrays(GL_POINTS, 0, self.nparticles)
    
    def invalid(self, p):
        if (p[0] < self.xmin or p[0] > self.xmin + self.width or
            p[1] < self.ymin or p[1] > self.ymin + self.height or
            p[2] <= 0):
            return True
        for (gx, gy, gv) in self.g:
            if gv < 0 and abs(p[0] - gx) < self.width / self.nx and abs(p[1] - gy) < self.height / self.ny:
                return True
        return False
        
    def generate(self):
        finished = False
        k = (self.cumdensity > random.uniform()).argmax()
        return ((random.uniform((-0.5, -0.5, 0), (0.5, 0.5, 1), 3) + array([self.mesh[k, 0], self.mesh[k, 1], 0])) *
                array([self.width / self.nx, self.height / self.ny, self.tmax - self.tmin]) +
                array([self.xmin, self.ymin, self.tmin]))
        
        for (gx, gy, gv) in self.g:
            if gv > 0:
                o = 2 * pi * random.uniform()
                r = min(self.width / self.nx, self.height / self.ny)
                return (gx + r * cos(o), gy + r * sin(o), 10 * random.uniform())
        
    def evalGrad(self, x, y):
        return array([self.interpolation(x, y, dx = 1)[0, 0], self.interpolation(x, y, dy = 1)[0, 0]])
    
    def sampleGrad(self, nx, ny):
        gx = linspace(self.xmin, self.xmin + self.width, nx + 1)
        gy = linspace(self.ymin, self.ymin + self.height, ny + 1)
        dx, dy = zeros((nx + 1, ny + 1)), zeros((nx + 1, ny + 1))
        for kx in range(nx + 1):
            for ky in range(ny + 1):
                dx[ky, kx], dy[ky, kx] = self.evalGrad(gx[kx], gy[ky])
        return dx, dy
    
    def evalPotential(self, x, y):
        return self.interpolation(x, y)[0, 0]
    
    def samplePotential(self, nx, ny):
        gx = linspace(self.xmin, self.xmin + self.width, nx + 1)
        gy = linspace(self.ymin, self.ymin + self.height, ny + 1)
        p = zeros((nx + 1, ny + 1))
        for kx in range(nx + 1):
            for ky in range(ny + 1):
                p[ky, kx] = self.evalPotential(gx[kx], gy[ky])
        return p
    
    @staticmethod
    def laplacian(A):
        return roll(A, 1, axis = 0) + roll(A, -1, axis = 0) + roll(A, 1, axis = 1) + roll(A, -1, axis = 1) - 4 * A

    @staticmethod
    def grad(A):
        return roll(A, -1, axis = 1) - roll(A, 1, axis = 1), roll(A, -1, axis = 0) - roll(A, 1, axis = 0)

