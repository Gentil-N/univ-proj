import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *

from numpy import *
from numpy.linalg import *

import PIL, PIL.Image

class Game:
    def __init__(self, area = [-1, 1, -1, 1], export = False, path = 'img', framerate = 50, duration = 5000, size = [800, 600]):
        pygame.init()
        pygame.key.set_repeat(400, 30)
        self.area = zeros(4)
        self.area[:] = area
        self.elements = []
        self.export = export
        self.path = path
        self.framerate = framerate
        self.maxindex = duration * framerate // 1000
        self.size = size
    
    def start(self):
#        pygame.display.gl_set_attribute(GL_MULTISAMPLEBUFFERS, 1)
        if self.export:
            pygame.display.set_mode((self.size[0], self.size[1]), DOUBLEBUF | OPENGL)
        else:
            pygame.display.set_mode((self.size[0], self.size[1]), DOUBLEBUF | OPENGL | RESIZABLE)
        try:
#            glEnable(GL_MULTISAMPLE)
            pygame.event.post(pygame.event.Event(VIDEORESIZE, w = self.size[0], h = self.size[1]))
            self.terminated = False
            if self.export:
                self.time = 0.0
                self.index = 0
            else:
                self.time = pygame.time.get_ticks() * 1e-3
            ready = False
            while not self.terminated:
                event = pygame.event.wait()
                if event.type == QUIT:
                    self.terminated = True
                if event.type == VIDEORESIZE:
                    cx, cy = 0.5 * (self.area[0] + self.area[1]), 0.5 * (self.area[2] + self.area[3])
                    w, h = self.area[1] - self.area[0], self.area[3] - self.area[2]
                    ws, hs = max(1, event.w), max(1, event.h)
                    if ws * h > w * hs:
                        w = h * ws / hs
                    else:
                        h = w * hs / ws
                    glViewport(0, 0, event.w, event.h)
                    glMatrixMode(GL_PROJECTION)
                    glLoadIdentity()
                    gluOrtho2D(cx - 0.5 * w, cx + 0.5 * w, cy - 0.5 * h, cy + 0.5 * h)
                    ready = True
                elif event.type == VIDEOEXPOSE:
                    if ready:
                        if self.export:
                            if (self.index >= self.maxindex):
                                self.stop()
                            else:
                                self.evolve(1 / self.framerate)
                                self.time += 1 / self.framerate
                                self.draw()
                                data = glReadPixels(0, 0, self.size[0], self.size[1], GL_RGB, GL_UNSIGNED_BYTE)
                                i = PIL.Image.frombytes("RGB", (self.size[0], self.size[1]), data).transpose(PIL.Image.FLIP_TOP_BOTTOM)
                                i.save(self.path + '{:0>4d}.png'.format(self.index))
                                pygame.display.flip()
                                pygame.event.post(pygame.event.Event(VIDEOEXPOSE))
                                self.index += 1
                        else:
                            t = pygame.time.get_ticks() * 1e-3
                            self.evolve(t - self.time)
                            self.time = t
                            self.draw()
                            pygame.display.flip()
                            pygame.event.post(pygame.event.Event(VIDEOEXPOSE))
                    else:
                        pygame.event.post(pygame.event.Event(VIDEOEXPOSE))

                elif event.type == KEYDOWN:
                    if event.key == K_UP:
                        self.input(0, 1)
                    elif event.key == K_DOWN:
                        self.input(0, -1)
                    elif event.key == K_RIGHT:
                        self.input(1, 0)
                    elif event.key == K_LEFT:
                        self.input(-1, 0)
                    elif event.key == K_q:
                        self.stop()
                    elif event.key == K_ESCAPE:
                        self.terminated = True
        finally:
            pygame.quit()
            
    def evolve(self, dt):
        for e in self.elements:
            e.evolve(dt)
    
    def draw(self):
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glClear(GL_COLOR_BUFFER_BIT)
        for e in self.elements:
            e.draw()
            
    def input(self, x, y):
        for e in self.elements:
            e.input(x, y)
        
    def stop(self):
        pygame.event.post(pygame.event.Event(QUIT))

class Element:
    def evolve(self, deltat):
        pass
    
    def draw(self):
        pass

    def input(self, x, y):
        pass

class Bound(Element):
    def intersect(self, p1, p2, r):
        pass

class BaseFlatWall(Bound):
    def __init__(self, p, n):
        self.P = zeros(2)
        self.P[:] = p
        self.N = zeros(2)
        self.N[:] = n
        self.N /= norm(self.N)
        self.U = array([self.N[1], -self.N[0]])
        
    def draw(self):
        glColor4d(0, 0, 1, 1)
        glBegin(GL_QUADS)
        for (x, y) in [(-100, 0), (100, 0), (100, -100), (-100, -100)]:
            glVertex2dv(self.P + x * self.U + y * self.N)
        glEnd()

class BaseRoundWall(Bound):
    def __init__(self, p, r):
        self.P = zeros(2)
        self.P[:] = p
        self.R = r
        
    def draw(self):
        n = 100
        p = zeros((2 * n, 2))
        o = linspace(0, 2 * pi, n)
        p[0::2, 0] = self.P[0] + self.R * cos(o)
        p[0::2, 1] = self.P[1] + self.R * sin(o)
        p[1::2, 0] = self.P[0] + 200 * cos(o)
        p[1::2, 1] = self.P[1] + 200 * sin(o)
        
        glColor4d(0, 0.5, 1, 1)
        glEnableClientState(GL_VERTEX_ARRAY)
        glVertexPointer(2, GL_DOUBLE, 0, p)
        glDrawArrays(GL_TRIANGLE_STRIP, 0, len(p))

class BaseBall(Element):
    def __init__(self, R = 1.0, P0 = [0, 0], dP0 = [0, 0], O0 = 0.0, dO0 = 0.0):
        self.R = R
        self.P = zeros(2)
        self.P[:] = P0
        self.dP = zeros(2)
        self.dP[:] = dP0
        self.O = O0
        self.dO = dO0
    
    def draw(self):
        n = 100
        p = zeros((n + 1, 2))
        p[0] = self.P
        o = linspace(0, 2 * pi, n)
        p[1:, 0] = self.P[0] + self.R * cos(o)
        p[1:, 1] = self.P[1] + self.R * sin(o)
        
        glColor4d(1.0, 0.0, 0.0, 1.0)
        glEnableClientState(GL_VERTEX_ARRAY)
        glVertexPointer(2, GL_DOUBLE, 0, p)
        glDrawArrays(GL_TRIANGLE_FAN, 0, len(p))
        
        glColor4d(1.0, 1.0, 1.0, 1.0)
        glBegin(GL_LINES)
        glVertex2f(self.P[0], self.P[1])
        glVertex2f(self.P[0] + self.R * cos(self.O), self.P[1] + self.R * sin(self.O))
        glEnd()

    def input(self, x, y):
        self.dP[0] += x
        self.dP[1] += y
