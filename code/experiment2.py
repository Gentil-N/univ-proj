from numpy import *
import ball as ball
import wind

b = ball.Ball(R = 0.15, P0 = [0.0, 0.0], dP0 = [0 * 4 * 5, 0 * 4 * 3], dO0 = 10.0)

Area = [-0.9, 0.9, -0.8, 0.9]
Elements = []
Wind = None

n = 5
r = 0.7
s = [(0, 0, -1)]
for k in range(n):
    o = 2 * k * pi / n + 3 * pi / 2 - 0.1
    w = ball.FlatWall([r * cos(o), r * sin(o)], [-cos(o), -sin(o)])
    ball.Bounds.append(w)
    s.append((0.8 * cos(o + pi / n), 0.8 * sin(o + pi / n), 1 / n))

Elements += ball.Bounds
ball.Wind = Wind
if (Wind):
    Elements.insert(0, Wind)
Elements.append(b)