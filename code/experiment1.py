import ball as ball

w1 = ball.FlatWall([0, 0], [1, 0])
w2 = ball.FlatWall([0, 0], [0, 1])
w3 = ball.FlatWall([10, 10], [-1, 0])
w4 = ball.FlatWall([10, 10], [0, -1])
b = ball.Ball(R = 0.5, P0 = [5.0, 5.5], dP0 = [5, 0], dO0 = -5.0)

Area = [-1, 11, -1, 11]
Elements = [w1, w2, w3, w4, b]
Wind = None

ball.Bounds = [w1, w2, w3, w4]
ball.Wind = Wind