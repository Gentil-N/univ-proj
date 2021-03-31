import game
import wind
import scipy.constants as sc_const
import math

# Une variable globale qui servira à stocker la liste des murs du problème
Bounds = []

# Une variable globale qui servira à stocker la soufflerie (s'il y en a une)
Wind = None

gravity = [0.0, -sc_const.g]


def normalize(vec):
    norm = math.sqrt(vec[0]*vec[0] + vec[1]*vec[1])
    return [vec[0]/norm, vec[1]/norm]

def dot(vec1, vec2):
    return vec1[0] * vec2[0] + vec1[1] * vec2[1]

def to_vec(point1, point2):
    return [point2[0] - point1[0], point2[1] - point1[1]]

def new_pos(deltat, old_speed):
    return deltat * old_speed

def new_speed(deltat, old_speed, coeff1, coeff2, mass):
    return deltat * (gravity + (coeff1/mass + coeff2/mass * abs(old_speed)) * old_speed)


class FlatWall(game.BaseFlatWall):
    def intersect(self, p1, p2, r):
        '''
        A faire:
        renvoie l'intersection (si elle existe) entre le mur et une boule de rayon r
        se déplaçant sur le segment d'extrémités p1 et p2.

        On utilisera:
        self.P un point qui appartient au plan du mur
        self.N un vecteur normal (sortant) du mur
        '''
        d = normalize([-self.N[1], self.N[0]])
        v = normalize(to_vec(p1, p2))
        det = v[0] * d[1] - v[1] * d[0]
        if det == 0 :
            return [False, [0, 0], [0, 0]]
        b = (d[1] * (self.P[0] - p1[0]) - d[0] * (self.P[1] + p1[1])) / det
        if b < 0 :
            return [False, [0, 0], [0, 0]]
        k = p1 + b * v
        theta = math.sin(math.acos(dot(v, d)))
        #to be continued...
        return None


class Ball(game.BaseBall):
    def __init__(self, R=1.0, M=1.0, P0=[0, 0], dP0=[0, 0], O0=0.0, dO0=0.0, eta=0.000018, cx=0.45, rho=1.2):
        '''
        Initialisation de la boule:
        self.R = rayon(mètre)
        self.M = masse(kilo)
        self.I = moment d'inertie
        self.P = position
        self.dP = vitesse
        self.O = angle
        self.dO = vitesse angulaire
        '''
        super().__init__(R, P0, dP0, O0, dO0)
        self.k1 = -6 * math.pi * self.R * eta
        self.k2 = 0  # -cx * rho * math.pi * self.R * self.R
        self.M = M
        self.I = 2 * self.M * self.R ** 2 / 5

    def evolve(self, deltat):
        '''
        A faire:
        mettre à jour P, dP, O, dO (connus à l'instant t) pour déterminer leurs valeurs à l'instant t+deltat.

        Ici, une version basique qui se contente de faire tourner la boule à vitesse constante.

        On utilisera la liste Bounds pour connaitre l'ensemble des murs du problème.
        '''
        self.P += new_pos(deltat, self.dP)
        self.dP += new_speed(deltat, self.dP, self.k1, self.k2, self.M)
