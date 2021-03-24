import game
import wind
import scipy.constants as sc_const
import math

# Une variable globale qui servira à stocker la liste des murs du problème
Bounds = []

# Une variable globale qui servira à stocker la soufflerie (s'il y en a une)
Wind = None

gravity = [0.0, -sc_const.g]

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
        self.k2 = -cx * rho * math.pi * self.R * self.R
        self.M = M
        self.I = 2 * self.M * self.R ** 2 / 5

    def evolve(self, deltat):
        '''
        A faire:
        mettre à jour P, dP, O, dO (connus à l'instant t) pour déterminer leurs valeurs à l'instant t+deltat.

        Ici, une version basique qui se contente de faire tourner la boule à vitesse constante.

        On utilisera la liste Bounds pour connaitre l'ensemble des murs du problème.
        '''
        self.P += deltat * self.dP
        self.dP += deltat * (gravity + (self.k1/self.M + self.k2/self.M * abs(self.dP)) * self.dP)
        #self.dP[1] += deltat * (gravity[1] + (self.k1/self.M + self.k2/self.M * abs(self.dP[1])) * self.dP[1])
