import game
import wind
import scipy.constants as sc_const
import math
import time
import numpy

# Une variable globale qui servira à stocker la liste des murs du problème
Bounds = []

# Une variable globale qui servira à stocker la soufflerie (s'il y en a une)
Wind = None

gravity = [0.0, -sc_const.g *2]


def print_vec(name, vec) :
    print(name, " = ", vec[0], ", ", vec[1])


def normalize(vec):
    norm = math.sqrt(vec[0]*vec[0] + vec[1]*vec[1])
    return [vec[0]/norm, vec[1]/norm]


def dot(vec1, vec2):
    return vec1[0] * vec2[0] + vec1[1] * vec2[1]


def lenght(vec):
    return math.sqrt(dot(vec, vec))


def to_vec(point1, point2):
    return [point2[0] - point1[0], point2[1] - point1[1]]


def new_pos(deltat, old_speed):
    return deltat * old_speed


def new_speed(deltat, old_speed, coeff1, coeff2, mass):
    coeff = float((coeff1/mass + coeff2/mass * abs(lenght(old_speed))))
    strenght = (gravity + [float(coeff * old_speed[0]), float(coeff * old_speed[1])])
    return [deltat * strenght[0], deltat * strenght[1]]


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

        '''
        Si la boule n'avance pas (le segment est en fait un point), il n'y a pas de collision.
        '''
        if p1[0] == p2[0] and p1[1] == p2[1] :
            return [False, [0, 0], 0, [0, 0]]
        
        '''
        "d" correspond à la "direction" du mur (on modélise le mur par une droite infinie).
        "v" correspond à la direction du mouvement de la balle.
        '''
        d = normalize([-self.N[1], self.N[0]])
        v = normalize(to_vec(p1, p2))

        '''
        On calcule le déterminant. Si celui est nul, les deux droites (mur et balle) sont parrallèle : pas de collision.
        '''
        det = v[0] * d[1] - v[1] * d[0]
        if det == 0:
            return [False, [0, 0], 0, [0, 0]]
        '''
        "b" correspond à l'avancement du centre de la balle jusqu'au point de collision (et pas le point de rebond).
        '''
        b = (d[1] * (self.P[0] - p1[0]) + d[0] * (p1[1] - self.P[1])) / det

        '''
        Si "b" est négatif, la collision a lieu dans le sens opposé au déplacement de la balle. il n'y a donc pas collision.
        '''
        if b < 0:
            return [False, [0, 0], 0, [0, 0]]
        
        '''
        On calule "k" qui est le point de collision.
        '''
        k = [p1[0] + b * v[0], p1[1] + b * v[1]]
        '''
        "sin_theta" est l'angle entre la direction de la balle et la direction du mur.
        '''
        sin_theta = math.sin(math.acos(dot(v, d)))
        '''
        "dist" est la distance entre le point de collision et le point p2 (centre de la boule sans collision)
        '''
        dist = [p2[0] - k[0], p2[1] - k[1]]

        '''
        Dans le cas où p2 est avant le mur, on regarde si la boule intersecte le mur.
        '''
        if sin_theta * lenght(dist) >= r:
            return [False, [0, 0], 0, [0, 0]]
        
        '''
        "h" est la distance entre le point de collision et le point de rebond.
        '''
        h = r / sin_theta
        '''
        "p3" correspond au point de rebond.
        '''
        p3 = [p1[0] + (b - h) * v[0], p1[1] + (b - h) * v[1]]

        '''
        "n" est la normale au mur
        '''
        n = normalize(self.N)
        '''
        On calcule la reflexion de la direction de la balle par rapport au mur avec "rflx".
        '''
        scl = 2 * dot(v, n)
        rflx = normalize([v[0] - scl * n[0], v[1] - scl * n[1]])
        '''
        "deltat" est l'avancement de la balle entre le point de rebond et le point final.
        '''
        delta = lenght(to_vec(p1, p2)) - lenght(to_vec(p1, p3))

        '''
        On retourne "True" pour la détection de collision, "p3" le point de rebond, "delta" l'avancement entre p3 et p4
        (le point final) et le vecteur reflexion ("rflx").
        '''
        return [True, p3, delta, rflx]


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
        '''
        "k1" correspond au frottement visqueux pour l'air.
        '''
        self.k1 = -6 * math.pi * self.R * eta
        '''
        "k2" correspond au frottement de l'air. Ici, on les néglige (voir le rapport pour plus de précisions).
        On peut enlever les commentaires pour observer la différence.
        '''
        self.k2 = 0  # -cx * rho * math.pi * self.R * self.R
        self.M = M
        self.I = 2 * self.M * self.R ** 2 / 5

    def evolve(self, deltat):
        '''
        on calcule la nouvelle position et la nouvelle vitesse sans la collision.
        '''
        nspeed = self.dP + new_speed(deltat, self.dP, self.k1, self.k2, self.M)
        p2 = self.P + new_pos(deltat, self.dP)

        '''
        Ce sont des variables temporaires qui ne seront modifiées uniquement s'il y a une collision.
        '''
        calc_speed = nspeed
        calc_pos = p2
        
        '''
        On traite tous les murs de la liste "Bounds".
        '''
        for wall in Bounds :
            '''
            Teste de l'intersection avec la fonction "instersect". La boule se déplace sur le segment [self.P, p2].
            '''
            vals = wall.intersect(self.P, p2, self.R)
            if vals[0] == True :
                '''
                Collision detecté !
                On calcule la nouvelle position de la boule après collision : "point de rebond" + "avancement" * "reflexion"
                '''
                calc_pos = [vals[1][0] + vals[2] * vals[3][0], vals[1][1] + vals[2] * vals[3][1]]
                '''
                "delta_p3" correspond à l'avancement jusqu'au point de rebond.
                Remarque : Il est donc inférieur à "deltat".
                '''
                delta_p3 = lenght(to_vec(self.P, vals[1])) * deltat / lenght(to_vec(self.P, p2))
                '''
                On calcule la norme de la vitesse au point p3 (point de rebond).
                '''
                speed_len_p3 = lenght(self.dP + new_speed(delta_p3, self.dP, self.k1, self.k2, self.M))
                '''
                La vitesse au point p3 est donc redirigée selon le vecteur de reflexion.
                '''
                speed_p3 = [speed_len_p3 * vals[3][0], speed_len_p3 * vals[3][1]]
                '''
                "delta_p4" est l'avancement proportionnel à "deltat" et à "vals[2]" : c'est l'avancement entre
                le point de rebond (p3) et la nouvelle position (p4).
                '''
                delta_p4 = deltat - delta_p3
                '''
                On calcule la progression de la vitesse entre p3 et p4
                '''
                deltat_speed = new_speed(delta_p4, speed_p3, self.k1, self.k2, self.M)
                '''
                Ici, on calcule la vitesse finale après la rebond avec un coefficient. Ce coefficient de "0.9955" est
                necéssaire sinon on remarque que l'énergie mécanique de la balle augmente (elle rebondit un peu plus haut).
                '''
                calc_speed = [(speed_p3[0] + deltat_speed[0]) * 0.9955, (speed_p3[1] + deltat_speed[1]) * 0.9955]

        '''
        On fait un cast pour transformer les valeurs dans le bon type sinon python renvoie parfois des erreurs.
        '''
        self.dP = numpy.array(calc_speed)
        self.P = numpy.array(calc_pos)
