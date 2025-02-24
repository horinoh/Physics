import numpy as np

def Sphere(Radius):
    return np.diag(np.full(3, 2.0 / 5.0 * pow(Radius, 2.0)))

def Box(X, Y, Z):
    return np.diag(np.array([Y + Z, X + Z, X + Y]) / 12.0)

def Tetrahedron(A, B, C, D):
    AB = B - A
    AC = C - A
    AD = D - A
    Det = np.linalg.det(np.array([
        [ AB[0], AC[0], AD[0] ],
        [ AB[1], AC[1], AD[1] ],
        [ AB[2], AC[2], AD[2] ],
    ]))

    Pts = [ A, B, C, D ]
    Len = len(Pts)
    XX = 0.0; YY = 0.0; ZZ = 0.0
    XY = 0.0; XZ = 0.0; YZ = 0.0
    for i in range(Len):
        for j in range(i, Len):
            XX += Pts[i][0] * Pts[j][0]
            YY += Pts[i][1] * Pts[j][1]
            ZZ += Pts[i][2] * Pts[j][2]

            XY +=  Pts[i][0] * Pts[j][1] + Pts[j][0] * Pts[i][1]
            XZ +=  Pts[i][0] * Pts[j][2] + Pts[j][0] * Pts[i][2]
            YZ +=  Pts[i][1] * Pts[j][2] + Pts[j][1] * Pts[i][2]
    
    return np.array([
        [ 2.0 * (YY + ZZ), -XY, -XZ ],
        [ -XY, 2.0 * (XX + ZZ), -YZ ],
        [ -XZ, -YZ, 2.0 * (XX + YY) ],
    ]) * Det / 120.0