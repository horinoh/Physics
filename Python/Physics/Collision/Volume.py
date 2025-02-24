import numpy as np

def Tetrahedron(A, B, C, D):
    return np.fabs((D - A) @ np.cross(D - B, D - C) / 6.0)
