# 最近接点
import sys

import numpy as np

import Physics

# 球 vs 球
def SphereSphere(PosA, RadA, 
                 PosB, RadB):
    AB = PosB - PosA
    AB /= np.linalg.norm(AB)
    return AB * RadA + PosA, -AB * RadB + PosB

# 凸包 vs 凸包
def GJK(ShA, PosA, RotA,
        ShB, PosB, RotB):
    return Physics.Collision.GJK.GJK(ShA, PosA, RotA,
                                     ShB, PosB, RotB,
                                     True)[1]