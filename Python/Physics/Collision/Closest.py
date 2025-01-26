# 最近接点
import numpy as np

# 球 vs 球
def SphereSphere(PosA, RadA, 
                 PosB, RadB):
    AB = PosB - PosA
    AB /= np.linalg.norm(AB)
    return AB * RadA + PosA, -AB * RadB + PosB
    