# 交差
import numpy as np
from Physics.Collision.Bound import AABB

# AABB vs AABB
def AABBAABB(AbA, AbB):
    if AbA.Max[0] < AbB.Min[0] or AbB.Max[0] < AbA.Min[0] or AbA.Max[1] < AbB.Min[1] or AbB.Max[1] < AbA.Min[1] or AbA.Max[2] < AbB.Min[2] or AbB.Max[2] < AbA.Min[2]:
        return False
    return True

# レイ vs 球
def RaySphere(RayPos, RayDir,
              SpPos, SpRad):
    M = np.array(RayPos) - SpPos
    A = RayDir @ RayDir
    B2 = M @ RayDir
    C = M @ M - SpRad * SpRad

    D4Sq = B2 * B2 - A * C
    if D4Sq >= 0:
        D4 = np.sqrt(D4Sq)
        T0 = (B2 - D4) / A
        T1 = (B2 + D4) / A
        return True, T0, T1
    return False, 0.0, 0.0

# 球 vs 球
def SphereSphere(PosA, RadA,
                 PosB, RadB):
    AB = np.array(PosB) - PosA
    return AB @ AB <= pow(RadA + RadB, 2.0); 

# 球 vs 球 (動的)
def SphereSphereDy(PosA, RadA, VelA, 
                   PosB, RadB, VelB):
    # A の相対速度
    Ray = VelA - VelB
    TotalRad = RadA + RadB
    T0 = 0.0
    T1 = 0.0
    #Eps2 = 0.001 * 0.001
    #if Ray @ Ray < Eps2:
    if np.isclose(Ray @ Ray, 0.0):
		# レイが十分短い場合は既に衝突しているかどうかのチェック
        AB = np.array(PosB) - PosA
        R = TotalRad + 0.001
        if AB @ AB > R * R:
            return False, 0.0
    else:
	    # レイ vs 球 に帰着
        B, T0, T1 = RaySphere(PosA, Ray, PosB, TotalRad)
        if False == B or T0 > 1.0 or T1 < 0.0:
                return False, 0.0

    return True, max(T0, 0.0)
    