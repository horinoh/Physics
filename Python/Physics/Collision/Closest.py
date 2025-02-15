# 最近接点
import sys

import numpy as np

from Physics.Collision.GJK import SupportPoint, GetSupportPoint, SignedVolume

# 球 vs 球
def SphereSphere(PosA, RadA, 
                 PosB, RadB):
    AB = PosB - PosA
    AB /= np.linalg.norm(AB)
    return AB * RadA + PosA, -AB * RadB + PosB

# 最近接店を見つける為、衝突判定の早期終了チェックを省いていいる
def GJK(ShA, PosA, RotA,
        ShB, PosB, RotB):
    Sps = []

    Dir = np.ones(3)
    Dir /= np.linalg.norm(Dir)
    Sps.append(GetSupportPoint(ShA, PosA, RotA,
                               ShB, PosB, RotB,
                               Dir, 0.0))
    
    Dir = -Sps[0].C
    ClosestSq = sys.float_info.max
    Intersect = False
    while Intersect == False:
        Dir /= np.linalg.norm(Dir)
        Pt = GetSupportPoint(ShA, PosA, RotA, 
                             ShB, PosB, RotB, Dir, 0.0)

        if list(filter(lambda rhs: np.isclose(rhs.C, Pt.C).all(), Sps)):
            break

        Sps.append(Pt)
        Lmd, Dir = SignedVolume(Sps, Dir)

        LenSq = Dir @ Dir        
        if LenSq >= ClosestSq:
            break
        ClosestSq = LenSq

        SpsLmd = list(map(lambda rhs: [Sps[rhs[0]], rhs[1]], filter(lambda rhs: rhs[1] != 0.0, enumerate(Lmd))))
        Sps = list(map(lambda rhs: rhs[0], SpsLmd))
        #Sps = list(map(lambda rhs: Sps[rhs[0]], filter(lambda rhs: rhs[1] != 0.0, enumerate(Lmd))))

        Intersect = (4 == len(Sps))

    return sum(map(lambda rhs: rhs[0].A * rhs[1], SpsLmd)), sum(map(lambda rhs: rhs[0].B * rhs[1], SpsLmd))
    