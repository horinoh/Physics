# 距離
import math # math.sqrt の方が numpy.sqrt より速いらしい
import numpy as np

# 点 vs レイ
def PointRaySq(Pt, 
               RayPos, RayDir):
    ToPt = Pt - RayPos
    Vert = (RayDir * ToPt @ RayDir) - ToPt
    return Vert @ Vert
def PointRay(Pt,
             RayPos, RayDir):
    return math.sqrt(PointRaySq(Pt, RayPos, RayDir))

# 点 vs 三角形
def PointTriangle(Pt, 
                  TriA, TriB, TriC):
    Nrm = np.cross(TriB - TriA, TriC - TriA)
    return (Pt - TriA) @ (Nrm / np.linalg.norm(Nrm))

# 点が三角形の前面にあるか
def IsFront(Pt, 
            TriA, TriB, TriC):
    return (Pt - TriA) @ np.cross(TriB - TriA, TriC - TriA) >= 0.0