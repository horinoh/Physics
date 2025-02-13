# 交差

import sys
import copy
import math # math.sqrt の方が numpy.sqrt より速いらしい
import numpy as np

from Physics.Collision.Bound import AABB
from Physics.Collision.GJK import GetSupportPoint, SignedVolume1, SignedVolume2, SignedVolume3

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
        D4 = math.sqrt(D4Sq)
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

# 原点を含むシンプレクスを作成する事で衝突を検出する
# ある方向に一番遠い点(サポートポイント)を見つける
def GJK(ShA, PosA, RotA,
        ShB, PosB, RotB):
    # サポートポイント保持先
    Sps = []

    # (1, 1, 1) 方向のサポートポイントを求める
    Dir = np.ones(3)
    Dir /= np.linalg.norm(Dir)
    Sps.append(GetSupportPoint(ShA, PosA, RotA,
                               ShB, PosB, RotB,
                               Dir, 0.0))
    
    # 原点に向かう方向 (逆向き) に次のサポートポイントを求める
    Dir = -Sps[0].C
    ClosestSq = sys.float_info.max
    Intersect = False
    while Intersect == False:
        Dir /= np.linalg.norm(Dir)
        Pt = GetSupportPoint(ShA, PosA, RotA, 
                             ShB, PosB, RotB, Dir, 0.0)

        # 新しいサポートポイント Pt が既存の場合これ以上拡張できない (衝突無し)
        if list(filter(lambda rhs: np.isclose(rhs.C, Pt.C).all(), Sps)):
            break

        # 新しいサポートポイント Pt が原点を超えていなければ、原点を含まない (衝突無し)
        if Dir @ Pt.C < 0.0:
            break

        # 新しいサポートポイント Pt を追加した上で、シンプレクスが原点を含むかどうかs
        Sps.append(Pt)
        match len(Sps):
            case 2:
                # シンプレクス (線分) 上での原点の重心座標
                Lmd = SignedVolume1(Sps[0].C, Sps[1].C)
                # 原点へのベクトル
                Dir = -(Sps[0].C * Lmd[0] + Sps[1].C * Lmd[1])
            case 3:
                # シンプレクス (三角形) 上
                Lmd = SignedVolume2(Sps[0].C, Sps[1].C, Sps[2].C)
                Dir = -(Sps[0].C * Lmd[0] + Sps[1].C * Lmd[1] + Sps[2].C * Lmd[2])
            case 4:
                # シンプレクス (四面体) 上
                Lmd = SignedVolume3(Sps[0].C, Sps[1].C, Sps[2].C, Sps[3].C)
                Dir = -(Sps[0].C * Lmd[0] + Sps[1].C * Lmd[1] + Sps[2].C * Lmd[2] + Sps[3].C * Lmd[3])
        LenSq = Dir @ Dir
        # 原点へ向かうベクトルの長さが 0.0 なら原点を含む (衝突)
        if np.isclose(LenSq, 0.0):
            Intersect = True
            break

        # 最短距離を更新できない (衝突無し)
        if LenSq >= ClosestSq:
            break
        ClosestSq = LenSq

        # 有効なサポートポイントのみを残す (Lmd が非 0.0 )
        #   enumerate で Lmd をインデックス付きにする (インデックス[0]、値[1])
        #   filter で値[1] が 0.0 でないもののみに絞り込み
        #   map で Sps[インデックス[0]] を抽出した新しいリストを作成
        #   Sps は新しいリストで上書き
        Sps = list(map(lambda rhs: Sps[rhs[0]], filter(lambda rhs: rhs[1] != 0.0, enumerate(Lmd))))

        # 四面体でここまで来たら原点を含む (衝突)
        Intersect = (4 == len(Sps))

    # 衝突確定
    if Intersect:
        # 衝突点を求め、EPA へ
        # EPA()
        return True
    
    return False