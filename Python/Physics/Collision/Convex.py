# 凸包
import math # math.sqrt の方が numpy.sqrt より速いらしい
import copy
import numpy as np

from Physics.Collision import Distance

def BuildTetrahedron(Pts):
    Vert = []

    # 特定の方向 (X軸) に一番遠い点を求める
    AxisX = [ 1.0, 0.0, 0.0 ]
    Val = []
    for i in Pts:
        Val.append(AxisX @ i)
    Vert.append(Pts[np.argmax(Val)])
    
    # 前述の逆向きの軸に一番遠い点
    Val.clear()
    for i in Pts:
        Val.append(-Vert[0] @ i)
    Vert.append(Pts[np.argmax(Val)])

    # 前述の2点からなるレイに一番遠い点
    Val.clear()
    Dir = Vert[1] - Vert[0]
    for i in Pts:
        Val.append(Distance.PointRaySq(i, Vert[0], Dir))
    Vert.append(Pts[np.argmax(Val)])

    # 前述の3点からなる三角形に一番遠い点
    Val.clear()
    for i in Pts:
        Val.append(Distance.PointTriangle(i, Vert[0], Vert[1], Vert[2]))
    Vert.append(Pts[np.argmax(Val)])

    # CCW になるように調整
    if Distance.IsFront(Vert[0], 
                        Vert[1], Vert[2], Vert[3]):
        Vert[0], Vert[1] = Vert[1], Vert[0]
    
    # バーテックス、インデックスを返す
    return Vert, [[0, 1, 2], [0, 2, 3], [2, 1, 3], [1, 0, 3]]

def RemoveInternal(Pts,
                   Verts, Inds):
    # 内部点を除外
    Pts = list(filter(lambda rhs: rhs, Pts))
   
    # 既存と同一とみなせる点も除外 TODO

def BuildConexHull(Pts):
    # なるべく包含するような四面体を求める
    Verts, Inds = BuildTetrahedron(Pts)

    # 内部点を除外し、外部点を残すss
    Ext = copy.deepcopy(Pts)

    for i in Inds:
        Pts = list(filter(lambda rhs: False == Distance.IsFront(rhs, Verts[i[0]], Verts[i[1]], Verts[i[2]]), Ext))

    RemoveInternal(Pts, 
                   Verts, Inds)
    
    # TODO
