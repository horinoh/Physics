# $pip install numpy
# $pip install numpy-quaternion
# $pip install vpython

# VisutalStudio の場合
#   ソリューション - Python Environments右クリック - View All Python Environments - Packages (PyPl) タブ - 検索欄で検索 - install

from math import e
import vpython as vp

import numpy as np
import quaternion

# Visual Studio の場合 Search Paths へ追加しておく
from LibPhysics import Scene
from LibPhysics import Shape
from LibPhysics import RigidBody

Radius = 0.5
Ext = Radius * 2.0
Y = 10.0

Scene = Scene.Scene()
Scene.Shapes.append(Shape.ShapeBox(Ext))
N = 4
N2 = N // 2
for y in range(1):
    for x in range(N):
        for z in range(N):
            Rb = RigidBody.RigidBody(Scene.Shapes[0])
            Rb.Position = np.array([(x - N2) * Radius * 2.0 * 1.5, Y + y * 1.5, (z - N2) * Radius * 2.0 * 1.5])
            Scene.RigidBodies.append(Rb)
            Rb.Vis = vp.box(pos = vp.vector(Rb.Position[0], Rb.Position[1], Rb.Position[2]), size = vp.vector(Ext, Ext, Ext), color = vp.color.white)

Radius = 20.0
Ext = Radius * 2.0
Y = -Radius
Scene.Shapes.append(Shape.ShapeBox(Ext))
Rb = RigidBody.RigidBody(Scene.Shapes[1], 0.0)
Rb.Position = np.array([0.0, Y, 0.0]);
Scene.RigidBodies.append(Rb)
Rb.Vis = vp.box(pos = vp.vector(Rb.Position[0], Rb.Position[1], Rb.Position[2]), size = vp.vector(Ext, Ext, Ext), color = vp.color.green)

FPS = 60.0
Pause = True
while True:
    Keys = vp.keysdown()
    if 'esc' in Keys:
        break
    if ' ' in Keys:
        Pause = Pause == False
 
    if not Pause:
        Scene.Update(1.0 / FPS)

    for i in Scene.RigidBodies:
        i.Vis.pos = vp.vector(i.Position[0], i.Position[1], i.Position[2])
        Axis = quaternion.as_rotation_vector(i.Rotation)
        Norm = np.linalg.norm(Axis)
        if Norm > 0.0:
            Axis = Axis / Norm
            # (Y 方向ではなく) X 方向がデフォルトなので注意
            i.Vis.axis = vp.vector(Axis[0], Axis[1], Axis[2])
    
    vp.rate(FPS)
    

