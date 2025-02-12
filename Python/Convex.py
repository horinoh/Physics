import sys
import random
import math

import numpy as np
import quaternion

from vispy import app
from vispy import scene
from vispy.visuals.transforms import MatrixTransform
from vispy.util.quaternion import Quaternion

from Physics import Scene
from Physics import RigidBody
from Physics import Shape
from Physics.Collision.Intersection import GJK

from Physics.Collision.GJK import TestSignedVolume

class App:
    def __init__(self):
        print("Python=", sys.version)
        print("Numpy+", np.__version__)

        # キャンバス
        self.Canvas = scene.SceneCanvas(keys = 'interactive', bgcolor = 'skyblue', size = (800, 600), show = True)
        # ビューを追加
        View = self.Canvas.central_widget.add_view()
        # カメラ設定
        View.camera = 'arcball'
        View.camera.set_range(x = [-10, 10])

        # 物理シーン
        self.Scene = Scene.Scene()

        # 剛体
        for i in range(2):
            Rb = RigidBody.RigidBody()
            Rb.InvMass = 0.0
            Rb.Shape = Shape.ShapeBox()
            #Rb.Shape = Shape.ShapeSphere()
            if i == 0:
                Rb.Position = [ -2.0, 0.0, 0.0 ]
            self.Scene.RigidBodies.append(Rb)

        # 描画用
        self.Visuals = []
        for i in self.Scene.RigidBodies:
            match i.Shape.GetType():
                case Shape.ShapeType.SPHERE:
                    Inst = scene.visuals.Sphere(radius = i.Shape.Radius, cols = 20, rows = 20, method = 'latitude', color = "yellow", edge_color = 'black', parent = View.scene)
                case Shape.ShapeType.BOX:
                    Inst = scene.visuals.Box(width = i.Shape.Extent[0], height = i.Shape.Extent[1], depth = i.Shape.Extent[2], color = "yellow", edge_color = 'black', parent = View.scene)
            Inst.transform = MatrixTransform()
            self.Visuals.append(Inst)

        # 更新処理
        FPS = 1.0 / 30.0
        self.Timer = app.Timer(interval = FPS, connect = self.Update)
        self.Timer.start()
        self.IsStop = False

        TestSignedVolume()

        @self.Canvas.events.key_press.connect
        def on_key_press(event):
            Rb = self.Scene.RigidBodies[0]
            Ang = quaternion.as_euler_angles(Rb.Rotation)
            
            # 移動、回転
            MvSpd = 0.2
            RotSpd = 10
            match event.key:
                case ' ':
                    self.IsStop = False if self.IsStop else True
                    if self.IsStop:
                        self.Timer.stop()
                    else:
                        self.Timer.start()
                case 'w':
                    Rb.Position[2] += MvSpd
                case 'a':
                    Rb.Position[0] -= MvSpd
                case 's':
                    Rb.Position[2] -= MvSpd
                case 'd':
                    Rb.Position[0] += MvSpd
                case 'q':
                    Rb.Position[1] -= MvSpd
                case 'e':
                    Rb.Position[1] += MvSpd
                case 'Up':
                    Ang[0] += np.deg2rad(RotSpd)
                case 'Left':
                    Ang[1] += np.deg2rad(RotSpd)
                case 'Down':
                    Ang[0] -= np.deg2rad(RotSpd)
                case 'Right':
                    Ang[1] -= np.deg2rad(RotSpd)

            if Ang[0] < 0.0:
                Ang[0] += np.pi
            if Ang[0] > np.pi:
                Ang[0] -= np.pi

            if Ang[1] < 0.0:
                Ang[1] += np.pi
            if Ang[1] > np.pi:
                Ang[1] -= np.pi

            Rb.Rotation = quaternion.from_euler_angles(Ang[0], Ang[1], Ang[2])

        if __name__ == '__main__' and sys.flags.interactive == 0:
           self.Canvas.app.run()

    # 更新関数
    def Update(self, event):
        self.Scene.Update(self.Timer.interval)

        Len = len(self.Scene.RigidBodies)

        # 衝突の有無で背景色を変更
        HasIntersection = False
        for i in range(Len):
            for j in range(i + 1, Len):
                RbA = self.Scene.RigidBodies[i]
                RbB = self.Scene.RigidBodies[j]
                if GJK(RbA.Shape, RbA.Position, RbA.Rotation,
                       RbB.Shape, RbB.Position, RbB.Rotation):
                    HasIntersection = True
        self.Canvas.bgcolor = "white" if HasIntersection else "skyblue"

        for i in range(Len):
            Rb = self.Scene.RigidBodies[i]
            
            Vis = self.Visuals[i]
            Vis.transform.reset()

            # 回転軸が取れれば回転する
            Axis = quaternion.as_rotation_vector(Rb.Rotation)
            LenSq = Axis @ Axis
            if False == np.isclose(LenSq, 0.0):
                Axis /= math.sqrt(LenSq)
                Vis.transform.rotate(-np.rad2deg(Rb.Rotation.angle()), [Axis[0], Axis[2], Axis[1]])
            
            # YZ が入れ替わるので注意s
            Pos = [Rb.Position[0], Rb.Position[2], Rb.Position[1]]
            # 剛体
            Vis.transform.translate(Pos)
App()

