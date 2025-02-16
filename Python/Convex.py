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
#from Physics.Collision.Intersection import GJK

import Physics.Collision.Closest
import Physics.Collision.GJK
import Physics.Collision.Intersection

#sfrom Physics.Collision.GJK import TestSignedVolume

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
                Rb.Position = [ -2.5, 0.0, 0.0 ]
            self.Scene.RigidBodies.append(Rb)

        # 描画用 (形状)
        self.Visuals = []
        Color = (0, 1, 0, 0.5)
        for i in self.Scene.RigidBodies:
            match i.Shape.GetType():
                case Shape.ShapeType.SPHERE:
                    Inst = scene.visuals.Sphere(radius = i.Shape.Radius, cols = 20, rows = 20, method = 'latitude', color = Color, edge_color = 'black', parent = View.scene)
                case Shape.ShapeType.BOX:
                    Inst = scene.visuals.Box(width = i.Shape.Extent[0], height = i.Shape.Extent[1], depth = i.Shape.Extent[2], color = Color, edge_color = 'black', parent = View.scene)
            Inst.transform = MatrixTransform()
            self.Visuals.append(Inst)

        # 描画用 (衝突点)
        self.IPos = scene.visuals.Box(width = 0.05, height = 0.05, depth = 0.05, color = "red", edge_color = 'black', parent = View.scene)
        self.IPos.transform = MatrixTransform()

        # 描画用 (最近接点)
        self.CPts = []
        Inst = scene.visuals.Box(width = 0.05, height = 0.05, depth = 0.05, color = "yellow", edge_color = 'black', parent = View.scene)
        Inst.transform = MatrixTransform()
        self.CPts.append(Inst)
        Inst = scene.visuals.Box(width = 0.05, height = 0.05, depth = 0.05, color = "yellow", edge_color = 'black', parent = View.scene)
        Inst.transform = MatrixTransform()
        self.CPts.append(Inst)

        # 更新処理
        FPS = 1.0 / 30.0
        self.Timer = app.Timer(interval = FPS, connect = self.Update)
        self.Timer.start()
        self.IsStop = False

        # SignedVolume 計算のテスト
        #TestSignedVolume()

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
        HasIntersection = False
        IPos = None
        Res = []
        for i in range(Len):
            for j in range(i + 1, Len):
                RbA = self.Scene.RigidBodies[i]
                RbB = self.Scene.RigidBodies[j]
                HasIntersection, Res = Physics.Collision.GJK.GJK(RbA.Shape, RbA.Position, RbA.Rotation,
                                                                 RbB.Shape, RbB.Position, RbB.Rotation,
                                                                 True)
                #HasIntersection = Physics.Collision.Intersection.GJK(RbA.Shape, RbA.Position, RbA.Rotation,
                #                                                     RbB.Shape, RbB.Position, RbB.Rotation)
                #if not HasIntersection:
                #    Res = Physics.Collision.Closest.GJK(RbA.Shape, RbA.Position, RbA.Rotation,
                #                                        RbB.Shape, RbB.Position, RbB.Rotation)

        # 衝突がある場合、衝突点を求める
        if HasIntersection and not Res is None:
            #for i in Res:
            #    print(i.C)
            Physics.Collision.GJK.EPA(RbA.Shape, RbA.Position, RbA.Rotation,
                                      RbB.Shape, RbB.Position, RbB.Rotation,
                                      Res, 0.1)

        # 衝突の有無で背景色を変更
        self.Canvas.bgcolor = "blue" if HasIntersection else "skyblue"

        # 描画
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
            
            # YZ が入れ替わるので注意
            Pos = [Rb.Position[0], Rb.Position[2], Rb.Position[1]]
            # 剛体
            Vis.transform.translate(Pos)
        
        # 衝突点
        self.IPos.transform.reset()
        if not IPos is None:
            self.IPos.transform.translate([IPos[0], IPos[2], IPos[1]])

        # 最近接点
        if not HasIntersection:
            if not Res is None:
                for i in range(len(self.CPts)):
                    CPt = self.CPts[i]
                    CPt.transform.reset()
                    CPt.transform.translate([Res[i][0], Res[i][2], Res[i][1]])

App()

