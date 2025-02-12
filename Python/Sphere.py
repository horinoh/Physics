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

        # 軸を表示
        #Axis = scene.visuals.XYZAxis(parent = View.scene, width = 1)
        # グリッド
        #View.add(scene.visuals.GridLines(color = "white", scale = (1.0, 1.0)))

        # 物理シーン
        self.Scene = Scene.Scene()

        # 固定剛体 (地面)
        for i in range(3):
            for j in range(3):
                Rb = RigidBody.RigidBody()
                Rb.InvMass = 0.0 # 固定
                Rb.Shape = Shape.ShapeSphere(50.0)
                Rb.Position = [ (i - 1) * Rb.Shape.Radius, -Rb.Shape.Radius, (j - 1) * Rb.Shape.Radius ]
                self.Scene.RigidBodies.append(Rb)

        # 剛体
        CXZ = 3; CY = 2
        DXZ = 3; DY = 3
        Ofs = CXZ * DXZ * 0.5
        for i in range(CY):
            for j in range(CXZ):
                for k in range(CXZ):
                    Rb = RigidBody.RigidBody()
                    Rb.Shape = Shape.ShapeSphere()
                    Rb.Position = [ j * DXZ - Ofs, i * DY + 3, k * DXZ - Ofs]
                    self.Scene.RigidBodies.append(Rb)

        # 球 (描画用)
        self.Spheres = []
        for i in self.Scene.RigidBodies:
            Inst = scene.visuals.Sphere(radius = i.Shape.Radius, cols = 20, rows = 20, method = 'latitude', color = "yellow" if i.InvMass != 0.0 else "green", edge_color = 'black', parent = View.scene)
            Inst.transform = MatrixTransform()
            self.Spheres.append(Inst)

        # AABB (描画用)
        self.AABBs = []
        for i in self.Scene.RigidBodies:
            Inst = scene.visuals.Box(width = 1, height = 1, depth = 1, color = (1, 1, 1, 0), edge_color = "white", parent = View.scene)
            Inst.transform = MatrixTransform()
            self.AABBs.append(Inst)

        # 更新処理
        #FPS = 1.0 / 20.0
        FPS = 1.0 / 30.0
        #FPS = 1.0 / 60.0
        #FPS = 1.0 / 120.0
        self.Timer = app.Timer(interval = FPS, connect = self.Update)
        self.Timer.start()
        self.IsStop = False

        @self.Canvas.events.key_press.connect
        def on_key_press(event):
             match event.key:
                case ' ':
                    self.IsStop = False if self.IsStop else True
                    if self.IsStop:
                        self.Timer.stop()
                    else:
                        self.Timer.start()

        # 小数点以下 4 桁までの制度で、同じ値とみなされるものはユニークにする
        a = [
            [0.1, 0.1, 0.12, 0.123, 0.1234],
            [0.1, 0.1, 0.12, 0.123, 0.1234],
            [0.1, 0.1, 0.12, 0.123, 0.123],
            [0.1, 0.1, 0.12, 0.12, 0.1234],
            [0.1, 0.1, 0.12, 0.123, 0.1234],
            [0.1, 0.1, 0.12, 0.12, 0.12],
            [0.1, 0.1, 0.12, 0.123, 0.0],
        ]
        print(np.unique(np.round(a, decimals = 4), axis = 0))

        if __name__ == '__main__' and sys.flags.interactive == 0:
            self.Canvas.app.run()

    # 更新関数
    def Update(self, event):
        self.Scene.Update(self.Timer.interval)

        IsDrawAABB = True
        #IsDrawAABB = False
        for i in range(len(self.Scene.RigidBodies)):
            Rb = self.Scene.RigidBodies[i]
            VisSp = self.Spheres[i]
            VisAB = self.AABBs[i]

            VisSp.transform.reset()
            VisAB.transform.reset()
            
            # 回転軸が取れれば回転する
            Axis = quaternion.as_rotation_vector(Rb.Rotation)
            LenSq = Axis @ Axis
            if False == np.isclose(LenSq, 0.0):
                Axis /= math.sqrt(LenSq)
                VisSp.transform.rotate(-np.rad2deg(Rb.Rotation.angle()), [Axis[0], Axis[2], Axis[1]])
            
            # YZ が入れ替わるので注意s
            Pos = [Rb.Position[0], Rb.Position[2], Rb.Position[1]]
            # 剛体
            VisSp.transform.translate(Pos)

            # AABB
            if IsDrawAABB and Rb.InvMass != 0.0:
                Ab = Rb.GetAABB()
                VisAB.transform.scale([Ab.Extent[0], Ab.Extent[2], Ab.Extent[1]])
                VisAB.transform.translate([Ab.Center[0], Ab.Center[2], Ab.Center[1]])
            else:
                VisAB.transform.translate([0, 0, -100])

App()

