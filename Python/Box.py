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

        # 物理シーン
        self.Scene = Scene.Scene()

        # 固定剛体 (地面)
        for i in range(3):
            for j in range(3):
                Rb = RigidBody.RigidBody()
                Rb.InvMass = 0.0 # 固定
                Rb.Shape = Shape.ShapeBox([20.0, 20.0, 20.0])
                Rb.Position = [ (i - 1) * Rb.Shape.Extent[0], -Rb.Shape.Extent[1], (j - 1) * Rb.Shape.Extent[2] ]
                self.Scene.RigidBodies.append(Rb)

        # 剛体
        CXZ = 2; CY = 2
        DXZ = 3; DY = 3
        Ofs = CXZ * DXZ * 0.5
        for i in range(CY):
            for j in range(CXZ):
                for k in range(CXZ):
                    Rb = RigidBody.RigidBody()
                    Rb.Shape = Shape.ShapeBox()
                    Rb.Position = [ j * DXZ - Ofs, i * DY + 3, k * DXZ - Ofs]
                    #Rb.Rotation = quaternion.from_euler_angles(i * 20, 0, j*20)
                    self.Scene.RigidBodies.append(Rb)

        # ボックス (描画用)
        self.Boxes = []
        for i in self.Scene.RigidBodies:
            Inst = scene.visuals.Box(width = i.Shape.Extent[0] * 2.0, height = i.Shape.Extent[1] * 2.0, depth = i.Shape.Extent[2] * 2.0, color = "yellow" if i.InvMass != 0.0 else "green", edge_color = 'black', parent = View.scene)
            Inst.transform = MatrixTransform()
            self.Boxes.append(Inst)

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

        if __name__ == '__main__' and sys.flags.interactive == 0:
            self.Canvas.app.run()

    # 更新関数
    def Update(self, event):
        self.Scene.Update(self.Timer.interval)

        IsDrawAABB = True
        #IsDrawAABB = False
        for i in range(len(self.Scene.RigidBodies)):
            Rb = self.Scene.RigidBodies[i]
            VisBox = self.Boxes[i]
            VisAB = self.AABBs[i]

            VisBox.transform.reset()
            VisAB.transform.reset()
            
            # 回転軸が取れれば回転する
            Axis = quaternion.as_rotation_vector(Rb.Rotation)
            LenSq = Axis @ Axis
            if False == np.isclose(LenSq, 0.0):
                Axis /= math.sqrt(LenSq)
                VisBox.transform.rotate(-np.rad2deg(Rb.Rotation.angle()), [Axis[0], Axis[2], Axis[1]])
            
            # YZ が入れ替わるので注意s
            Pos = [Rb.Position[0], Rb.Position[2], Rb.Position[1]]
            # 剛体
            VisBox.transform.translate(Pos)

            # AABB
            if IsDrawAABB and Rb.InvMass != 0.0:
                Ab = Rb.GetAABB()
                VisAB.transform.scale([Ab.Extent[0] * 2.0, Ab.Extent[2] * 2.0, Ab.Extent[1] * 2.0])
                # AABB がジャストフィットなので、表示用に少し大きくする
                VisAB.transform.scale([1.1, 1.1, 1.1])
                VisAB.transform.translate([Ab.Center[0], Ab.Center[2], Ab.Center[1]])
            else:
                VisAB.transform.translate([0, 0, -100])
                
App()

