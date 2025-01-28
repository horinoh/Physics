import sys
import random

import numpy as np
import quaternion

# 要インストール
# pip install vispy, Glfw
from vispy import app
from vispy import scene
from vispy.visuals.transforms import MatrixTransform
from vispy.util.quaternion import Quaternion

from Physics import Scene
from Physics import RigidBody
from Physics import Shape

class App:
    def __init__(self):
        # キャンバス
        Canvas = scene.SceneCanvas(keys = 'interactive', bgcolor = 'skyblue', size = (800, 600), show = True)
        # ビューを追加
        View = Canvas.central_widget.add_view()
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

        # ボックス (描画用 未使用)
        self.Boxes = []
        for i in range(0):
            Inst = scene.visuals.Box(1, 1, 1, color = "green", edge_color = "black", parent = View.scene)
            Inst.transform = MatrixTransform()
            self.Boxes.append(Inst)

        # 更新処理
        self.Timer = app.Timer(interval = 1.0 / 30.0, connect = self.Update)
        #self.Timer = app.Timer(interval = 'auto', connect = self.Update)
        #self.Timer = app.Timer(interval = 1.0 / 120.0, connect = self.Update)
        self.Timer.start()

        if __name__ == '__main__' and sys.flags.interactive == 0:
            Canvas.app.run()

    # 更新関数
    def Update(self, event):
        self.Scene.Update(self.Timer.interval)

        for i in range(len(self.Scene.RigidBodies)):
            Rb = self.Scene.RigidBodies[i]
            Vis = self.Spheres[i]

            Vis.transform.reset()

            Axis = quaternion.as_rotation_vector(Rb.Rotation)
            LenSq = Axis @ Axis
            if False == np.isclose(LenSq, 0.0):
                Axis /= np.sqrt(LenSq)
                Vis.transform.rotate(-np.rad2deg(Rb.Rotation.angle()), [Axis[0], Axis[2], Axis[1]])
            
            Vis.transform.translate([Rb.Position[0], Rb.Position[2], Rb.Position[1]])
App()

