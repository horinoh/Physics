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

from Physics.Collision.GJK import TestSignedVolume

class App:
    def __init__(self):
        print("Python=", sys.version)
        print("Numpy+", np.__version__)

        # キャンバス
        Canvas = scene.SceneCanvas(keys = 'interactive', bgcolor = 'skyblue', size = (800, 600), show = True)
        # ビューを追加
        View = Canvas.central_widget.add_view()
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
            if i == 0:
                Rb.Position = [ -2.0, 0.0, 0.0 ]
            self.Scene.RigidBodies.append(Rb)

        # ボックス (描画用)
        self.Boxes = []
        for i in self.Scene.RigidBodies:
            Inst = scene.visuals.Box(width = i.Shape.Extent[0], height = i.Shape.Extent[1], depth = i.Shape.Extent[2], color = "yellow" if i.InvMass != 0.0 else "green", edge_color = 'black', parent = View.scene)
            Inst.transform = MatrixTransform()
            self.Boxes.append(Inst)

        # (描画用)
        #self.Poly = scene.visuals.Polygon(pos = [[-5, 0, 0],[5, 0, 0],[0, 5, 0]], border_method="agg" )
        #self.Poly.transform = MatrixTransform()

        # 更新処理
        FPS = 1.0 / 30.0
        self.Timer = app.Timer(interval = FPS, connect = self.Update)
        self.Timer.start()
        self.IsStop = False

        TestSignedVolume()

        @Canvas.events.key_press.connect
        def on_key_press(event):
            Rb = self.Scene.RigidBodies[0]

            match event.key:
                case ' ':
                    self.IsStop = False if self.IsStop else True
                    if self.IsStop:
                        self.Timer.stop()
                    else:
                        self.Timer.start()
                case 'w':
                    Rb.Position[2] += 0.1
                case 'a':
                    Rb.Position[0] -= 0.1
                case 's':
                    Rb.Position[2] -= 0.1
                case 'd':
                    Rb.Position[0] += 0.1
                case 'Up':
                    print("Up")
                case 'Left':
                    print("Left")
                case 'Down':
                    print("Down")
                case 'Right':
                    print("Right")

        if __name__ == '__main__' and sys.flags.interactive == 0:
            Canvas.app.run()

    # 更新関数
    def Update(self, event):
        self.Scene.Update(self.Timer.interval)
        
        for i in range(len(self.Scene.RigidBodies)):
            Rb = self.Scene.RigidBodies[i]
            
            VisBox = self.Boxes[i]
            VisBox.transform.reset()

            # 回転軸が取れれば回転する
            Axis = quaternion.as_rotation_vector(Rb.Rotation)
            LenSq = Axis @ Axis
            if False == np.isclose(LenSq, 0.0):
                Axis /= math.sqrt(LenSq)
                VisBox.transform.rotate(-np.rad2deg(Rb.Rotation.angle()), [Axis[0], Axis[2], Axis[1]])
            
            # YZ が入れ替わるので注意s
            Pos = [Rb.Position[0], Rb.Position[2], Rb.Position[1]]
            # ボックス
            VisBox.transform.translate(Pos)
App()

