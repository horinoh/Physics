import sys
import random
import math

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

from Physics.Collision.GJK import TestSignedVolume

class App:
    def __init__(self):
        # キャンバス
        Canvas = scene.SceneCanvas(keys = 'interactive', bgcolor = 'skyblue', size = (800, 600), show = True)
        # ビューを追加
        View = Canvas.central_widget.add_view()
        # カメラ設定
        View.camera = 'arcball'
        View.camera.set_range(x = [-10, 10])

        # (描画用)
        self.Poly = scene.visuals.Polygon(pos = [[-5, 0, 0],[5, 0, 0],[0, 5, 0]], border_method="agg" )
        self.Poly.transform = MatrixTransform()

        # 更新処理
        FPS = 1.0 / 30.0
        self.Timer = app.Timer(interval = FPS, connect = self.Update)
        self.Timer.start()
        self.IsStop = False

        TestSignedVolume()

        @Canvas.events.key_press.connect
        def on_key_press(event):
            if event.key == ' ':
                self.IsStop = False if self.IsStop else True
                if self.IsStop:
                    self.Timer.stop()
                else:
                    self.Timer.start()

        if __name__ == '__main__' and sys.flags.interactive == 0:
            Canvas.app.run()

    # 更新関数
    def Update(self, event):
        self.Poly.transform.reset()
        self.Poly.transform.rotate(self.Timer.elapsed*100, [1, 1, 1])
App()

