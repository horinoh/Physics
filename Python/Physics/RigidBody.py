import numpy as np
import quaternion

from Physics import Shape

class RigidBody:
    """RigidBody"""

    def __init__(self):
        self.Position = np.zeros(3)
        self.Rotation = np.quaternion(1.0, 0.0, 0.0, 0.0)

        self.LinearVelocity = np.zeros(3)
        self.AngularVelocity = np.zeros(3)

        self.InvMass = 1.0
        self.InvInertiaTensor = np.zeros((3, 3))
    
        self.Elasticity = 0.5
        self.Friction = 0.5
    
        self._Shape = None
    def __del__(self):
        pass         

    # シェイプ
    @property
    def Shape(self):
        return self._Shape
    @Shape.setter
    def Shape(self, rhs):
        self._Shape = rhs
        self.InvInertiaTensor = self.Shape.InvInertiaTensor * self.InvMass
    @property
    
    # 慣性テンソル
    def InertiaTensor(self):
        return self.Shape.InertiaTensor

    # 重心
    @property
    def CenterOfMass(self):
        return self.Shape.CenterOfMass
    @property    
    def WorldCenterOfMass(self):
         return self.Position + quaternion.rotate_vectors(self.Rotation, self.CenterOfMass)

    # AABB
    def GetAABB(self):
        return self.Shape.GetAABB(self.Position, self.Rotation)

    # 方向 (ワールド変換)
    def ToWorldDir(self, rhs):
        return quaternion.rotate_vectors(self.Rotation, rhs)
    # 位置 (ワールド変換)
    def ToWorldPos(self, rhs):
        return self.WorldCenterOfMass + self.ToWorldDir(rhs)
    # 方向 (ローカル変換)
    def ToLocalDir(self, rhs):
        return quaternion.inverse().rotate_vectors(self.Rotation, rhs)
    # 位置 (ローカル変換)
    def ToLocalPos(self, rhs):
        return self.ToLocalDir(rhs - self.WorldCenterOfMass)
    
    # 行列をワールドスペースへ
    def ToWorld(self, rhs):
        Mat3 = quaternion.as_rotation_matrix(self.Rotation)
        return Mat3 @ rhs @ Mat3.T
    
    # 線形力積
    def ApplyLinearImpulse(self, Impulse):
        if self.InvMass != 0.0:
            self.LinearVelocity += Impulse * self.InvMass
    # 角力積
    def ApplyAngularImpulse(self, Impulse):
        if self.InvMass != 0.0:
            self.AngularVelocity += self.ToWorld(self.InvInertiaTensor) @ Impulse
    # 力積 (総合)
    def ApplyImpulse(self, ImpactPoint, Impulse):
        if self.InvMass != 0.0:
            self.ApplyLinearImpulse(Impulse)    
            self.ApplyAngularImpulse(np.cross(ImpactPoint - self.WorldCenterOfMass, Impulse))
    
    def Update(self, DeltaSec):
        # 位置
        self.Position += self.LinearVelocity * DeltaSec

        # 角速度
        WInvIner = self.ToWorld(self.InvInertiaTensor)
        WIner = self.ToWorld(self.InertiaTensor)
        AngAccel = WInvIner @ np.cross(self.AngularVelocity, WIner @ self.AngularVelocity)
        self.AngularVelocity += AngAccel * DeltaSec

        # 回転
        DAng = self.AngularVelocity * DeltaSec
        DQuat = quaternion.from_rotation_vector(DAng)
        self.Rotation = (DQuat * self.Rotation).normalized()
        
        # 回転後の位置
        WCOM = self.WorldCenterOfMass
        self.Position = WCOM + quaternion.rotate_vectors(DQuat, self.Position - WCOM)

    
