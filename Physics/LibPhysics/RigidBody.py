from token import EQUAL
import numpy as np
import quaternion

from LibPhysics import Shape

class RigidBody:
    """RigidBody"""

    def __init__(self, Shape, InvMass = 1.0):
        self.Gravity = np.array([0.0, -9.8, 0.0])

        self.Position = np.zeros(3)
        self.Rotation = np.quaternion(1.0, 0.0, 0.0, 0.0)
        
        self.LinearVelocity = np.zeros(3)
        self.AngularVelocity = np.zeros(3)

        self.InvMass = InvMass
        
        self.Elasticity = 0.5
        self.Friction = 0.5

        self.Shape = Shape
        if self.InvMass != 0.0:
            self.InertiaTensor = self.Shape.InertiaTensor / self.InvMass
        else:
            self.InertiaTensor = np.zeros((3, 3))
        self.InvInertiaTensor = self.Shape.InvInertiaTensor * self.InvMass

    def __del__(self):
        pass

    def GetWorldCenterOfMass(self):
        return self.Position + quaternion.rotate_vectors(self.Rotation, self.Shape.CenterOfMass)

    def ToLocal(self, rhs, Center):
        return quaternion.inverse().rotate_vectors(self.Rotation, rhs - Center)
    def ToLocalPos(self, rhs):
        return self.ToLocal(rhs, self.GetWorldCenterOfMass())
    def ToLocalDir(self, rhs):
        return self.ToLocal(rhs, np.zeros(3))
    
    def ToWorld(self, rhs, Center):
        return Center + quaternion.rotate_vectors(self.Rotation, rhs)
    def ToWorldPos(self, rhs):
        return self.ToWorld(rhs, self.GetWorldSpaceCenterOfMass())
    def ToWorldDir(self, rhs):
        return self.ToWorld(rhs, np.zeros(3))

    def ToWorld(self, rhs):
        Mat3 = quaternion.as_rotation_matrix(self.Rotation)
        return Mat3 @ rhs @ Mat3.T
    def GetWorldInertiaTensor(self):
        return self.ToWorld(self.InertiaTensor)
    def GetWorldInverseInertiaTensor(self):
        return self.ToWorld(self.InvInertiaTensor)

    def ApplyGravity(self, DeltaSec):
        if 0.0 != self.InvMass:
	        self.LinearVelocity += self.Gravity * DeltaSec
    def ApplyLinearImpulse(self, Impulse):
        if 0.0 != self.InvMass:
            self.LinearVelocity += Impulse * self.InvMass
    def ApplyAngularImpulse(self, Impulse):
        if 0.0 != self.InvMass:
            self.AngularVelocity += self.WorldInverseInertiaTensor() @ Impulse
            Limit = 30.0
            LenSq = self.AngularVelocity @ self.AngularVelocity
            if LenSq > Limit * Limit:
                self.AngularVelocity = self.AngularVelocity / np.sqrt(LenSq) * Limit
    def ApplyImpulse(self, ImpactPoint, Impulse):
        if 0.0 != self.InvMass:
            self.ApplyLinearImpulse(Impulse)
            Radius = ImpactPoint - self.GetWorldCenterOfMass()
            self.ApplyAngularImpulse(np.cross(Radius, Impulse))

    def Update(self, DeltaSec):
        self.Position += self.LinearVelocity * DeltaSec
    
        InvIT = self.GetWorldInverseInertiaTensor()
        IT = self.GetWorldInertiaTensor()

        AngAccel = InvIT @ np.cross(self.AngularVelocity, IT @ self.AngularVelocity)
        self.AngularVelocity += AngAccel * DeltaSec

        DeltaAng = self.AngularVelocity * DeltaSec
        DeltaQuat = quaternion.from_rotation_vector(DeltaAng)

        self.Rotation = (DeltaQuat * self.Rotation).normalized()

        CM = self.GetWorldCenterOfMass();
        self.Position = CM + quaternion.rotate_vectors(DeltaQuat, self.Position - CM)
