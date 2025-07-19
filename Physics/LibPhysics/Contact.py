import numpy as np
import math
import copy

from LibPhysics import RigidBody

class ContactBase:
    """ContactBase"""

    def __init__(self):
        self.TimeOfImpact = 0.0

        self.RigidBodyA = None
        self.RigidBodyB = None

        self.WPointA = np.zeros(3)
        self.WPointB = np.zeros(3)
        self.WNormal = np.zeros(3)
    
    def __del__(self):
        pass

    def Swap(self):
        self.RigidBodyA, self.RigidBodyB = self.RigidBodyB, self.RigidBodyA
        self.WPointA, self.WPointB = self.WPointB, self.WPointA
        self.WNormal = -self.WNormal

class Contact(ContactBase):
    """Contact"""

    def __init__(self):
        super(Contact, self).__init__()
        self.LPointA = np.zeros(3)
        self.LPointB = np.zeros(3)

    def __del__(self):
        super(Contact, self).__del__()

    def Swap(self):
        super(Contact, self).Swap()
        self.LPointA, self.LPointB = self.LPointB, self.LPointA

    def CalcLocal(self):
        self.LPointA = self.RigidBodyA.ToLocalPos(self.WPointA)
        self.LPointB = self.RigidBodyB.ToLocalPos(self.WPointB)