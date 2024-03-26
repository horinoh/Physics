#include "RigidBody.h"
#include "Shape.h"

void Physics::RigidBody::Init(const Physics::Shape* Sh) 
{
	Shape = Sh;
	//!< InvMass �������������� (�������Ȃ����̂̓V�F�C�v����擾���邱��)
	InvInertiaTensor = Shape->GetInvInertiaTensor() * InvMass;
}

Math::Vec3 Physics::RigidBody::GetCenterOfMass() const 
{
	return Shape->GetCenterOfMass(); 
};

void Physics::RigidBody::Update(const float DeltaSec)
{
	//!< (���x�ɂ��) �ʒu�̍X�V
	{
		Position += LinearVelocity * DeltaSec;
	}

	//!< (�p���x�ɂ��) �ʒu�A��]�̍X�V
	{
		//!< �p�����x AngAccel = Inv(I) * (w x (I �E w))
		const auto AngAccel = ToWorld(Shape->GetInvInertiaTensor()) * (AngularVelocity.Cross(ToWorld(Shape->GetInertiaTensor()) * AngularVelocity));
		//!< �p���x
		AngularVelocity += AngAccel * DeltaSec;

		//!< ��]�̍X�V Quat' = dQuat * Quat
		const auto DeltaAng = AngularVelocity * DeltaSec;
		const auto DeltaQuat = Math::Quat(DeltaAng, DeltaAng.Length());
		Rotation = (DeltaQuat * Rotation).Normalize();

		//!< �ʒu�̍X�V
		const auto WorldCenter = GetWorldSpaceCenterOfMass();
		Position = WorldCenter + DeltaQuat.Rotate(Position - WorldCenter);
	}
}