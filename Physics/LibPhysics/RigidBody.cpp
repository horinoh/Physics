#include "RigidBody.h"
#include "Shape.h"

Physics::RigidBody::RigidBody(const Physics::Shape* Sh, const float InvMass)
	: Shape(Sh), InvMass(InvMass)
{
	if (0.0f != InvMass) {
		InertiaTensor = Shape->GetInertiaTensor() / InvMass;
	}
	//!< InvMass �������������� (�������Ȃ����̂̓V�F�C�v����擾���邱��)
	InvInertiaTensor = Shape->GetInvInertiaTensor() * InvMass;
}

LinAlg::Vec3 Physics::RigidBody::GetCenterOfMass() const 
{
	return Shape->GetCenterOfMass(); 
};

void Physics::RigidBody::Update(const float DeltaSec)
{
	//!< ���ʖ������������������̂�����̂ő����I���ł��Ȃ��A�����Ȃ����Ƃ��������Ă���̂ł���Ή\
	//if (0.0f == InvMass) {
	//	return;
	//}

	//!< (���x�ɂ��) �ʒu�̍X�V
	{
		Position += LinearVelocity * DeltaSec;
	}

	//!< (�p���x�ɂ��) �ʒu�A��]�̍X�V
	{
		//!< ���[���h��Ԃ� (�t) �����e���\��
		const auto InvWIT = GetWorldInverseInertiaTensor();
		const auto WIT = GetWorldInertiaTensor();

		//!< �p�����x AngAccel = InvWIT * (w x (WIT * w))
		const auto AngAccel = InvWIT * (AngularVelocity.Cross(WIT * AngularVelocity));
		
		//!< �p���x�̍X�V
		AngularVelocity += AngAccel * DeltaSec;

		//!< �p�ω��̎l�����\��
		const auto DeltaAng = AngularVelocity * DeltaSec;
		const auto DeltaQuat = LinAlg::Quat(DeltaAng, DeltaAng.Length());
	
		//!< (�p�ω��ɂ��) ��]�̍X�V
		Rotation = (DeltaQuat * Rotation).Normalize();

		//!< (�p�ω��ɂ��) �ʒu�̍X�V
		const auto WorldCenter = GetWorldCenterOfMass();
		Position = WorldCenter + DeltaQuat.Rotate(Position - WorldCenter);
	}
}