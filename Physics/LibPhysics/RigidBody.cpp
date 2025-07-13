#include "RigidBody.h"
#include "Shape.h"

Physics::RigidBody::RigidBody(const Physics::Shape* Sh, const float InvMass)
	: Shape(Sh), InvMass(InvMass)
{
	if (0.0f != InvMass) {
		InertiaTensor = Shape->GetInertiaTensor() / InvMass;
	}
	//!< InvMass を加味したもの (加味しないものはシェイプから取得すること)
	InvInertiaTensor = Shape->GetInvInertiaTensor() * InvMass;
}

LinAlg::Vec3 Physics::RigidBody::GetCenterOfMass() const 
{
	return Shape->GetCenterOfMass(); 
};

void Physics::RigidBody::Update(const float DeltaSec)
{
	//!< 質量無限扱いだが動くものがあるので早期終了できない、動かないことが分かっているのであれば可能
	//if (0.0f == InvMass) {
	//	return;
	//}

	//!< (速度による) 位置の更新
	{
		Position += LinearVelocity * DeltaSec;
	}

	//!< (角速度による) 位置、回転の更新
	{
		//!< ワールド空間の (逆) 慣性テンソル
		const auto InvWIT = GetWorldInverseInertiaTensor();
		const auto WIT = GetWorldInertiaTensor();

		//!< 角加速度 AngAccel = InvWIT * (w x (WIT * w))
		const auto AngAccel = InvWIT * (AngularVelocity.Cross(WIT * AngularVelocity));
		
		//!< 角速度の更新
		AngularVelocity += AngAccel * DeltaSec;

		//!< 角変化の四元数表現
		const auto DeltaAng = AngularVelocity * DeltaSec;
		const auto DeltaQuat = LinAlg::Quat(DeltaAng, DeltaAng.Length());
	
		//!< (角変化による) 回転の更新
		Rotation = (DeltaQuat * Rotation).Normalize();

		//!< (角変化による) 位置の更新
		const auto WorldCenter = GetWorldCenterOfMass();
		Position = WorldCenter + DeltaQuat.Rotate(Position - WorldCenter);
	}
}