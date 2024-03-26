#include "RigidBody.h"
#include "Shape.h"

void Physics::RigidBody::Init(const Physics::Shape* Sh) 
{
	Shape = Sh;
	//!< InvMass を加味したもの (加味しないものはシェイプから取得すること)
	InvInertiaTensor = Shape->GetInvInertiaTensor() * InvMass;
}

Math::Vec3 Physics::RigidBody::GetCenterOfMass() const 
{
	return Shape->GetCenterOfMass(); 
};

void Physics::RigidBody::Update(const float DeltaSec)
{
	//!< (速度による) 位置の更新
	{
		Position += LinearVelocity * DeltaSec;
	}

	//!< (角速度による) 位置、回転の更新
	{
		//!< 角加速度 AngAccel = Inv(I) * (w x (I ・ w))
		const auto AngAccel = ToWorld(Shape->GetInvInertiaTensor()) * (AngularVelocity.Cross(ToWorld(Shape->GetInertiaTensor()) * AngularVelocity));
		//!< 角速度
		AngularVelocity += AngAccel * DeltaSec;

		//!< 回転の更新 Quat' = dQuat * Quat
		const auto DeltaAng = AngularVelocity * DeltaSec;
		const auto DeltaQuat = Math::Quat(DeltaAng, DeltaAng.Length());
		Rotation = (DeltaQuat * Rotation).Normalize();

		//!< 位置の更新
		const auto WorldCenter = GetWorldSpaceCenterOfMass();
		Position = WorldCenter + DeltaQuat.Rotate(Position - WorldCenter);
	}
}