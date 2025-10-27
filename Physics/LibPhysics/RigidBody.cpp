#include "RigidBody.h"
#include "Shape.h"

Physics::RigidBody::RigidBody(const Physics::Shape* Sh, const float InvMass)
	: Shape(Sh), Mass_Inverse(InvMass)
{
	if (0.0f != InvMass) {
		InertiaTensor = Shape->GetInertiaTensor() / InvMass;
	}
	//!< InvMass を加味したもの (加味しないものはシェイプから取得すること)
	InertiaTensor_Inverse = Shape->GetInertiaTensor_Inverse() * Mass_Inverse;
}

LinAlg::Vec3 Physics::RigidBody::GetCenterOfMass() const 
{
	return Shape->GetCenterOfMass(); 
};

void Physics::RigidBody::ApplyImpulse(const Collision::Contact& Ct)
{
	const auto& WPointA = Ct.WPointA;
	const auto& WPointB = Ct.WPointB;

	const auto TotalInvMass = Ct.RigidBodyA->Mass_Inverse + Ct.RigidBodyB->Mass_Inverse;
	{
		//!< 半径 (重心 -> 衝突点)
		const auto RA = WPointA - Ct.RigidBodyA->GetCenterOfMass_World();
		const auto RB = WPointB - Ct.RigidBodyB->GetCenterOfMass_World();
		{
			//!< 逆慣性テンソル (ワールドスペース)
			const auto InvIA = Ct.RigidBodyA->GetInertiaTensor_Inverse_World();
			const auto InvIB = Ct.RigidBodyB->GetInertiaTensor_Inverse_World();

			//!< (A 視点の) 相対速度
			const auto VelA = Ct.RigidBodyA->Velocity_Linear + Ct.RigidBodyA->Velocity_Angular.Cross(RA);
			const auto VelB = Ct.RigidBodyB->Velocity_Linear + Ct.RigidBodyB->Velocity_Angular.Cross(RB);
			const auto RelVelA = VelA - VelB;

			//!< c = 1 + e(弾性の場合), \mu(摩擦の場合)
			//!< v = v_b - v_a
			//!< m = (m_a^-1 + m_b^-1)
			//!< n = 法線ベクトル(弾性の場合), 接線ベクトル(摩擦の場合)
			//!< J_a = (I_a^-1 r_a \cross n) \cross r_a
			//!< J_b = (I_b^-1 r_b \cross n) \cross r_b
			//!< とすると
			//!< J = \frac{c v}{m + (J_a + J_b) n}
			auto Apply = [&](const auto& N, const auto& Vel, const float Coef) {
				const auto JA = (InvIA * RA.Cross(N)).Cross(RA);
				const auto JB = (InvIB * RB.Cross(N)).Cross(RB);
				const auto J = Vel * Coef / (TotalInvMass + (JA + JB).Dot(N));
				Ct.RigidBodyA->ApplyImpulse(WPointA, -J);
				Ct.RigidBodyB->ApplyImpulse(WPointB, J);
				};

			//!< 法線方向 力積J (運動量変化)
			const auto& Nrm = Ct.WNormal;
			const auto VelN = Nrm * RelVelA.Dot(Nrm);
			//!< ここでは両者の弾性係数を掛けただけの簡易な実装とする
			const auto TotalElas = 1.0f + Ct.RigidBodyA->Elasticity * Ct.RigidBodyB->Elasticity;
			Apply(Nrm, VelN, TotalElas);

			//!< 接線方向 力積J (摩擦力)
			const auto VelT = RelVelA - VelN;
			const auto Tan = VelT.Normalize();
			//!< ここでは両者の摩擦係数を掛けただけの簡易な実装とする
			const auto TotalFric = Ct.RigidBodyA->Friction * Ct.RigidBodyB->Friction;
			Apply(Tan, VelT, TotalFric);
		}
	}
}

void Physics::RigidBody::Update(const float DeltaSec)
{
	//!< 質量無限扱いだが動くものがあるので早期終了できない、動かないことが分かっているのであれば可能
	//if (0.0f == InvMass) {
	//	return;
	//}

	//!< (速度による) 位置の更新
	{
		Position += Velocity_Linear * DeltaSec;
	}

	//!< (角速度による) 位置、回転の更新
	{
		//!< トルク \tau = \omega \cross (I \omega) = I \alpha より
		//!< 角加速度 \alpha = I^-1 (\omega \cross (I \omega))
		const auto I_Inv = GetInertiaTensor_Inverse_World();
		const auto I = GetInertiaTensor_World();
		const auto AngAccel = I_Inv * (Velocity_Angular.Cross(I * Velocity_Angular));
		
		//!< 角速度の更新
		Velocity_Angular += AngAccel * DeltaSec;

		//!< デルタ角速度の四元数表現
		const auto DeltaVel = Velocity_Angular * DeltaSec;
		const auto DeltaQuat = LinAlg::Quat(DeltaVel, DeltaVel.Length());
		
		//!< 回転の更新 q^' = dq * q
		Rotation = (DeltaQuat * Rotation).Normalize();

		//!< (回転による) 位置の更新
		const auto CenterOfMass = GetCenterOfMass_World();
		Position = CenterOfMass + DeltaQuat.Rotate(Position - CenterOfMass);
	}
}