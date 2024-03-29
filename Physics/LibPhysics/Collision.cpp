#include "Collision.h"
#include "RigidBody.h"
#include "Shape.h"
#include "GJK.h"

bool Collision::Intersection::RaySphere(const Math::Vec3& RayPos, const Math::Vec3& RayDir, const Math::Vec3& SpPos, const float SpRad, float& T0, float& T1)
{
	//!< 1)球	(x - SpPos)^2 = SpRad^2
	//!<		x^2 - 2 * x * SpPos + SpPos^2 - SpRad^2 = 0
	//!< 2)レイ	RayPos + RayDir * t 
	//!<	1) の x に 2) を代入 
	//!< (RayPos + RayDir * t - SpPos)^2 = SpRad^2
	//!< (RayDir * t + M)^2 - SpRad^2 = 0 ... M = RayPos - SpPos
	//!< RayDir^2 * t^2 + 2 * M * RayDir * t + M^2 - SpRad^2 = 0
	//!< A * t^2 + B * t + C = 0 ... A = RayDir^2, B = 2 * M * RayDir, C = M^2 - SpRad^2
	const auto M = RayPos - SpPos;
	const auto A = RayDir.Dot(RayDir);
	const auto B2 = M.Dot(RayDir);
	const auto C = M.Dot(M) - SpRad * SpRad;

	const auto D4 = B2 * B2 - A * C;
	if (D4 >= 0) {
		const auto D4Sqrt = sqrt(D4);
		T0 = (B2 - D4Sqrt) / A;
		T1 = (B2 + D4Sqrt) / A;
		return true;
	}
	return false;
}

bool Collision::Intersection::SphereShpere(const float RadA, const float RadB,
	const Math::Vec3& PosA, const Math::Vec3& PosB,
	const Math::Vec3& VelA, const Math::Vec3& VelB,
	const float DeltaSec, float& T)
{
	const auto Ray = (VelB - VelA) * DeltaSec;
	const auto TotalRadius = RadA + RadB;

	auto T0 = 0.0f, T1 = 0.0f;
	constexpr auto Eps2 = 0.001f * 0.001f;
	if (Ray.LengthSq() < Eps2) {
		//!< レイが十分短い場合は既に衝突しているかどうかのチェック
		const auto PosAB = PosB - PosA;
		const auto R = TotalRadius + 0.001f;
		if (PosAB.LengthSq() > R * R) {
			return false;
		}
	}
	//!< レイ vs 球 に帰着
	else if (false == Intersection::RaySphere(PosA, Ray, PosB, TotalRadius, T0, T1) || (T0 > 1.0f || T1 < 0.0f)) {
		return false;
	}

	T0 *= DeltaSec;
	//T1 *= DeltaSec;

	T = (std::max)(T0, 0.0f);

	return true;
}
bool Collision::Intersection::RigidBodyRigidBody(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const float DeltaSec, Contact& Ct)
{
	if (RbA->Shape->GetShapeType() == Physics::Shape::SHAPE::SPHERE && RbB->Shape->GetShapeType() == Physics::Shape::SHAPE::SPHERE) {
		const auto SpA = static_cast<const Physics::ShapeSphere*>(RbA->Shape);
		const auto SpB = static_cast<const Physics::ShapeSphere*>(RbB->Shape);
		float T;
		if (Intersection::SphereShpere(SpA->Radius, SpB->Radius, RbA->Position, RbB->Position, RbA->LinearVelocity, RbB->LinearVelocity, DeltaSec, T)) {
			Ct.TimeOfImpact = T;

			//!< 衝突時刻の(中心)位置
			const auto CPosA = RbA->Position + RbA->LinearVelocity * T;
			const auto CPosB = RbB->Position + RbB->LinearVelocity * T;

			//!< 法線 A -> B
			Ct.Normal = (CPosB - CPosA).Normalize();

			//!< 衝突点 (半径の分オフセット)
			Ct.PointA = CPosA + Ct.Normal * SpA->Radius;
			Ct.PointB = CPosB - Ct.Normal * SpB->Radius;

			//!< 衝突剛体を覚えておく
			Ct.RigidBodyA = const_cast<Physics::RigidBody*>(RbA);
			Ct.RigidBodyB = const_cast<Physics::RigidBody*>(RbB);

			return true;
		}
		return false;
	}
	else {
		//!< ワーク
		auto WRbA = *RbA, WRbB = *RbB;

		auto DT = DeltaSec;
		auto TOI = 0.0f;
		auto ItCount = 0;
		while (DT > 0.0f) {
			//!< 衝突点、最近接点
			Math::Vec3 OnA, OnB;
			constexpr auto Bias = 0.001f;
			if (Intersection::GJK_EPA(&WRbA, &WRbB, Bias, OnA, OnB)) {
				Ct.TimeOfImpact = TOI;

				//!< 法線 A -> B
#if 0
				//!< 法線が小さすぎる場合があるので、拡大してから正規化するテスト
				constexpr auto Scale = 100.0f;
				Ct.Normal = ((OnB - OnA) * Scale).Normalize();
#else
				Ct.Normal = (OnB - OnA).Normalize();
#endif
				//!< シンプレックスを拡張しているので、その分をキャンセルする
				OnA -= Ct.Normal * Bias;
				OnB += Ct.Normal * Bias;

				//!< 衝突点
				Ct.PointA = OnA;
				Ct.PointB = OnB;

				//!< 衝突剛体を覚えておく
				Ct.RigidBodyA = const_cast<Physics::RigidBody*>(RbA);
				Ct.RigidBodyB = const_cast<Physics::RigidBody*>(RbB);

				return true;
			}

			//!< 移動せずその場で回転しているような場合、ループから抜け出さない事があるのでループに上限回数を設ける
			++ItCount;
			if (ItCount > 10) {
				break;
			}

			//!< 回転を考慮した相対速度を求める
			//!< A -> B 方向
			const auto Dir = (OnB - OnA).Normalize();
			//!< A の相対速度、角速度
			const auto LVel = (WRbA.LinearVelocity - WRbB.LinearVelocity).Dot(Dir);
			const auto AVel = WRbA.Shape->GetFastestPointSpeed(WRbA.AngularVelocity, Dir) - WRbB.Shape->GetFastestPointSpeed(WRbB.AngularVelocity, Dir);
			const auto OrthoSpeed = LVel + AVel;
			if (OrthoSpeed <= 0.0f) {
				//!< 近づいていない
				break;
			}

			//!< 衝突するであろう直前までの時間を求める
			const auto SeparationDistance = (OnB - OnA).Length();
			const auto TimeToGo = SeparationDistance / OrthoSpeed;
			if (TimeToGo > DT) {
				//!< DT 以内には存在しない
				break;
			}

			//!< 衝突するであろう直前まで時間を進める
			DT -= TimeToGo;
			TOI += TimeToGo;
			WRbA.Update(TimeToGo);
			WRbB.Update(TimeToGo);
		}
	}
	return false;
}
void Collision::ResolveContact(const Contact& Ct)
{
	const auto TotalInvMass = Ct.RigidBodyA->InvMass + Ct.RigidBodyB->InvMass;
	{
		//!< 半径 (重心 -> 衝突点)
		const auto RA = Ct.PointA - Ct.RigidBodyA->GetWorldSpaceCenterOfMass();
		const auto RB = Ct.PointB - Ct.RigidBodyB->GetWorldSpaceCenterOfMass();
		{
			//!< 逆慣性テンソル (ワールドスペース)
			const auto WorldInvInertiaA = Ct.RigidBodyA->GetWorldSpaceInverseInertiaTensor();
			const auto WorldInvInertiaB = Ct.RigidBodyB->GetWorldSpaceInverseInertiaTensor();

			//!< (A 視点の) 相対速度 A -> B
			const auto VelA = Ct.RigidBodyA->LinearVelocity + Ct.RigidBodyA->AngularVelocity.Cross(RA);
			const auto VelB = Ct.RigidBodyB->LinearVelocity + Ct.RigidBodyB->AngularVelocity.Cross(RB);
			const auto VelAB = VelA - VelB;
			//!< 速度の法線成分
			const auto& Nrm = Ct.Normal;
			const auto NVelAB = Nrm * VelAB.Dot(Nrm);

			//!< 法線方向 力積J (運動量変化)
			{
				//!< 両者の弾性係数を掛けただけの簡易な実装とする
				const auto TotalElasticity = 1.0f + Ct.RigidBodyA->Elasticity * Ct.RigidBodyB->Elasticity;
				{
					const auto AngJA = (WorldInvInertiaA * RA.Cross(Nrm)).Cross(RA);
					const auto AngJB = (WorldInvInertiaB * RB.Cross(Nrm)).Cross(RB);
					const auto AngFactor = (AngJA + AngJB).Dot(Nrm);

					const auto J = NVelAB * TotalElasticity / (TotalInvMass + AngFactor);

					Ct.RigidBodyA->ApplyImpulse(Ct.PointA, -J);
					Ct.RigidBodyB->ApplyImpulse(Ct.PointB, J);
				}
			}

			//!< 接線方向 力積J (摩擦力)
			{
				//!< 速度の接線成分
				const auto TVelAB = VelAB - NVelAB;
				const auto Tan = TVelAB.Normalize();

				const auto TotalFriction = Ct.RigidBodyA->Friction * Ct.RigidBodyB->Friction;
				{
					const auto AngJA = (WorldInvInertiaA * RA.Cross(Tan)).Cross(RA);
					const auto AngJB = (WorldInvInertiaB * RB.Cross(Tan)).Cross(RB);
					const auto AngFactor = (AngJA + AngJB).Dot(Tan);

					const auto J = TVelAB * TotalFriction / (TotalInvMass + AngFactor);

					Ct.RigidBodyA->ApplyImpulse(Ct.PointA, -J);
					Ct.RigidBodyB->ApplyImpulse(Ct.PointB, J);
				}
			}
		}
	}

	//!< めり込みの追い出し (TOI == 0.0f の時点で衝突している場合)
	if (0.0f == Ct.TimeOfImpact) {
		//!< 質量により追い出し割合を考慮
		const auto DistAB = Ct.PointB - Ct.PointA;
		Ct.RigidBodyA->Position += DistAB * (Ct.RigidBodyA->InvMass / TotalInvMass);
		Ct.RigidBodyB->Position -= DistAB * (Ct.RigidBodyB->InvMass / TotalInvMass);
	}
}
