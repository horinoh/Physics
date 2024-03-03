#include "Collision.h"
#include "RigidBody.h"
#include "Shape.h"
#include "GJK.h"

bool Collision::Intersection::RaySphere(const Math::Vec3& RayPos, const Math::Vec3& RayDir, const Math::Vec3& SpPos, const float SpRad, float& T0, float& T1)
{
	//!< 1)��	(x - SpPos)^2 = SpRad^2
	//!<		x^2 - 2 * x * SpPos + SpPos^2 - SpRad^2 = 0
	//!< 2)���C	RayPos + RayDir * t 
	//!<	1) �� x �� 2) ���� 
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
		T0 = (-B2 - D4Sqrt) / A;
		T1 = (-B2 + D4Sqrt) / A;
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
		//!< ���C���\���Z���ꍇ�͊��ɏՓ˂��Ă��邩�ǂ����̃`�F�b�N
		const auto PosAB = PosB - PosA;
		const auto R = TotalRadius + 0.001f;
		if (PosAB.LengthSq() > R * R) {
			return false;
		}
	}
	//!< ���C vs �� �ɋA��
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
	if (RbA->Shape->GetShapeTyoe() == Physics::Shape::SHAPE::SPHERE && RbB->Shape->GetShapeTyoe() == Physics::Shape::SHAPE::SPHERE) {
		const auto SpA = static_cast<const Physics::ShapeSphere*>(RbA->Shape);
		const auto SpB = static_cast<const Physics::ShapeSphere*>(RbB->Shape);
		float T;
		if (Intersection::SphereShpere(SpA->Radius, SpB->Radius, RbA->Position, RbB->Position, RbA->LinearVelocity, RbB->LinearVelocity, DeltaSec, T)) {
			Ct.TimeOfImpact = T;

			//!< �Փˎ��Ԃ̃I�u�W�F�N�g�̈ʒu
			const auto CPosA = RbA->Position + RbA->LinearVelocity * T;
			const auto CPosB = RbB->Position + RbB->LinearVelocity * T;
			//!< �@�� A -> B
			Ct.Normal = (CPosB - CPosA).Normalize();

			//!< �Փ˓_
			Ct.PointA = CPosA + Ct.Normal * SpA->Radius;
			Ct.PointB = CPosB - Ct.Normal * SpB->Radius;

			//!< �Փˍ��̂��o���Ă���
			Ct.RigidBodyA = const_cast<Physics::RigidBody*>(RbA);
			Ct.RigidBodyB = const_cast<Physics::RigidBody*>(RbB);

			return true;
		}
		return false;
	}
	else {
		auto DT = DeltaSec;
		auto TOI = 0.0f;
		auto ItCount = 0;
		while (DT > 0.0f) {
			//!< �Փ˓_�A�ŋߐړ_
			Math::Vec3 OnA, OnB;
			constexpr auto Bias = 0.001f;
			if (Intersection::GJK_EPA(RbA, RbB, Bias, OnA, OnB)) {
				Ct.TimeOfImpact = TOI;

				//!< �@�� A -> B
				Ct.Normal = (OnB - OnA).Normalize();
				OnA -= Ct.Normal * Bias;
				OnB += Ct.Normal * Bias;

				//!< �Փ˓_
				Ct.PointA = OnA;
				Ct.PointB = OnB;

				//!< �Փˍ��̂��o���Ă���
				Ct.RigidBodyA = const_cast<Physics::RigidBody*>(RbA);
				Ct.RigidBodyB = const_cast<Physics::RigidBody*>(RbB);

				return true;
			}
			else {
				//!< �Փ˂������ꍇ�́A�ŋߐړ_�����߂�
				Closest::GJK(RbA, RbB, OnA, OnB);
			}

			//!< �ړ��������̏�ŉ�]���Ă���悤�ȏꍇ�A���[�v���甲���o���Ȃ���������̂Ń��[�v�ɏ���񐔂�݂���
			++ItCount;
			if (ItCount > 10) {
				break;
			}

			//!< ��]���l���������Α��x�����߂�
			const auto Dir = (OnB - OnA).Normalize();
			const auto LVel = RbA->LinearVelocity - RbB->LinearVelocity;
			const auto AngVelA = RbA->Shape->GetFastestPointSpeed(RbA->AngularVelocity, Dir);
			const auto AngVelB = RbB->Shape->GetFastestPointSpeed(RbB->AngularVelocity, -Dir);
			const auto OrthoSpeed = LVel.Dot(Dir) + AngVelA + AngVelB;
			if (OrthoSpeed <= 0.0f) {
				break;
			}

			//!< �Փ˒��O�܂Ŏ��Ԃ�i�߂�
			const auto SeparationDistance = (OnB - OnA).Length();
			const auto TimeToGo = SeparationDistance / OrthoSpeed;
			if (TimeToGo > DT) {
				break;
			}

			DT -= TimeToGo;
			TOI += TimeToGo;
		}
	}
	return false;
}
void Collision::Resolve(const Contact& Ct)
{
	const auto TotalInvMass = Ct.RigidBodyA->InvMass + Ct.RigidBodyB->InvMass;
	{
		//!< ���a (�d�S -> �Փ˓_)
		const auto RadiusA = Ct.PointA - Ct.RigidBodyA->GetWorldSpaceCenterOfMass();
		const auto RadiusB = Ct.PointB - Ct.RigidBodyB->GetWorldSpaceCenterOfMass();
		{
			//!< �t�����e���\�� (���[���h�X�y�[�X)
			const auto WorldInvInertiaA = Ct.RigidBodyA->GetWorldSpaceInverseInertiaTensor();
			const auto WorldInvInertiaB = Ct.RigidBodyB->GetWorldSpaceInverseInertiaTensor();

			//!< (A ���_��)���x A -> B
			const auto VelA = Ct.RigidBodyA->LinearVelocity + Ct.RigidBodyA->AngularVelocity.Cross(RadiusA);
			const auto VelB = Ct.RigidBodyB->LinearVelocity + Ct.RigidBodyB->AngularVelocity.Cross(RadiusB);
			const auto VelAB = VelB - VelA;
			//!< ���x�̖@������
			const auto NrmVelAB = Ct.Normal * VelAB.Dot(Ct.Normal);

			//!< �@������ �͐�J (�^���ʕω�)
			{
				//!< ���҂̒e���W�����|���������̊ȈՂȎ����Ƃ���
				const auto TotalElasticity = 1.0f + Ct.RigidBodyA->Elasticity * Ct.RigidBodyB->Elasticity;
				{
					const auto AngularJA = (WorldInvInertiaA * RadiusA.Cross(Ct.Normal)).Cross(RadiusA);
					const auto AngularJB = (WorldInvInertiaB * RadiusB.Cross(Ct.Normal)).Cross(RadiusB);
					const auto AngularFactor = (AngularJA + AngularJB).Dot(Ct.Normal);

					const auto J = NrmVelAB * TotalElasticity / (TotalInvMass + AngularFactor);

					Ct.RigidBodyA->ApplyTotalImpulse(Ct.PointA, J);
					Ct.RigidBodyB->ApplyTotalImpulse(Ct.PointB, -J);
				}
			}

			//!< �ڐ����� �͐�J (���C��)
			{
				//!< ���x�̐ڐ�����
				const auto TanVelAB = VelAB - NrmVelAB;
				const auto Tangent = TanVelAB.Normalize();

				const auto TotalFriction = Ct.RigidBodyA->Friction * Ct.RigidBodyB->Friction;
				{
					const auto AngularJA = (WorldInvInertiaA * RadiusA.Cross(Tangent)).Cross(RadiusA);
					const auto AngularJB = (WorldInvInertiaB * RadiusB.Cross(Tangent)).Cross(RadiusB);
					const auto AngularFactor = (AngularJA + AngularJB).Dot(Tangent);

					const auto J = TanVelAB * TotalFriction / (TotalInvMass + AngularFactor);

					Ct.RigidBodyA->ApplyTotalImpulse(Ct.PointA, J);
					Ct.RigidBodyB->ApplyTotalImpulse(Ct.PointB, -J);
				}
			}
		}
	}

	//!< �߂荞�݂̒ǂ��o�� (TOI == 0.0f �̎��_�ŏՓ˂��Ă���ꍇ)
	if (0.0f == Ct.TimeOfImpact) {
		//!< ���ʂɂ��ǂ��o���������l��
		const auto DistAB = Ct.PointB - Ct.PointA;
		Ct.RigidBodyA->Position += DistAB * (Ct.RigidBodyA->InvMass / TotalInvMass);
		Ct.RigidBodyB->Position -= DistAB * (Ct.RigidBodyB->InvMass / TotalInvMass);
	}
}
