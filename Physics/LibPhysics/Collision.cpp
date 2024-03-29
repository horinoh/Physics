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
	if (RbA->Shape->GetShapeType() == Physics::Shape::SHAPE::SPHERE && RbB->Shape->GetShapeType() == Physics::Shape::SHAPE::SPHERE) {
		const auto SpA = static_cast<const Physics::ShapeSphere*>(RbA->Shape);
		const auto SpB = static_cast<const Physics::ShapeSphere*>(RbB->Shape);
		float T;
		if (Intersection::SphereShpere(SpA->Radius, SpB->Radius, RbA->Position, RbB->Position, RbA->LinearVelocity, RbB->LinearVelocity, DeltaSec, T)) {
			Ct.TimeOfImpact = T;

			//!< �Փˎ�����(���S)�ʒu
			const auto CPosA = RbA->Position + RbA->LinearVelocity * T;
			const auto CPosB = RbB->Position + RbB->LinearVelocity * T;

			//!< �@�� A -> B
			Ct.Normal = (CPosB - CPosA).Normalize();

			//!< �Փ˓_ (���a�̕��I�t�Z�b�g)
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
		//!< ���[�N
		auto WRbA = *RbA, WRbB = *RbB;

		auto DT = DeltaSec;
		auto TOI = 0.0f;
		auto ItCount = 0;
		while (DT > 0.0f) {
			//!< �Փ˓_�A�ŋߐړ_
			Math::Vec3 OnA, OnB;
			constexpr auto Bias = 0.001f;
			if (Intersection::GJK_EPA(&WRbA, &WRbB, Bias, OnA, OnB)) {
				Ct.TimeOfImpact = TOI;

				//!< �@�� A -> B
#if 0
				//!< �@��������������ꍇ������̂ŁA�g�債�Ă��琳�K������e�X�g
				constexpr auto Scale = 100.0f;
				Ct.Normal = ((OnB - OnA) * Scale).Normalize();
#else
				Ct.Normal = (OnB - OnA).Normalize();
#endif
				//!< �V���v���b�N�X���g�����Ă���̂ŁA���̕����L�����Z������
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

			//!< �ړ��������̏�ŉ�]���Ă���悤�ȏꍇ�A���[�v���甲���o���Ȃ���������̂Ń��[�v�ɏ���񐔂�݂���
			++ItCount;
			if (ItCount > 10) {
				break;
			}

			//!< ��]���l���������Α��x�����߂�
			//!< A -> B ����
			const auto Dir = (OnB - OnA).Normalize();
			//!< A �̑��Α��x�A�p���x
			const auto LVel = (WRbA.LinearVelocity - WRbB.LinearVelocity).Dot(Dir);
			const auto AVel = WRbA.Shape->GetFastestPointSpeed(WRbA.AngularVelocity, Dir) - WRbB.Shape->GetFastestPointSpeed(WRbB.AngularVelocity, Dir);
			const auto OrthoSpeed = LVel + AVel;
			if (OrthoSpeed <= 0.0f) {
				//!< �߂Â��Ă��Ȃ�
				break;
			}

			//!< �Փ˂���ł��낤���O�܂ł̎��Ԃ����߂�
			const auto SeparationDistance = (OnB - OnA).Length();
			const auto TimeToGo = SeparationDistance / OrthoSpeed;
			if (TimeToGo > DT) {
				//!< DT �ȓ��ɂ͑��݂��Ȃ�
				break;
			}

			//!< �Փ˂���ł��낤���O�܂Ŏ��Ԃ�i�߂�
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
		//!< ���a (�d�S -> �Փ˓_)
		const auto RA = Ct.PointA - Ct.RigidBodyA->GetWorldSpaceCenterOfMass();
		const auto RB = Ct.PointB - Ct.RigidBodyB->GetWorldSpaceCenterOfMass();
		{
			//!< �t�����e���\�� (���[���h�X�y�[�X)
			const auto WorldInvInertiaA = Ct.RigidBodyA->GetWorldSpaceInverseInertiaTensor();
			const auto WorldInvInertiaB = Ct.RigidBodyB->GetWorldSpaceInverseInertiaTensor();

			//!< (A ���_��) ���Α��x A -> B
			const auto VelA = Ct.RigidBodyA->LinearVelocity + Ct.RigidBodyA->AngularVelocity.Cross(RA);
			const auto VelB = Ct.RigidBodyB->LinearVelocity + Ct.RigidBodyB->AngularVelocity.Cross(RB);
			const auto VelAB = VelA - VelB;
			//!< ���x�̖@������
			const auto& Nrm = Ct.Normal;
			const auto NVelAB = Nrm * VelAB.Dot(Nrm);

			//!< �@������ �͐�J (�^���ʕω�)
			{
				//!< ���҂̒e���W�����|���������̊ȈՂȎ����Ƃ���
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

			//!< �ڐ����� �͐�J (���C��)
			{
				//!< ���x�̐ڐ�����
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

	//!< �߂荞�݂̒ǂ��o�� (TOI == 0.0f �̎��_�ŏՓ˂��Ă���ꍇ)
	if (0.0f == Ct.TimeOfImpact) {
		//!< ���ʂɂ��ǂ��o���������l��
		const auto DistAB = Ct.PointB - Ct.PointA;
		Ct.RigidBodyA->Position += DistAB * (Ct.RigidBodyA->InvMass / TotalInvMass);
		Ct.RigidBodyB->Position -= DistAB * (Ct.RigidBodyB->InvMass / TotalInvMass);
	}
}
