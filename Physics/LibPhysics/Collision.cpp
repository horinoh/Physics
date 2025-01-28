#include "Collision.h"
#include "RigidBody.h"
#include "Shape.h"
#include "GJK.h"

float Collision::Distance::CapsulePointSq(const Math::Vec3& CapA, const Math::Vec3& CapB, const float CapR,
	const Math::Vec3& Pt) 
{
	if (Collision::Intersection::CapsulePoint(CapA, CapB, CapR, Pt)) { return 0.0f; }
	return (Pt - Collision::Closest::CapsulePoint(CapA, CapB, CapR, Pt)).LengthSq();
}

float Collision::Distance::CylinderPointSq(const Math::Vec3& CyA, const Math::Vec3& CyB, const float CyR,
	const Math::Vec3& Pt) 
{
	if (Collision::Intersection::CylinderPoint(CyA, CyB, CyR, Pt)) { return 0.0f; }
	return (Pt - Collision::Closest::CylinderPoint(CyA, CyB, CyR, Pt)).LengthSq();
}

float Collision::Distance::SegmentSegmentSq(const Math::Vec3& SegA, const Math::Vec3& SegB,
	const Math::Vec3& SegC, const Math::Vec3& SegD,
	float& T0, float& T1)
{
	Collision::Closest::SegmentSegment(SegA, SegB, SegC, SegD, T0, T1);
	return ((SegA + (SegB - SegA) * T0) - (SegC + (SegD - SegC) * T1)).LengthSq();
}

bool Collision::Intersection::AABBAABB(const AABB& AbA, const AABB& AbB,
	const Math::Vec3& VelA, const Math::Vec3& VelB,
	float& T)
{
	if (AABBAABB(AbA, AbB)) {
		T = 0.0f;
		return true;
	}
	const auto V = VelB - VelA;
	constexpr auto Eps2 = 0.01f * 0.01f;
	if (V.LengthSq() < Eps2) {
		return false;
	}
	T = 0.0f;
	auto T1 = 1.0f;
	for (auto i = 0; i < 3; ++i) {
		if (std::abs(V[i]) > std::numeric_limits<float>::epsilon()) {
			if (V[i] < 0.0f) {
				if (AbB.Max[i] < AbA.Min[i]) { return false; }
				if (AbA.Max[i] < AbB.Min[i]) T = (std::max)((AbA.Max[i] - AbB.Min[i]) / V[i], T);
				if (AbB.Max[i] > AbA.Min[i]) T1 = (std::min)((AbA.Min[i] - AbB.Max[i]) / V[i], T1);
			}
			else {
				if (AbB.Min[i] < AbA.Max[i]) { return false; }
				if (AbB.Max[i] < AbA.Min[i]) T = (std::max)((AbA.Min[i] - AbB.Max[i]) / V[i], T);
				if (AbA.Max[i] > AbB.Min[i]) T1 = (std::min)((AbA.Max[i] - AbB.Min[i]) / V[i], T1);
			}
		}
	}
	return T < T1;
}

bool Collision::Intersection::AABBRay(const AABB& Ab,
	const Math::Vec3& RayPos, const Math::Vec3& RayDir,
	float& T) 
{
	T = 0.0f;
	auto TMax = std::numeric_limits<float>::max();

	for (auto i = 0; i < 3; ++i) {
		if (std::abs(RayDir[i]) < std::numeric_limits<float>::epsilon()) {
			if (RayPos[i] < Ab.Min[i] || RayPos[i] > Ab.Max[i]) { return false; }
		}
		else {
			auto t1 = (Ab.Min[i] - RayPos[i]) / RayDir[i];
			auto t2 = (Ab.Max[i] - RayPos[i]) / RayDir[i];
			if (t1 > t2) {
				std::swap(t1, t2);
			}
			T = std::max(T, t1);
			TMax = std::min(TMax, t2);
			if (T > TMax) { return false; }
		}
	}
	return true;
}
bool Collision::Intersection::CapsulePoint(const Math::Vec3& CapA, const Math::Vec3& CapB, const float CapR,
	const Math::Vec3& Pt) 
{
	return Collision::Distance::PointSegmentSq(CapA, CapB, Pt) <= std::pow(CapR, 2.0f);
}

bool Collision::Intersection::CylinderPoint(const Math::Vec3& CyA, const Math::Vec3& CyB, const float CyR,
	const Math::Vec3& Pt) 
{
	if (!CapsulePoint(CyA, CyB, CyR, Pt)) { return false; }
	const auto AB = CyB - CyA;
	if ((Pt - CyB).Dot(-AB) < 0.0f) { return false; }
	if ((Pt - CyA).Dot(AB) < 0.0f) { return false; }
	return true;
}

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
	float& T)
{
	// A �̑��Α��x
	const auto Ray = VelA - VelB;
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

	T = (std::max)(T0, 0.0f);

	return true;
}

Math::Vec3 Collision::Closest::CapsulePoint(const Math::Vec3& CapA, const Math::Vec3& CapB, const float CapR,
	const Math::Vec3& Pt) 
{
	const auto C = PointSegment(Pt, CapA, CapB);
	return C + (Pt - C).Normalize() * CapR;
}

Math::Vec3 Collision::Closest::CylinderPoint(const Math::Vec3& CyA, const Math::Vec3& CyB, const float CyR,
	const Math::Vec3& Pt) 
{
	const auto AB = CyB - CyA;
	if ((Pt - CyB).Dot(-AB) < 0.0f) {
		const auto N = AB.Normalize();
		const auto C = PlanePoint(N, N.Dot(CyB), Pt);
		const auto BC(C - CyB);
		const auto LenSq = BC.LengthSq();
		if (LenSq < std::pow(CyR, 2.0f)) {
			return C;
		}
		return CyB + BC * CyR / std::sqrt(LenSq);
	}
	else if ((Pt - CyA).Dot(AB) < 0.0f) {
		const auto N = -AB.Normalize();
		const auto C = PlanePoint(N, N.Dot(CyA), Pt);
		const auto AC = C - CyA;
		const auto LenSq = AC.LengthSq();
		if (LenSq < std::pow(CyR, 2.0f)) {
			return C;
		}
		return CyA + AC * CyR / std::sqrt(LenSq);
	}
	else {
		const auto C = PointSegment(Pt, CyA, CyB);
		return C + (Pt - C).Normalize() * CyR;
	}
}

Math::Vec3 Collision::Closest::PlanePoint(const Math::Vec3 & PlN, const float PlD,
	const Math::Vec3& Pt) 
{
	return Pt - Collision::Distance::PlanePoint(PlN, PlD, Pt) * PlN;
}

Math::Vec3 Collision::Closest::SegmentSegment(const Math::Vec3& SegA, const Math::Vec3& SegB,
	const Math::Vec3& SegC, const Math::Vec3& SegD,
	float& T0, float& T1)
{
	const auto AB = SegB - SegA;
	const auto CD = SegD - SegC;
	const auto A = AB.LengthSq();
	const auto E = CD.LengthSq();

	//< �Q�̐����Ƃ����_�ɏk�ނ��Ă���
	if (A <= std::numeric_limits<float>::epsilon() && E <= std::numeric_limits<float>::epsilon()) { T0 = 0.0f; T1 = 0.0f; return Math::Vec3::Zero(); }

	const auto CA = SegA - SegC;
	const auto F = CD.Dot(CA);

	float S, T;
	//< ����A���_�ɏk�ނ��Ă���
	if (A <= std::numeric_limits<float>::epsilon()) {
		S = 0.0f; T = std::clamp(F / E, 0.0f, 1.0f);
	}
	else {
		const auto C = AB.Dot(CA);
		//< ����B���_�ɏk�ނ��Ă���
		if (E <= std::numeric_limits<float>::epsilon()) { 
			S = std::clamp(-C / A, 0.0f, 1.0f); T = 0.0f; 
		}
		else {
			const auto b = AB.Dot(CD);
			const auto Denom(A * E - std::pow(b, 2.0f));
			//< ���s�ȏꍇ
			if (std::abs(Denom) <= std::numeric_limits<float>::epsilon()) { S = 0.0f; }
			else { S = std::clamp((b * F - C * E) / Denom, 0.0f, 1.0f); }

			const auto Tnom(b * S + F);
			if (Tnom < 0.0f) { T = 0.0f; S = std::clamp(-C / A, 0.0f, 1.0f); }
			else if (Tnom > E) { T = 1.0f; S = std::clamp((b - C) / A, 0.0f, 1.0f); }
			else { T = Tnom / E; }
		}
	}

	T0 = S;
	T1 = T;
}

bool Collision::Intersection::RigidBodyRigidBody(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const float DeltaSec, Contact& Ct)
{
	if (RbA->Shape->GetShapeType() == Physics::Shape::SHAPE::SPHERE && RbB->Shape->GetShapeType() == Physics::Shape::SHAPE::SPHERE) {
		const auto SpA = static_cast<const Physics::ShapeSphere*>(RbA->Shape);
		const auto SpB = static_cast<const Physics::ShapeSphere*>(RbB->Shape);
		float T;
		//!< TOI �����ڋ��߂���
		//!< ���x�̓f���^���Ԃ̂��̂�n������
		if (Intersection::SphereShpere(SpA->Radius, SpB->Radius, RbA->Position, RbB->Position, RbA->LinearVelocity * DeltaSec, RbB->LinearVelocity * DeltaSec, T)) {
			Ct.TimeOfImpact = T * DeltaSec;

			//!< �Փˎ�����(���S)�ʒu
			const auto CPosA = RbA->Position + RbA->LinearVelocity * T;
			const auto CPosB = RbB->Position + RbB->LinearVelocity * T;

			//!< �@�� A -> B
			Ct.WNormal = (CPosB - CPosA).Normalize();

			//!< �Փ˓_ (���a�̕��I�t�Z�b�g)
#ifdef WORLD_CONTACT_POINT
			Ct.WPointA = CPosA + Ct.Normal * SpA->Radius;
			Ct.WPointB = CPosB + Ct.Normal * SpB->Radius;
			Ct.PointA = RbA->ToLocalPos(Ct.WPointA);
			Ct.PointB = RbB->ToLocalPos(Ct.WPointB);
#else
			Ct.LPointA = RbA->ToLocalPos(CPosA + Ct.WNormal * SpA->Radius);
			Ct.LPointB = RbB->ToLocalPos(CPosB - Ct.WNormal * SpB->Radius); 
#endif
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

		//!< TOI �����ڋ��܂�Ȃ��̂ŁA�V�~�����[�V������i�߂邱�Ƃŋ��߂� (Conservative Advance)
		auto DT = DeltaSec;
		auto TOI = 0.0f;
		//!< ���
		auto ItCount = 0;
		while (DT > 0.0f) {
			//!< �Փ˓_�A�ŋߐړ_
			Math::Vec3 OnA, OnB;
			constexpr auto Bias = 0.001f;
			//if (Intersection::GJK_EPA(&WRbA, &WRbB, Bias, OnA, OnB)) {
			if (Intersection::GJK_EPA(WRbA.Shape, WRbA.Position, WRbA.Rotation, WRbB.Shape, WRbB.Position, WRbB.Rotation, Bias, OnA, OnB)) {
				Ct.TimeOfImpact = TOI;

				//!< �@�� A -> B #TODO
				Ct.WNormal = -(OnB - OnA).Normalize();

				//!< �V���v���b�N�X���g�����Ă���̂ŁA���̕����L�����Z������
				OnA -= Ct.WNormal * Bias;
				OnB += Ct.WNormal * Bias;

				//!< �Փ˓_
#ifdef WORLD_CONTACT_POINT
				Ct.WPointA = OnA;
				Ct.WPointB = OnB;
#endif
				Ct.LPointA = RbA->ToLocalPos(OnA);
				Ct.LPointB = RbB->ToLocalPos(OnB);

				//!< �Փˍ��̂��o���Ă���
				Ct.RigidBodyA = const_cast<Physics::RigidBody*>(RbA);
				Ct.RigidBodyB = const_cast<Physics::RigidBody*>(RbB);

				return true;
			}
			else {
				//!< �Փ˂������ꍇ�͍ŋߐړ_�����߂�
				Closest::GJK(&WRbA, &WRbB, OnA, OnB);
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
	const auto WPointA = Ct.RigidBodyA->ToWorldPos(Ct.LPointA);
	const auto WPointB = Ct.RigidBodyB->ToWorldPos(Ct.LPointB);
	const auto TotalInvMass = Ct.RigidBodyA->InvMass + Ct.RigidBodyB->InvMass;
	{
		//!< ���a (�d�S -> �Փ˓_)
		const auto RA = WPointA - Ct.RigidBodyA->GetWorldSpaceCenterOfMass();
		const auto RB = WPointB - Ct.RigidBodyB->GetWorldSpaceCenterOfMass();
		{
			//!< �t�����e���\�� (���[���h�X�y�[�X)
			const auto WorldInvInertiaA = Ct.RigidBodyA->GetWorldSpaceInverseInertiaTensor();
			const auto WorldInvInertiaB = Ct.RigidBodyB->GetWorldSpaceInverseInertiaTensor();

			//!< (A ���_��) ���Α��x A -> B
			const auto VelA = Ct.RigidBodyA->LinearVelocity + Ct.RigidBodyA->AngularVelocity.Cross(RA);
			const auto VelB = Ct.RigidBodyB->LinearVelocity + Ct.RigidBodyB->AngularVelocity.Cross(RB);
			const auto VelAB = VelA - VelB;
	
			auto Apply = [&](const auto& Axis, const auto& Vel, const float Coef) {
				const auto AngJA = (WorldInvInertiaA * RA.Cross(Axis)).Cross(RA);
				const auto AngJB = (WorldInvInertiaB * RB.Cross(Axis)).Cross(RB);
				const auto AngFactor = (AngJA + AngJB).Dot(Axis);

				const auto J = Vel * Coef / (TotalInvMass + AngFactor);

				Ct.RigidBodyA->ApplyImpulse(WPointA, -J);
				Ct.RigidBodyB->ApplyImpulse(WPointB, J);
			};

			//!< �@������ �͐�J (�^���ʕω�)
			//!< ���x�̖@������
			const auto NVelAB = Ct.WNormal * VelAB.Dot(Ct.WNormal);
			//!< ���҂̒e���W�����|���������̊ȈՂȎ����Ƃ���
			const auto TotalElasticity = 1.0f + Ct.RigidBodyA->Elasticity * Ct.RigidBodyB->Elasticity;
			Apply(Ct.WNormal, NVelAB, TotalElasticity);

			//!< �ڐ����� �͐�J (���C��)
			//!< ���x�̐ڐ�����
			const auto TVelAB = VelAB - NVelAB;
			const auto Tan = TVelAB.Normalize();
			const auto TotalFriction = Ct.RigidBodyA->Friction * Ct.RigidBodyB->Friction;
			Apply(Tan, TVelAB, TotalFriction);
		}
	}

	//!< �߂荞�݂̒ǂ��o�� (TOI == 0.0f �̎��_�ŏՓ˂��Ă���ꍇ)
	if (0.0f == Ct.TimeOfImpact) {
		//!< ���ʂɂ��ǂ��o���������l��
		const auto DistAB = WPointB - WPointA;
		Ct.RigidBodyA->Position += DistAB * (Ct.RigidBodyA->InvMass / TotalInvMass);
		Ct.RigidBodyB->Position -= DistAB * (Ct.RigidBodyB->InvMass / TotalInvMass);
	}
}
