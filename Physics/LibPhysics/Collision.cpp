#include "Collision.h"
#include "RigidBody.h"
#include "Shape.h"
#include "GJK.h"

void Collision::Contact::CalcLocal() 
{
	LPointA = RigidBodyA->ToLocalPos(WPointA);
	LPointB = RigidBodyB->ToLocalPos(WPointB);
}

float Collision::Distance::CapsulePointSq(const LinAlg::Vec3& CapA, const LinAlg::Vec3& CapB, const float CapR,
	const LinAlg::Vec3& Pt) 
{
	if (Collision::Intersection::CapsulePoint(CapA, CapB, CapR, Pt)) { return 0.0f; }
	return (Pt - Collision::Closest::CapsulePoint(CapA, CapB, CapR, Pt)).LengthSq();
}

float Collision::Distance::CylinderPointSq(const LinAlg::Vec3& CyA, const LinAlg::Vec3& CyB, const float CyR,
	const LinAlg::Vec3& Pt) 
{
	if (Collision::Intersection::CylinderPoint(CyA, CyB, CyR, Pt)) { return 0.0f; }
	return (Pt - Collision::Closest::CylinderPoint(CyA, CyB, CyR, Pt)).LengthSq();
}

float Collision::Distance::SegmentSegmentSq(const LinAlg::Vec3& SegA, const LinAlg::Vec3& SegB,
	const LinAlg::Vec3& SegC, const LinAlg::Vec3& SegD,
	float& T0, float& T1)
{
	LinAlg::Vec3 OnAB, OnCD;
	Collision::Closest::SegmentSegment(SegA, SegB, SegC, SegD, T0, T1, &OnAB, &OnCD);
	return (OnAB - OnCD).LengthSq();
}
float Collision::Distance::SegmentSegmentSq(const LinAlg::Vec3& SegA, const LinAlg::Vec3& SegB,
	const LinAlg::Vec3& SegC, const LinAlg::Vec3& SegD) {
	float T0, T1;
	return SegmentSegmentSq(SegA, SegB,
		SegC, SegD,
		T0, T1);
}

bool Collision::Intersection::AABBAABB(const AABB& AbA, const AABB& AbB,
	const LinAlg::Vec3& VelA, const LinAlg::Vec3& VelB,
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
		if (std::abs(V[i]) > (std::numeric_limits<float>::epsilon)()) {
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
	const LinAlg::Vec3& RayPos, const LinAlg::Vec3& RayDir,
	float& T) 
{
	T = 0.0f;
	auto TMax = std::numeric_limits<float>::max();

	for (auto i = 0; i < 3; ++i) {
		if (std::abs(RayDir[i]) < (std::numeric_limits<float>::epsilon)()) {
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
bool Collision::Intersection::CapsulePoint(const LinAlg::Vec3& CapA, const LinAlg::Vec3& CapB, const float CapR,
	const LinAlg::Vec3& Pt) 
{
	return Collision::Distance::PointSegmentSq(CapA, CapB, Pt) <= std::pow(CapR, 2.0f);
}

bool Collision::Intersection::CylinderPoint(const LinAlg::Vec3& CyA, const LinAlg::Vec3& CyB, const float CyR,
	const LinAlg::Vec3& Pt) 
{
	if (!CapsulePoint(CyA, CyB, CyR, Pt)) { return false; }
	const auto AB = CyB - CyA;
	if ((Pt - CyB).Dot(-AB) < 0.0f) { return false; }
	if ((Pt - CyA).Dot(AB) < 0.0f) { return false; }
	return true;
}

std::optional<std::pair<float, float>> Collision::Intersection::RaySphere(const LinAlg::Vec3& RayPos, const LinAlg::Vec3& RayDir, const LinAlg::Vec3& SpPos, const float SpRad)
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
		const auto D4Sqrt = std::sqrt(D4);
		return std::make_pair((B2 - D4Sqrt) / A, (B2 + D4Sqrt) / A);
	}
	return std::nullopt;
}

std::optional<float> Collision::Intersection::SphereShpere(const float RadA, const float RadB,
	const LinAlg::Vec3& PosA, const LinAlg::Vec3& PosB,
	const LinAlg::Vec3& VelA, const LinAlg::Vec3& VelB)
{
	// A �̑��Α��x
	const auto Ray = VelA - VelB;
	const auto TotalRadius = RadA + RadB;

	constexpr auto Eps2 = 0.001f * 0.001f;
	if (Ray.LengthSq() < Eps2) {
		//!< ���C���\���Z���ꍇ�͊��ɏՓ˂��Ă��邩�ǂ����̃`�F�b�N
		const auto PosAB = PosB - PosA;
		const auto R = TotalRadius + 0.001f;
		if (PosAB.LengthSq() > R * R) {
			return std::nullopt;
		}
		return 0.0f;
	}
	//!< ���C vs �� �ɋA��
	else {
		const auto Ret = Intersection::RaySphere(PosA, Ray, PosB, TotalRadius);
		if (Ret == std::nullopt || Ret.value().first > 1.0f || Ret.value().second < 0.0f) {
			return std::nullopt;
		}
		return (std::max)(Ret.value().first, 0.0f);
	}
}

LinAlg::Vec3 Collision::Closest::CapsulePoint(const LinAlg::Vec3& CapA, const LinAlg::Vec3& CapB, const float CapR,
	const LinAlg::Vec3& Pt) 
{
	const auto C = PointSegment(Pt, CapA, CapB);
	return C + (Pt - C).Normalize() * CapR;
}

LinAlg::Vec3 Collision::Closest::CylinderPoint(const LinAlg::Vec3& CyA, const LinAlg::Vec3& CyB, const float CyR,
	const LinAlg::Vec3& Pt) 
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

LinAlg::Vec3 Collision::Closest::PlanePoint(const LinAlg::Vec3 & PlN, const float PlD,
	const LinAlg::Vec3& Pt) 
{
	return Pt - Collision::Distance::PlanePoint(PlN, PlD, Pt) * PlN;
}

void Collision::Closest::SegmentSegment(const LinAlg::Vec3& SegA, const LinAlg::Vec3& SegB,
	const LinAlg::Vec3& SegC, const LinAlg::Vec3& SegD,
	float& T0, float& T1,
	LinAlg::Vec3* OnAB, LinAlg::Vec3* OnCD)
{
	const auto AB = SegB - SegA;
	const auto CD = SegD - SegC;
	const auto A = AB.LengthSq();
	const auto E = CD.LengthSq();

	//< �Q�̐����Ƃ����_�ɏk�ނ��Ă���
	if (A <= std::numeric_limits<float>::epsilon() && E <= (std::numeric_limits<float>::epsilon)()) { 
		T0 = 0.0f; 
		T1 = 0.0f;
		if (nullptr != OnAB) {
			*OnAB = SegA;
		}
		if (nullptr != OnCD) {
			*OnCD = SegC;
		}
		return;
	}

	const auto CA = SegA - SegC;
	const auto F = CD.Dot(CA);

	float S, T;
	//< ����A���_�ɏk�ނ��Ă���
	if (A <= (std::numeric_limits<float>::epsilon)()) {
		S = 0.0f; T = std::clamp(F / E, 0.0f, 1.0f);
	}
	else {
		const auto C = AB.Dot(CA);
		//< ����B���_�ɏk�ނ��Ă���
		if (E <= (std::numeric_limits<float>::epsilon)()) { 
			S = std::clamp(-C / A, 0.0f, 1.0f); T = 0.0f; 
		}
		else {
			const auto b = AB.Dot(CD);
			const auto Denom(A * E - std::pow(b, 2.0f));
			//< ���s�ȏꍇ
			if (std::abs(Denom) <= (std::numeric_limits<float>::epsilon)()) { S = 0.0f; }
			else { S = std::clamp((b * F - C * E) / Denom, 0.0f, 1.0f); }

			const auto Tnom(b * S + F);
			if (Tnom < 0.0f) { T = 0.0f; S = std::clamp(-C / A, 0.0f, 1.0f); }
			else if (Tnom > E) { T = 1.0f; S = std::clamp((b - C) / A, 0.0f, 1.0f); }
			else { T = Tnom / E; }
		}
	}

	T0 = S;
	T1 = T;
	if (nullptr != OnAB) {
		*OnAB = SegA + AB * T0;
	}
	if (nullptr != OnCD) {
		*OnCD = SegC + CD * T1;
	}
	return;
}
void Collision::Closest::SegmentSegment(const LinAlg::Vec3& SegA, const LinAlg::Vec3& SegB,
	const LinAlg::Vec3& SegC, const LinAlg::Vec3& SegD,
	LinAlg::Vec3& OnAB, LinAlg::Vec3& OnCD) 
{
	float T0, T1;
	SegmentSegment(SegA, SegB,
		SegC, SegD,
		T0, T1,
		&OnAB, &OnCD);
}

[[nodiscard]] bool Collision::Intersection::SphereSphere(const Physics::RigidBody* RbA,
	const Physics::RigidBody* RbB,
	const float DeltaSec, ContactBase& Ct) 
{
	const auto SpA = static_cast<const Physics::ShapeSphere*>(RbA->Shape);
	const auto SpB = static_cast<const Physics::ShapeSphere*>(RbB->Shape);
	//!< ���x�̓f���^���Ԃ̂��̂�n������
	const auto T = Intersection::SphereShpere(SpA->Radius, SpB->Radius, RbA->Position, RbB->Position, RbA->LinearVelocity * DeltaSec, RbB->LinearVelocity * DeltaSec);
	if (T != std::nullopt) {
		//!< �Փˍ��̂��o���Ă���
		Ct.RigidBodyA = const_cast<Physics::RigidBody*>(RbA);
		Ct.RigidBodyB = const_cast<Physics::RigidBody*>(RbB); 

		Ct.TimeOfImpact = T.value() * DeltaSec;

		//!< �Փˎ�����(���S)�ʒu
		const auto CPosA = RbA->Position + RbA->LinearVelocity * Ct.TimeOfImpact;
		const auto CPosB = RbB->Position + RbB->LinearVelocity * Ct.TimeOfImpact;

		//!< �@�� A -> B
		Ct.WNormal = (CPosB - CPosA).Normalize();

		//!< �Փ˓_ (���a�̕��I�t�Z�b�g)
		Ct.WPointA = CPosA + Ct.WNormal * SpA->Radius;
		Ct.WPointB = CPosB - Ct.WNormal * SpB->Radius;

		//!< �Փ˓_�̃��[�J�����W (���ł͕s�v�Ȃ̂� ContactBase �ŏ\��)
		//Ct.CalcLocal();		

		return true;
	}
	return false;
}

bool Collision::Intersection::RigidBodyRigidBody(const Physics::RigidBody* RbA,
	const Physics::RigidBody* RbB, 
	const float DeltaSec, Contact& Ct)
{
	//!< ���[�N
	auto WRbA = *RbA, WRbB = *RbB;

	//!< TOI �����ڋ��܂�Ȃ��̂ŁA�V�~�����[�V������i�߂邱�Ƃŋ��߂� (Conservative Advance)
	auto DT = DeltaSec;
	auto TOI = 0.0f;
	//!< �C�e���[�V������ (���[�v���甲���o���Ȃ��΍�)
	auto ItCount = 0;
	constexpr auto Bias = 0.001f;
	while (DT > 0.0f) {
		//!< �Փ˓_�A�ŋߐړ_
		LinAlg::Vec3 OnA, OnB;
		if (Intersection::GJK_EPA(&WRbA, &WRbB,
				Bias,
				OnA, OnB)) {
			//!< �Փˍ��̂��o���Ă���
			Ct.RigidBodyA = const_cast<Physics::RigidBody*>(RbA);
			Ct.RigidBodyB = const_cast<Physics::RigidBody*>(RbB);

			Ct.TimeOfImpact = TOI;
			//!< �@�� A -> B
			Ct.WNormal = (OnB - OnA).Normalize();
			//!< �V���v���b�N�X���g�����Ă���̂ŁA���̕����L�����Z������
			OnA -= Ct.WNormal * Bias;
			OnB += Ct.WNormal * Bias;

			//!< �Փ˓_
			Ct.WPointA = OnA;
			Ct.WPointB = OnB;
			Ct.CalcLocal();

			return true;
		}
		//!< �ړ��������̏�ŉ�]���Ă���悤�ȏꍇ�A���[�v���甲���o���Ȃ���������̂Ń��[�v�ɏ���񐔂�݂���
		++ItCount;
		if (ItCount > 10) {
			break;
		}

		const auto AB = OnB - OnA;
		const auto SepDist = AB.Length();

		//!< ��]���l���������Α��x�����߂�
		//!< A -> B ����
		const auto Dir = AB / SepDist;
		//!< A �̑��Α��x�A�p���x
		const auto LVel = (WRbA.LinearVelocity - WRbB.LinearVelocity).Dot(Dir);
		const auto AVel = WRbA.Shape->GetFastestPointSpeed(WRbA.AngularVelocity, Dir) - WRbB.Shape->GetFastestPointSpeed(WRbB.AngularVelocity, Dir);
		const auto OrthoSpeed = LVel + AVel;
		if (OrthoSpeed <= 0.0f) {
			//!< �߂Â��Ă��Ȃ�
			break;
		}

		//!< �Փ˂���ł��낤���O�܂ł̎��Ԃ����߂�
		const auto TimeToGo = SepDist / OrthoSpeed;
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
	return false;
}
