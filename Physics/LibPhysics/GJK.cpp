#include "GJK.h"
#include "RigidBody.h"
#include "Shape.h"
#include "Collision.h"
#include "Convex.h"

Math::Vec2 Collision::SignedVolume(const Math::Vec3& A, const Math::Vec3& B)
{
	const auto AB = B - A;
	//!< ���_�� AB ���Ɏˉe�� P �Ƃ���
	const auto P = A + AB * AB.Dot(-A) / AB.LengthSq(); //!< ���������Ȃ���̂Ő��K�����Ȃ������ǂ�

	//!< X, Y, Z ���̓��A��Βl���ő�̂��̂�������
	auto Index = 0;
	auto MaxVal = 0.0f;
	for (auto i = 0; i < 3; ++i) {
		if (std::abs(MaxVal) < std::abs(AB[i])) {
			MaxVal = AB[i];
			Index = i;
		}
	}

	//!< �uP �Ɛ����v��I���������֎ˉe
	const auto PrjA = A[Index];
	const auto PrjB = B[Index];
	const auto PrjP = P[Index];

	//!< P �� [A, B] �̓����ɂ���ꍇ
	if ((PrjP > PrjA && PrjP < PrjB) || (PrjP > PrjB && PrjP < PrjA)) {
		return Math::Vec2(PrjB - PrjP, PrjP - PrjA) / MaxVal;
	}
	//!< P �� A ���̊O��
	if ((PrjA <= PrjB && PrjP <= PrjA) || (PrjA >= PrjB && PrjP >= PrjA)) {
		return Math::Vec2::AxisX();
	}
	//!< P �� B ���̊O��
	else {
		return Math::Vec2::AxisY();
	}
}

Math::Vec3 Collision::SignedVolume(const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C)
{
	const auto N = (B - A).Cross(C - A);
	if (N.NearlyEqual(Math::Vec3::Zero())) { return Math::Vec3::AxisX(); }
	const auto P = N * A.Dot(N) / N.LengthSq(); //!< ���������Ȃ���̂Ő��K�����Ȃ������ǂ�

	//!< XY, YZ, ZX ���ʂ̓��A�ˉe�ʐς��ő�̂��̂�������
	auto Index = 0;
	auto MaxVal = 0.0f;
	for (auto i = 0; i < 3; ++i) {
		const auto j = (i + 1) % 3;
		const auto k = (i + 2) % 3;

		const auto a = Math::Vec2(A[j], A[k]);
		const auto b = Math::Vec2(B[j], B[k]);
		const auto c = Math::Vec2(C[j], C[k]);
		const auto AB = b - a;
		const auto AC = c - a;
		const auto Area = Math::Mat2(AB, AC).Determinant();

		if (std::abs(Area) > std::abs(MaxVal)) {
			MaxVal = Area;
			Index = i;
		}
	}

	//!< �uP �ƎO�p�`�v��I���������ʂɎˉe (X ���I�����ꂽ�ꍇ Index1, Index2 �͂��ꂼ�� Y, Z �Ƃ�������ɂȂ�)
	const auto X = (Index + 1) % 3;
	const auto Y = (Index + 2) % 3;
	const std::array PrjABC = { Math::Vec2(A[X], A[Y]), Math::Vec2(B[X], B[Y]), Math::Vec2(C[X], C[Y]) };
	const auto PrjP = Math::Vec2(P[X], P[Y]);

	//!< �ˉe�_�ƕӂ���Ȃ�T�u�O�p�`�̖ʐ�
	Math::Vec3 Areas;
	for (auto i = 0; i < 3; ++i) {
		const auto j = (i + 1) % 3;
		const auto k = (i + 2) % 3;

		Areas[i] = Math::Mat2(PrjABC[j] - PrjP, PrjABC[k] - PrjP).Determinant();
	}
	//!< P �� [A, B, C] �̓����ɂ���ꍇ (�T�u�O�p�`�̖ʐς̕������番����)
	if (Sign(MaxVal) == Sign(Areas.X()) && Sign(MaxVal) == Sign(Areas.Y()) && Sign(MaxVal) == Sign(Areas.Z())) {
		return Areas / MaxVal;
	}

	//!< 3 �ӂɎˉe���Ĉ�ԋ߂����̂������� (1-SignedVolume �ɋA��)
	const std::array EdgePts = { A, B, C };
	Math::Vec3 Lambda;
	auto MinVal = (std::numeric_limits<float>::max)();
	for (auto i = 0; i < 3; ++i) {
		const auto j = (i + 1) % 3;
		const auto k = (i + 2) % 3;

		const auto LambdaEdge = SignedVolume(EdgePts[j], EdgePts[k]);
		const auto LenSq = (EdgePts[j] * LambdaEdge[0] + EdgePts[k] * LambdaEdge[1]).LengthSq();
		if (LenSq < MinVal) {
			Lambda[i] = 0.0f;
			Lambda[j] = LambdaEdge[0];
			Lambda[k] = LambdaEdge[1];
			MinVal = LenSq;
		}
	}
	return Lambda;
}

Math::Vec4 Collision::SignedVolume(const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C, const Math::Vec3& D)
{
	//const auto Cofactor = Vec4(-Mat3(B, C, D).Determinant(), Mat3(A, C, D).Determinant(), -Mat3(A, B, D).Determinant(), Mat3(A, B, C).Determinant());
	const auto M = Math::Mat4(Math::Vec4(A.X(), B.X(), C.X(), D.X()), Math::Vec4(A.Y(), B.Y(), C.Y(), D.Y()), Math::Vec4(A.Z(), B.Z(), C.Z(), D.Z()), Math::Vec4::One());
	const auto Cofactor = Math::Vec4(M.Cofactor(3, 0), M.Cofactor(3, 1), M.Cofactor(3, 2), M.Cofactor(3, 3));
	const auto Det = Cofactor.X() + Cofactor.Y() + Cofactor.Z() + Cofactor.W();

	//!< �l�ʑ̓����ɂ���΁A�d�S���W��Ԃ�
	if (Sign(Det) == Sign(Cofactor.X()) && Sign(Det) == Sign(Cofactor.Y()) && Sign(Det) == Sign(Cofactor.Z()) && Sign(Det) == Sign(Cofactor.W())) {
		return Cofactor / Det;
	}

	//!< 3 �ʂɎˉe���Ĉ�ԋ߂����̂������� (2-SignedVolume �ɋA��)
	const std::array FacePts = { A, B, C, D };
	Math::Vec4 Lambda;
	auto MinVal = (std::numeric_limits<float>::max)();
	for (auto i = 0; i < 4; ++i) {
		const auto j = (i + 1) % 4;
		const auto k = (i + 2) % 4;

		const auto LambdaFace = SignedVolume(FacePts[i], FacePts[j], FacePts[k]);
		const auto LenSq = (FacePts[i] * LambdaFace[0] + FacePts[j] * LambdaFace[1] + FacePts[k] * LambdaFace[2]).LengthSq();
		if (LenSq < MinVal) {
			Lambda.ToZero();
			Lambda[i] = LambdaFace[0];
			Lambda[j] = LambdaFace[1];
			Lambda[k] = LambdaFace[2];
			MinVal = LenSq;
		}
	}
	return Lambda;
}

Math::Vec3 Collision::Barycentric(const Math::Vec3& Pt, const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C)
{
	const auto PA = A - Pt;
	const auto PB = B - Pt;
	const auto PC = C - Pt;

	const auto N = Math::Vec3::Normal(PA, PB, PC);
	const auto P = N * A.Dot(N) / N.LengthSq();

	//!< XY, YZ, ZX ���ʂ̓��A�ˉe�ʐς��ő�̂��̂�������
	auto Index = 0;
	auto MaxVal = 0.0f;
	for (auto i = 0; i < 3; ++i) {
		const auto j = (i + 1) % 3;
		const auto k = (i + 2) % 3;

		const auto A = Math::Vec2(PA[j], PA[k]);
		const auto B = Math::Vec2(PB[j], PB[k]);
		const auto C = Math::Vec2(PC[j], PC[k]);
		const auto AB = B - A;
		const auto AC = C - A;
		const auto Area = Math::Mat2(AB, AC).Determinant();

		if (std::abs(Area) > std::abs(MaxVal)) {
			MaxVal = Area;
			Index = i;
		}
	}

	//!< �uP �ƎO�p�`�v��I���������ʂɎˉe (X ���I�����ꂽ�ꍇ Index1, Index2 �͂��ꂼ�� Y, Z �Ƃ�������ɂȂ�)
	const auto X = (Index + 1) % 3;
	const auto Y = (Index + 2) % 3;
	const std::array PrjABC = { Math::Vec2(PA[X], PA[Y]), Math::Vec2(PB[X], PB[Y]), Math::Vec2(PC[X], PC[Y]) };
	const auto PrjP = Math::Vec2(P[X], P[Y]);

	//!< �ˉe�_�ƕӂ���Ȃ�T�u�O�p�`�̖ʐ�
	Math::Vec3 Areas;
	for (auto i = 0; i < 3; i++) {
		const auto j = (i + 1) % 3;
		const auto k = (i + 2) % 3;

		Areas[i] = Math::Mat2(PrjABC[j] - PrjP, PrjABC[k] - PrjP).Determinant();
	}

	//!< P �� [A, B, C] �̓����ɂ���ꍇ (�T�u�O�p�`�̖ʐς̕������番����)
	if (Sign(MaxVal) == Sign(Areas.X()) && Sign(MaxVal) == Sign(Areas.Y()) && Sign(MaxVal) == Sign(Areas.Z())) {
		return Areas / MaxVal;
	}

	return Math::Vec3::AxisX();
}
Collision::SupportPoint::Points Collision::SupportPoint::GetPoints(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& UDir, const float Bias)
{
	return Collision::SupportPoint::Points(RbA->Shape->GetSupportPoint(RbA->Position, RbA->Rotation, UDir, Bias), RbB->Shape->GetSupportPoint(RbB->Position, RbB->Rotation, -UDir, Bias));
}
void Collision::SupportPoint::ToTetrahedron(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, std::vector<Collision::SupportPoint::Points>& Sps)
{
	if (1 == std::size(Sps)) {
		Sps.emplace_back(Collision::SupportPoint::GetPoints(RbA, RbB, -Sps[0].GetC().Normalize(), 0.0f));
	}
	if (2 == std::size(Sps)) {
		const auto AB = Sps[1].GetC() - Sps[0].GetC();
		Math::Vec3 U, V;
		AB.GetOrtho(U, V);
		Sps.emplace_back(Collision::SupportPoint::GetPoints(RbA, RbB, U, 0.0f));
	}
	if (3 == std::size(Sps)) {
		const auto AB = Sps[1].GetC() - Sps[0].GetC();
		const auto AC = Sps[2].GetC() - Sps[0].GetC();
		Sps.emplace_back(Collision::SupportPoint::GetPoints(RbA, RbB, AB.Cross(AC).Normalize(), 0.0f));
	}
}
void Collision::SupportPoint::Expand(const float Bias, std::vector<Collision::SupportPoint::Points>& Sps)
{
	const auto Center = std::accumulate(std::cbegin(Sps), std::cend(Sps), Math::Vec3::Zero(), [](const auto& Acc, const auto& i) { return Acc + i.GetC(); }) / static_cast<float>(std::size(Sps));
	std::ranges::transform(Sps, std::begin(Sps), [&](const auto& i) {
		const auto Dir = (i.GetC() - Center).Normalize() * Bias;
		return Collision::SupportPoint::Points(i.GetA() + Dir, i.GetB() - Dir);
		});
}

//!< #TODO �v����			
//!< �~���R�t�X�L�[���̓ʕ�𐶐��������Ɍ��_���܂ނ悤�ȃV���v���b�N�X (�P��) �𐶐����鎖�ő�p����
//!< A, B �̃~���R�t�X�L�[�� C �����_���܂߂ΏՓ˂ƂȂ�
//!< A, B �̃T�|�[�g�|�C���g�̍��� C �̃T�|�[�g�|�C���g�ƂȂ�
//!<	�ŏ��̃T�|�[�g�|�C���g 1 ��������
//!<	���_�����̎��̃T�|�[�g�|�C���g 2 ��������
//!<	1, 2 �̐������猴�_�����̎��̃T�|�[�g�|�C���g 3 ��������
//!<	1, 2, 3 ���Ȃ��O�p�`�����_���܂߂ΏՓˁA�I��
//!<	���_�������@�������̎��̃T�|�[�g�|�C���g 4 ��������
//!<	1, 2, 3, 4 ���Ȃ��l�ʑ̂����_���܂߂ΏՓˁA�I��
//!<	��ԋ߂��O�p�` (�Ⴆ�� 1, 2, 4) ����A���_�������@�������̎��̃T�|�[�g�|�C���g 5 ��������
//!<	�l�ʑ� (1, 2, 4, 5) �����_���܂ނ��A�T�|�[�g�|�C���g�������Ȃ�܂ő�����
bool Collision::Intersection::GJK(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, OnIntersectGJK OnIntersect, const float Bias, Math::Vec3& OnA, Math::Vec3& OnB)
{
	std::vector<Collision::SupportPoint::Points> Sps;
	Sps.reserve(4); //!< 4 �g

	//!< (1, 1, 1) �����̃T�|�[�g�|�C���g�����߂�
	Sps.emplace_back(Collision::SupportPoint::GetPoints(RbA, RbB, Math::Vec3::One().Normalize(), 0.0f));

	auto ClosestDist = (std::numeric_limits<float>::max)();
	auto Dir = -Sps.back().GetC();
	auto ContainOrigin = false;
	Math::Vec4 Lambda;
	do {
		const auto Pt = Collision::SupportPoint::GetPoints(RbA, RbB, Dir.ToNormalized(), 0.0f);
		Dir *= -1.0f;

		//!< �����̓_�Ƃ������Ƃ͂�������ȏ�g���ł��Ȃ� -> �Փ˖���
		if (std::ranges::any_of(Sps, [&](const auto& i) { return Pt.GetC().NearlyEqual(i.GetC()); })) {
			break;
		}

		Sps.emplace_back(Pt);

		//!< �V���v���b�N�X�����_���܂� -> �Փ�
		//!< (Dir ���X�V�ALambda ��Ԃ�)
		if ((ContainOrigin = Collision::SupportPoint::SimplexSignedVolumes(Sps, Dir, Lambda))) {
			break;
		}

		//!< �ŒZ�������X�V�A�X�V�ł��Ȃ�ΏI��
		const auto Dist = Dir.LengthSq();
		if (Dist < ClosestDist) {
			ClosestDist = Dist;
		}
		else {
			break;
		}

		//!< �L�� (Lambda[n] �� �� 0) �� Sps �������c��
		const auto Range = std::ranges::remove_if(Sps, [&](const auto& i) {
			return 0.0f == Lambda[static_cast<int>(IndexOf(Sps, i))];
			});
		Sps.erase(std::ranges::cbegin(Range), std::ranges::cend(Range));

		//!< �V���v���b�N�X���l�ʑ̂̏�Ԃł����܂ŗ����猴�_���܂�
		ContainOrigin = (4 == size(Sps));
	} while (!ContainOrigin); //!< ���_���܂܂������܂ŗ����烋�[�v

	if (ContainOrigin) {
		//!< �Փˊm�� (EPA ��)
		OnIntersect(RbA, RbB, Sps, Bias, OnA, OnB);
		return true;
	}
	else {
		//!< �l���L�� (��[��) �Ȃ��̂����A�O�l�߂ɂ��� (�Ԃ�l�͔j��)
		const auto Discard = std::ranges::remove_if(static_cast<Math::Component4&>(Lambda), [](const auto i) { return i == 0.0f; });
		//!< �Փ˂������ꍇ�́A�ŋߐړ_�����߂�
		OnA.ToZero();
		OnB.ToZero();
		for (auto i = 0; i < std::size(Sps); ++i) {
			OnA += Sps[i].GetA() * Lambda[i];
			OnB += Sps[i].GetB() * Lambda[i];
		}
		return false;
	}
}

void Collision::Intersection::EPA(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const std::vector<SupportPoint::Points>& SupportPoints, const float Bias, Math::Vec3& OnA, Math::Vec3& OnB)
{
	//!< ��Ɨp�T�|�[�g�|�C���g
	std::vector<SupportPoint::Points> Sps;
	Sps.assign(std::cbegin(SupportPoints), std::cend(SupportPoints));

	//!< EPA �̑O����
	{
		//!< EPA �͎l�ʑ̂�K�v�Ƃ���̂ŁA�l�ʑ̂֊g������
		SupportPoint::ToTetrahedron(RbA, RbB, Sps);
		//!< �V���v���b�N�X���o�C�A�X�̕������g������
		SupportPoint::Expand(Bias, Sps);
	}

	//!< ���_����O�p�`�̃C���f�b�N�X���X�g�𐶐�
	std::vector<Collision::TriInds> Tris;
	for (uint32_t i = 0; i < 4; ++i) {
		const auto j = (i + 1) % 4;
		const auto k = (i + 2) % 4;

		const auto l = (i + 3) % 4;

		if (Collision::Distance::IsFront(Sps[l].GetC(), Sps[i].GetC(), Sps[j].GetC(), Sps[k].GetC())) {
			Tris.emplace_back(Collision::TriInds({ j, i, k }));
		}
		else {
			Tris.emplace_back(Collision::TriInds({ i, j, k }));
		}
	}

	const auto Center = (Sps[0].GetC() + Sps[1].GetC() + Sps[2].GetC() + Sps[3].GetC()) * 0.25f;

	while (true) {
		//!< ���_�ɍł��߂��O�p�`���擾
		const auto& CTri = *SupportPoint::Distance::Closest(Math::Vec3::Zero(), Sps, Tris);
		const auto A = CTri[0], B = CTri[1], C = CTri[2];
		//!< �O�p�`�̖@��
		const auto N = Math::Vec3::UnitNormal(Sps[A].GetC(), Sps[B].GetC(), Sps[C].GetC());

		//!< �@�������̃T�|�[�g�|�C���g���擾
		const auto Pt = Collision::SupportPoint::GetPoints(RbA, RbB, N, Bias);

		//!< �T�|�[�g�|�C���g�����o�̏ꍇ�́A����ȏ�g���ł��Ȃ�
		if (std::ranges::any_of(Tris, [&](const auto i) {
			constexpr auto Eps = 0.01f;
			return Sps[i[0]].GetC().NearlyEqual(Pt.GetC(), Eps) || Sps[i[1]].GetC().NearlyEqual(Pt.GetC(), Eps) || Sps[i[2]].GetC().NearlyEqual(Pt.GetC(), Eps);
		})) {
			break;
		}

		//!< �T�|�[�g�|�C���g�Ƃ̋����������ꍇ�́A����ȏ�g���ł��Ȃ�
		if (Distance::PointTriangle(Pt.GetC(), Sps[A].GetC(), Sps[B].GetC(), Sps[C].GetC()) <= 0.0f) {
			break;
		}

		Sps.emplace_back(Pt);

		//!< �T�|�[�g�|�C���g���������Ă���O�p�`���폜�A�폜�ł��Ȃ��ꍇ�͏I��
		{
			const auto Range = std::ranges::remove_if(Tris, [&](const auto& i) {
				return Distance::IsFront(Pt.GetC(), Sps[i[0]].GetC(), Sps[i[1]].GetC(), Sps[i[2]].GetC());
			});
			if (std::ranges::cbegin(Range) == std::ranges::cend(Range)) {
				break;
			}
			Tris.erase(std::ranges::cbegin(Range), std::ranges::cend(Range));
		}

		//!< ���j�[�N�ȕӂ�T���A������ΏI��
		{
			std::vector<Collision::EdgeIndsCount> DanglingEdges;
			Convex::CollectUniqueEdges(Tris, DanglingEdges);
			if (std::empty(DanglingEdges)) {
				break;
			}

			//!< �T�|�[�g�|�C���g�ƃ��j�[�N�ӂ���Ȃ�O�p�`�Q��ǉ�
			const auto A = static_cast<uint32_t>(std::size(Sps)) - 1;
			std::ranges::transform(DanglingEdges, std::back_inserter(Tris), [&](const auto& i) {
				const auto B = i.first[0], C = i.first[1];
				if (Distance::IsFront(Center, Sps[A].GetC(), Sps[C].GetC(), Sps[B].GetC())) {
					return Collision::TriInds({ A, B, C });
				}
				else {
					return Collision::TriInds({ A, C, B });
				}
			});
		}
	}

	{
		//!< ���_�ɍł��߂��O�p�`���擾
		const auto& CTri = *SupportPoint::Distance::Farthest(Math::Vec3::Zero(), Sps, Tris);
		//const auto& CTri = *SupportPoint::Distance::Closest(Math::Vec3::Zero(), Sps, Tris);
		const auto A = CTri[0], B = CTri[1], C = CTri[2];
		//!< �����ł́A���_�̏d�S���W���擾
		const auto Lambda = Barycentric(Math::Vec3::Zero(), Sps[A].GetC(), Sps[B].GetC(), Sps[C].GetC());
		OnA = Sps[A].GetA() * Lambda[0] + Sps[B].GetA() * Lambda[1] + Sps[C].GetA() * Lambda[2];
		OnB = Sps[A].GetB() * Lambda[0] + Sps[B].GetB() * Lambda[1] + Sps[C].GetB() * Lambda[2];
	}
}

#ifdef _DEBUG
void Collision::SignedVolumeTest()
{
	constexpr auto Eps = 0.001f;
	{
		const std::vector OrgPts = {
			Math::Vec3(0.0f, 0.0f, 0.0f),
			Math::Vec3(1.0f, 0.0f, 0.0f),
			Math::Vec3(0.0f, 1.0f, 0.0f),
			Math::Vec3(0.0f, 0.0f, 1.0f),
		};
		{
			//!< ���؍�
			std::vector<Math::Vec3> Pts;
			Pts.resize(std::size(OrgPts));
			std::ranges::transform(OrgPts, std::begin(Pts), std::bind(std::plus(), std::placeholders::_1, Math::Vec3(1.0f, 1.0f, 1.0f)));
			const auto Lambda = SignedVolume(Pts[0], Pts[1], Pts[2], Pts[3]);
			const auto V = Pts[0] * Lambda[0] + Pts[1] * Lambda[1] + Pts[2] * Lambda[2] + Pts[3] * Lambda[3];

			const auto CorrectLambda = Math::Vec4(1.0f, 0.0f, 0.0f, 0.0f);
			const auto CorrectV = Math::Vec3::One();
			if (!Lambda.NearlyEqual(CorrectLambda, Eps)) {
				__debugbreak();
			}
			if (!V.NearlyEqual(CorrectV, Eps)) {
				__debugbreak();
			}
		}
		{
			//!< ���؍�
			std::vector<Math::Vec3> Pts;
			Pts.resize(std::size(OrgPts));
			std::ranges::transform(OrgPts, std::begin(Pts), std::bind(std::plus(), std::placeholders::_1, Math::Vec3(-1.0f, -1.0f, -1.0f) * 0.25f));
			const auto Lambda = SignedVolume(Pts[0], Pts[1], Pts[2], Pts[3]);
			const auto V = Pts[0] * Lambda[0] + Pts[1] * Lambda[1] + Pts[2] * Lambda[2] + Pts[3] * Lambda[3];

			const auto CorrectLambda = Math::Vec4(0.25f, 0.25f, 0.25f, 0.25f);
			const auto CorrectV = Math::Vec3::Zero();
			if (!Lambda.NearlyEqual(CorrectLambda, Eps)) {
				__debugbreak();
			}
			if (!V.NearlyEqual(CorrectV, Eps)) {
				__debugbreak();
			}
		}
		{
			//!< ���؍�
			std::vector<Math::Vec3> Pts;
			Pts.resize(std::size(OrgPts));
			std::ranges::transform(OrgPts, std::begin(Pts), std::bind(std::plus(), std::placeholders::_1, Math::Vec3(-1.0f, -1.0f, -1.0f)));
			const auto Lambda = SignedVolume(Pts[0], Pts[1], Pts[2], Pts[3]);
			const auto V = Pts[0] * Lambda[0] + Pts[1] * Lambda[1] + Pts[2] * Lambda[2] + Pts[3] * Lambda[3];

			const auto CorrectLambda = Math::Vec4(0.0f, 0.333f, 0.333f, 0.333f);
			const auto CorrectV = Math::Vec3(-0.667f, -0.667f, -0.667f);
			if (!Lambda.NearlyEqual(CorrectLambda, Eps)) {
				__debugbreak();
			}
			if (!V.NearlyEqual(CorrectV, Eps)) {
				__debugbreak();
			}
		}
		{
			//!< ���؍�
			std::vector<Math::Vec3> Pts;
			Pts.resize(std::size(OrgPts));
			std::ranges::transform(OrgPts, std::begin(Pts), std::bind(std::plus(), std::placeholders::_1, Math::Vec3(1.0f, 1.0f, -0.5f)));
			const auto Lambda = SignedVolume(Pts[0], Pts[1], Pts[2], Pts[3]);
			const auto V = Pts[0] * Lambda[0] + Pts[1] * Lambda[1] + Pts[2] * Lambda[2] + Pts[3] * Lambda[3];

			const auto CorrectLambda = Math::Vec4(0.5f, 0.0f, 0.0f, 0.5f);
			const auto CorrectV = Math::Vec3(1.0f, 1.0f, 0.0f);
			if (!Lambda.NearlyEqual(CorrectLambda, Eps)) {
				__debugbreak();
			}
			if (!V.NearlyEqual(CorrectV, Eps)) {
				__debugbreak();
			}
		}
	}
	{
		//!< ���؍�
		const std::array Pts = {
			Math::Vec3(51.1996613f, 26.1989613f, 1.91339576f),
			Math::Vec3(-51.0567360f, -26.0565681f, -0.436143428f),
			Math::Vec3(50.8978920f, -24.1035538f, -1.04042661f),
			Math::Vec3(-49.1021080f, 25.8964462f, -1.04042661f)
		};
		const auto Lambda = SignedVolume(Pts[0], Pts[1], Pts[2], Pts[3]);
		const auto V = Pts[0] * Lambda[0] + Pts[1] * Lambda[1] + Pts[2] * Lambda[2] + Pts[3] * Lambda[3];

		//!< �������킹
		const auto CorrectLambda = Math::Vec4(0.290f, 0.302f, 0.206f, 0.202f);
		const auto CorrectV = Math::Vec3::Zero();
		if (!Lambda.NearlyEqual(CorrectLambda, Eps)) {
			__debugbreak();
		}
		if (!V.NearlyEqual(CorrectV, Eps)) {
			__debugbreak();
		}
	}
}
#endif
