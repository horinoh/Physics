#pragma once

#include "Math.h"
using namespace Math;

#include <algorithm>
#include <functional>

#include "Shape.h"
#include "RigidBody.h"
using namespace Physics;

namespace Collision
{
	//!< SignedVolue : �ˉe���ő�ƂȂ鎲�╽�ʂ������A����ɑ΂����_���ˉe���ē����ɂ���Ώd�S��Ԃ�

	//!< TODO ������
	[[nodiscard]] static Vec2 SignedVolume(const Vec3& A, const Vec3& B)
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
		//const std::array PrjSeg = { A[Index] , B[Index] };
		const auto PrjA = A[Index];
		const auto PrjB = B[Index];
		const auto PrjP = P[Index];

		//!< P �� [A, B] �̓����ɂ���ꍇ
		if ((PrjP > PrjA && PrjP < PrjB) || (PrjP > PrjB && PrjP < PrjA)) {
			return Vec2(PrjB - PrjP, PrjP - PrjA) / MaxVal;
		}
		//!< P �� A ���̊O��
		if ((PrjA <= PrjB && PrjP <= PrjA) || (PrjA >= PrjB && PrjP >= PrjA)) {
			return Vec2::AxisX();
		}
		//!< P �� B ���̊O��
		else {
			return Vec2::AxisY();
		}
	}
	//!< TODO ������
	[[nodiscard]] static Vec3 SignedVolume(const Vec3& A, const Vec3& B, const Vec3& C)
	{
		const auto N = (B - A).Cross(C - A);
		const auto P = N * A.Dot(N) / N.LengthSq(); //!< ���������Ȃ���̂Ő��K�����Ȃ������ǂ�

		//!<  XY, YZ, ZX ���ʂ̓��A�ˉe�ʐς��ő�̂��̂�������
		auto Index = 0;
		auto MaxVal = 0.0f;
		for (auto i = 0; i < 3; ++i) {
			const auto j = (i + 1) % 3;
			const auto k = (i + 2) % 3;

			const auto a = Vec2(A[j], A[k]);
			const auto b = Vec2(B[j], B[k]);
			const auto c = Vec2(C[j], C[k]);
			const auto AB = b - a;
			const auto AC = c - a;
			const auto Area = Mat2(AB, AC).Determinant();
			//const auto Area = AB.X() * AC.Y() - AB.Y() * AC.X();

			if (std::abs(Area) > std::abs(MaxVal)) {
				MaxVal = Area;
				Index = i;
			}
		}

		//!< �uP �ƎO�p�`�v��I���������ʂɎˉe (X ���I�����ꂽ�ꍇ Index1, Index2 �͂��ꂼ�� Y, Z �Ƃ�������ɂȂ�)
		const auto X = (Index + 1) % 3;
		const auto Y = (Index + 2) % 3;
		const auto PrjA = Vec2(A[X], A[Y]);
		const auto PrjB = Vec2(B[X], B[Y]);
		const auto PrjC = Vec2(C[X], C[Y]);
		const std::array PrjTri = { PrjA, PrjB, PrjC };
		const auto PrjP = Vec2(P[X], P[Y]);

		//!< �ˉe�_�ƕӂ���Ȃ�T�u�O�p�`�̖ʐ�
		Vec3 Areas;
		for (auto i = 0; i < 3; ++i) {
			const auto j = (i + 1) % 3;
			const auto k = (i + 2) % 3;

			Areas[i] = Mat2(PrjTri[j] - PrjP, PrjTri[k] - PrjP).Determinant();
			//Areas[i] = AB.X() * AC.Y() - AB.Y() * AC.X();
		}
		//!< P �� [A, B, C] �̓����ɂ���ꍇ (�T�u�O�p�`�̖ʐς̕������番����)
		if (Sign(MaxVal) == Sign(Areas.X()) && Sign(MaxVal) == Sign(Areas.Y()) && Sign(MaxVal) == Sign(Areas.Z())) {
			return Areas / MaxVal;
		}

		//!< 3 �ӂɎˉe���Ĉ�ԋ߂����̂������� (1-SignedVolume �ɋA��)
		const std::array EdgesPts = { A, B, C };
		Vec3 Lambda;
		auto MinVal = (std::numeric_limits<float>::max)();
		for (auto i = 0; i < 3; ++i) {
			const auto j = (i + 1) % 3;
			const auto k = (i + 2) % 3;

			const auto LambdaEdge = SignedVolume(EdgesPts[j], EdgesPts[k]);
			const auto LenSq = (EdgesPts[j] * LambdaEdge[0] + EdgesPts[k] * LambdaEdge[1]).LengthSq();
			if (LenSq < MinVal) {
				Lambda[i] = 0.0f;
				Lambda[j] = LambdaEdge[0];
				Lambda[k] = LambdaEdge[1];
				MinVal = LenSq;
			}
		}
		return Lambda;
	}
	//!< ���؍�
	[[nodiscard]] static Vec4 SignedVolume(const Vec3& A, const Vec3& B, const Vec3& C, const Vec3& D)
	{
		//const auto Cofactor = Vec4(-Mat3(B, C, D).Determinant(), Mat3(A, C, D).Determinant(), -Mat3(A, B, D).Determinant(), Mat3(A, B, C).Determinant());
		const auto M = Mat4(Vec4(A.X(), B.X(), C.X(), D.X()), Vec4(A.Y(), B.Y(), C.Y(), D.Y()), Vec4(A.Z(), B.Z(), C.Z(), D.Z()), Vec4::One());
		const auto Cofactor = Vec4(M.Cofactor(3, 0), M.Cofactor(3, 1), M.Cofactor(3, 2), M.Cofactor(3, 3));
		const auto Det = Cofactor.X() + Cofactor.Y() + Cofactor.Z() + Cofactor.W();

		//!< �l�ʑ̓����ɂ���΁A�d�S���W��Ԃ�
		if (Sign(Det) == Sign(Cofactor.X()) && Sign(Det) == Sign(Cofactor.Y()) && Sign(Det) == Sign(Cofactor.Z()) && Sign(Det) == Sign(Cofactor.W())) {
			return Cofactor / Det;
		}

		//!< 3 �ʂɎˉe���Ĉ�ԋ߂����̂������� (2-SignedVolume �ɋA��)
		const std::array FacePts = { A, B, C, D };
		Vec4 Lambda;
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

	//!< �T�|�[�g�|�C���g : ����̕����ɍł������_
	class SupportPoints
	{
	public:
		SupportPoints(const Vec3& A, const Vec3& B) : Data({ A, B, A - B }) { }

		const Vec3 GetA() const { return std::get<0>(Data); }
		const Vec3 GetB() const { return std::get<1>(Data); }
		const Vec3 GetC() const { return std::get<2>(Data); }

		bool operator == (const SupportPoints& rhs) const {
			return GetA() == rhs.GetA() && GetB() == rhs.GetB() && GetC() == rhs.GetC();
		}
	private:
		std::tuple<Vec3, Vec3, Vec3> Data;
	};
	//!< A, B �̃T�|�[�g�|�C���g�̍����AC (A, B �̃~���R�t�X�L�[��) �̃T�|�[�g�|�C���g�ƂȂ�
	static [[nodiscard]] SupportPoints GetSupportPoints(const RigidBody* RbA, const RigidBody* RbB, const Vec3& NDir, const float Bias) {
		return SupportPoints(RbA->Shape->GetSupportPoint(RbA->Position, RbA->Rotation, NDir, Bias), RbB->Shape->GetSupportPoint(RbB->Position, RbB->Rotation, -NDir, Bias));
	}
	static [[nodiscard]] bool SimplexSignedVolumes(const std::vector<SupportPoints>& Sps, Vec3& Dir, Vec4& OutLambda)
	{
		//constexpr auto Eps2 = (std::numeric_limits<float>::epsilon)() * (std::numeric_limits<float>::epsilon)();
		constexpr auto Eps2 = 0.0001f * 0.00001f;

		switch (size(Sps)) {
		case 2:
			OutLambda = SignedVolume(Sps[0].GetC(), Sps[1].GetC());
			Dir = - (Sps[0].GetC() * OutLambda[0] + Sps[1].GetC() * OutLambda[1]);
		break;
		case 3:
			OutLambda = SignedVolume(Sps[0].GetC(), Sps[1].GetC(), Sps[2].GetC());
			Dir = - (Sps[0].GetC() * OutLambda[0] + Sps[1].GetC() * OutLambda[1] + Sps[2].GetC() * OutLambda[2]);
			break;
		case 4:
			OutLambda = SignedVolume(Sps[0].GetC(), Sps[1].GetC(), Sps[2].GetC(), Sps[3].GetC());
			Dir = - (Sps[0].GetC() * OutLambda[0] + Sps[1].GetC() * OutLambda[1] + Sps[2].GetC() * OutLambda[2] + Sps[3].GetC() * OutLambda[3]);
			break;
		default:
			__debugbreak();
			break;
		}

		//!< ���_���܂� -> �Փ�
		return Dir.LengthSq() < Eps2;
	}

	namespace Intersection {
		//!< #TODO �v����			
		//!< �~���R�t�X�L�[���̓ʕ�𐶐��������Ɍ��_���܂ނ悤�ȒP�̂𐶐����鎖�ő�p����
		//!< A, B �̃~���R�t�X�L�[�� C �����_���܂߂ΏՓ˂ƂȂ�
		//!< A, B �̃T�|�[�g�|�C���g�̍��� C �̃T�|�[�g�|�C���g�ƂȂ�
		//!<	�ŏ��̃T�|�[�g�|�C���g 1 ��������
		//!<	���_�����̎��̃T�|�[�g�|�C���g 2 ��������
		//!<	1, 2 �̐������猴�_�����̎��̃T�|�[�g�|�C���g 3 ��������
		//!<	1, 2, 3 ���Ȃ��O�p�`�����_���܂߂ΏՓˁA�I��
		//!<	���_�������@�������̎��̃T�|�[�g�|�C���g 4 ��������
		//!<	1, 2, 3, 4 ���Ȃ��l�ʑ̂����_���܂߂ΏՓˁA�I��
		//!<	��ԋ߂��O�p�` (�Ⴆ�� 1, 2, 4) ����A���_�������@�������̎��̃T�|�[�g�|�C���g 5 ��������
		//!<	�l�ʑ̂����_���܂ނ��A�T�|�[�g�|�C���g�������Ȃ�܂ő�����
		[[nodiscard]] static bool GJK(const RigidBody* RbA, const RigidBody* RbB, std::function<void(const RigidBody*, const RigidBody*, const std::vector<SupportPoints>&, const float, Vec3&, Vec3&)> OnIntersect, const float Bias, Vec3& OnA, Vec3& OnB)
		{
			std::vector<SupportPoints> Sps;
			Sps.reserve(4); //!< 4 �g

			//!< (1, 1, 1) �����̃T�|�[�g�|�C���g�����߂�
			Sps.emplace_back(GetSupportPoints(RbA, RbB, Vec3::One().Normalize(), 0.0f));

			auto ClosestDist = (std::numeric_limits<float>::max)();
			auto Dir = -Sps.back().GetC();
			auto ContainOrigin = false;
			do {
				const auto Pt = GetSupportPoints(RbA, RbB, Dir.ToNormalized(), 0.0f);
				Dir *= -1.0f;

				//!< �����̓_�Ƃ������Ƃ͂�������ȏ�g���ł��Ȃ� -> �Փ˖���
				if (std::end(Sps) != std::ranges::find_if(Sps, [&](const auto& rhs) { return Pt == rhs; })) {
					break;
				}

				Sps.emplace_back(Pt);

				//!< �V�����_�����_�𒴂��Ă��Ȃ��ꍇ�A���_�������Ɋ܂܂�Ȃ� -> �Փ˖���
				if (Dir.Dot(Pt.GetC()) < 0.0f) {
					break;
				}

				//!< 1, 2, 3-�V���v���b�N�X���̏��� (Dir ���X�V�ALambda ��Ԃ�)
				Vec4 Lambda;
				if ((ContainOrigin = SimplexSignedVolumes(Sps, Dir, Lambda))) {
					//!< �V���v���b�N�X�����_���܂� -> �Փ�
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

				//!< �L���� (Lambda �� �� 0) Sps �������c��
				const auto [Beg, End] = std::ranges::remove_if(Sps, [&](const auto& rhs) {
					return 0.0f == Lambda[static_cast<int>(IndexOf(Sps, rhs))];
				});
				Sps.erase(Beg, End);

				ContainOrigin = (4 == size(Sps));
			} while (!ContainOrigin); //!< ���_���܂܂������܂ŗ����烋�[�v (�V���v���b�N�X�����_���܂ޏꍇ�A�V���v���b�N�X�l�ʑ̂Ń��[�v���I�����ꍇ�̓��[�v����o��)

			//!< ���_���܂܂Ȃ���Ԃł����܂ŗ�����A�Փ˖���
			if (!ContainOrigin) {
				return false;
			}

			//!< ���̐�Փˊm��

			//!< �Փˊm�莞�ɌĂяo���֐� (EPA ��)
			if (4 == size(Sps)) {
				Vec3 OnA, OnB;
				OnIntersect(RbA, RbB, Sps, Bias, OnA, OnB);
				return true;
			}

			return true;
		}
		[[nodiscard]] static bool GJK(const RigidBody* RbA, const RigidBody* RbB) {
			Vec3 OnA, OnB;
			return GJK(RbA, RbB, [](const RigidBody*, const RigidBody*, const std::vector<SupportPoints>&, const float, Vec3&, Vec3&) {}, 0.001f, OnA, OnB);
		}
		[[nodiscard]] float EPA(const RigidBody*, const RigidBody*, const std::vector<SupportPoints>& Sps, const float Bias, Vec3& OnA, Vec3& OnB) {
			return 0.0f;
		}
	}
	namespace Closest {
		static void GJK(const RigidBody* RbA, const RigidBody* RbB, Vec3& OnA, Vec3& OnB) {
			std::vector<SupportPoints> Sps;
			Sps.reserve(4);

			Sps.emplace_back(GetSupportPoints(RbA, RbB, Vec3::One().Normalize(), 0.0f));

			auto ClosestDist = (std::numeric_limits<float>::max)();
			auto Dir = -Sps.back().GetC();
			Vec4 Lambda;
			do {
				const auto Pt = GetSupportPoints(RbA, RbB, Dir.ToNormalized(), 0.0f);
				Dir *= -1.0f;

				if (std::end(Sps) != std::ranges::find_if(Sps, [&](const auto& rhs) { return Pt == rhs; })) {
					break;
				}

				Sps.emplace_back(Pt);

				SimplexSignedVolumes(Sps, Dir, Lambda);

				const auto [Beg, End] = std::ranges::remove_if(Sps, [&](const auto& rhs) {
					return 0.0f == Lambda[static_cast<int>(IndexOf(Sps, rhs))];
				});
				Sps.erase(Beg, End);

				const auto Dist = Dir.LengthSq();
				if (Dist < ClosestDist) {
					ClosestDist = Dist;
				}
				else {
					break;
				}
			} while (std::size(Sps) < 4);

			if (4 == std::size(Sps)) {
				OnA = Sps[0].GetA() * Lambda[0] + Sps[1].GetA() * Lambda[1] + Sps[2].GetA() * Lambda[2] + Sps[3].GetA() * Lambda[3];
				OnB = Sps[0].GetB() * Lambda[0] + Sps[1].GetB() * Lambda[1] + Sps[2].GetB() * Lambda[2] + Sps[3].GetB() * Lambda[3];
			}
		}
	}

#ifdef _DEBUG
	static void SignedVolumeTest()
	{
		constexpr auto Eps = 0.001f;	
		{
			const std::vector OrgPts = {
				Vec3(0.0f, 0.0f, 0.0f),
				Vec3(1.0f, 0.0f, 0.0f),
				Vec3(0.0f, 1.0f, 0.0f),
				Vec3(0.0f, 0.0f, 1.0f),
			};
			{
				//!< ���؍�
				std::vector<Vec3> Pts;
				Pts.resize(std::size(OrgPts));
				std::ranges::transform(OrgPts, std::begin(Pts), std::bind(std::plus(), std::placeholders::_1, Vec3(1.0f, 1.0f, 1.0f)));
				const auto Lambda = SignedVolume(Pts[0], Pts[1], Pts[2], Pts[3]);
				const auto V = Pts[0] * Lambda[0] + Pts[1] * Lambda[1] + Pts[2] * Lambda[2] + Pts[3] * Lambda[3];

				const auto CorrectLambda = Vec4(1.0f, 0.0f, 0.0f, 0.0f);
				const auto CorrectV = Vec3::One();
				if (!Lambda.NearlyEqual(CorrectLambda, Eps)) {
					__debugbreak();
				}
				if (!V.NearlyEqual(CorrectV, Eps)) {
					__debugbreak();
				}
			}
			{
				//!< ���؍�
				std::vector<Vec3> Pts;
				Pts.resize(std::size(OrgPts));
				std::ranges::transform(OrgPts, std::begin(Pts), std::bind(std::plus(), std::placeholders::_1, Vec3(-1.0f, -1.0f, -1.0f) * 0.25f));
				const auto Lambda = SignedVolume(Pts[0], Pts[1], Pts[2], Pts[3]);
				const auto V = Pts[0] * Lambda[0] + Pts[1] * Lambda[1] + Pts[2] * Lambda[2] + Pts[3] * Lambda[3];

				const auto CorrectLambda = Vec4(0.25f, 0.25f, 0.25f, 0.25f);
				const auto CorrectV = Vec3::Zero();
				if (!Lambda.NearlyEqual(CorrectLambda, Eps)) {
					__debugbreak();
				}
				if (!V.NearlyEqual(CorrectV, Eps)) {
					__debugbreak();
				}
			}
			{
				//!< ���؍�
				std::vector<Vec3> Pts;
				Pts.resize(std::size(OrgPts));
				std::ranges::transform(OrgPts, std::begin(Pts), std::bind(std::plus(), std::placeholders::_1, Vec3(-1.0f, -1.0f, -1.0f)));
				const auto Lambda = SignedVolume(Pts[0], Pts[1], Pts[2], Pts[3]);
				const auto V = Pts[0] * Lambda[0] + Pts[1] * Lambda[1] + Pts[2] * Lambda[2] + Pts[3] * Lambda[3];

				const auto CorrectLambda = Vec4(0.0f, 0.333f, 0.333f, 0.333f);
				const auto CorrectV = Vec3(-0.667f, -0.667f, -0.667f);
				if (!Lambda.NearlyEqual(CorrectLambda, Eps)) {
					__debugbreak();
				}
				if (!V.NearlyEqual(CorrectV, Eps)) {
					__debugbreak();
				}
			}
			{
				//!< ���؍�
				std::vector<Vec3> Pts;
				Pts.resize(std::size(OrgPts));
				std::ranges::transform(OrgPts, std::begin(Pts), std::bind(std::plus(), std::placeholders::_1, Vec3(1.0f, 1.0f, -0.5f)));
				const auto Lambda = SignedVolume(Pts[0], Pts[1], Pts[2], Pts[3]);
				const auto V = Pts[0] * Lambda[0] + Pts[1] * Lambda[1] + Pts[2] * Lambda[2] + Pts[3] * Lambda[3];

				const auto CorrectLambda = Vec4(0.5f, 0.0f, 0.0f, 0.5f);
				const auto CorrectV = Vec3(1.0f, 1.0f, 0.0f);
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
				Vec3(51.1996613f, 26.1989613f, 1.91339576f),
				Vec3(-51.0567360f, -26.0565681f, -0.436143428f),
				Vec3(50.8978920f, -24.1035538f, -1.04042661f),
				Vec3(-49.1021080f, 25.8964462f, -1.04042661f)
			};
			const auto Lambda = SignedVolume(Pts[0], Pts[1], Pts[2], Pts[3]);
			const auto V = Pts[0] * Lambda[0] + Pts[1] * Lambda[1] + Pts[2] * Lambda[2] + Pts[3] * Lambda[3];

			//!< �������킹
			const auto CorrectLambda = Vec4(0.290f, 0.302f, 0.206f, 0.202f);
			const auto CorrectV = Vec3::Zero();
			if (!Lambda.NearlyEqual(CorrectLambda, Eps)) {
				__debugbreak();
			}
			if (!V.NearlyEqual(CorrectV, Eps)) {
				__debugbreak();
			}
		}
	}
#endif
}