#pragma once

#include <algorithm>
#include <format>
#include <vector>

#include "Physics.h"
#include "RigidBody.h"
#include "Collision.h"
#include "PhysicsMath.h"

namespace Convex
{
	//!< ���_���u�Ȃ�ׂ��v��܂���悤�Ȏl�ʑ̂��쐬
	static void BuildTetrahedron(const std::vector<Math::Vec3>& Pts, std::vector<Math::Vec3>& Vertices, std::vector<Collision::TriInds > & Indices)
	{
		//!< ����̎� (�����ł�X) �Ɉ�ԉ����_
		std::array<Math::Vec3, 4> P = { *Collision::Distance::Farthest(Pts, Math::Vec3::AxisX()) };
		//< �O�o�̋t�����̎����Ɉ�ԉ����_
		P[1] = *Collision::Distance::Farthest(Pts, -P[0]);
		//!< �O�o�� 2 �_����Ȃ�����Ɉ�ԉ����_
		P[2] = *Collision::Distance::Farthest(Pts, P[0], P[1]);
		//!< �O�o�� 3 �_����Ȃ�O�p�`�Ɉ�ԉ����_
		P[3] = *Collision::Distance::Farthest(Pts, P[0], P[1], P[2]);

		//!< CCW �ɂȂ�悤�ɒ���
		if (Collision::Distance::IsFront(P[0], P[1], P[2], P[3])) {
			std::swap(P[0], P[1]);
		}

		//!< �l�ʑ̂̒��_
		Vertices.emplace_back(P[0]);
		Vertices.emplace_back(P[1]);
		Vertices.emplace_back(P[2]);
		Vertices.emplace_back(P[3]);

		//!< �l�ʑ̂̃C���f�b�N�X
		Indices.emplace_back(Collision::TriInds({ 0, 1, 2 }));
		Indices.emplace_back(Collision::TriInds({ 0, 2, 3 }));
		Indices.emplace_back(Collision::TriInds({ 2, 1, 3 }));
		Indices.emplace_back(Collision::TriInds({ 1, 0, 3 }));
	}

	//!< �w��̓_���ʕ�̓����_���ǂ���
	[[nodiscard]] static bool IsInternal(const Math::Vec3& Pt, const std::vector<Math::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices)
	{
		//!< �S�Ă̎O�p�`�ɑ΂��A���̑��ɂ���Γ����_
		return std::ranges::all_of(Indices, [&](const auto rhs) {
			return !Collision::Distance::IsFront(Pt, Vertices[rhs[0]], Vertices[rhs[1]], Vertices[rhs[2]]);
		});
	}
	//!< �ʕ�̓����_���폜
	static void RemoveInternal(const std::vector<Math::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices, std::vector<Math::Vec3>& Pts)
	{
		//!< �����_�����O
		{
			const auto Range = std::ranges::remove_if(Pts, [&](const auto& Pt) {
				return IsInternal(Pt, Vertices, Indices);
			});
			Pts.erase(std::ranges::cbegin(Range), std::ranges::cend(Range));
		}

		//!< �����Ɠ���Ƃ݂Ȃ���_�͏��O
		{
			const auto Range = std::ranges::remove_if(Pts, [&](const auto& Pt) {
				//!< ����Ƃ݂Ȃ���_
				return std::ranges::any_of(Vertices, [&](const auto rhs) {
					return rhs.NearlyEqual(Pt);
				});
			});
			Pts.erase(std::ranges::cbegin(Range), std::ranges::cend(Range));
		}
	}
	static void CollectUniqueEdges(std::vector<Collision::TriInds>::const_iterator Begin, std::vector<Collision::TriInds>::const_iterator End, std::vector<Collision::EdgeIndsCount>& EdgeCounts)
	{
		std::for_each(Begin, End, [&](const auto& i) {
			const std::array Edges = {
				Collision::EdgeInds({ i[0], i[1] }),
				Collision::EdgeInds({ i[1], i[2] }),
				Collision::EdgeInds({ i[2], i[0] }),
			};
			for (auto& j : Edges) {
				//!< ���o�̕ӂ��ǂ����𒲂ׂ� (�^�t�����o�Ƃ��Ĉ���)
				const auto It = std::ranges::find_if(EdgeCounts, [&](const auto& k) {
					return (k.first[0] == j[0] && k.first[1] == j[1]) || (k.first[0] == j[1] && k.first[1] == j[0]);
				});
				if (std::cend(EdgeCounts) == It) {
					//!< �V�K�̕ӂȂ̂ŃJ�E���^�[���Ƃ��Ēǉ�
					EdgeCounts.emplace_back(Collision::EdgeIndsCount({j, 0}));
				}
				else {
					//!< (�ǉ��ς݂̕ӂ�) ���o�̕ӂƂȂ�����J�E���^���C���N�������g���ď����X�V���Ă���
					++It->second;
				}
			}
		});

		//!< ���j�[�N�łȂ��� (�J�E���^�� 0 ���傫��) ���폜
		const auto Range = std::ranges::remove_if(EdgeCounts, [](const auto& i) { return i.second > 0; });
		EdgeCounts.erase(std::ranges::cbegin(Range), std::ranges::cend(EdgeCounts));
	}
	static void CollectUniqueEdges(const std::vector<Collision::TriInds>& Indices, std::vector<Collision::EdgeIndsCount>& EdgeCounts) { CollectUniqueEdges(std::ranges::cbegin(Indices), std::ranges::cend(Indices), EdgeCounts); }

#if 0
	static void BuildConvexHull(const std::vector<Vec3>& Pts, std::vector<Vec3>& Vertices, std::vector<TriInds>& Indices);
#else
	//!< �n�C�|����H�킹��Ƃ��Ȃ莞�Ԃ��������Ɍ��ǃn�C�|���̓ʕ�ł��邾���Ȃ̂ŃR���W�����Ƃ��Č����I�ł͂Ȃ��A���[�|����H�킹�邱��
	//!< ���؍�
	static void BuildConvexHull(const std::vector<Math::Vec3>& Pts, std::vector<Math::Vec3>& Vertices, std::vector<Collision::TriInds>& Indices)
	{
		//LOG(data(std::format("Building convex hull...\n")));

		//!< �܂��́u�Ȃ�ׂ��v��܂���悤�Ȏl�ʑ̂��쐬
		BuildTetrahedron(Pts, Vertices, Indices);

		//!< �����_�̏��O -> �O���_���c��
		auto External = Pts;
		RemoveInternal(Vertices, Indices, External);

		//!< �O���_�������Ȃ�܂ŌJ��Ԃ�
		while (!std::empty(External)) {
			//LOG(data(std::format("Rest vertices = {}\n", size(External))));

			//!< �ŉ��_��������
			const auto ExFarIt = Collision::Distance::Farthest(External, External[0]);

			std::vector<Collision::EdgeIndsCount> EdgeCounts;
			{
				//!< �ŉ��_�������Ă��Ȃ��O�p�` (A) �ƁA�����Ă���O�p�` (B) �ɕ���
				//!< partition �͈ȉ��̂悤�ɕԂ�
				//!<	A �����_���� true	: [begin(Indices), begin(Range)]
				//!<	B �����_���� false	: [begin(Range), end(Range)]
				const auto Range = std::ranges::partition(Indices, [&](const auto& i) {
					return !Collision::Distance::IsFront(*ExFarIt, Vertices[i[0]], Vertices[i[1]], Vertices[i[2]]);
				});

				//!< A, B �̋��E�ƂȂ�悤�ȕӂ����W���� (B �̒����瑼�̎O�p�`�ƕӂ����L���Ȃ����j�[�N�ȕӂ݂̂����W����Ηǂ�)
				CollectUniqueEdges(std::ranges::cbegin(Range), std::ranges::cend(Range), EdgeCounts);

				//!< (�ӂ͎��W��) �����܂ŗ����� B �͍폜���Ă悢  
				Indices.erase(std::ranges::cbegin(Range), std::ranges::cend(Range));
			}

			//!< �ʕ�̍X�V
			{
				//!< �ŉ��_�𒸓_�Ƃ��Ēǉ�����
				Vertices.emplace_back(*ExFarIt);

				//!< �ŉ��_�̃C���f�b�N�X
				const auto FarIndex = static_cast<uint32_t>(std::size(Vertices) - 1);
				//!< �ŉ��_�ƃ��j�[�N�ӂ���Ȃ�O�p�`�Q��ǉ�
				std::ranges::transform(EdgeCounts, std::back_inserter(Indices), [&](const auto& i) {
					return Collision::TriInds({ i.first[0], i.first[1], FarIndex });
				});
			}

			//!< �O���_�̍X�V
			{
				//!< �����܂ōς񂾂�ŉ��_�͍폜���Ă悢
				External.erase(ExFarIt);

				//!< �X�V�����ʕ�ɑ΂��ē����_���폜����
				RemoveInternal(Vertices, Indices, External);
			}
		}
	}
#endif

	//!< #TODO �v����
	[[nodiscard]] static Math::Vec3 CalcCenterOfMass(const Collision::AABB & Aabb, const std::vector<Math::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices) {
		//!< �e���ɃT���v�����O�����
		constexpr auto SampleCount = 100;

		auto CenterOfMass = Math::Vec3::Zero();
		auto Sampled = 0;
		const auto Delta = Aabb.GetExtent() / static_cast<float>(SampleCount);
		for (auto x = 0; x < SampleCount; ++x) {
			for (auto y = 0; y < SampleCount; ++y) {
				for (auto z = 0; z < SampleCount; ++z) {
					//!< AABB ���̃T���v���_
					const auto Pt = Aabb.Min + Math::Vec3(Delta.X() * x, Delta.Y() * y, Delta.Z() * z);
					if (IsInternal(Pt, Vertices, Indices)) {
						//!< �����_�Ȃ���W
						CenterOfMass += Pt;
						++Sampled;
					}
				}
			}
		}
		return CenterOfMass / static_cast<float>(Sampled);
	}
	//!< #TODO �v����
	[[nodiscard]] static Math::Mat3 CalcInertiaTensor(const Collision::AABB& Aabb, const std::vector<Math::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices, const Math::Vec3& CenterOfMass) {
		//!< �e���ɃT���v�����O�����
		constexpr auto SampleCount = 100;

		auto InertiaTensor = Math::Mat3::Zero();
		auto Sampled = 0;
		const auto Delta = Aabb.GetExtent() / static_cast<float>(SampleCount);
		for (auto x = 0; x < SampleCount; ++x) {
			for (auto y = 0; y < SampleCount; ++y) {
				for (auto z = 0; z < SampleCount; ++z) {
					//!< AABB ���̃T���v���_ (�d�S����̑���)
					const auto Pt = Aabb.Min + Math::Vec3(Delta.X() * x, Delta.Y() * y, Delta.Z() * z) - CenterOfMass;
					if (IsInternal(Pt, Vertices, Indices)) {
						//!< �����_�Ȃ���W����
#if 0
						InertiaTensor[0][0] += Pt.Y() * Pt.Y() + Pt.Z() * Pt.Z();
						InertiaTensor[1][1] += Pt.Z() * Pt.Z() + Pt.X() * Pt.X();
						InertiaTensor[2][2] += Pt.X() * Pt.X() + Pt.Y() * Pt.Y();

						InertiaTensor[0][1] += -Pt.X() * Pt.Y();
						InertiaTensor[0][2] += -Pt.X() * Pt.Z();
						InertiaTensor[1][2] += -Pt.Y() * Pt.Z();

						InertiaTensor[1][0] += -Pt.X() * Pt.Y();
						InertiaTensor[2][0] += -Pt.X() * Pt.Z();
						InertiaTensor[2][1] += -Pt.Y() * Pt.Z();
#else
						InertiaTensor += {
							{ Pt.Y() * Pt.Y() + Pt.Z() * Pt.Z(), -Pt.X() * Pt.Y(), -Pt.X() * Pt.Z() },
							{ -Pt.X() * Pt.Y(), Pt.Z() * Pt.Z() + Pt.X() * Pt.X(), -Pt.Y() * Pt.Z() },
							{ -Pt.X() * Pt.Z(), -Pt.Y() * Pt.Z(), Pt.X() * Pt.X() + Pt.Y() * Pt.Y() },
						};
#endif
						++Sampled;
					}
				}
			}
		}
		return InertiaTensor / static_cast<float>(Sampled);
	}
	//[[nodiscard]] static Mat3 CalcInertiaTensor(const AABB& Aabb, const std::vector<Vec3>& Vertices, const std::vector<TriInds>& Indices) {
	//	return CalcInertiaTensor(Aabb, Vertices, Indices, CalcCenterOfMass(Aabb, Vertices, Indices));
	//}
}
