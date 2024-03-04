#pragma once

#include <algorithm>
#include <format>
#include <vector>

#include "RigidBody.h"
#include "Collision.h"
#include "PhysicsMath.h"

namespace Convex
{
	void BuildTetrahedron(const std::vector<Math::Vec3>& Pts, std::vector<Math::Vec3>& Vertices, std::vector<Collision::TriInds >& Indices);

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
	void CollectUniqueEdges(std::vector<Collision::TriInds>::const_iterator Begin, std::vector<Collision::TriInds>::const_iterator End, std::vector<Collision::EdgeIndsCount>& EdgeCounts);
	static void CollectUniqueEdges(const std::vector<Collision::TriInds>& Indices, std::vector<Collision::EdgeIndsCount>& EdgeCounts) { CollectUniqueEdges(std::ranges::cbegin(Indices), std::ranges::cend(Indices), EdgeCounts); }

	void BuildConvexHull(const std::vector<Math::Vec3>& Pts, std::vector<Math::Vec3>& Vertices, std::vector<Collision::TriInds>& Indices);

	//!< #TODO �v����
	[[nodiscard]] static Math::Vec3 CalcCenterOfMass(const Collision::AABB & Aabb, const std::vector<Math::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices) {
		//!< �e���ɃT���v�����O����� (����͌v�Z�Ɏ��Ԃ�������)
		//constexpr auto SampleCount = 100;
		constexpr auto SampleCount = 10;

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
		//!< �e���ɃT���v�����O����� (����͌v�Z�Ɏ��Ԃ�������)
		//constexpr auto SampleCount = 100;
		constexpr auto SampleCount = 10;

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
