#pragma once

#include "Math.h"
using namespace Math;

#include <algorithm>

#include "Shape.h"
#include "RigidBody.h"
using namespace Physics;

namespace Convex
{
	//!< ���_���u�Ȃ�ׂ��v��܂���悤�Ȏl�ʑ̂��쐬
	static void BuildTetrahedron(const std::vector<Vec3>& Pts, std::vector<Vec3>& Vertices, std::vector<TriInds>& Indices)
	{
		//!< ����̎� (�����ł�X) �Ɉ�ԉ����_
		std::array<Vec3, 4> P = { *Distance::Farthest(Pts, Vec3::AxisX()) };
		//< �O�o�̋t�����̎����Ɉ�ԉ����_
		P[1] = *Distance::Farthest(Pts, -P[0]);
		//!< �O�o�� 2 �_����Ȃ�����Ɉ�ԉ����_
		P[2] = *Distance::Farthest(Pts, P[0], P[1]);
		//!< �O�o�� 3 �_����Ȃ�O�p�`�Ɉ�ԉ����_
		P[3] = *Distance::Farthest(Pts, P[0], P[1], P[2]);

		//!< CCW �ɂȂ�悤�ɒ���
		if (Distance::PointTriangle(P[0], P[1], P[2], P[3]) > 0.0f) {
			std::swap(P[0], P[1]);
		}

		//!< �l�ʑ̂̒��_
		Vertices.emplace_back(P[0]);
		Vertices.emplace_back(P[1]);
		Vertices.emplace_back(P[2]);
		Vertices.emplace_back(P[3]);

		//!< �l�ʑ̂̃C���f�b�N�X
		Indices.emplace_back(TriInds({ 0, 1, 2 }));
		Indices.emplace_back(TriInds({ 0, 2, 3 }));
		Indices.emplace_back(TriInds({ 2, 1, 3 }));
		Indices.emplace_back(TriInds({ 1, 0, 3 }));
	}

	//!< �w��̓_���ʕ�̓����_���ǂ���
	[[nodiscard]] static bool IsInternal(const Vec3& Pt, const std::vector<Vec3>& Vertices, const std::vector<TriInds>& Indices)
	{
		//!< �S�Ă̎O�p�`�ɑ΂��A���̑��ɂ���Γ����_
		return std::ranges::all_of(Indices, [&](const auto rhs) {
			return Distance::PointTriangle(Pt, Vertices[rhs[0]], Vertices[rhs[1]], Vertices[rhs[2]]) <= 0.0f;
		});
	}
	//!< �ʕ�̓����_���폜
	static void RemoveInternal(const std::vector<Vec3>& Vertices, const std::vector<TriInds>& Indices, std::vector<Vec3>& Pts)
	{
		//!< �����_�����O
		{
			const auto Range = std::ranges::remove_if(Pts, [&](const auto& Pt) {
				return IsInternal(Pt, Vertices, Indices);
			});
			Pts.erase(std::begin(Range), std::end(Range));
		}

		//!< �����Ɠ���Ƃ݂Ȃ���_�͏��O
		{
			const auto Range = std::ranges::remove_if(Pts, [&](const auto& Pt) {
				//!< ����Ƃ݂Ȃ���_
				return std::ranges::any_of(Vertices, [&](const auto rhs) {
					return rhs.NearlyEqual(Pt);
				});
			});
			Pts.erase(std::begin(Range), std::end(Range));
		}
	}
	//!< �n�C�|����H�킹��Ƃ��Ȃ莞�Ԃ��������Ɍ��ǃn�C�|���̓ʕ�ł��邾���Ȃ̂ŃR���W�����Ƃ��Č����I�ł͂Ȃ��A���[�|����H�킹�邱��
	//!< ���؍�
	static void BuildConvexHull(const std::vector<Vec3>& Pts, std::vector<Vec3>& Vertices, std::vector<TriInds>& Indices)
	{
		LOG(data(std::format("Building convex hull...\n")));

		//!< �܂��́u�Ȃ�ׂ��v��܂���悤�Ȏl�ʑ̂��쐬
		BuildTetrahedron(Pts, Vertices, Indices);

		//!< �����_�̏��O -> �O���_���c��
		auto External = Pts;
		RemoveInternal(Vertices, Indices, External);

		//!< �O���_�������Ȃ�܂ŌJ��Ԃ�
		while (!std::empty(External)) {
			LOG(data(std::format("Rest vertices = {}\n", size(External))));

			//!< �ŉ��_��������
			const auto ExFarIt = Distance::Farthest(External, External[0]);

			//!< �ŉ��X�������Ă���O�p�` (A �Ƃ���) �ƁA�����Ă��Ȃ��O�p�` (B �Ƃ���) �̋��E�ƂȂ�ӂ����W���܂�
			std::vector<EdgeIndsCount> EdgeCounts;
			{
				//!< B ��O���AA ������ɕ�������
				const auto Range = std::ranges::partition(Indices, [&](const auto& i) {
					return Distance::PointTriangle(*ExFarIt, Vertices[i[0]], Vertices[i[1]], Vertices[i[2]]) <= 0.0f;
				});

				//!< A �� B �̋��E�ƂȂ�ӂ����W���� (A �̒����瑼�̎O�p�`�ƕӂ����L���Ȃ����j�[�N�ȕӂ݂̂����W����Ηǂ�)
				std::for_each(std::begin(Range), std::end(Indices), [&](const auto& i) {
					const std::array Edges = {
						EdgeInds({ i[0], i[1] }),
						EdgeInds({ i[1], i[2] }),
						EdgeInds({ i[2], i[0] }),
					};
					for (auto& j : Edges) {
						//!< ���o�̕ӂ��ǂ����𒲂ׂ� (�^�t�����o�Ƃ��Ĉ���)
						const auto It = std::ranges::find_if(EdgeCounts, [&](const auto& rhs) {
							return (rhs.first[0] == j[0] && rhs.first[1] == j[1]) || (rhs.first[0] == j[1] && rhs.first[1] == j[0]);
						});
						if (std::end(EdgeCounts) == It) {
							//!< �V�K�̕ӂȂ̂ŃJ�E���^�[���Ƃ��Ēǉ�
							EdgeCounts.emplace_back(EdgeIndsCount({ j, 0 }));
						}
						else {
							//!< (�ǉ��ς݂̕ӂ�) ���o�̕ӂƂȂ�����J�E���^���C���N�������g���ď����X�V���Ă���
							++It->second;
						}
					}
				});
				//!< (�ӂ͎��W�ς݂Ȃ̂�) �����܂ŗ����� A �͍폜���Ă悢  
				Indices.erase(std::begin(Range), std::end(Indices));
			}

			//!< �ʕ�̍X�V
			{
				//!< �ŉ��_�𒸓_�Ƃ��Ēǉ�����
				Vertices.emplace_back(*ExFarIt);

				//!< ���j�[�N�łȂ��� (�J�E���^�� 0 ���傫��) ���폜
				const auto Range = std::ranges::remove_if(EdgeCounts, [](const auto& i) { 
					return i.second > 0; 
				});
				EdgeCounts.erase(std::begin(Range), std::end(EdgeCounts));

				//!< �ŉ��_�̃C���f�b�N�X
				const auto FarIndex = static_cast<uint32_t>(std::size(Vertices) - 1);
				//!< �ŉ��_�ƃ��j�[�N�ӂ���Ȃ�O�p�`�Q��ǉ�
				std::ranges::transform(EdgeCounts, std::back_inserter(Indices), [&](const auto& i) {
					return TriInds({ i.first[0], i.first[1], FarIndex });
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

	//!< #TODO �v����
	[[nodiscard]] static Vec3 CalcCenterOfMass(const AABB& Aabb, const std::vector<Vec3>& Vertices, const std::vector<TriInds>& Indices) {
		constexpr auto SampleCount = 100;

		auto Sampled = 0;
		auto CenterOfMass = Vec3::Zero();
		const Vec3 d = Aabb.GetExtent() / static_cast<float>(SampleCount);
		for (auto x = 0; x < SampleCount; ++x) {
			for (auto y = 0; y < SampleCount; ++y) {
				for (auto z = 0; z < SampleCount; ++z) {
					const auto Pt = Vec3(Aabb.Min.X() + d.X() * x, Aabb.Min.Y() + d.Y() * y, Aabb.Min.Z() + d.Z() * z);
					if (IsInternal(Pt, Vertices, Indices)) {
						CenterOfMass += Pt;
						++Sampled;
					}
				}
			}
		}
		return CenterOfMass / static_cast<float>(Sampled);
	}
	//!< #TODO �v����
	[[nodiscard]] static Mat3 CalcInertiaTensor(const AABB& Aabb, const std::vector<Vec3>& Vertices, const std::vector<TriInds>& Indices, const Vec3& CenterOfMass) {
		constexpr auto SampleCount = 100;

		auto Sampled = 0;
		auto InertiaTensor = Mat3::Zero();
		const Vec3 d = Aabb.GetExtent() / static_cast<float>(SampleCount);
		for (auto x = 0; x < SampleCount; ++x) {
			for (auto y = 0; y < SampleCount; ++y) {
				for (auto z = 0; z < SampleCount; ++z) {
					const auto Pt = Vec3(Aabb.Min.X() + d.Z() * x, Aabb.Min.Y() + d.Y() * y, Aabb.Min.Z() + d.Z() * z) - CenterOfMass;
					if (IsInternal(Pt, Vertices, Indices)) {
						InertiaTensor[0][0] += Pt.Y() * Pt.Y() + Pt.Z() * Pt.Z();
						InertiaTensor[1][1] += Pt.Z() * Pt.Z() + Pt.X() * Pt.X();
						InertiaTensor[2][2] += Pt.X() * Pt.X() + Pt.Y() * Pt.Y();

						InertiaTensor[0][1] += -Pt.X() * Pt.Y();
						InertiaTensor[0][2] += -Pt.X() * Pt.Z();
						InertiaTensor[1][2] += -Pt.Y() * Pt.Z();

						InertiaTensor[1][0] += -Pt.X() * Pt.Y();
						InertiaTensor[2][0] += -Pt.X() * Pt.Z();
						InertiaTensor[2][1] += -Pt.Y() * Pt.Z();

						++Sampled;
					}
				}
			}
		}
		return InertiaTensor / static_cast<float>(Sampled);
	}
	[[nodiscard]] static Mat3 CalcInertiaTensor(const AABB& Aabb, const std::vector<Vec3>& Vertices, const std::vector<TriInds>& Indices) {
		return CalcInertiaTensor(Aabb, Vertices, Indices, CalcCenterOfMass(Aabb, Vertices, Indices));
	}
}
