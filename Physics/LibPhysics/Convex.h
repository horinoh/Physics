#pragma once

#include <vector>
#include <algorithm>

#include "PhysicsMath.h"
#include "RigidBody.h"
#include "Collision.h"

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
	void RemoveInternal(const std::vector<Math::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices, std::vector<Math::Vec3>& Pts);
	
	void CollectUniqueEdges(std::vector<Collision::TriInds>::const_iterator Begin, std::vector<Collision::TriInds>::const_iterator End, std::vector<Collision::EdgeIndsCount>& EdgeCounts);
	static void CollectUniqueEdges(const std::vector<Collision::TriInds>& Indices, std::vector<Collision::EdgeIndsCount>& EdgeCounts) { CollectUniqueEdges(std::ranges::cbegin(Indices), std::ranges::cend(Indices), EdgeCounts); }

	void BuildConvexHull(const std::vector<Math::Vec3>& Pts, std::vector<Math::Vec3>& Vertices, std::vector<Collision::TriInds>& Indices);

	namespace Uniform {
		//!< #TODO �v����
		[[nodiscard]] Math::Vec3 CalcCenterOfMass(const Collision::AABB& Aabb, const std::vector<Math::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices);
		//!< #TODO �v����
		[[nodiscard]] Math::Mat3 CalcInertiaTensor(const Collision::AABB& Aabb, const std::vector<Math::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices, const Math::Vec3& CenterOfMass);
	}
	namespace MonteCarlo
	{
		[[nodiscard]] Math::Vec3 CalcCenterOfMass(const Collision::AABB& Aabb, const std::vector<Math::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices);
		[[nodiscard]] Math::Mat3 CalcInertiaTensor(const Collision::AABB& Aabb, const std::vector<Math::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices, const Math::Vec3& CenterOfMass);
	}
	namespace Tetrahedron 
	{
		[[nodiscard]] Math::Vec3 CalcCenterOfMass(const std::vector<Math::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices);
		[[nodiscard]] Math::Mat3 CalcInertiaTensor(const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C, const Math::Vec3& D);
		[[nodiscard]] Math::Mat3 CalcInertiaTensor(const std::vector<Math::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices, const Math::Vec3& CenterOfMass);
	}
}
