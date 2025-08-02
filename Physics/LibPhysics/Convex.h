#pragma once

#include <vector>
#include <algorithm>

#include "LinAlg.h"
#include "RigidBody.h"
#include "Collision.h"

namespace Convex
{
	void BuildTetrahedron(const std::vector<LinAlg::Vec3>& Mesh, std::vector<LinAlg::Vec3>& Vertices, std::vector<Collision::TriInds >& Indices);

	//!< �w��̓_���ʕ�̓����_���ǂ���
	[[nodiscard]] static bool IsInternal(const LinAlg::Vec3& Pt, const std::vector<LinAlg::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices)
	{
		//!< �S�Ă̎O�p�`�ɑ΂��A���̑��ɂ���Γ����_
		return std::ranges::all_of(Indices,
			[&](const auto rhs) {
				return !Collision::Distance::IsFrontTriangle(Pt, Vertices[rhs[0]], Vertices[rhs[1]], Vertices[rhs[2]]);
			});
	}

	//!< �ʕ�̓����_���폜
	void RemoveInternal(const std::vector<LinAlg::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices, std::vector<LinAlg::Vec3>& Mesh);
	
	void CollectUniqueEdges(std::vector<Collision::TriInds>::const_iterator Begin, std::vector<Collision::TriInds>::const_iterator End, std::vector<Collision::EdgeIndsWithCount>& EdgeCounts);
	static void CollectUniqueEdges(const std::vector<Collision::TriInds>& Indices, std::vector<Collision::EdgeIndsWithCount>& EdgeCounts) { 
		CollectUniqueEdges(std::ranges::cbegin(Indices), std::ranges::cend(Indices), EdgeCounts);
	}

	void BuildConvexHull(const std::vector<LinAlg::Vec3>& Pts, std::vector<LinAlg::Vec3>& Vertices, std::vector<Collision::TriInds>& Indices);

	namespace Uniform {
		[[nodiscard]] LinAlg::Vec3 CalcCenterOfMass(const Collision::AABB& Aabb, const std::vector<LinAlg::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices);
		[[nodiscard]] LinAlg::Mat3 CalcInertiaTensor(const Collision::AABB& Aabb, const std::vector<LinAlg::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices, const LinAlg::Vec3& CenterOfMass);
	}
	namespace MonteCarlo
	{
		[[nodiscard]] LinAlg::Vec3 CalcCenterOfMass(const Collision::AABB& Aabb, const std::vector<LinAlg::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices);
		[[nodiscard]] LinAlg::Mat3 CalcInertiaTensor(const Collision::AABB& Aabb, const std::vector<LinAlg::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices, const LinAlg::Vec3& CenterOfMass);
	}
	namespace Tetrahedron 
	{
		[[nodiscard]] LinAlg::Vec3 CalcCenterOfMass(const std::vector<LinAlg::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices);
		[[nodiscard]] LinAlg::Mat3 CalcInertiaTensor(const LinAlg::Vec3& A, const LinAlg::Vec3& B, const LinAlg::Vec3& C, const LinAlg::Vec3& D);
		[[nodiscard]] LinAlg::Mat3 CalcInertiaTensor(const std::vector<LinAlg::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices, const LinAlg::Vec3& CenterOfMass);
	}
}
