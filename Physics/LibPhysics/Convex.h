#pragma once

#include <vector>
#include <algorithm>

#include "LinAlg.h"
#include "RigidBody.h"
#include "Collision.h"

namespace Convex
{
	void BuildTetrahedron(const std::vector<LinAlg::Vec3>& Mesh, std::vector<LinAlg::Vec3>& Vertices, std::vector<Collision::TriInds >& Indices);

	//!< 指定の点が凸包の内部点かどうか
	[[nodiscard]] static bool IsInternal(const LinAlg::Vec3& Pt, const std::vector<LinAlg::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices)
	{
		//!< 全ての三角形に対し、負の側にあれば内部点
		return std::ranges::all_of(Indices,
			[&](const auto rhs) {
				return !Collision::Distance::IsFrontTriangle(Pt, Vertices[rhs[0]], Vertices[rhs[1]], Vertices[rhs[2]]);
			});
	}

	//!< 凸包の内部点を削除
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
