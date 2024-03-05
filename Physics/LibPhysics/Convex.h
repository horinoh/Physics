#pragma once

#include <vector>
#include <algorithm>

#include "PhysicsMath.h"
#include "RigidBody.h"
#include "Collision.h"

namespace Convex
{
	void BuildTetrahedron(const std::vector<Math::Vec3>& Pts, std::vector<Math::Vec3>& Vertices, std::vector<Collision::TriInds >& Indices);

	//!< 指定の点が凸包の内部点かどうか
	[[nodiscard]] static bool IsInternal(const Math::Vec3& Pt, const std::vector<Math::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices)
	{
		//!< 全ての三角形に対し、負の側にあれば内部点
		return std::ranges::all_of(Indices, [&](const auto rhs) {
			return !Collision::Distance::IsFront(Pt, Vertices[rhs[0]], Vertices[rhs[1]], Vertices[rhs[2]]);
		});
	}

	//!< 凸包の内部点を削除
	void RemoveInternal(const std::vector<Math::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices, std::vector<Math::Vec3>& Pts);
	
	void CollectUniqueEdges(std::vector<Collision::TriInds>::const_iterator Begin, std::vector<Collision::TriInds>::const_iterator End, std::vector<Collision::EdgeIndsCount>& EdgeCounts);
	static void CollectUniqueEdges(const std::vector<Collision::TriInds>& Indices, std::vector<Collision::EdgeIndsCount>& EdgeCounts) { CollectUniqueEdges(std::ranges::cbegin(Indices), std::ranges::cend(Indices), EdgeCounts); }

	void BuildConvexHull(const std::vector<Math::Vec3>& Pts, std::vector<Math::Vec3>& Vertices, std::vector<Collision::TriInds>& Indices);

	namespace Uniform {
		//!< #TODO 要検証
		[[nodiscard]] Math::Vec3 CalcCenterOfMass(const Collision::AABB& Aabb, const std::vector<Math::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices);
		//!< #TODO 要検証
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
