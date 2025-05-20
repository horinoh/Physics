#pragma once

#include <vector>

#include "Collision.h"
#include "Constraint.h"
#include "Util.h"

//#define USE_BRUTE_FORCE

namespace Physics
{
	class Shape;
	class RigidBody;
	class Constraint;

	class Scene 
	{
	public:
		using CollidablePair = std::pair<int, int>;

		//virtual ~Scene() { for (auto& i : RigidBodies) { i.reset(); } }
	
		//!< BruteForce の場合チェックする回数 (Check count on brute force)
		int BruteForceCount() const {
			auto Index = 0;
			return std::accumulate(std::cbegin(RigidBodies), std::cend(RigidBodies), 0,
				[&](const auto& Acc, const auto& rhs) {
					return Acc + Index++;
				});
		}
#ifdef USE_BRUTE_FORCE
		virtual void BruteForce(const float DeltaSec, std::vector<Collision::Contact>& Contacts);
#else
		//!< SAP (Sweep And Prune)
		virtual void BroadPhase(const float DeltaSec, std::vector<CollidablePair>& CollidablePairs);
		virtual void NarrowPhase(const float DeltaSec, const std::vector<CollidablePair>& CollidablePairs, std::vector<Collision::Contact>& Contacts);
#endif

		virtual void SolveConstraint(const float DeltaSec, const uint32_t ItCount);

		virtual void Update(const float DeltaSec);

		static const Math::Vec3 BroadPhaseAxis;

		std::vector<std::unique_ptr<Physics::Shape>> Shapes;
		std::vector<std::unique_ptr<Physics::RigidBody>> RigidBodies;

		// 静的なコンストレイント (距離、ヒンジ、...)
		std::vector<std::unique_ptr<Physics::Constraint>> Constraints;
		// 動的なコンストレイント (貫通)
		Physics::ManifoldCollector Manifolds;
	};
}