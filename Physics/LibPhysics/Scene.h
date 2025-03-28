#pragma once

#include <vector>

#include "Collision.h"
#include "Constraint.h"

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
		
#ifdef USE_BRUTE_FORCE
		virtual void BruteForce(const float DeltaSec, std::vector<Collision::Contact>& Contacts);
#else
		//!< SAP (Sweep And Prune)
		virtual void BroadPhase(const float DeltaSec, std::vector<CollidablePair>& CollidablePairs);
		virtual void NarrowPhase(const float DeltaSec, const std::vector<CollidablePair>& CollidablePairs, std::vector<Collision::Contact>& Contacts);
#endif

		virtual void SolveConstraint(const float DeltaSec, const uint32_t ItCount);

		virtual void Update(const float DeltaSec);

		std::vector<std::unique_ptr<Physics::Shape>> Shapes;
		std::vector<std::unique_ptr<Physics::RigidBody>> RigidBodies;

		// 静的なコンストレイント (距離、ヒンジ、...)
		std::vector<std::unique_ptr<Physics::Constraint>> Constraints;
		// 動的なコンストレイント (貫通)
		Physics::ManifoldCollector Manifolds;
	};
}