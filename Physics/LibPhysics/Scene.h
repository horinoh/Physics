#pragma once

#include <vector>

#include "Collision.h"

//#define USE_BRUTE_FORCE

namespace Physics
{
	class Shape;
	class RigidBody;

	class Scene 
	{
	public:
		using CollidablePair = std::pair<int, int>;

		virtual ~Scene();
		
#ifdef USE_BRUTE_FORCE
		virtual void BruteForce(const float DeltaSec, std::vector<Contact>& Contacts);
#else
		//!< SAP (Sweep And Prune)
		virtual void BroadPhase(const float DeltaSec, std::vector<CollidablePair>& CollidablePairs);
		virtual void NarrowPhase(const float DeltaSec, const std::vector<CollidablePair>& CollidablePairs, std::vector<Collision::Contact>& Contacts);
#endif
		virtual void Update(const float DeltaSec);

		std::vector<Physics::Shape*> Shapes;
		std::vector<Physics::RigidBody*> RigidBodies;
	};
}