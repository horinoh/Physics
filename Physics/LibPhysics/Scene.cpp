#include <algorithm>
#include <vector>

#include "Scene.h"
#include "Shape.h"
#include "RigidBody.h"
#include "Constraint.h"

Physics::Scene::~Scene() 
{
	for (auto i : Shapes) {
		if (nullptr != i) {
			delete i;
		}
	}
	for (auto i : RigidBodies) {
		if (nullptr != i) {
			delete i;
		}
	}
	for (auto i : Constraints) {
		if (nullptr != i) {
			delete i;
		}
	}
}

#ifdef USE_BRUTE_FORCE
void Scene::BruteForce(const float DeltaSec, std::vector<Contact>& Contacts)
{
	const auto RbCount = size(RigidBodies);
	Contacts.reserve(RbCount * RbCount);
	Contacts.clear();
	for (auto i = 0; i < RbCount; ++i) {
		for (auto j = i + 1; j < RbCount; ++j) {
			const auto RbA = RigidBodies[i];
			const auto RbB = RigidBodies[j];
			if (0.0f != RbA->InvMass || 0.0f != RbB->InvMass) {
				Contact Ct;
				if (Collision::Intersection::RigidBodyRigidBody(RbA, RbB, DeltaSec, Ct)) {
					Contacts.emplace_back(Ct);
				}
			}
		}
	}
	std::ranges::sort(Contacts, std::ranges::less{}, &Contact::TimeOfImpact);
}
#else
//!< SAP (Sweep And Prune)
void Physics::Scene::BroadPhase(const float DeltaSec, std::vector<CollidablePair>& CollidablePairs)
{
	std::vector<Collision::BoundEdge> BoundEdges;
	{
		//!< (�����ł�) ���̎��Ɏˉe���� AABB �͈̔͂��v�Z
		const auto Axis = Math::Vec3(1.0f, 1.0f, 1.0f).Normalize();

		const auto RbCount = size(RigidBodies);
		BoundEdges.reserve(RbCount * 2);
		for (auto i = 0; i < RbCount; ++i) {
			const auto Rb = RigidBodies[i];

			auto Aabb = Rb->Shape->GetAABB(Rb->Position, Rb->Rotation);
			//!< ���x�� AABB ���g������
			Aabb.Expand(Aabb.Min + Rb->LinearVelocity * DeltaSec);
			Aabb.Expand(Aabb.Max + Rb->LinearVelocity * DeltaSec);

			//!< ����ɏ����g�� (��肱�ڂ��h�~�H)
			const auto Epsilon = 0.01f;
			Aabb.Expand(Aabb.Min - Math::Vec3::One() * Epsilon);
			Aabb.Expand(Aabb.Max + Math::Vec3::One() * Epsilon);

			BoundEdges.emplace_back(Collision::BoundEdge({ i, Axis.Dot(Aabb.Min), true })); //!< ����
			BoundEdges.emplace_back(Collision::BoundEdge({ i, Axis.Dot(Aabb.Max), false }));//!< ���
		}

		//!< ���Ɏˉe�����l�Ń\�[�g
		std::ranges::sort(BoundEdges, std::ranges::less{}, &Collision::BoundEdge::Value);
	}

	//!< �ˉe AABB ����A���ݓI�Փ˃y�A���X�g���\�z
	const auto BoundsCount = size(BoundEdges);
	for (auto i = 0; i < BoundsCount; ++i) {
		const auto& A = BoundEdges[i];
		//!< Min �Ȃ�΂ƂȂ� Max ��������܂ŒT��
		if (A.IsMin) {
			for (auto j = i + 1; j < BoundsCount; ++j) {
				const auto& B = BoundEdges[j];
				//!< �΂ƂȂ� Max (�����C���f�b�N�X) ��������ΏI��
				if (A.Index == B.Index) {
					break;
				}
				//!< ���I�u�W�F�N�g�����������ꍇ�́A���ݓI�Փˑ���Ƃ��Ď��W
				if (B.IsMin) {
					CollidablePairs.emplace_back(CollidablePair({ A.Index, B.Index }));
				}
			}
		}
	}
}
void Physics::Scene::NarrowPhase(const float DeltaSec, const std::vector<CollidablePair>& CollidablePairs, std::vector<Collision::Contact>& Contacts)
{
	Contacts.reserve(std::size(CollidablePairs));
	Contacts.clear();

	//!< ���ݓI�Փˑ���ƁA���ۂɏՓ˂��Ă��邩�𒲂ׂ�
	for (auto i : CollidablePairs) {
		const auto RbA = RigidBodies[i.first];
		const auto RbB = RigidBodies[i.second];
		if (0.0f != RbA->InvMass || 0.0f != RbB->InvMass) {
			Collision::Contact Ct;
			if (Collision::Intersection::RigidBodyRigidBody(RbA, RbB, DeltaSec, Ct)) {
				//!< �Փ˂����W
				Contacts.emplace_back(Ct);
			}
		}
	}
	//!< TOI �Ń\�[�g
	std::ranges::sort(Contacts, std::ranges::less{}, &Collision::Contact::TimeOfImpact);
}
#endif

void Physics::Scene::SolveConstraint(const float DeltaSec, const uint32_t ItCount)
{
	for (auto i : Constraints) {
		i->PreSolve(DeltaSec);
	}

	//!< �P�x�ɂ͂Q���̊Ԃ̃R���X�g���C���g�����������Ȃ��̂ŁA�J��Ԃ��Ȃ��Ǝ������Ȃ�
	for (uint32_t c = 0; c < ItCount; ++c) {
		for (auto i : Constraints) {
			i->Solve();
		}
	}

	for (auto i : Constraints) {
		i->PostSolve();
	}
}

void Physics::Scene::Update(const float DeltaSec)
{
	//!< �d��
	for (auto i : RigidBodies) {
		i->ApplyGravity(DeltaSec);
	}

	//!< �Փ� Contacts �����W
	std::vector<Collision::Contact> Contacts;
#ifdef USE_BRUTE_FORCE
	BruteForce(DeltaSec, Contacts);
#else
	{
		std::vector<CollidablePair> CollidablePairs;
		BroadPhase(DeltaSec, CollidablePairs);
		NarrowPhase(DeltaSec, CollidablePairs, Contacts);
	}
#endif

	//!< �R���X�g���C���g�̉���
	SolveConstraint(DeltaSec, 5);

	//!< TOI ���Ɏ��Ԃ��X���C�X���āA�V�~�����[�V������i�߂�
	auto AccumTime = 0.0f;
	for (auto i : Contacts) {
		//!< ���̏Փ˂܂ł̎���
		const auto Delta = i.TimeOfImpact - AccumTime;

		//!< ���̏Փ˂܂ŃV�~�����[�V������i�߂�
		for (auto j : RigidBodies) {
			j->Update(Delta);
		}

		//!< �Փ˂̉���
		Resolve(i);

		AccumTime += Delta;
	}

	//!< �c��̃V�~�����[�V������i�߂�
	const auto Delta = DeltaSec - AccumTime;
	if (0.0f < Delta) {
		for (auto i : RigidBodies) {
			i->Update(Delta);
		}
	}
}