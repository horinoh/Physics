#include <algorithm>
#include <vector>

#include "Scene.h"
#include "Shape.h"
#include "RigidBody.h"
#include "Constraint.h"

#include "Log.h"

const Math::Vec3 Physics::Scene::BroadPhaseAxis = Math::Vec3::One().Normalize();

#ifdef USE_BRUTE_FORCE
void Physics::Scene::BruteForce(const float DeltaSec, std::vector<Collision::Contact>& Contacts)
{
	const auto RbCount = std::size(RigidBodies);
	Contacts.reserve(RbCount * RbCount);
	Contacts.clear();
	for (auto i = 0; i < RbCount; ++i) {
		for (auto j = i + 1; j < RbCount; ++j) {
			const auto RbA = RigidBodies[i];
			const auto RbB = RigidBodies[j];
			if (0.0f != RbA->InvMass || 0.0f != RbB->InvMass) {
				Collision::Contact Ct;
				if (Collision::Intersection::RigidBodyRigidBody(RbA, RbB, DeltaSec, Ct)) {
					if (0.0f == Ct.TimeOfImpact) {
						Manifolds.Add(Ct);
					}
					else {
						Contacts.emplace_back(Ct);
					}
				}
			}
		}
	}
	std::ranges::sort(Contacts, std::ranges::less{}, &Collision::Contact::TimeOfImpact);
}
#else
//!< SAP (Sweep And Prune)
void Physics::Scene::BroadPhase(const float DeltaSec, std::vector<CollidablePair>& CollidablePairs)
{
	std::vector<Collision::BoundEdge> BoundEdges;
	{
		const auto RbCount = std::size(RigidBodies);
		BoundEdges.reserve(RbCount * 2);
		for (auto i = 0; i < RbCount; ++i) {
			const auto Rb = RigidBodies[i].get();

			auto Aabb = Rb->Shape->GetAABB(Rb->Position, Rb->Rotation);
			//!< �f���^���ԂɈړ����镪���� AABB ���g������
			const auto DVel = Rb->LinearVelocity * DeltaSec;
			Aabb.Expand(Aabb.Min + DVel);
			Aabb.Expand(Aabb.Max + DVel);

			//!< ����ɏ����g�� (��肱�ڂ��h�~�H)
			const auto Epsilon = 0.01f;
			const auto DEp = Math::Vec3::One() * Epsilon;
			Aabb.Expand(Aabb.Min - DEp);
			Aabb.Expand(Aabb.Max + DEp);

			//!< ���E�[ (����A����) ���o���Ă���
			BoundEdges.emplace_back(Collision::BoundEdge({ i, BroadPhaseAxis.Dot(Aabb.Min), true })); //!< ����
			BoundEdges.emplace_back(Collision::BoundEdge({ i, BroadPhaseAxis.Dot(Aabb.Max), false }));//!< ���
		}

		//!< ���Ɏˉe�������ϒl�Ń\�[�g
		std::ranges::sort(BoundEdges, std::ranges::less{}, &Collision::BoundEdge::Value);
	}

	//!< �ˉe AABB ����A���ݓI�Փ˃y�A���X�g���\�z
	const auto BoundsCount = std::size(BoundEdges);
	for (auto i = 0; i < BoundsCount; ++i) {
		const auto& A = BoundEdges[i];
		//!< �����Ȃ�΂ƂȂ�����������܂ŒT��
		if (A.IsMin) {
			for (auto j = i + 1; j < BoundsCount; ++j) {
				const auto& B = BoundEdges[j];
				//!< �΂ƂȂ��� (�����C���f�b�N�X) ��������ΏI��
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
	Contacts.clear();
	Contacts.reserve(std::size(CollidablePairs));

	//!< ���ݓI�Փˑ���ƁA���ۂɏՓ˂��Ă��邩�𒲂ׂ�
	for (const auto& i : CollidablePairs) {
		const auto RbA = RigidBodies[i.first].get();
		const auto RbB = RigidBodies[i.second].get();
		
		if (0.0f == RbA->InvMass && 0.0f == RbB->InvMass) {
			continue;
		}

		Collision::Contact Ct;
		if (RbA->Shape->GetShapeType() == Physics::Shape::SHAPE_TYPE::SPHERE && RbB->Shape->GetShapeType() == Physics::Shape::SHAPE_TYPE::SPHERE) {
			if (Collision::Intersection::SphereSphere(RbA, RbB, DeltaSec, Ct)) {
				Contacts.emplace_back(Ct);
			}
		}
		else {
			if (Collision::Intersection::RigidBodyRigidBody(RbA, RbB, DeltaSec, Ct)) {
				if (0.0f == Ct.TimeOfImpact) {
					//!< �ÓI�Փ�
					Manifolds.Add(Ct);
				}
				else {
					//!< ���I�Փ�
					Contacts.emplace_back(Ct);
				}
			}
		}
	}
	//!< TOI �Ń\�[�g
	std::ranges::sort(Contacts, std::ranges::less{}, &Collision::Contact::TimeOfImpact);
}
#endif

// �ÓI�R���X�g���C���g�A���I�R���X�g���C���g������
void Physics::Scene::SolveConstraint(const float DeltaSec, const uint32_t ItCount)
{
	for (auto& i : Constraints) {
		i->PreSolve(DeltaSec);
	}
	Manifolds.PreSolve(DeltaSec);

	//!< Solve() �͌J��Ԃ����ƂŎ��������� (�P�x�ɂ͂Q���̊Ԃ̃R���X�g���C���g�����������Ȃ���)
	for (uint32_t c = 0; c < ItCount; ++c) {
		for (auto& i : Constraints) {
			i->Solve();
		}
		Manifolds.Solve();
	}

	for (auto& i : Constraints) {
		i->PostSolve();
	}
	Manifolds.PostSolve();
}

void Physics::Scene::Update(const float DeltaSec)
{
	Manifolds.RemoveExpired();

	//!< �d��
	for (auto& i : RigidBodies) {
		i->ApplyGravity(DeltaSec);
	}

	//!< �Փ� Contacts �����W
	std::vector<Collision::Contact> Contacts;
	{
#ifdef USE_BRUTE_FORCE
		//!< �������� (�u���[�g�t�H�[�X)
		BruteForce(DeltaSec, Contacts);
#else
		//!< �u���[�h�t�F�[�Y�A�i���[�t�F�[�Y
		std::vector<CollidablePair> CollidablePairs;
		BroadPhase(DeltaSec, CollidablePairs);
#ifdef _DEBUG
		{
			//!< BruteForce �ɔ�ׂĂǂ̂��炢�팸�ł��Ă��邩
			const auto BFC = BruteForceCount();
			LOG(std::data(std::format("Collidable / BruteForce = {} / {} = {} %\n", std::size(CollidablePairs), BFC, std::size(CollidablePairs) * 100 / BFC)));
		}
#endif
		NarrowPhase(DeltaSec, CollidablePairs, Contacts);
#endif
	}

	//!< �R���X�g���C���g�̉���
	SolveConstraint(DeltaSec, 5);

	//!< TOI ���Ɏ��Ԃ��X���C�X���āA�V�~�����[�V������i�߂�
	auto AccumTime = 0.0f;
	for (const auto& i : Contacts) {
		//!< ���̏Փ˂܂ł̎���
		const auto DeltaTime = i.TimeOfImpact - AccumTime;

		//!< ���̏Փ˂܂ŃV�~�����[�V������i�߂�
		for (auto& j : RigidBodies) {
			j->Update(DeltaTime);
		}

		//!< �Փ˂̉���
		ResolveContact(i);

		AccumTime += DeltaTime;
	}

	//!< �c��̃V�~�����[�V������i�߂�
	const auto DeltaTime = DeltaSec - AccumTime;
	if (0.0f < DeltaTime) {
		for (auto& i : RigidBodies) {
			i->Update(DeltaTime);
		}
	}

	//!< ���������h�~
#if 1
	for (auto& i : RigidBodies) {
		if (i->Position.Y() < -100.0f) {
			i->Position[1] = -100.0f;
			i->InvMass = 0.0f;
			i->LinearVelocity = i->AngularVelocity = Math::Vec3::Zero();
		}
	}
#endif
}