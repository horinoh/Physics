#include <algorithm>
#include <vector>

#include "Scene.h"
#include "Shape.h"
#include "RigidBody.h"
#include "Constraint.h"

#include "Log.h"

const LinAlg::Vec3 Physics::Scene::BroadPhaseAxis = LinAlg::Vec3::One().Normalize();

#ifdef USE_BRUTE_FORCE
void Physics::Scene::BruteForce(const float DeltaSec, std::vector<Collision::Contact>& Contacts)
{
	const auto RbCount = std::size(RigidBodies);
	Contacts.reserve(RbCount * RbCount);
	Contacts.clear();
	for (auto i = 0; i < RbCount - 1; ++i) {
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

#if false
			//!< ����ɏ����g�� (��肱�ڂ��h�~�H)
			const auto Epsilon = 0.01f;
			const auto DEp = LinAlg::Vec3::One() * Epsilon;
			Aabb.Expand(Aabb.Min - DEp);
			Aabb.Expand(Aabb.Max + DEp);
#endif

			//!< ���E�[ (����A����) ���o���Ă���
			BoundEdges.emplace_back(Collision::BoundEdge({ i, BroadPhaseAxis.Dot(Aabb.Min), true })); //!< ����
			BoundEdges.emplace_back(Collision::BoundEdge({ i, BroadPhaseAxis.Dot(Aabb.Max), false }));//!< ���
		}

		//!< ���Ɏˉe�������ϒl�Ń\�[�g
		std::ranges::sort(BoundEdges, std::ranges::less{}, &Collision::BoundEdge::Value);
	}

	//!< �ˉe AABB ����A���ݓI�Փ˃y�A���X�g���\�z
	const auto BoundsCount = std::size(BoundEdges);
	for (auto i = 0; i < BoundsCount - 1; ++i) {
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
#if true
		//!< �����m�� GJK �ł͂Ȃ��A�J�X�^���Փ˔���
		if (RbA->Shape->GetShapeType() == Physics::Shape::SHAPE_TYPE::SPHERE && RbB->Shape->GetShapeType() == Physics::Shape::SHAPE_TYPE::SPHERE) {
			if (Collision::Intersection::SphereSphere(RbA, RbB, DeltaSec, Ct)) {
				Contacts.emplace_back(Ct);
			}	
		}
		//!< ���ȊO�� GJK �ŏՓ˔���
		else {
			if (Collision::Intersection::ConservativeAdvance(RbA, RbB, DeltaSec, Ct)) {
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
#else
		//!< �S�� GJK �ŏՓ˔���
		if (Collision::Intersection::RigidBodyRigidBody(RbA, RbB, DeltaSec, Ct)) {
			if (0.0f == Ct.TimeOfImpact) {
				Manifolds.Add(Ct);
			}
			else {
				Contacts.emplace_back(Ct);
			}
		}
#endif
	}
	//!< TOI �Ń\�[�g
	std::ranges::sort(Contacts, std::ranges::less{}, &Collision::Contact::TimeOfImpact);
}
#endif

//!< �R���X�g���C���g������ 
//!< GJK �g�p���̊ђʉ����� Manifolds �R���X�g���C���g�����ɋA��
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
//!< �ђʉ��� (GJK �s�g�p��)
//!< GJK �s�g�p���� TOI == 0 �� Contacts �֒ǉ�����Ă���̂ŁA�����ŉ�������
void Physics::Scene::SolvePenetration(std::span<Collision::Contact> Contacts)
{
	for (const auto& i : Contacts) {
		//!< �� GJK �̏ꍇ�݂̂����Ŋђʉ������s��
		if (0.0f == i.TimeOfImpact) {
			const auto TotalInvMass = i.RigidBodyA->InvMass + i.RigidBodyB->InvMass;
			const auto DistAB = i.WPointB - i.WPointA;
			if (0.0f != i.RigidBodyA->InvMass) {
				i.RigidBodyA->Position += DistAB * (i.RigidBodyA->InvMass / TotalInvMass);
			}
			if (0.0f != i.RigidBodyB->InvMass) {
				i.RigidBodyB->Position -= DistAB * (i.RigidBodyB->InvMass / TotalInvMass);
			}
		}
	}
}

void Physics::Scene::ApplyImpulse(const Collision::Contact& Ct)
{
	const auto& WPointA = Ct.WPointA;
	const auto& WPointB = Ct.WPointB;

	const auto TotalInvMass = Ct.RigidBodyA->InvMass + Ct.RigidBodyB->InvMass;
	{
		//!< ���a (�d�S -> �Փ˓_)
		const auto RA = WPointA - Ct.RigidBodyA->GetWorldCenterOfMass();
		const auto RB = WPointB - Ct.RigidBodyB->GetWorldCenterOfMass();
		{
			//!< �t�����e���\�� (���[���h�X�y�[�X)
			const auto InvWITA = Ct.RigidBodyA->GetWorldInverseInertiaTensor();
			const auto InvWITB = Ct.RigidBodyB->GetWorldInverseInertiaTensor();

			//!< (A ���_��) ���Α��x
			const auto VelA = Ct.RigidBodyA->LinearVelocity + Ct.RigidBodyA->AngularVelocity.Cross(RA);
			const auto VelB = Ct.RigidBodyB->LinearVelocity + Ct.RigidBodyB->AngularVelocity.Cross(RB);
			const auto RelVelA = VelA - VelB;

			//!< �@���A�ڐ������̗͐� J ��K�p����̋��ʏ���
			auto Apply = [&](const auto& Axis, const auto& Vel, const float Coef) {
				const auto AngJA = (InvWITA * RA.Cross(Axis)).Cross(RA);
				const auto AngJB = (InvWITB * RB.Cross(Axis)).Cross(RB);
				const auto AngFactor = (AngJA + AngJB).Dot(Axis);
				const auto J = Vel * Coef / (TotalInvMass + AngFactor);
				Ct.RigidBodyA->ApplyImpulse(WPointA, -J);
				Ct.RigidBodyB->ApplyImpulse(WPointB, J);
				};

			//!< �@������ �͐�J (�^���ʕω�)
			const auto& Nrm = Ct.WNormal;
			const auto VelN = Nrm * RelVelA.Dot(Nrm);
			//!< �����ł͗��҂̒e���W�����|���������̊ȈՂȎ����Ƃ���
			const auto TotalElas = 1.0f + Ct.RigidBodyA->Elasticity * Ct.RigidBodyB->Elasticity;
			Apply(Nrm, VelN, TotalElas);

			//!< �ڐ����� �͐�J (���C��)
			const auto VelT = RelVelA - VelN;
			const auto Tan = VelT.Normalize();
			//!< �����ł͗��҂̖��C�W�����|���������̊ȈՂȎ����Ƃ���
			const auto TotalFric = Ct.RigidBodyA->Friction * Ct.RigidBodyB->Friction;
			Apply(Tan, VelT, TotalFric);
		}
	}
}

void Physics::Scene::Update(const float DeltaSec)
{
	PERFORMANCE_COUNTER_FUNC();

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

	//!< �R���X�g���C���g�̉��� (�ђʉ������܂�)
	SolveConstraint(DeltaSec, 5);
	//!< GJK �s�g�p���̊ђʉ���
	SolvePenetration(Contacts);
	
	//!< TOI ���Ɏ��Ԃ��X���C�X���āA�V�~�����[�V������i�߂�
	auto AccumTime = 0.0f;
	for (const auto& i : Contacts) {
		//!< ���̏Փ˂܂ł̎���
		const auto Delta = i.TimeOfImpact - AccumTime;

		//!< ���̏Փ˂܂ŃV�~�����[�V������i�߂�
		for (auto& j : RigidBodies) {
			j->Update(Delta);
		}

		//!< �Փ˂ɂ��͐ς̓K�p
		ApplyImpulse(i);

		AccumTime += Delta;
	}
	//!< �c��̃V�~�����[�V������i�߂�
	const auto Delta = DeltaSec - AccumTime;
	if (0.0f < Delta) {
		for (auto& i : RigidBodies) {
			i->Update(Delta);
		}
	}

	//!< ���������h�~
#if 1
	for (auto& i : RigidBodies) {
		if (i->Position.Y() < -100.0f) {
			i->Position[1] = -100.0f;
			i->InvMass = 0.0f;
			i->LinearVelocity = i->AngularVelocity = LinAlg::Vec3::Zero();
		}
	}
#endif
}

//PerformanceCounter::~PerformanceCounter() 
//{
//	const auto End = std::chrono::system_clock::now();
//	const auto MilliSec = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(End - Start).count() / 1000.0);
//	LOG(std::data(std::format("{} msec\n", MilliSec)));
//}