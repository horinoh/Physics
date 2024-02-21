#pragma once

#include <vector>
#include <algorithm>

#include "RigidBody.h"
#include "Collision.h"

using namespace Math;
using namespace Collision;

namespace Physics
{
	class Scene 
	{
	public:
		using CollidablePair = std::pair<int, int>;

		virtual ~Scene() {
			for (auto i : RigidBodies) {
				if (nullptr != i) {
					delete i;
				}
			}
		}

		virtual void BroadPhase(std::vector<CollidablePair>& CollidablePairs, const float DeltaSec)
		{
			std::vector<BoundEdge> BoundEdges;
			{
				//!< (�����ł�) ���̎��Ɏˉe���� AABB �͈̔͂��v�Z
				const auto Axis = Vec3(1.0f, 1.0f, 1.0f).Normalize();

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
					Aabb.Expand(Aabb.Min - Vec3::One() * Epsilon);
					Aabb.Expand(Aabb.Max + Vec3::One() * Epsilon);

					BoundEdges.emplace_back(BoundEdge({ i, Axis.Dot(Aabb.Min), true })); //!< ����
					BoundEdges.emplace_back(BoundEdge({ i, Axis.Dot(Aabb.Max), false }));//!< ���
				}

				//!< ���Ɏˉe�����l�Ń\�[�g
				std::ranges::sort(BoundEdges, std::ranges::less{}, &BoundEdge::Value);
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
		virtual void NarrowPhase(std::vector<Contact>& Contacts, const std::vector<CollidablePair>& CollidablePairs, const float DeltaSec)
		{
			Contacts.reserve(std::size(CollidablePairs));
			Contacts.clear();

			//!< ���ݓI�Փˑ���ƁA���ۂɏՓ˂��Ă��邩�𒲂ׂ�
			for (auto i : CollidablePairs) {
				const auto RbA = RigidBodies[i.first];
				const auto RbB = RigidBodies[i.second];
				if (0.0f != RbA->InvMass || 0.0f != RbB->InvMass) {
					Contact Ct;
					if (Collision::Intersection::RigidBodies(RbA, RbB, DeltaSec, Ct)) {
						//!< �Փ˂����W
						Contacts.emplace_back(Ct);
					}
				}
			}
			//!< TOI �Ń\�[�g
			std::ranges::sort(Contacts, std::ranges::less{}, &Contact::TimeOfImpact);
		}
		virtual void BruteForce(std::vector<Contact>& Contacts, const float DeltaSec)
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
						if (Collision::Intersection::RigidBodies(RbA, RbB, DeltaSec, Ct)) {
							Contacts.emplace_back(Ct);
						}
					}
				}
			}
			std::ranges::sort(Contacts, std::ranges::less{}, &Contact::TimeOfImpact);
		}
		virtual void Update(const float DeltaSec)
		{
			//!< �d��
			for (auto i : RigidBodies) {
				i->ApplyGravity(DeltaSec);
			}

			//!< �Փ� Contacts �����W
			std::vector<Contact> Contacts;
#if true
			{
				std::vector<CollidablePair> CollidablePairs;
				BroadPhase(CollidablePairs, DeltaSec);
				NarrowPhase(Contacts, CollidablePairs, DeltaSec);
			}
#else
			BruteForce(Contacts, DeltaSec);
#endif

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

		std::vector<RigidBody *> RigidBodies;
	};
}