#include <random>

#include "Convex.h"
#include "Log.h"

//!< ���_���u�Ȃ�ׂ��v��܂���悤�Ȏl�ʑ̂��쐬
void Convex::BuildTetrahedron(const std::vector<Math::Vec3>& Pts, std::vector<Math::Vec3>& Vertices, std::vector<Collision::TriInds >& Indices)
{
	//!< ����̎� (�����ł�X) �Ɉ�ԉ����_
	std::array<Math::Vec3, 4> P = { *Collision::Distance::Farthest(Pts, Math::Vec3::AxisX()) };
	//< �O�o�̋t�����̎����Ɉ�ԉ����_
	P[1] = *Collision::Distance::Farthest(Pts, -P[0]);
	//!< �O�o�� 2 �_����Ȃ�����Ɉ�ԉ����_
	P[2] = *Collision::Distance::Farthest(Pts, P[0], P[1]);
	//!< �O�o�� 3 �_����Ȃ�O�p�`�Ɉ�ԉ����_
	P[3] = *Collision::Distance::Farthest(Pts, P[0], P[1], P[2]);

	//!< CCW �ɂȂ�悤�ɒ���
	if (Collision::Distance::IsFront(P[0], P[1], P[2], P[3])) {
		std::swap(P[0], P[1]);
	}

	//!< �l�ʑ̂̒��_
	Vertices.emplace_back(P[0]);
	Vertices.emplace_back(P[1]);
	Vertices.emplace_back(P[2]);
	Vertices.emplace_back(P[3]);

	//!< �l�ʑ̂̃C���f�b�N�X
	Indices.emplace_back(Collision::TriInds({ 0, 1, 2 }));
	Indices.emplace_back(Collision::TriInds({ 0, 2, 3 }));
	Indices.emplace_back(Collision::TriInds({ 2, 1, 3 }));
	Indices.emplace_back(Collision::TriInds({ 1, 0, 3 }));
}

//!< �ʕ�̓����_���폜
void Convex::RemoveInternal(const std::vector<Math::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices, std::vector<Math::Vec3>& Pts)
{
	//!< �����_�����O
	{
		const auto Range = std::ranges::remove_if(Pts, [&](const auto& Pt) {
			return IsInternal(Pt, Vertices, Indices);
			});
		Pts.erase(std::ranges::cbegin(Range), std::ranges::cend(Range));
	}

	//!< �����Ɠ���Ƃ݂Ȃ���_�͏��O
	{
		const auto Range = std::ranges::remove_if(Pts, [&](const auto& Pt) {
			//!< ����Ƃ݂Ȃ���_
			return std::ranges::any_of(Vertices, [&](const auto rhs) {
				return rhs.NearlyEqual(Pt);
				});
			});
		Pts.erase(std::ranges::cbegin(Range), std::ranges::cend(Range));
	}
}

void Convex::CollectUniqueEdges(std::vector<Collision::TriInds>::const_iterator Begin, std::vector<Collision::TriInds>::const_iterator End, std::vector<Collision::EdgeIndsCount>& EdgeCounts)
{
	std::for_each(Begin, End, [&](const auto& i) {
		const std::array Edges = {
			Collision::EdgeInds({ i[0], i[1] }),
			Collision::EdgeInds({ i[1], i[2] }),
			Collision::EdgeInds({ i[2], i[0] }),
		};
		std::ranges::for_each(Edges, [&](const auto& j) {
			//!< ���o�̕ӂ��ǂ����𒲂ׂ� (�^�t�����o�Ƃ��Ĉ���)
			const auto It = std::ranges::find_if(EdgeCounts, [&](const auto& k) {
				return (k.first[0] == j[0] && k.first[1] == j[1]) || (k.first[0] == j[1] && k.first[1] == j[0]);
			});
			if (std::cend(EdgeCounts) == It) {
				//!< �V�K�̕ӂƂ��Ēǉ�
				EdgeCounts.emplace_back(Collision::EdgeIndsCount({ j, 0 }));
			}
			else {
				//!< ���o�̕ӂƂȂ�����J�E���^���C���N�������g���ď����X�V
				++It->second;
			}
		});
	});

	//!< ���j�[�N�łȂ��� (�J�E���^�� 0 ���傫��) ���폜
	const auto Range = std::ranges::remove_if(EdgeCounts, [](const auto& i) { return i.second > 0; });
	EdgeCounts.erase(std::ranges::cbegin(Range), std::ranges::cend(EdgeCounts));
}

//!< �n�C�|����H�킹��Ƃ��Ȃ莞�Ԃ��������Ɍ��ǃn�C�|���̓ʕ�ł��邾���Ȃ̂ŃR���W�����Ƃ��Č����I�ł͂Ȃ��A���[�|����H�킹�邱��
void Convex::BuildConvexHull(const std::vector<Math::Vec3>& Pts, std::vector<Math::Vec3>& Vertices, std::vector<Collision::TriInds>& Indices)
{
	LOG(std::data(std::format("Building convex hull...\n")));

	//!< �܂��́u�Ȃ�ׂ��v��܂���悤�Ȏl�ʑ̂��쐬
	BuildTetrahedron(Pts, Vertices, Indices);

	//!< �����_�̏��O -> �O���_���c��
	auto External = Pts;
	RemoveInternal(Vertices, Indices, External);

	//!< �O���_�������Ȃ�܂ŌJ��Ԃ�
	while (!std::empty(External)) {
		LOG(std::data(std::format("Rest vertices = {}\n", size(External))));

		//!< �ŉ��_��������
		const auto ExFarIt = Collision::Distance::Farthest(External, External[0]);

		std::vector<Collision::EdgeIndsCount> DanglingEdges;
		{
			//!< �ŉ��_�������Ă��Ȃ��O�p�` (A) �ƁA�����Ă���O�p�` (B) �ɕ���
			//!< partition �͈ȉ��̂悤�ɕԂ�
			//!<	A �����_���� true	: [begin(Indices), begin(Range)]
			//!<	B �����_���� false	: [begin(Range), end(Range)]
			const auto Range = std::ranges::partition(Indices, [&](const auto& i) {
				return !Collision::Distance::IsFront(*ExFarIt, Vertices[i[0]], Vertices[i[1]], Vertices[i[2]]);
			});

			//!< A, B �̋��E�ƂȂ�悤�ȕӂ����W���� (B �̒����瑼�̎O�p�`�ƕӂ����L���Ȃ����j�[�N�ȕӂ݂̂����W����Ηǂ�)
			CollectUniqueEdges(std::ranges::cbegin(Range), std::ranges::cend(Range), DanglingEdges);

			//!< (�ӂ͎��W��) �����܂ŗ����� B �͍폜���Ă悢  
			Indices.erase(std::ranges::cbegin(Range), std::ranges::cend(Range));
		}

		//!< �ʕ�̍X�V
		{
			//!< �ŉ��_�𒸓_�Ƃ��Ēǉ�����
			Vertices.emplace_back(*ExFarIt);

			//!< �ŉ��_�̃C���f�b�N�X
			const auto FarIndex = static_cast<uint32_t>(std::size(Vertices) - 1);
			//!< �ŉ��_�ƃ��j�[�N�ӂ���Ȃ�O�p�`�Q��ǉ�
			std::ranges::transform(DanglingEdges, std::back_inserter(Indices), [&](const auto& i) {
				return Collision::TriInds({ i.first[0], i.first[1], FarIndex });
			});
		}

		//!< �O���_�̍X�V
		{
			//!< �����܂ōς񂾂�ŉ��_�͍폜���Ă悢
			External.erase(ExFarIt);

			//!< �X�V�����ʕ�ɑ΂��ē����_���폜����
			RemoveInternal(Vertices, Indices, External);
		}
	}
}

Math::Vec3 Convex::Uniform::CalcCenterOfMass(const Collision::AABB& Aabb, const std::vector<Math::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices)
{
	LOG(std::data(std::format("Calculating center of mass (Uniform)...\n")));

	//!< �e���ɃT���v�����O����� (�v�Z�Ɏ��Ԃ�������)
	constexpr auto SampleCount = 100;

	auto CenterOfMass = Math::Vec3::Zero();
	auto Sampled = 0;
	const auto Delta = Aabb.GetExtent() / static_cast<float>(SampleCount);
	for (auto x = 0; x < SampleCount; ++x) {
		for (auto y = 0; y < SampleCount; ++y) {
			for (auto z = 0; z < SampleCount; ++z) {
				//!< AABB ���̃T���v���_
				const auto Pt = Aabb.Min + Math::Vec3(Delta.X() * x, Delta.Y() * y, Delta.Z() * z);
				if (IsInternal(Pt, Vertices, Indices)) {
					//!< �����_�Ȃ���W
					CenterOfMass += Pt;
					++Sampled;
				}
			}
		}
	}
	return CenterOfMass / static_cast<float>(Sampled);
}

Math::Mat3 Convex::Uniform::CalcInertiaTensor(const Collision::AABB& Aabb, const std::vector<Math::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices, const Math::Vec3& CenterOfMass)
{
	LOG(std::data(std::format("Calculating inertia tensor (Uniform)...\n")));

	//!< �e���ɃT���v�����O����� (�v�Z�Ɏ��Ԃ�������)
	constexpr auto SampleCount = 100;

	auto InertiaTensor = Math::Mat3::Zero();
	auto Sampled = 0;
	const auto Delta = Aabb.GetExtent() / static_cast<float>(SampleCount);
	for (auto x = 0; x < SampleCount; ++x) {
		for (auto y = 0; y < SampleCount; ++y) {
			for (auto z = 0; z < SampleCount; ++z) {
				//!< AABB ���̃T���v���_ (�d�S����̑���)
				const auto Pt = Aabb.Min + Math::Vec3(Delta.X() * x, Delta.Y() * y, Delta.Z() * z) - CenterOfMass;
				if (IsInternal(Pt, Vertices, Indices)) {
					//!< �����_�Ȃ���W����
					InertiaTensor += {
						{ Pt.Y() * Pt.Y() + Pt.Z() * Pt.Z(), -Pt.X() * Pt.Y(), -Pt.X() * Pt.Z() },
						{ -Pt.X() * Pt.Y(), Pt.Z() * Pt.Z() + Pt.X() * Pt.X(), -Pt.Y() * Pt.Z() },
						{ -Pt.X() * Pt.Z(), -Pt.Y() * Pt.Z(), Pt.X() * Pt.X() + Pt.Y() * Pt.Y() },
					};
					++Sampled;
				}
			}
		}
	}
	return InertiaTensor / static_cast<float>(Sampled);
}

Math::Vec3 Convex::MonteCarlo::CalcCenterOfMass(const Collision::AABB& Aabb, const std::vector<Math::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices)
{
	LOG(std::data(std::format("Calculating center of mass (Monte carlo)...\n")));

#if 0
	//!< �^�����ŃV�[�h�𐶐�����
	std::random_device SeedGen;
	//!< �^������ (�����Z���k�c�C�X�^�[) (�V�[�h�g�p)
	std::mt19937 MersenneTwister(SeedGen());
#else
	//!< �^������ (�����Z���k�c�C�X�^�[) (���񓯂�)
	std::mt19937 MersenneTwister;
#endif
	//!< ���z
	std::uniform_real_distribution<float> Distribution(0.0f, 1.0f);

	//!< �T���v�����O�����
	constexpr auto SampleCount = 10000;

	auto CenterOfMass = Math::Vec3::Zero();
	auto Sampled = 0;
	const auto Ext = Aabb.GetExtent();
	for (auto i = 0; i < SampleCount; ++i) {
		//!< AABB ���̃T���v���_
		const auto Pt = Aabb.Min + Math::Vec3(Ext.X() * Distribution(MersenneTwister), Ext.Y() * Distribution(MersenneTwister), Ext.Z() * Distribution(MersenneTwister));
		if (IsInternal(Pt, Vertices, Indices)) {
			//!< �����_�Ȃ���W
			CenterOfMass += Pt;
			++Sampled;
		}
	}
	return CenterOfMass / static_cast<float>(Sampled);
}

Math::Mat3 Convex::MonteCarlo::CalcInertiaTensor(const Collision::AABB& Aabb, const std::vector<Math::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices, const Math::Vec3& CenterOfMass)
{
	LOG(std::data(std::format("Calculating inertia tensor (Monte carlo)...\n")));

#if 0
	std::random_device SeedGen;
	std::mt19937 MersenneTwister(SeedGen());
#else
	std::mt19937 MersenneTwister;
#endif
	std::uniform_real_distribution<float> Distribution(0.0f, 1.0f);

	//!< �T���v�����O�����
	constexpr auto SampleCount = 10000;

	auto InertiaTensor = Math::Mat3::Zero();
	auto Sampled = 0;
	const auto Ext = Aabb.GetExtent();
	for (auto i = 0; i < SampleCount; ++i) {
		//!< AABB ���̃T���v���_ (�d�S����̑���)
		const auto Pt = Aabb.Min + Math::Vec3(Ext.X() * Distribution(MersenneTwister), Ext.Y() * Distribution(MersenneTwister), Ext.Z() * Distribution(MersenneTwister)) - CenterOfMass;
		if (IsInternal(Pt, Vertices, Indices)) {
			//!< �����_�Ȃ���W����
			InertiaTensor += {
				{ Pt.Y() * Pt.Y() + Pt.Z() * Pt.Z(), -Pt.X() * Pt.Y(), -Pt.X() * Pt.Z() },
				{ -Pt.X() * Pt.Y(), Pt.Z() * Pt.Z() + Pt.X() * Pt.X(), -Pt.Y() * Pt.Z() },
				{ -Pt.X() * Pt.Z(), -Pt.Y() * Pt.Z(), Pt.X() * Pt.X() + Pt.Y() * Pt.Y() },
			};
			++Sampled;
		}
	}
	return InertiaTensor / static_cast<float>(Sampled);
}

Math::Vec3 Convex::Tetrahedron::CalcCenterOfMass(const std::vector<Math::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices)
{
	LOG(std::data(std::format("Calculating center of mass (Tetrahedron)...\n")));

	const auto MeshCenter = std::accumulate(std::begin(Vertices), std::end(Vertices), Math::Vec3::Zero()) / static_cast<float>(std::size(Vertices));
	auto CenterOfMass = Math::Vec3::Zero();
	auto TotalVolume = 0.0f;
	std::ranges::for_each(Indices, [&](const auto& i) {
		const auto& A = MeshCenter;
		const auto& B = Vertices[i[0]];
		const auto& C = Vertices[i[1]];
		const auto& D = Vertices[i[2]];

		const auto TetraCenter = (A + B + C + D) * 0.25f;
		const auto Volume = Collision::Volume::Tetrahedron(A, B, C, D);

		CenterOfMass += TetraCenter * Volume;
		TotalVolume += Volume;
	});
	return CenterOfMass / TotalVolume;
}
Math::Mat3 Convex::Tetrahedron::CalcInertiaTensor(const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C, const Math::Vec3& D)
{
	//!< �l�ʑ̂� 4 ���_����Ȃ�s��
	const auto M = Math::Mat3(
		Math::Vec3(B.X() - A.X(), C.X() - A.X(), D.X() - A.X()),
		Math::Vec3(B.Y() - A.Y(), C.Y() - A.Y(), D.Y() - A.Y()),
		Math::Vec3(B.Z() - A.Z(), C.Z() - A.Z(), D.Z() - A.Z()));
	const auto Det = M.Determinant();

	auto XX = 0.0f, YY = 0.0f, ZZ = 0.0f, XY = 0.0f, XZ = 0.0f, YZ = 0.0f;
	const std::array Pts = { A, B, C, D };
	for (auto i = 0; i < std::size(Pts); ++i) {
		for (auto j = i; j < std::size(Pts); ++j) {
			//!< �Ίp��
			XX += Pts[i].X() * Pts[j].X();
			YY += Pts[i].Y() * Pts[j].Y();
			ZZ += Pts[i].Z() * Pts[j].Z();

			XY += Pts[i].X() * Pts[j].Y() + Pts[j].X() * Pts[i].Y();
			XZ += Pts[i].X() * Pts[j].Z() + Pts[j].X() * Pts[i].Z();
			YZ += Pts[i].Y() * Pts[j].Z() + Pts[j].Y() * Pts[i].Z();
		}
	}

	return Math::Mat3(
		Math::Vec3(2.0f * (YY + ZZ), -XY, -XZ),
		Math::Vec3(-XY, 2.0f * (XX + ZZ), -YZ),
		Math::Vec3(-XZ, -YZ, 2.0f * (XX + YY))) * Det / 120.0f;
}
Math::Mat3 Convex::Tetrahedron::CalcInertiaTensor(const std::vector<Math::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices, const Math::Vec3& CenterOfMass)
{
	LOG(std::data(std::format("Calculating inertia tensor (Tetrahedron)...\n")));

	auto TotalInertiaTensor = Math::Mat3::Zero();
	auto TotalVolume = 0.0f;
	const auto A = Math::Vec3::Zero(); //!< CenterOfMass - CenterOfMass �Ȃ̂�
	std::ranges::for_each(Indices, [&](const auto& i) {
		const auto B = Vertices[i[0]] - CenterOfMass;
		const auto C = Vertices[i[1]] - CenterOfMass;
		const auto D = Vertices[i[2]] - CenterOfMass;
		TotalInertiaTensor += CalcInertiaTensor(A, B, C, D);
		TotalVolume += Collision::Volume::Tetrahedron(A, B, C, D);
	});
	return TotalInertiaTensor / TotalVolume;
}