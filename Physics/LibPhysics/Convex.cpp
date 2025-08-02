#include <random>

#include "Convex.h"
#include "Log.h"

//!< ���_���u�Ȃ�ׂ��v��܂���悤�Ȏl�ʑ̂��쐬
void Convex::BuildTetrahedron(const std::vector<LinAlg::Vec3>& Mesh, std::vector<LinAlg::Vec3>& Vertices, std::vector<Collision::TriInds>& Indices)
{
	//!< ����̎� (�����ł�X) �Ɉ�ԉ����_
	std::array<LinAlg::Vec3, 4> Pts = { *Collision::Distance::Farthest(Mesh, LinAlg::Vec3::AxisX()) };
	//< �O�o�̋t�����Ɉ�ԉ����_
	Pts[1] = *Collision::Distance::Farthest(Mesh, -Pts[0]);
	//!< �O�o�� 2 �_����Ȃ�����Ɉ�ԉ����_
	Pts[2] = *Collision::Distance::Farthest(Mesh, Pts[0], Pts[1] - Pts[0]);
	//!< �O�o�� 3 �_����Ȃ�O�p�`�Ɉ�ԉ����_
	Pts[3] = *Collision::Distance::Farthest(Mesh, Pts[0], Pts[1], Pts[2]);

	//!< CCW �ɂȂ�悤�ɒ���
	if (Collision::Distance::IsFrontTriangle(Pts[0], Pts[1], Pts[2], Pts[3])) {
		std::swap(Pts[0], Pts[1]);
	}

	//!< �l�ʑ̂̒��_
	Vertices.insert(std::end(Vertices), std::begin(Pts), std::end(Pts));

	//!< �l�ʑ̂̃C���f�b�N�X
	Indices.insert(std::end(Indices), {{0, 1, 2}, {0, 2, 3}, {2, 1, 3}, {1, 0, 3}});
}

//!< �ʕ�̓����_���폜
void Convex::RemoveInternal(const std::vector<LinAlg::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices, std::vector<LinAlg::Vec3>& Mesh)
{
	//!< �����_�����O
	{
		const auto Range = std::ranges::remove_if(Mesh,
			[&](const auto& Pt) {
				return IsInternal(Pt, Vertices, Indices);
			});
		Mesh.erase(std::ranges::cbegin(Range), std::ranges::cend(Range));
	}

	//!< �����Ɠ���Ƃ݂Ȃ���_�͏��O
	{
		const auto Range = std::ranges::remove_if(Mesh,
			[&](const auto& Pt) {
				//!< ����Ƃ݂Ȃ���_������
				return std::ranges::any_of(Vertices, 
					[&](const auto rhs) {
						return rhs.NearlyEqual(Pt);
					});
			});
		Mesh.erase(std::ranges::cbegin(Range), std::ranges::cend(Range));
	}
}

//!< �ӂ����L���Ȃ����j�[�N�ȕӂ����W
void Convex::CollectUniqueEdges(std::vector<Collision::TriInds>::const_iterator Begin, std::vector<Collision::TriInds>::const_iterator End, std::vector<Collision::EdgeIndsWithCount>& EdgeCounts)
{
	std::for_each(Begin, End, 
		[&](const auto& i) {
			//!< �O�p�`�̎O��
			const std::array Edges = {
				Collision::EdgeInds({ i[0], i[1] }),
				Collision::EdgeInds({ i[1], i[2] }),
				Collision::EdgeInds({ i[2], i[0] }),
			};
			std::ranges::for_each(Edges,
				[&](const auto& j) {
					//!< (�^�t�����o�Ƃ��Ĉ���) ���o�̕ӂ��ǂ����𒲂ׂ� 
					const auto It = std::ranges::find_if(EdgeCounts, 
						[&](const auto& k) {
							return (k.first[0] == j[0] && k.first[1] == j[1]) || (k.first[0] == j[1] && k.first[1] == j[0]);
						});
					if (std::cend(EdgeCounts) == It) {
						//!< ������Ȃ������̂ŁA�V�K�̕ӂƂ��Ēǉ�
						EdgeCounts.emplace_back(Collision::EdgeIndsWithCount({ j, 0 }));
					}
					else {
						//!< ���o�̕ӂȂ̂ŃJ�E���^���C���N�������g (�X�V)
						++It->second;
					}
			});
		});

	//!< ���j�[�N�łȂ��ӂ��폜 (�J�E���^�� 0 �̂��̂����c��) 
	const auto Range = std::ranges::remove_if(EdgeCounts,
		[](const auto& i) {
			return i.second > 0;
		});
	EdgeCounts.erase(std::ranges::cbegin(Range), std::ranges::cend(EdgeCounts));
}

//!< �n�C�|����H�킹��Ƃ��Ȃ莞�Ԃ��������Ɍ��ǃn�C�|���̓ʕ�ł��邾���Ȃ̂ŃR���W�����Ƃ��Č����I�ł͂Ȃ��A���[�|����H�킹�邱��
void Convex::BuildConvexHull(const std::vector<LinAlg::Vec3>& Mesh, std::vector<LinAlg::Vec3>& Vertices, std::vector<Collision::TriInds>& Indices)
{
	//PERFORMANCE_COUNTER_FUNC();
	PERFORMANCE_COUNTER("BuildConvexHull()");

	LOG(std::data(std::format("Building convex hull...\n")));

	//!< �܂��͊T�˕�܂���悤�Ȏl�ʑ̂��쐬����
	BuildTetrahedron(Mesh, Vertices, Indices);

	//!< �����_�̏��O -> �O���_���c��
	auto External = Mesh;
	RemoveInternal(Vertices, Indices, External);

	//!< �O���_�������Ȃ�܂ŌJ��Ԃ�
	while (!std::empty(External)) {
		LOG(std::data(std::format("Rest vertices = {}\n", std::size(External))));

		//!< (�����ł� External[0]�̕�����) �ŉ��_��������
		const auto FarIt = Collision::Distance::Farthest(External, External[0]);

		//!< �Ԃ牺�������ӂ�������
		std::vector<Collision::EdgeIndsWithCount> DanglingEdges;
		{
			//!< �ŉ��_�������Ă��Ȃ��O�p�` (A) �ƁA�����Ă���O�p�` (B) �ɕ���
			//!< partition �͈ȉ��̂悤�ɕԂ�
			//!<	A �����_���� true		: [begin(Indices), begin(Range)]
			//!<	B �����_���� false	: [begin(Range), end(Range)]
			const auto Range = std::ranges::partition(Indices, 
				[&](const auto& i) {
					return !Collision::Distance::IsFrontTriangle(*FarIt, Vertices[i[0]], Vertices[i[1]], Vertices[i[2]]);
				});

			//!< A, B �̋��E�ƂȂ�悤�ȕӂ����W���� (B �̒����瑼�̎O�p�`�ƕӂ����L���Ȃ����j�[�N�ȕӂ����W)
			CollectUniqueEdges(std::ranges::cbegin(Range), std::ranges::cend(Range), DanglingEdges);

			//!< (�ӂ͎��W��) �����܂ŗ����� B �͍폜���Ă悢  
			Indices.erase(std::ranges::cbegin(Range), std::ranges::cend(Range));
		}

		//!< �ʕ�̍X�V
		{
			//!< �ŉ��_�𒸓_�Ƃ��Ēǉ�����
			Vertices.emplace_back(*FarIt);

			//!< �ŉ��_ (�Ō�̗v�f) �̃C���f�b�N�X 
			const auto FarIndex = static_cast<uint32_t>(std::size(Vertices) - 1);
			//!< �ŉ��_�ƃ��j�[�N�ӂ���Ȃ�O�p�`�Q��ǉ�
			std::ranges::transform(DanglingEdges, std::back_inserter(Indices), 
				[&](const auto& i) {
					return Collision::TriInds({ i.first[0], i.first[1], FarIndex });
				});
		}

		//!< �O���_�̍X�V
		{
			//!< �����܂ōς񂾂�ŉ��_�͍폜���Ă悢
			External.erase(FarIt);

			//!< �X�V�����ʕ�ɑ΂��ē����_�ɂȂ������̂��폜����
			RemoveInternal(Vertices, Indices, External);
		}
	}
}

LinAlg::Vec3 Convex::Uniform::CalcCenterOfMass(const Collision::AABB& Ab, const std::vector<LinAlg::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices)
{
	LOG(std::data(std::format("Calculating center of mass (Uniform)...\n")));

	//!< �e���ɃT���v�����O����� (�v�Z�Ɏ��Ԃ�������)
	constexpr auto SampleCount = 100;

	auto CenterOfMass = LinAlg::Vec3::Zero();
	auto Sampled = 0;
	const auto Delta = Ab.GetExtent() / static_cast<float>(SampleCount);
	for (auto x = 0; x < SampleCount; ++x) {
		for (auto y = 0; y < SampleCount; ++y) {
			for (auto z = 0; z < SampleCount; ++z) {
				//!< AABB ���̃T���v���_
				const auto Pt = Ab.Min + LinAlg::Vec3(Delta.X() * x, Delta.Y() * y, Delta.Z() * z);
				if (IsInternal(Pt, Vertices, Indices)) {
					//!< �����_�Ȃ���W
					CenterOfMass += Pt;
					++Sampled;
				}
			}
		}
		LOG(std::data(std::format("\t{} / {}\n", x, SampleCount)));
	}
	return CenterOfMass / static_cast<float>(Sampled);
}

LinAlg::Mat3 Convex::Uniform::CalcInertiaTensor(const Collision::AABB& Ab, const std::vector<LinAlg::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices, const LinAlg::Vec3& CenterOfMass)
{
	LOG(std::data(std::format("Calculating inertia tensor (Uniform)...\n")));

	//!< �e���ɃT���v�����O����� (�v�Z�Ɏ��Ԃ�������)
	constexpr auto SampleCount = 100;

	auto InertiaTensor = LinAlg::Mat3::Zero();
	auto Sampled = 0;
	const auto Delta = Ab.GetExtent() / static_cast<float>(SampleCount);
	for (auto x = 0; x < SampleCount; ++x) {
		for (auto y = 0; y < SampleCount; ++y) {
			for (auto z = 0; z < SampleCount; ++z) {
				//!< AABB ���̃T���v���_ (�d�S����̑���)
				const auto Pt = Ab.Min + LinAlg::Vec3(Delta.X() * x, Delta.Y() * y, Delta.Z() * z) - CenterOfMass;
				if (IsInternal(Pt, Vertices, Indices)) {
					const auto XX = Pt.X() * Pt.X();
					const auto YY = Pt.Y() * Pt.Y();
					const auto ZZ = Pt.Z() * Pt.Z();
					const auto XY = Pt.X() * Pt.Y();
					const auto XZ = Pt.X() * Pt.Z();
					const auto YZ = Pt.Y() * Pt.Z();
					//!< �����_�Ȃ���W����
					InertiaTensor += {
						{ YY + ZZ,     -XY,     -XZ },
						{     -XY, ZZ + XX,     -YZ },
						{     -XZ,     -YZ, XX + YY },
					};
					++Sampled;
				}
			}
		}
		LOG(std::data(std::format("\t{} / {}\n", x, SampleCount)));
	}
	return InertiaTensor / static_cast<float>(Sampled);
}

LinAlg::Vec3 Convex::MonteCarlo::CalcCenterOfMass(const Collision::AABB& Ab, const std::vector<LinAlg::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices)
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
	constexpr auto SampleCount = 100 * 100;

	auto CenterOfMass = LinAlg::Vec3::Zero();
	auto Sampled = 0;
	const auto Ext = Ab.GetExtent();
	for (auto i = 0; i < SampleCount; ++i) {
		//!< AABB ���̃T���v���_
		const auto Pt = Ab.Min + LinAlg::Vec3(Ext.X() * Distribution(MersenneTwister), 
			Ext.Y() * Distribution(MersenneTwister), 
			Ext.Z() * Distribution(MersenneTwister));
		if (IsInternal(Pt, Vertices, Indices)) {
			//!< �����_�Ȃ���W
			CenterOfMass += Pt;
			++Sampled;
		}
		LOG(std::data(std::format("\t{} / {}\n", i, SampleCount)));
	}
	return CenterOfMass / static_cast<float>(Sampled);
}

LinAlg::Mat3 Convex::MonteCarlo::CalcInertiaTensor(const Collision::AABB& Ab, const std::vector<LinAlg::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices, const LinAlg::Vec3& CenterOfMass)
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
	constexpr auto SampleCount = 100 * 100;

	auto InertiaTensor = LinAlg::Mat3::Zero();
	auto Sampled = 0;
	const auto Ext = Ab.GetExtent();
	for (auto i = 0; i < SampleCount; ++i) {
		//!< AABB ���̃T���v���_ (�d�S����̑���)
		const auto Pt = Ab.Min + LinAlg::Vec3(Ext.X() * Distribution(MersenneTwister), 
			Ext.Y() * Distribution(MersenneTwister),
			Ext.Z() * Distribution(MersenneTwister)) - CenterOfMass;
		if (IsInternal(Pt, Vertices, Indices)) {
			const auto XX = Pt.X() * Pt.X();
			const auto YY = Pt.Y() * Pt.Y();
			const auto ZZ = Pt.Z() * Pt.Z();
			const auto XY = Pt.X() * Pt.Y();
			const auto XZ = Pt.X() * Pt.Z();
			const auto YZ = Pt.Y() * Pt.Z();
			//!< �����_�Ȃ���W����
			InertiaTensor += {
				{ YY + ZZ,     -XY,     -XZ },
				{     -XY, ZZ + XX,     -YZ },
				{     -XZ,     -YZ, XX + YY },
			};
			++Sampled;
		}
		LOG(std::data(std::format("\t{} / {}\n", i, SampleCount)));
	}
	return InertiaTensor / static_cast<float>(Sampled);
}

LinAlg::Vec3 Convex::Tetrahedron::CalcCenterOfMass(const std::vector<LinAlg::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices)
{
	LOG(std::data(std::format("Calculating center of mass (Tetrahedron)...\n")));

	//!< ���_�̕��� (���S) ������ A �ƕ\��
	const auto A = std::accumulate(std::cbegin(Vertices), std::cend(Vertices), LinAlg::Vec3::Zero()) / static_cast<float>(std::size(Vertices));
	const auto CenterVolumeSum = std::accumulate(std::cbegin(Indices), std::cend(Indices), std::pair<LinAlg::Vec3, float>(LinAlg::Vec3::Zero(), 0.0f),
		[&](const auto& Acc, const auto& rhs) {
			//!< A �𒸓_�Ƃ����l�ʑ�
			const auto& B = Vertices[rhs[0]];
			const auto& C = Vertices[rhs[1]];
			const auto& D = Vertices[rhs[2]];

			//!< �l�ʑ̂̒��S�ɑ̐ς̏d�ݕt�� (���S�ɑ̐ς��Ïk�����̂ōl����)
			const auto Volume = Collision::Volume::Tetrahedron(A, B, C, D);
			const auto Center = (A + B + C + D) * 0.25f;

			return std::pair<LinAlg::Vec3, float>(Acc.first + Center * Volume, Acc.second + Volume);
		});
	return CenterVolumeSum.first / CenterVolumeSum.second;
}
LinAlg::Mat3 Convex::Tetrahedron::CalcInertiaTensor(const LinAlg::Vec3& A, const LinAlg::Vec3& B, const LinAlg::Vec3& C, const LinAlg::Vec3& D)
{
	//!< �l�ʑ̂� 4 ���_����Ȃ�s��
	const auto AB = B - A;
	const auto AC = C - A;
	const auto AD = D - A;
	const auto Det = LinAlg::Mat3(AB, AC, AD).Transpose().Determinant();

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

	return LinAlg::Mat3(
		LinAlg::Vec3(2.0f * (YY + ZZ), -XY, -XZ),
		LinAlg::Vec3(-XY, 2.0f * (XX + ZZ), -YZ),
		LinAlg::Vec3(-XZ, -YZ, 2.0f * (XX + YY))) * Det / 120.0f;
}
LinAlg::Mat3 Convex::Tetrahedron::CalcInertiaTensor(const std::vector<LinAlg::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices, const LinAlg::Vec3& CenterOfMass)
{
	LOG(std::data(std::format("Calculating inertia tensor (Tetrahedron)...\n")));

	const auto A = LinAlg::Vec3::Zero(); //!< CenterOfMass - CenterOfMass �Ȃ̂�
	const auto TensorVolumeSum = std::accumulate(std::cbegin(Indices), std::cend(Indices), std::pair<LinAlg::Mat3, float>(LinAlg::Mat3::Zero(), 0.0f),
		[&](const auto& Acc, const auto& rhs) {
			const auto B = Vertices[rhs[0]] - CenterOfMass;
			const auto C = Vertices[rhs[1]] - CenterOfMass;
			const auto D = Vertices[rhs[2]] - CenterOfMass;

			return std::pair<LinAlg::Mat3, float>(Acc.first + CalcInertiaTensor(A, B, C, D), Acc.second + Collision::Volume::Tetrahedron(A, B, C, D));
		});

	return TensorVolumeSum.first / TensorVolumeSum.second;
}