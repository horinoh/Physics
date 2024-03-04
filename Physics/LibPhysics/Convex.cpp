#include "Convex.h"

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
	//LOG(data(std::format("Building convex hull...\n")));

	//!< �܂��́u�Ȃ�ׂ��v��܂���悤�Ȏl�ʑ̂��쐬
	BuildTetrahedron(Pts, Vertices, Indices);

	//!< �����_�̏��O -> �O���_���c��
	auto External = Pts;
	RemoveInternal(Vertices, Indices, External);

	//!< �O���_�������Ȃ�܂ŌJ��Ԃ�
	while (!std::empty(External)) {
		//LOG(data(std::format("Rest vertices = {}\n", size(External))));

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