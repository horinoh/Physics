#include "GJK.h"
#include "RigidBody.h"
#include "Shape.h"
#include "Collision.h"
#include "Convex.h"
#include "Util.h"

#include "Log.h"

//!< 1-�V���v���N�X (����) ... ���_�̃V���v���b�N�X��ł̏d�S���W (�V���v���b�N�X��ɂ��邩) ��Ԃ�
Math::Vec2 Collision::SignedVolume(const Math::Vec3& A, const Math::Vec3& B)
{
	const auto AB = B - A;
	//!< ���_�� AB ���Ɏˉe�� P �Ƃ���
	const auto P = A + AB * AB.Dot(-A) / AB.LengthSq(); //!< ���������Ȃ���̂Ő��K�����Ȃ������ǂ�

	//!< ���� AB �� X, Y, Z �� �֎ˉe�������ꂼ��̐���
	std::vector<float> Segs;
	std::ranges::transform(static_cast<const Math::Component3&>(AB), std::back_inserter(Segs),
		[](const auto& rhs) { 
			return std::abs(rhs); 
		});
	//!< �������Œ��ƂȂ鎲�̃C���f�b�N�X
	const auto Index = static_cast<int>(std::ranges::max_element(Segs) - std::cbegin(Segs));
	
	//!< P�A���� AB ��I���������֎ˉe
	const auto PrjA = A[Index], PrjB = B[Index], PrjP = P[Index];

	//!< P �� ���� AB �̓����ɂ���Ώd�S���W��Ԃ�
	if ((PrjP > PrjA && PrjP < PrjB) || (PrjP > PrjB && PrjP < PrjA)) {
		return Math::Vec2(PrjB - PrjP, PrjP - PrjA) / AB[Index];
	}

	//!< �O���m��AP �� A ���� B ����
	return (PrjP <= PrjA) ? Math::Vec2::AxisX() : Math::Vec2::AxisY();
}

//!< 2-�V���v���N�X (�O�p�`)
Math::Vec3 Collision::SignedVolume(const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C)
{
	//!< ABC ��ł� ���_ �̏d�S���W
	const auto BC = Barycentric(A, B, C);
	if (BC != std::nullopt) {
		return BC.value();
	}
	
	//!< YZ, ZX, XY ����
	constexpr std::array XYZ = { 0, 1, 2 };
	constexpr auto XYZSize = static_cast<int>(std::size(XYZ));

	//!< P �� 3 �ӂɎˉe���Ĉ�ԋ߂����̂������� (1-�V���v���N�X �ɋA��)
	const std::array Edges = { A, B, C };
	//!< 3 �ӂɑ΂��āA�d�S���W�Ƌ���(���)�̃y�A�����W
	std::vector<std::pair<Math::Vec2, float>> LmdLen;
	std::ranges::transform(XYZ, std::back_inserter(LmdLen),
		[&](const auto& rhs) {
			const auto j = (rhs + 1) % XYZSize;
			const auto k = (rhs + 2) % XYZSize;
			
			const auto Lmd = SignedVolume(Edges[j], Edges[k]);
			return std::pair<Math::Vec2, float>({ Lmd, (Edges[j] * Lmd[0] + Edges[k] * Lmd[1]).LengthSq() });
		});
	//!< �������ŏ��ƂȂ�ӂƂ��̃C���f�b�N�X
	const auto MinIt = std::ranges::min_element(LmdLen,
		[](const auto& lhs, const auto& rhs) {
			return lhs.second < rhs.second;
		});
	const auto Index = static_cast<int>(MinIt - std::cbegin(LmdLen));

	//!< �d�S���W�� Vec3 �Ƃ��ĕԂ�
	Math::Vec3 Lambda;
	Lambda[(Index + 0) % XYZSize] = 0.0f;
	Lambda[(Index + 1) % XYZSize] = MinIt->first[0];
	Lambda[(Index + 2) % XYZSize] = MinIt->first[1];
	return Lambda;
}

//!< 3-�V���v���N�X (�l�ʑ�)
Math::Vec4 Collision::SignedVolume(const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C, const Math::Vec3& D)
{
	//!< �l�ʑ̂����W�n�ƍl����A���̋t(�]�u)�s��̎l���q
	const auto M = Math::Mat4(Math::Vec4(A, 1.0f), Math::Vec4(B, 1.0f), Math::Vec4(C, 1.0f), Math::Vec4(D, 1.0f)).Transpose();
	const auto Cofactors = Math::Vec4(M.Cofactor(3, 0), M.Cofactor(3, 1), M.Cofactor(3, 2), M.Cofactor(3, 3));
	const auto& CofCmps = static_cast<const Math::Component4&>(Cofactors);
	const auto Det = std::accumulate(std::cbegin(CofCmps), std::cend(CofCmps), 0.0f);
	//!< ���_���l�ʑ̓����ɂ���΁A�d�S���W��Ԃ�
	if (std::ranges::all_of(CofCmps,
		[&](const auto rhs) {
			return std::signbit(Det) == std::signbit(rhs);
		})) {
		return Cofactors / Det;
	}

	constexpr std::array XYZW = { 0, 1, 2, 3 };
	constexpr auto XYZWSize = static_cast<int>(std::size(XYZW));

	//!< 4 �ʂɎˉe���Ĉ�ԋ߂����̂������� (2-�V���v���N�X �ɋA��)
	const std::array Faces = { A, B, C, D };
	//!< 4 �ʂɑ΂��āA�d�S���W�Ƌ���(���)�̃y�A�����W
	std::vector<std::pair<Math::Vec3, float>> LmdLen;
	std::ranges::transform(XYZW, std::back_inserter(LmdLen),
		[&](const auto& rhs) {
			const auto i = rhs;
			const auto j = (rhs + 1) % XYZWSize;
			const auto k = (rhs + 2) % XYZWSize;

			const auto Lmd = SignedVolume(Faces[i], Faces[j], Faces[k]);
			return std::pair<Math::Vec3, float>({ Lmd, (Faces[i] * Lmd[0] + Faces[j] * Lmd[1] + Faces[k] * Lmd[2]).LengthSq() });
		});
	//!< �������ŏ��ƂȂ�ʂƂ��̃C���f�b�N�X
	const auto MinIt = std::ranges::min_element(LmdLen,
		[](const auto& lhs, const auto& rhs) {
			return lhs.second < rhs.second;
		});
	const auto Index = static_cast<int>(MinIt - std::cbegin(LmdLen));
	
	Math::Vec4 Lambda;
	Lambda[(Index + 0) % XYZWSize] = MinIt->first[0];
	Lambda[(Index + 1) % XYZWSize] = MinIt->first[1];
	Lambda[(Index + 2) % XYZWSize] = MinIt->first[2];
	Lambda[(Index + 3) % XYZWSize] = 0.0f;
	return Lambda;
}

std::optional<Math::Vec3> Collision::Barycentric(const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C)
{
	const auto N = Math::Vec3::Normal(A, B, C);
	if (N.NearlyEqual(Math::Vec3::Zero())) {
		return std::nullopt;
	}

	//!< YZ, ZX, XY ����
	constexpr std::array XYZ = { 0, 1, 2 }; 
	constexpr auto XYZSize = static_cast<int>(std::size(XYZ));

	//!< ���_�� �O�p�` ABC �Ɏˉe�� P �Ƃ���
	const auto P = N * A.Dot(N) / N.LengthSq();

	//!< �O�p�` ABC �� YZ, ZX, XY ���ʂ֎ˉe�������ꂼ��̖ʐ�
	std::vector<float> Areas;
	std::ranges::transform(XYZ, std::back_inserter(Areas),
		[&](const auto& rhs) {
			const auto j = (rhs + 1) % XYZSize;
			const auto k = (rhs + 2) % XYZSize;

			const auto a = Math::Vec2(A[j], A[k]);
			const auto b = Math::Vec2(B[j], B[k]);
			const auto c = Math::Vec2(C[j], C[k]);
			const auto AB = b - a;
			const auto AC = c - a;
			return Math::Mat2(AB, AC).Determinant();
		});
	//!< �ʐς̐�Βl���ő�ƂȂ镽�ʂƂ��̃C���f�b�N�X
	const auto AbsMaxIt = std::ranges::max_element(Areas,
		[](const auto& lhs, const auto& rhs) {
			return std::abs(lhs) < std::abs(rhs);
		});
	const auto Index = static_cast<int>(AbsMaxIt - std::cbegin(Areas));

	//!< P�A�O�p�` ABC ��I���������ʂɎˉe (Z ���I�����ꂽ�ꍇ�� XY ���ʂƂ�������ɂȂ�)
	const auto X = (Index + 1) % XYZSize;
	const auto Y = (Index + 2) % XYZSize;
	const std::array PrjABC = { Math::Vec2(A[X], A[Y]), Math::Vec2(B[X], B[Y]), Math::Vec2(C[X], C[Y]) };
	const auto PrjP = Math::Vec2(P[X], P[Y]);

	//!< �ˉe�_�ƕӂ���Ȃ�T�u�O�p�`���ꂼ��̖ʐ�
	std::vector<float> SubAreas;
	std::ranges::transform(XYZ, std::back_inserter(SubAreas),
		[&](const auto& rhs) {
			const auto j = (rhs + 1) % XYZSize;
			const auto k = (rhs + 2) % XYZSize;
			return Math::Mat2(PrjABC[j] - PrjP, PrjABC[k] - PrjP).Determinant();
		});

	//!< P �� �O�p�` ABC �̓����ɂ���� (�T�u�O�p�`�̖ʐς̕������画�f)�A�d�S���W��Ԃ�
	if (std::ranges::all_of(SubAreas,
		[&](const auto rhs) {
			return std::signbit(Areas[Index]) == std::signbit(rhs);
		})) {
		return Math::Vec3(SubAreas[0], SubAreas[1], SubAreas[2]) / Areas[Index];
	}

	return std::nullopt;
}
std::optional<Math::Vec3> Collision::Barycentric(const Math::Vec3& Pt, const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C)
{
	//!< ABC ���� Pt ���������ƂŁA���_�̏d�S���W�֋A��
	return Barycentric(A - Pt, B - Pt, C - Pt);
}

//!< A, B �̃T�|�[�g�|�C���g�A�y�т��̍� C �����߂�
Collision::SupportPoint::Points Collision::SupportPoint::Get(const Physics::Shape* ShA, const Math::Vec3& PosA, const Math::Quat& RotA,
	const Physics::Shape* ShB, const Math::Vec3& PosB, const Math::Quat& RotB, 
	const Math::Vec3& UDir, const float Bias)
{
	return { ShA->GetSupportPoint(PosA, RotA, UDir, Bias), ShB->GetSupportPoint(PosB, RotB, -UDir, Bias) };
}
Collision::SupportPoint::Points Collision::SupportPoint::Get(const Physics::RigidBody* RbA, 
	const Physics::RigidBody* RbB, 
	const Math::Vec3& UDir, const float Bias)
{
	return Get(RbA->Shape, RbA->Position, RbA->Rotation, RbB->Shape, RbB->Position, RbB->Rotation, UDir, Bias);
}

void Collision::SupportPoint::ToTetrahedron(const Physics::Shape* ShA, const Math::Vec3& PosA, const Math::Quat& RotA, 
	const Physics::Shape* ShB, const Math::Vec3& PosB, const Math::Quat& RotB, 
	std::vector<SupportPoint::Points>& Sps)
{
	if (1 == std::size(Sps)) {
		Sps.emplace_back(Collision::SupportPoint::Get(ShA, PosA, RotA, ShB, PosB, RotB, -Sps[0].GetC().Normalize(), 0.0f));
	}
	if (2 == std::size(Sps)) {
		const auto AB = Sps[1].GetC() - Sps[0].GetC();
		Math::Vec3 U, V;
		AB.GetOrtho(U, V);
		Sps.emplace_back(Collision::SupportPoint::Get(ShA, PosA, RotA, ShB, PosB, RotB, U, 0.0f));
	}
	if (3 == std::size(Sps)) {
		const auto AB = Sps[1].GetC() - Sps[0].GetC();
		const auto AC = Sps[2].GetC() - Sps[0].GetC();
		Sps.emplace_back(Collision::SupportPoint::Get(ShA, PosA, RotA, ShB, PosB, RotB, AB.Cross(AC).Normalize(), 0.0f));
	}
}
void Collision::SupportPoint::ToTetrahedron(const Physics::RigidBody* RbA,
	const Physics::RigidBody* RbB, 
	std::vector<Collision::SupportPoint::Points>& Sps)
{
	ToTetrahedron(RbA->Shape, RbA->Position, RbA->Rotation, RbB->Shape, RbB->Position, RbB->Rotation, Sps);
}

void Collision::SupportPoint::Expand(const float Bias, std::vector<Collision::SupportPoint::Points>& Sps)
{
	const auto Center = std::accumulate(std::cbegin(Sps), std::cend(Sps), Math::Vec3::Zero(), 
		[](const auto& Acc, const auto& i) { 
			return Acc + i.GetC(); 
		}) / static_cast<float>(std::size(Sps));
	std::ranges::transform(Sps, std::begin(Sps),
		[&](const auto& i) {
			const auto Dir = (i.GetC() - Center).Normalize() * Bias;
			return Collision::SupportPoint::Points(i.GetA() + Dir, i.GetB() - Dir);
		});
}

bool Collision::Intersection::GJK(const Physics::Shape* ShA, const Math::Vec3& PosA, const Math::Quat& RotA,
	const Physics::Shape* ShB, const Math::Vec3& PosB, const Math::Quat& RotB,
	OnIntersectGJK OnIntersect, const float Bias,
	const bool WithClosestPoint,
	Math::Vec3& OnA, Math::Vec3& OnB)
{
	std::vector<Collision::SupportPoint::Points> Sps;
	Sps.reserve(4); //!< 4 �g

	//!< (1, 1, 1) �����̃T�|�[�g�|�C���g�����߂�
	Sps.emplace_back(Collision::SupportPoint::Get(ShA, PosA, RotA, ShB, PosB, RotB, Math::Vec3::One().Normalize(), 0.0f));

	//!< ���_�����ɔ��Α�
	auto Dir = -Sps.back().GetC();
	auto ClosestDistSq = (std::numeric_limits<float>::max)();
	auto HasIntersection = false;
	Math::Vec4 Lambda;
	do {
		//!< Dir �����̃T�|�[�g�|�C���g�����߂�
		const auto Pt = Collision::SupportPoint::Get(ShA, PosA, RotA, ShB, PosB, RotB, Dir.ToNormalized(), 0.0f);

		//!< Pt �͊����̓_�A��������ȏ�g���ł��Ȃ� -> �Փ˖���
		if (std::ranges::any_of(Sps,
			[&](const auto& i) {
				return Pt.GetC().NearlyEqual(i.GetC());
			})) {
			break;
		}

		//!< �ŋߐړ_�����߂Ȃ��ꍇ�͑����I���ł���\��������
		if (!WithClosestPoint) {
			//!< �V�����T�|�[�g�|�C���g Pt �����_�𒴂��Ă��Ȃ��ꍇ�͌��_���܂܂Ȃ��A�����I��
			if (Dir.Dot(Pt.GetC()) < 0.0f) {
				break;
			}
		}

		//!< �V�����T�|�[�g�|�C���g��ǉ�
		Sps.emplace_back(Pt);

		//!< �V���v���b�N�X�����_���܂ނȂ�Փ�
		if ((HasIntersection = Collision::SupportPoint::SimplexSignedVolumes(Sps, Dir, Lambda))) {
			break;
		}

		//!< �ŒZ�������X�V�A�X�V�ł��Ȃ�ΏI��
		//!< (2 �̎O�p�`����Ȃ�l�p�`�̑Ίp����Ɍ��_���ʒu����悤�ȏꍇ�A2 �̎O�p�`�ԂŐ؂�ւ����[�v�ɂȂ�̂����)
		const auto DistSq = Dir.LengthSq();
		if (DistSq >= ClosestDistSq) {
			break;
		}
		ClosestDistSq = DistSq;

		//!< �L���ȃT�|�[�g�|�C���g (�Ή����� Lambda ���� 0.0 �̃R���|�[�l���g) �������c��
		auto Index = 0;
		const auto SpsRange = std::ranges::remove_if(Sps,
			[&](const auto& i) {
				return 0.0f == Lambda[Index++];
			});
		Sps.erase(std::ranges::cbegin(SpsRange), std::ranges::cend(SpsRange));

		//!< Lambda �̗L���� (�� 0.0) �v�f��O�֏W�߂� (�P�c�ɂ̓S�~���c�邪�ASps ���������A�N�Z�X���Ȃ�)
		[[maybe_unused]] const auto Dmy = std::ranges::remove(static_cast<Math::Component4&>(Lambda), 0.0f);

		//!< 3-�V���v���b�N�X (�l�ʑ�) �ł����܂ŗ����猴�_���܂�
		HasIntersection = (4 == std::size(Sps));
	} while (!HasIntersection); //!< ���_���܂܂������܂ŗ����烋�[�v

	//!< (EPA ����) �Փ˓_�����߂�
	if (HasIntersection && nullptr != OnIntersect) {
		OnIntersect(ShA, PosA, RotA,
			ShB, PosB, RotB,
			Sps,
			Bias, OnA, OnB);
		return true;
	}

	//!< �ŋߐړ_�����߂�
	if (WithClosestPoint) {
		auto Index = 0;
		OnA = std::accumulate(std::cbegin(Sps), std::cend(Sps), Math::Vec3::Zero(),
			[&](const auto& Acc, const auto& i) {
				return Acc + i.GetA() * Lambda[Index++];
			});
		Index = 0;
		OnB = std::accumulate(std::cbegin(Sps), std::cend(Sps), Math::Vec3::Zero(),
			[&](const auto& Acc, const auto& i) {
				return Acc + i.GetB() * Lambda[Index++];
			});
	}

	return false;
}
bool Collision::Intersection::GJK(const Physics::RigidBody* RbA,
	const Physics::RigidBody* RbB,
	OnIntersectGJK OnIntersect, const float Bias,
	Math::Vec3& OnA, Math::Vec3& OnB) {
	return GJK(RbA->Shape, RbA->Position, RbA->Rotation,
		RbB->Shape, RbB->Position, RbB->Rotation,
		OnIntersect, Bias,
		true,
		OnA, OnB);
}

void Collision::Intersection::EPA(const Physics::Shape* ShA, const Math::Vec3& PosA, const Math::Quat& RotA,
	const Physics::Shape* ShB, const Math::Vec3& PosB, const Math::Quat& RotB, 
	const std::vector<SupportPoint::Points>& SupportPoints, const float Bias, 
	Math::Vec3& OnA, Math::Vec3& OnB)
{
	//!< ��Ɨp�T�|�[�g�|�C���g
	std::vector<SupportPoint::Points> Sps;
	Sps.assign(std::cbegin(SupportPoints), std::cend(SupportPoints));

	//!< EPA �̑O����
	{
		//!< EPA �ł͎l�ʑ̂�K�v�Ƃ���̂ŁA(�l�ʑ̂łȂ��ꍇ��) �l�ʑ̂֊g������
		SupportPoint::ToTetrahedron(ShA, PosA, RotA, ShB, PosB, RotB, Sps);

		//!< �V���v���b�N�X���g������ (A, B �̏Փ˓_���قړ���̏ꍇ�ɖ@������肭���߂��Ȃ��ꍇ�����邽�߁A�o�C�A�X���g������)
		SupportPoint::Expand(Bias, Sps);
	}

	//!< �O���ɖ@���������悤�ɃC���f�b�N�X�𐶐�����
	constexpr std::array XYZW = { 0, 1, 2, 3 };
	constexpr auto XYZWSize = static_cast<uint32_t>(std::size(XYZW));
	std::vector<Collision::TriInds> Tris;
	std::ranges::transform(XYZW, std::back_inserter(Tris),
		[&](const auto& rhs) {
			const auto i = (rhs + 0) % XYZWSize;
			const auto j = (rhs + 1) % XYZWSize;
			const auto k = (rhs + 2) % XYZWSize;
			const auto l = (rhs + 3) % XYZWSize; //!< �O�p�` i, j, k ���ӂƂ���l�ʑ̂̒��_
			return Collision::Distance::IsFront(Sps[l].GetC(), Sps[i].GetC(), Sps[j].GetC(), Sps[k].GetC()) ? 
				Collision::TriInds({ j, i, k }) : Collision::TriInds({ i, j, k });
		});

	//!< �l�ʑ̂̒��S�����߂�
	const auto Center = std::accumulate(std::cbegin(Sps), std::cend(Sps), Math::Vec3::Zero(),
		[](const auto& Acc, const auto& i) {
			return Acc + i.GetC(); 
		}) / static_cast<float>(std::size(Sps));

	//!< ���_�ɍł��߂��ʂ�������ׂɃV���v���b�N�X���g������ (���_�̌����Ɍ��肵�ēʕ���`�����Ă�������)
	while (true) {
		//!< ���_�ɍł��߂��O�p�` ABC ���擾
		const auto& CTri = *SupportPoint::Distance::Closest(Math::Vec3::Zero(), Sps, Tris);
		const auto& A = Sps[CTri[0]].GetC(), B = Sps[CTri[1]].GetC(), C = Sps[CTri[2]].GetC();
		//!< �O�p�`�̖@��
		const auto N = Math::Vec3::UnitNormal(A, B, C);

		//!< �@�������̃T�|�[�g�|�C���g���擾
		const auto Pt = Collision::SupportPoint::Get(ShA, PosA, RotA, ShB, PosB, RotB, N, Bias);

		//!< ����ȏ�g���ł��Ȃ� (Pt �� ABC �@���̔��Α�) �Ȃ�I�� 
		//if (Distance::PointTriangle(Pt.GetC(), A, B, C) <= 0.0f) {
		//	break;
		//}
		if ((Pt.GetC() - A).Dot(N) <= 0.0f) {
			break;
		}

		//!< �T�|�[�g�|�C���g�����o�̏ꍇ�́A����ȏ�g���ł��Ȃ����Ƃ��Ӗ�����̂ŏI��
		if (std::ranges::any_of(Tris, 
			[&](const auto& i) {
				//!< �O�p�`�Q
				return std::ranges::any_of(i,
					[&](const auto& j) { 
						//!< 3 ���_�̂����ꂩ�ɓ������ꍇ�A���o
						constexpr auto Eps = 0.01f; 
						return Sps[j].GetC().NearlyEqual(Pt.GetC(), Eps);
					});
			})) {
			break;
		}

		Sps.emplace_back(Pt);

		//!< �T�|�[�g�|�C���g���������Ă���O�p�`���폜
		{
			const auto Range = std::ranges::remove_if(Tris,
				[&](const auto& i) {
					return Distance::IsFront(Pt.GetC(), Sps[i[0]].GetC(), Sps[i[1]].GetC(), Sps[i[2]].GetC());
				});
			Tris.erase(std::ranges::cbegin(Range), std::ranges::cend(Range));
			//!< �폜�ł��Ȃ��ꍇ�͏I��
			if (std::ranges::empty(Range)) {
				break; 
			}
		}

		//!< �Ԃ牺�������� (�폜�����O�p�`�ƁA�c�����O�p�`�̋��E�ƂȂ�悤�ȕ�) �����W����
		{
			//!< �����̎O�p�`���狤�L����Ă��Ȃ��悤�ȃ��j�[�N�ȕӂ����W����΂悢
			std::vector<Collision::EdgeIndsWithCount> DanglingEdges;
			Convex::CollectUniqueEdges(Tris, DanglingEdges);
			//!< ���̂悤�ȕӂ�������ΏI��
			if (std::empty(DanglingEdges)) {
				break;
			}

			//!< �T�|�[�g�|�C���g A �Ƌ��E�� BC ����Ȃ�O�p�`�Q��ǉ�
			const auto A = static_cast<uint32_t>(std::size(Sps)) - 1;
			std::ranges::transform(DanglingEdges, std::back_inserter(Tris),
				[&](const auto& i) {
					const auto B = i.first[0], C = i.first[1];
					//!< �@�����O���������悤�ɃC���f�b�N�X�𐶐�����
					if (Distance::IsFront(Center, Sps[A].GetC(), Sps[B].GetC(), Sps[C].GetC())) {
						return Collision::TriInds({ A, C, B });
					}
					else {
						return Collision::TriInds({ A, B, C });
					}
				});
		}
	}

	{
		//!< ���_�ɍł��߂��O�p�` ABC ���擾
		const auto& CTri = *SupportPoint::Distance::Closest(Math::Vec3::Zero(), Sps, Tris);
		const auto& A = Sps[CTri[0]], B = Sps[CTri[1]], C = Sps[CTri[2]];

		//!< ABC ��ł̌��_�̏d�S���W���擾
		const auto Lambda = Barycentric(A.GetC(), B.GetC(), C.GetC());
		if (Lambda != std::nullopt) {
			OnA = A.GetA() * Lambda.value()[0] + B.GetA() * Lambda.value()[1] + C.GetA() * Lambda.value()[2];
			OnB = A.GetB() * Lambda.value()[0] + B.GetB() * Lambda.value()[1] + C.GetB() * Lambda.value()[2];
#ifdef _DEBUG
			//LOG(std::data(std::format("PenetrationDistSq = {}\n", (OnB - OnA).LengthSq())));
#endif
		}
		else {
			//!< #TODO ���_���O�p�`�̊O���ɂ���
			LOG(std::data(std::format("EPA: Barycentric failed\n")));
		}
	}
}

//!< #TODO �v����			
//!< �~���R�t�X�L�[���̓ʕ�𐶐��������Ɍ��_���܂ނ悤�ȃV���v���b�N�X (�P��) �𐶐����鎖�ő�p����
//!< A, B �̃~���R�t�X�L�[�� C �����_���܂߂ΏՓ˂ƂȂ�
//!< A, B �̃T�|�[�g�|�C���g�̍��� C �̃T�|�[�g�|�C���g�ƂȂ�
//!<	�ŏ��̃T�|�[�g�|�C���g 1 ��������
//!<	���_�����̎��̃T�|�[�g�|�C���g 2 ��������
//!<	1, 2 �̐������猴�_�����̎��̃T�|�[�g�|�C���g 3 ��������
//!<	1, 2, 3 ���Ȃ��O�p�`�����_���܂߂ΏՓˁA�I��
//!<	���_�������@�������̎��̃T�|�[�g�|�C���g 4 ��������
//!<	1, 2, 3, 4 ���Ȃ��l�ʑ̂����_���܂߂ΏՓˁA�I��
//!<	��ԋ߂��O�p�` (�Ⴆ�� 1, 2, 4) ����A���_�������@�������̎��̃T�|�[�g�|�C���g 5 ��������
//!<	�l�ʑ� (1, 2, 4, 5) �����_���܂ނ��A�T�|�[�g�|�C���g�������Ȃ�܂ő�����
bool Collision::Intersection::GJK_EPA(const Physics::RigidBody* RbA, 
	const Physics::RigidBody* RbB, 
	const float Bias, 
	bool WithClosestPoint,
	Math::Vec3& OnA, Math::Vec3& OnB)
{
	return GJK_EPA(RbA->Shape, RbA->Position, RbA->Rotation,
		RbB->Shape, RbB->Position, RbB->Rotation,
		Bias,
		WithClosestPoint,
		OnA, OnB);
}
bool Collision::Intersection::GJK(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB) 
{
	return GJK(RbA->Shape, RbA->Position, RbA->Rotation,
		RbB->Shape, RbB->Position, RbB->Rotation);
}

void Collision::Closest::GJK(const Physics::Shape* ShA, const Math::Vec3& PosA, const Math::Quat& RotA, 
	const Physics::Shape* ShB, const Math::Vec3& PosB, const Math::Quat& RotB, 
	Math::Vec3& OnA, Math::Vec3& OnB)
{
	Intersection::GJK(ShA, PosA, RotA,
		ShB, PosB, RotB,
		Intersection::OnIntersectDummy, 0.0f,
		true,
		OnA, OnB);
}
void Collision::Closest::GJK(const Physics::RigidBody* RbA, 
	const Physics::RigidBody* RbB, 
	Math::Vec3& OnA, Math::Vec3& OnB)
{
	GJK(RbA->Shape, RbA->Position, RbA->Rotation, 
		RbB->Shape, RbB->Position, RbB->Rotation, 
		OnA, OnB);
}

#ifdef _DEBUG
void Collision::SignedVolumeTest()
{
	constexpr auto Eps = 0.001f;
	{
		const std::vector OrgPts = {
			Math::Vec3(0.0f, 0.0f, 0.0f),
			Math::Vec3(1.0f, 0.0f, 0.0f),
			Math::Vec3(0.0f, 1.0f, 0.0f),
			Math::Vec3(0.0f, 0.0f, 1.0f),
		};
		{
			//!< ���؍�
			std::vector<Math::Vec3> Pts;
			Pts.resize(std::size(OrgPts));
			std::ranges::transform(OrgPts, std::begin(Pts), std::bind(std::plus(), std::placeholders::_1, Math::Vec3(1.0f, 1.0f, 1.0f)));
			const auto Lambda = SignedVolume(Pts[0], Pts[1], Pts[2], Pts[3]);
			const auto V = Pts[0] * Lambda[0] + Pts[1] * Lambda[1] + Pts[2] * Lambda[2] + Pts[3] * Lambda[3];

			const auto CorrectLambda = Math::Vec4(1.0f, 0.0f, 0.0f, 0.0f);
			const auto CorrectV = Math::Vec3::One();
			if (!Lambda.NearlyEqual(CorrectLambda, Eps)) {
				__debugbreak();
			}
			if (!V.NearlyEqual(CorrectV, Eps)) {
				__debugbreak();
			}
		}
		{
			//!< ���؍�
			std::vector<Math::Vec3> Pts;
			Pts.resize(std::size(OrgPts));
			std::ranges::transform(OrgPts, std::begin(Pts), std::bind(std::plus(), std::placeholders::_1, Math::Vec3(-1.0f, -1.0f, -1.0f) * 0.25f));
			const auto Lambda = SignedVolume(Pts[0], Pts[1], Pts[2], Pts[3]);
			const auto V = Pts[0] * Lambda[0] + Pts[1] * Lambda[1] + Pts[2] * Lambda[2] + Pts[3] * Lambda[3];

			const auto CorrectLambda = Math::Vec4(0.25f, 0.25f, 0.25f, 0.25f);
			const auto CorrectV = Math::Vec3::Zero();
			if (!Lambda.NearlyEqual(CorrectLambda, Eps)) {
				__debugbreak();
			}
			if (!V.NearlyEqual(CorrectV, Eps)) {
				__debugbreak();
			}
		}
		{
			//!< ���؍�
			std::vector<Math::Vec3> Pts;
			Pts.resize(std::size(OrgPts));
			std::ranges::transform(OrgPts, std::begin(Pts), std::bind(std::plus(), std::placeholders::_1, Math::Vec3(-1.0f, -1.0f, -1.0f)));
			const auto Lambda = SignedVolume(Pts[0], Pts[1], Pts[2], Pts[3]);
			const auto V = Pts[0] * Lambda[0] + Pts[1] * Lambda[1] + Pts[2] * Lambda[2] + Pts[3] * Lambda[3];

			const auto CorrectLambda = Math::Vec4(0.0f, 0.333f, 0.333f, 0.333f);
			const auto CorrectV = Math::Vec3(-0.667f, -0.667f, -0.667f);
			if (!Lambda.NearlyEqual(CorrectLambda, Eps)) {
				__debugbreak();
			}
			if (!V.NearlyEqual(CorrectV, Eps)) {
				__debugbreak();
			}
		}
		{
			//!< ���؍�
			std::vector<Math::Vec3> Pts;
			Pts.resize(std::size(OrgPts));
			std::ranges::transform(OrgPts, std::begin(Pts), std::bind(std::plus(), std::placeholders::_1, Math::Vec3(1.0f, 1.0f, -0.5f)));
			const auto Lambda = SignedVolume(Pts[0], Pts[1], Pts[2], Pts[3]);
			const auto V = Pts[0] * Lambda[0] + Pts[1] * Lambda[1] + Pts[2] * Lambda[2] + Pts[3] * Lambda[3];

			const auto CorrectLambda = Math::Vec4(0.5f, 0.0f, 0.0f, 0.5f);
			const auto CorrectV = Math::Vec3(1.0f, 1.0f, 0.0f);
			if (!Lambda.NearlyEqual(CorrectLambda, Eps)) {
				__debugbreak();
			}
			if (!V.NearlyEqual(CorrectV, Eps)) {
				__debugbreak();
			}
		}
	}
	{
		//!< ���؍�
		const std::array Pts = {
			Math::Vec3(51.1996613f, 26.1989613f, 1.91339576f),
			Math::Vec3(-51.0567360f, -26.0565681f, -0.436143428f),
			Math::Vec3(50.8978920f, -24.1035538f, -1.04042661f),
			Math::Vec3(-49.1021080f, 25.8964462f, -1.04042661f)
		};
		const auto Lambda = SignedVolume(Pts[0], Pts[1], Pts[2], Pts[3]);
		const auto V = Pts[0] * Lambda[0] + Pts[1] * Lambda[1] + Pts[2] * Lambda[2] + Pts[3] * Lambda[3];

		//!< �������킹
		const auto CorrectLambda = Math::Vec4(0.290f, 0.302f, 0.206f, 0.202f);
		const auto CorrectV = Math::Vec3::Zero();
		if (!Lambda.NearlyEqual(CorrectLambda, Eps)) {
			__debugbreak();
		}
		if (!V.NearlyEqual(CorrectV, Eps)) {
			__debugbreak();
		}
	}
}
#endif
