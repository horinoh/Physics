#include "GJK.h"
#include "RigidBody.h"
#include "Shape.h"
#include "Collision.h"
#include "Convex.h"

#include "Log.h"

//!< 1-シンプレクス (線分)
LinAlg::Vec2 Collision::SignedVolume(const LinAlg::Vec3& A, const LinAlg::Vec3& B)
{
	const auto AB = B - A;
	//!< 原点を AB 軸に射影し P とする
	const auto P = A + AB * AB.Dot(-A) / AB.LengthSq(); //!< 平方根を省けるので正規化しない方が良い

	//!< 線分 AB を X, Y, Z 軸 へ射影したそれぞれの線分
	std::vector<float> Segs;
	std::ranges::transform(static_cast<const LinAlg::Float3&>(AB), std::back_inserter(Segs),
		[](const auto& rhs) { 
			return std::abs(rhs); 
		});
	//!< 線分が最長となる軸のインデックス
	const auto Index = static_cast<int>(std::ranges::max_element(Segs) - std::cbegin(Segs));
	
	//!< P、AB を選択した軸へ射影
	const auto PrjA = A[Index], PrjB = B[Index], PrjP = P[Index];

	//!< P が AB の内部にあれば重心パラメータを返す
	if ((PrjP > PrjA && PrjP < PrjB) || (PrjP > PrjB && PrjP < PrjA)) {
		return LinAlg::Vec2(PrjB - PrjP, PrjP - PrjA) / AB[Index];
	}

	//!< 外部確定、P が A 側か B 側か
	if ((PrjA <= PrjB && PrjP <= PrjA) || (PrjA >= PrjB && PrjP >= PrjA)) {
		return LinAlg::Vec2::AxisX();
	}
	return LinAlg::Vec2::AxisY();
}

//!< 2-シンプレクス (三角形)
LinAlg::Vec3 Collision::SignedVolume(const LinAlg::Vec3& A, const LinAlg::Vec3& B, const LinAlg::Vec3& C)
{
	//!< 原点 を ABC 上へ射影し、内部にあれば重心パラメータを返す
	{
		LinAlg::Vec3 Lambda;
		if (Barycentric(A, B, C, Lambda)) {
			return Lambda;
		}
	}
	
	//!< 3 辺インデックス
	constexpr std::array XYZ = { 0, 1, 2 };
	constexpr auto XYZSize = static_cast<int>(std::size(XYZ));
	//!< P を射影する 3 辺
	const std::array Edges = { A, B, C };

	//!< 3 辺に対して、重心パラメータと距離 (二乗) のペアを収集する
	std::vector<std::pair<LinAlg::Vec2, float>> LmdLen;
	std::ranges::transform(XYZ, std::back_inserter(LmdLen),
		[&](const auto& rhs) {
			const auto j = (rhs + 1) % XYZSize;
			const auto k = (rhs + 2) % XYZSize;
			//!< 1-シンプレクス に帰着して求める
			const auto Lmd = SignedVolume(Edges[j], Edges[k]);
			return std::pair<LinAlg::Vec2, float>({ Lmd, (Edges[j] * Lmd[0] + Edges[k] * Lmd[1]).LengthSq() });
		});
	//!< 距離が最小となるインデックスを求める
	const auto MinIt = std::ranges::min_element(LmdLen,
		[](const auto& lhs, const auto& rhs) {
			return lhs.second < rhs.second;
		});
	const auto Index = static_cast<int>(MinIt - std::cbegin(LmdLen));

	//!< 重心パラメータを返す
	LinAlg::Vec3 Lambda;
	Lambda[(Index + 0) % XYZSize] = 0.0f;
	Lambda[(Index + 1) % XYZSize] = MinIt->first[0];
	Lambda[(Index + 2) % XYZSize] = MinIt->first[1];
	return Lambda;
}

//!< 3-シンプレクス (四面体)
LinAlg::Vec4 Collision::SignedVolume(const LinAlg::Vec3& A, const LinAlg::Vec3& B, const LinAlg::Vec3& C, const LinAlg::Vec3& D)
{
	//!< 四面体を座標系と考える、その逆(転置)行列の四因子
	const auto M = LinAlg::Mat4(LinAlg::Vec4(A, 1.0f), LinAlg::Vec4(B, 1.0f), LinAlg::Vec4(C, 1.0f), LinAlg::Vec4(D, 1.0f)).Transpose();
	const auto Cofactors = LinAlg::Vec4(M.Cofactor(3, 0), M.Cofactor(3, 1), M.Cofactor(3, 2), M.Cofactor(3, 3));
	const auto& CofCmps = static_cast<const LinAlg::Float4&>(Cofactors);
	const auto Det = std::accumulate(std::cbegin(CofCmps), std::cend(CofCmps), 0.0f);
	//!< 原点が四面体内部にあれば、重心座標を返す
	if (std::ranges::all_of(CofCmps,
		[&](const auto rhs) {
			return std::signbit(Det) == std::signbit(rhs);
		})) {
		return Cofactors / Det;
	}

	//!< 4 面インデックス
	constexpr std::array XYZW = { 0, 1, 2, 3 };
	constexpr auto XYZWSize = static_cast<int>(std::size(XYZW));
	//!< P を射影する 4 面
	const std::array Faces = { A, B, C, D };

	//!< 4 面に対して、重心パラメータと距離 (二乗) のペアを収集
	std::vector<std::pair<LinAlg::Vec3, float>> LmdLen;
	std::ranges::transform(XYZW, std::back_inserter(LmdLen),
		[&](const auto& rhs) {
			const auto i = rhs;
			const auto j = (rhs + 1) % XYZWSize;
			const auto k = (rhs + 2) % XYZWSize;
			//!< 2-シンプレクス に帰着して求める
			const auto Lmd = SignedVolume(Faces[i], Faces[j], Faces[k]);
			return std::pair<LinAlg::Vec3, float>({ Lmd, (Faces[i] * Lmd[0] + Faces[j] * Lmd[1] + Faces[k] * Lmd[2]).LengthSq() });
		});
	//!< 距離が最小となるインデックスを求める
	const auto MinIt = std::ranges::min_element(LmdLen,
		[](const auto& lhs, const auto& rhs) {
			return lhs.second < rhs.second;
		});
	const auto Index = static_cast<int>(MinIt - std::cbegin(LmdLen));
	
	//!< 重心パラメータを返す
	LinAlg::Vec4 Lambda;
	Lambda[(Index + 0) % XYZWSize] = MinIt->first[0];
	Lambda[(Index + 1) % XYZWSize] = MinIt->first[1];
	Lambda[(Index + 2) % XYZWSize] = MinIt->first[2];
	Lambda[(Index + 3) % XYZWSize] = 0.0f;
	return Lambda;
}

bool Collision::Barycentric(const LinAlg::Vec3& A, const LinAlg::Vec3& B, const LinAlg::Vec3& C, 
	LinAlg::Vec3& Lambda)
{
	const auto N = LinAlg::Vec3::Normal(A, B, C);

	//!< 原点を 三角形 ABC に射影し P とする
	const auto P = N * A.Dot(N) / N.LengthSq();

	//!< 3 面インデックス
	constexpr std::array XYZ = { 0, 1, 2 };
	constexpr auto XYZSize = static_cast<int>(std::size(XYZ));

	//!< 3 面へ射影した面積を収集
	std::vector<float> Areas;
	std::ranges::transform(XYZ, std::back_inserter(Areas),
		[&](const auto& rhs) {
			const auto j = (rhs + 1) % XYZSize;
			const auto k = (rhs + 2) % XYZSize;

			const auto a = LinAlg::Vec2(A[j], A[k]);
			const auto b = LinAlg::Vec2(B[j], B[k]);
			const auto c = LinAlg::Vec2(C[j], C[k]);
			const auto AB = b - a;
			const auto AC = c - a;
			return LinAlg::Mat2(AB, AC).Determinant();
		});
	//!< 面積 (絶対値) が最大となるインデックスを求める
	const auto AbsMaxIt = std::ranges::max_element(Areas,
		[](const auto& lhs, const auto& rhs) {
			return std::abs(lhs) < std::abs(rhs);
		});
	const auto Index = static_cast<int>(AbsMaxIt - std::cbegin(Areas));

	//!< P、ABC を選択した面に射影 (Z が選択された場合は XY 平面といった具合になる)
	const auto X = (Index + 1) % XYZSize;
	const auto Y = (Index + 2) % XYZSize;
	const std::array PrjABC = { LinAlg::Vec2(A[X], A[Y]), LinAlg::Vec2(B[X], B[Y]), LinAlg::Vec2(C[X], C[Y]) };
	const auto PrjP = LinAlg::Vec2(P[X], P[Y]);

	//!< 射影後の P, ABC からなるサブ三角形それぞれの面積
	std::vector<float> SubAreas;
	std::ranges::transform(XYZ, std::back_inserter(SubAreas),
		[&](const auto& rhs) {
			const auto j = (rhs + 1) % XYZSize;
			const auto k = (rhs + 2) % XYZSize;
			return LinAlg::Mat2(PrjABC[j] - PrjP, PrjABC[k] - PrjP).Determinant();
		});

	//!< 重心パラメータ
	Lambda = LinAlg::Vec3(SubAreas[0], SubAreas[1], SubAreas[2]) / Areas[Index];

	//!< P が ABC の内部にあるかどうか (サブ三角形の面積の符号から判断)
	return std::ranges::all_of(SubAreas,
		[&](const auto rhs) {
			return std::signbit(Areas[Index]) == std::signbit(rhs);
		});
}
bool Collision::Barycentric(const LinAlg::Vec3& Pt, 
	const LinAlg::Vec3& A, const LinAlg::Vec3& B, const LinAlg::Vec3& C, 
	LinAlg::Vec3& Lambda) 
{
	//!< ABC から Pt を引くことで、原点の重心座標へ帰着
	return Barycentric(A - Pt, B - Pt, C - Pt, Lambda);
}

//!< A, B のサポートポイント、及びその差 C を求める
Collision::SupportPoint::Points Collision::SupportPoint::Get(const Physics::Shape* ShA, const LinAlg::Vec3& PosA, const LinAlg::Quat& RotA,
	const Physics::Shape* ShB, const LinAlg::Vec3& PosB, const LinAlg::Quat& RotB, 
	const LinAlg::Vec3& UDir, const float Bias)
{
	return { ShA->GetSupportPoint(PosA, RotA, UDir, Bias), ShB->GetSupportPoint(PosB, RotB, -UDir, Bias) };
}
Collision::SupportPoint::Points Collision::SupportPoint::Get(const Physics::RigidBody* RbA, 
	const Physics::RigidBody* RbB, 
	const LinAlg::Vec3& UDir, const float Bias)
{
	return Get(RbA->GetShape(), RbA->GetPosition(), RbA->GetRotation(), 
		RbB->GetShape(), RbB->GetPosition(), RbB->GetRotation(),
		UDir, Bias);
}

//!< サポートポイントが四面体をなしていない場合、四面体を形成する
void Collision::SupportPoint::ToTetrahedron(const Physics::Shape* ShA, const LinAlg::Vec3& PosA, const LinAlg::Quat& RotA, 
	const Physics::Shape* ShB, const LinAlg::Vec3& PosB, const LinAlg::Quat& RotB, 
	std::vector<SupportPoint::Points>& Sps)
{
	if (1 == std::size(Sps)) {
		Sps.emplace_back(Collision::SupportPoint::Get(ShA, PosA, RotA, ShB, PosB, RotB, -Sps[0].GetC().Normalize(), 0.0f));
	}
	if (2 == std::size(Sps)) {
		const auto AB = Sps[1].GetC() - Sps[0].GetC();
		LinAlg::Vec3 U, V;
		AB.GetOrtho(U, V);
		Sps.emplace_back(Collision::SupportPoint::Get(ShA, PosA, RotA, ShB, PosB, RotB, U, 0.0f));
	}
	if (3 == std::size(Sps)) {
		const auto AB = Sps[1].GetC() - Sps[0].GetC();
		const auto AC = Sps[2].GetC() - Sps[0].GetC();
		Sps.emplace_back(Collision::SupportPoint::Get(ShA, PosA, RotA, ShB, PosB, RotB, AB.Cross(AC).Normalize(), 0.0f));
	}
}

//!< サポートポイントを Bias 分だけ拡張する
LinAlg::Vec3 Collision::SupportPoint::Expand(const float Bias, std::vector<Collision::SupportPoint::Points>& Sps)
{
	const auto Center = std::accumulate(std::cbegin(Sps), std::cend(Sps), LinAlg::Vec3::Zero(), 
		[](const auto& Acc, const auto& i) { 
			return Acc + i.GetC(); 
		}) / static_cast<float>(std::size(Sps));
	std::ranges::transform(Sps, std::begin(Sps),
		[&](const auto& i) {
			const auto Dir = (i.GetC() - Center).Normalize() * Bias;
			return Collision::SupportPoint::Points(i.GetA() + Dir, i.GetB() - Dir);
		});
	return Center;
}

bool Collision::Intersection::GJK(const Physics::Shape* ShA, const LinAlg::Vec3& PosA, const LinAlg::Quat& RotA,
	const Physics::Shape* ShB, const LinAlg::Vec3& PosB, const LinAlg::Quat& RotB,
	OnIntersectGJK OnIntersect, const float Bias, const bool WithClosestPoint,
	LinAlg::Vec3& OnA, LinAlg::Vec3& OnB)
{
	std::vector<Collision::SupportPoint::Points> Sps;
	Sps.reserve(4); //!< 4 枠

	//!< (1, 1, 1) 方向のサポートポイントを求める
	Sps.emplace_back(Collision::SupportPoint::Get(ShA, PosA, RotA, ShB, PosB, RotB, LinAlg::Vec3::UnitXYZ(), 0.0f));

	//!< 原点を元に反対側
	auto Dir = -Sps.back().GetC();
	auto ClosestDistSq = (std::numeric_limits<float>::max)();
	auto HasIntersection = false;
	LinAlg::Vec4 Lambda;
	while (!HasIntersection) {
		//!< Dir 方向のサポートポイントを求める
		const auto Pt = Collision::SupportPoint::Get(ShA, PosA, RotA, ShB, PosB, RotB, (Dir = Dir.Normalize()), 0.0f);

		//!< Pt が既存の点の場合、もうこれ以上拡張できない、衝突無し
		if (std::ranges::any_of(Sps,
			[&](const auto& i) {
				return Pt.GetC().NearlyEqual(i.GetC());
			})) {
			break;
		}

		//!< 新しいサポートポイントを追加
		Sps.emplace_back(Pt);

		//!< 最近接点を求めない場合のみ早期終了可能
		if (!WithClosestPoint) {
			//!< Pt が原点を超えて反対側でない場合は原点を含まない、衝突無し
			if (Dir.Dot(Pt.GetC()) < 0.0f) {
				break;
			}
		}

		//!< シンプレクス上での原点の重心パラメータと次の探索方向を求める
		//!< (Dir の長さがほぼゼロなら原点を含むとみなし true を返す、衝突)
		if ((HasIntersection = Collision::SupportPoint::SimplexSignedVolumes(Sps, Dir, Lambda))) {
			break;
		}

		//!< 最短距離を更新する、更新できない場合は終了
		//!< (2 つの三角形からなる四角形の対角線上に原点が位置するような場合、2 つの三角形間で切り替わり続けるループになるのを回避する為にこうしている)
		const auto DistSq = Dir.LengthSq();
		if (DistSq >= ClosestDistSq) {
			break;
		}
		ClosestDistSq = DistSq;

		//!< 有効なサポートポイントの要素 (Lambda が非 0.0) のみを残す
		auto Index = 0;
		const auto SpsRange = std::ranges::remove_if(Sps,
			[&](const auto& i) {
				return 0.0f == Lambda[Index++];
			});
		Sps.erase(std::ranges::cbegin(SpsRange), std::ranges::cend(SpsRange));

		//!< Lambda の有効な (非 0.0) 要素を前へ集める (ケツにはゴミが残るが、Sps 個数分しかアクセスしない)
		[[maybe_unused]] const auto DmyRange = std::ranges::remove(static_cast<LinAlg::Float4&>(Lambda), 0.0f);

		//!< 3-シンプレックス (四面体) になっていれば原点を含む、衝突
		HasIntersection = (4 == std::size(Sps));
	}

	//!< 衝突あり (GJK では衝突の有無のみ、衝突点は EPA 等で求める)
	if (HasIntersection) {
		if (nullptr != OnIntersect) {
			//!< 衝突点を求める (EPA 等)
			OnIntersect(ShA, PosA, RotA,
				ShB, PosB, RotB,
				Sps, Bias, 
				OnA, OnB);
		}
		return true;
	}

	//!< 最近接点を求める
	if (WithClosestPoint) {
		auto Index = 0;
		OnA = std::accumulate(std::cbegin(Sps), std::cend(Sps), LinAlg::Vec3::Zero(),
			[&](const auto& Acc, const auto& i) {
				return Acc + i.GetA() * Lambda[Index++];
			});
		Index = 0;
		OnB = std::accumulate(std::cbegin(Sps), std::cend(Sps), LinAlg::Vec3::Zero(),
			[&](const auto& Acc, const auto& i) {
				return Acc + i.GetB() * Lambda[Index++];
			});
	}

	return false;
}
bool Collision::Intersection::GJK(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB,
	OnIntersectGJK OnIntersect, const float Bias,
	LinAlg::Vec3& OnA, LinAlg::Vec3& OnB) {
	return GJK(RbA->GetShape(), RbA->GetPosition(), RbA->GetRotation(),
		RbB->GetShape(), RbB->GetPosition(), RbB->GetRotation(),
		OnIntersect, Bias,
		true,
		OnA, OnB);
}

void Collision::Intersection::EPA(const Physics::Shape* ShA, const LinAlg::Vec3& PosA, const LinAlg::Quat& RotA,
	const Physics::Shape* ShB, const LinAlg::Vec3& PosB, const LinAlg::Quat& RotB, 
	std::vector<SupportPoint::Points>& Sps, const float Bias, 
	LinAlg::Vec3& OnA, LinAlg::Vec3& OnB)
{
	//!< EPA では四面体を必要とするので、(四面体でない場合は) 四面体へ拡張する
	SupportPoint::ToTetrahedron(ShA, PosA, RotA, ShB, PosB, RotB, Sps);

	//!< シンプレックスを拡張する (A, B の衝突点がほぼ同一の場合に法線が上手く求められない場合があるため、バイアス分拡張する)
	//!< この過程で中心が求まるので覚えておく
	const auto Center = SupportPoint::Expand(Bias, Sps);
	
	//!< 3 シンプレクス (四面体) の 4 面 に対して法線が外側に向くように三角形のインデックスを生成する
	constexpr std::array Faces = { 0, 1, 2, 3 };
	constexpr auto FacesSize = static_cast<uint32_t>(std::size(Faces));
	std::vector<Collision::TriInds> Tris;
	std::ranges::transform(Faces, std::back_inserter(Tris),
		[&](const auto& rhs) {
			const auto i = (rhs + 0) % FacesSize;
			const auto j = (rhs + 1) % FacesSize;
			const auto k = (rhs + 2) % FacesSize;
			const auto l = (rhs + 3) % FacesSize; //!< 四面体の頂点
			return Collision::Distance::IsFrontTriangle(Sps[l].GetC(), Sps[i].GetC(), Sps[j].GetC(), Sps[k].GetC()) ? 
				Collision::TriInds({ j, i, k }) : Collision::TriInds({ i, j, k });
		});

	//!< 原点に最も近い面を見つける為にシンプレックスを拡張する (原点の向きに限定して凸包を形成していく感じ)
	while (true) {
		//!< 原点に最も近い三角形 ABC を取得
		const auto& CTri = *SupportPoint::Distance::Closest(LinAlg::Vec3::Zero(), Sps, Tris);
		const auto& A = Sps[CTri[0]].GetC(), B = Sps[CTri[1]].GetC(), C = Sps[CTri[2]].GetC();

		//!< 三角形の法線
		const auto N = LinAlg::Vec3::UnitNormal(A, B, C);

		//!< 法線方向のサポートポイントを取得
		const auto Pt = Collision::SupportPoint::Get(ShA, PosA, RotA, ShB, PosB, RotB, N, Bias);

		//!< Pt が ABC 法線の反対側ならこれ以上拡張できない
		if ((Pt.GetC() - A).Dot(N) <= 0.0f) {
			break;
		}

		//!< 既存の場合これ以上拡張できない
		if (std::ranges::any_of(Tris, 
			[&](const auto& i) {
				//!< 三角形群
				return std::ranges::any_of(i,
					[&](const auto& j) { 
						//!< 3 頂点のいずれかに等しい場合、既出
						constexpr auto Eps = 0.01f; 
						return Sps[j].GetC().NearlyEqual(Pt.GetC(), Eps);
					});
			})) {
			break;
		}

		Sps.emplace_back(Pt);

		//!< Pt を向く三角形を削除、削除できない場合は終了
		{
			const auto Range = std::ranges::remove_if(Tris,
				[&](const auto& i) {
					return Distance::IsFrontTriangle(Pt.GetC(), Sps[i[0]].GetC(), Sps[i[1]].GetC(), Sps[i[2]].GetC());
				});
			Tris.erase(std::ranges::cbegin(Range), std::ranges::cend(Range));
			//!< 削除できない場合は終了
			if (std::ranges::empty(Range)) {
				break; 
			}
		}

		//!< ぶら下がった辺 (削除した三角形群と、残った三角形群の境界となるような辺) を収集、見つからなければ終了
		{
			//!< 複数の三角形から共有されていないようなユニークな辺を収集すればよい
			std::vector<Collision::EdgeIndsWithCount> DanglingEdges;
			Convex::CollectUniqueEdges(Tris, DanglingEdges);
			//!< そのような辺が無ければ終了
			if (std::empty(DanglingEdges)) {
				break;
			}

			//!< Pt(A) とぶら下がり辺(BC)との新しい三角形を追加
			const auto A = static_cast<uint32_t>(std::size(Sps)) - 1;
			std::ranges::transform(DanglingEdges, std::back_inserter(Tris),
				[&](const auto& i) {
					const auto B = i.first[0], C = i.first[1];
					//!< 法線が外側を向くようにインデックスを生成する
					if (Distance::IsFrontTriangle(Center, Sps[A].GetC(), Sps[B].GetC(), Sps[C].GetC())) {
						return Collision::TriInds({ A, C, B });
					}
					else {
						return Collision::TriInds({ A, B, C });
					}
				});
		}
	}

	{
		//!< 原点に最も近い三角形 ABC を取得
		const auto& CTri = *SupportPoint::Distance::Closest(LinAlg::Vec3::Zero(), Sps, Tris);
		const auto& A = Sps[CTri[0]], B = Sps[CTri[1]], C = Sps[CTri[2]];

		//!< ABC 上での原点の重心座標を取得
		LinAlg::Vec3 Lambda;
		if (Barycentric(A.GetC(), B.GetC(), C.GetC(), Lambda)) {
			if(!Lambda.IsValid()) {
				Lambda = LinAlg::Vec3::AxisX();
			}
			OnA = A.GetA() * Lambda[0] + B.GetA() * Lambda[1] + C.GetA() * Lambda[2];
			OnB = A.GetB() * Lambda[0] + B.GetB() * Lambda[1] + C.GetB() * Lambda[2];
#ifdef _DEBUG
			//LOG(std::data(std::format("PenetrationDistSq = {}\n", (OnB - OnA).LengthSq())));
#endif
		}
		else {
			//!< #TODO 原点が三角形の外側にある
			LOG(std::data(std::format("EPA: Barycentric failed\n")));
		}
	}
}
bool Collision::Intersection::GJK_EPA(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, 
	const float Bias,
	LinAlg::Vec3& OnA, LinAlg::Vec3& OnB)
{
	return GJK(RbA->GetShape(), RbA->GetPosition(), RbA->GetRotation(),
		RbB->GetShape(), RbB->GetPosition(), RbB->GetRotation(),
		EPA, Bias, true,
		OnA, OnB);
}

void Collision::Closest::GJK(const Physics::RigidBody* RbA, 
	const Physics::RigidBody* RbB, 
	LinAlg::Vec3& OnA, LinAlg::Vec3& OnB)
{
	GJK(RbA->GetShape(), RbA->GetPosition(), RbA->GetRotation(),
		RbB->GetShape(), RbB->GetPosition(), RbB->GetRotation(),
		OnA, OnB);
}

#ifdef _DEBUG
void Collision::SignedVolumeTest()
{
	constexpr auto Eps = 0.001f;
	{
		const std::vector OrgPts = {
			LinAlg::Vec3(0.0f, 0.0f, 0.0f),
			LinAlg::Vec3(1.0f, 0.0f, 0.0f),
			LinAlg::Vec3(0.0f, 1.0f, 0.0f),
			LinAlg::Vec3(0.0f, 0.0f, 1.0f),
		};
		{
			//!< 検証済
			std::vector<LinAlg::Vec3> Pts;
			Pts.resize(std::size(OrgPts));
			std::ranges::transform(OrgPts, std::begin(Pts), std::bind(std::plus(), std::placeholders::_1, LinAlg::Vec3(1.0f, 1.0f, 1.0f)));
			const auto Lambda = SignedVolume(Pts[0], Pts[1], Pts[2], Pts[3]);
			const auto V = Pts[0] * Lambda[0] + Pts[1] * Lambda[1] + Pts[2] * Lambda[2] + Pts[3] * Lambda[3];

			const auto CorrectLambda = LinAlg::Vec4(1.0f, 0.0f, 0.0f, 0.0f);
			const auto CorrectV = LinAlg::Vec3::One();
			if (!Lambda.NearlyEqual(CorrectLambda, Eps)) {
				__debugbreak();
			}
			if (!V.NearlyEqual(CorrectV, Eps)) {
				__debugbreak();
			}
		}
		{
			//!< 検証済
			std::vector<LinAlg::Vec3> Pts;
			Pts.resize(std::size(OrgPts));
			std::ranges::transform(OrgPts, std::begin(Pts), std::bind(std::plus(), std::placeholders::_1, LinAlg::Vec3(-1.0f, -1.0f, -1.0f) * 0.25f));
			const auto Lambda = SignedVolume(Pts[0], Pts[1], Pts[2], Pts[3]);
			const auto V = Pts[0] * Lambda[0] + Pts[1] * Lambda[1] + Pts[2] * Lambda[2] + Pts[3] * Lambda[3];

			const auto CorrectLambda = LinAlg::Vec4(0.25f, 0.25f, 0.25f, 0.25f);
			const auto CorrectV = LinAlg::Vec3::Zero();
			if (!Lambda.NearlyEqual(CorrectLambda, Eps)) {
				__debugbreak();
			}
			if (!V.NearlyEqual(CorrectV, Eps)) {
				__debugbreak();
			}
		}
		{
			//!< 検証済
			std::vector<LinAlg::Vec3> Pts;
			Pts.resize(std::size(OrgPts));
			std::ranges::transform(OrgPts, std::begin(Pts), std::bind(std::plus(), std::placeholders::_1, LinAlg::Vec3(-1.0f, -1.0f, -1.0f)));
			const auto Lambda = SignedVolume(Pts[0], Pts[1], Pts[2], Pts[3]);
			const auto V = Pts[0] * Lambda[0] + Pts[1] * Lambda[1] + Pts[2] * Lambda[2] + Pts[3] * Lambda[3];

			const auto CorrectLambda = LinAlg::Vec4(0.0f, 0.333f, 0.333f, 0.333f);
			const auto CorrectV = LinAlg::Vec3(-0.667f, -0.667f, -0.667f);
			if (!Lambda.NearlyEqual(CorrectLambda, Eps)) {
				__debugbreak();
			}
			if (!V.NearlyEqual(CorrectV, Eps)) {
				__debugbreak();
			}
		}
		{
			//!< 検証済
			std::vector<LinAlg::Vec3> Pts;
			Pts.resize(std::size(OrgPts));
			std::ranges::transform(OrgPts, std::begin(Pts), std::bind(std::plus(), std::placeholders::_1, LinAlg::Vec3(1.0f, 1.0f, -0.5f)));
			const auto Lambda = SignedVolume(Pts[0], Pts[1], Pts[2], Pts[3]);
			const auto V = Pts[0] * Lambda[0] + Pts[1] * Lambda[1] + Pts[2] * Lambda[2] + Pts[3] * Lambda[3];

			const auto CorrectLambda = LinAlg::Vec4(0.5f, 0.0f, 0.0f, 0.5f);
			const auto CorrectV = LinAlg::Vec3(1.0f, 1.0f, 0.0f);
			if (!Lambda.NearlyEqual(CorrectLambda, Eps)) {
				__debugbreak();
			}
			if (!V.NearlyEqual(CorrectV, Eps)) {
				__debugbreak();
			}
		}
	}
	{
		//!< 検証済
		const std::array Pts = {
			LinAlg::Vec3(51.1996613f, 26.1989613f, 1.91339576f),
			LinAlg::Vec3(-51.0567360f, -26.0565681f, -0.436143428f),
			LinAlg::Vec3(50.8978920f, -24.1035538f, -1.04042661f),
			LinAlg::Vec3(-49.1021080f, 25.8964462f, -1.04042661f)
		};
		const auto Lambda = SignedVolume(Pts[0], Pts[1], Pts[2], Pts[3]);
		const auto V = Pts[0] * Lambda[0] + Pts[1] * Lambda[1] + Pts[2] * Lambda[2] + Pts[3] * Lambda[3];

		//!< 答え合わせ
		const auto CorrectLambda = LinAlg::Vec4(0.290f, 0.302f, 0.206f, 0.202f);
		const auto CorrectV = LinAlg::Vec3::Zero();
		if (!Lambda.NearlyEqual(CorrectLambda, Eps)) {
			__debugbreak();
		}
		if (!V.NearlyEqual(CorrectV, Eps)) {
			__debugbreak();
		}
	}
}
#endif
