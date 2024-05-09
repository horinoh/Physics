#include <random>

#include "Convex.h"
#include "Log.h"

//!< 頂点を「なるべく」包含するような四面体を作成
void Convex::BuildTetrahedron(const std::vector<Math::Vec3>& Pts, std::vector<Math::Vec3>& Vertices, std::vector<Collision::TriInds >& Indices)
{
	//!< 特定の軸 (ここではX) に一番遠い点
	std::array<Math::Vec3, 4> P = { *Collision::Distance::Farthest(Pts, Math::Vec3::AxisX()) };
	//< 前出の逆向きの軸軸に一番遠い点
	P[1] = *Collision::Distance::Farthest(Pts, -P[0]);
	//!< 前出の 2 点からなる線分に一番遠い点
	P[2] = *Collision::Distance::Farthest(Pts, P[0], P[1]);
	//!< 前出の 3 点からなる三角形に一番遠い点
	P[3] = *Collision::Distance::Farthest(Pts, P[0], P[1], P[2]);

	//!< CCW になるように調整
	if (Collision::Distance::IsFront(P[0], P[1], P[2], P[3])) {
		std::swap(P[0], P[1]);
	}

	//!< 四面体の頂点
	Vertices.emplace_back(P[0]);
	Vertices.emplace_back(P[1]);
	Vertices.emplace_back(P[2]);
	Vertices.emplace_back(P[3]);

	//!< 四面体のインデックス
	Indices.emplace_back(Collision::TriInds({ 0, 1, 2 }));
	Indices.emplace_back(Collision::TriInds({ 0, 2, 3 }));
	Indices.emplace_back(Collision::TriInds({ 2, 1, 3 }));
	Indices.emplace_back(Collision::TriInds({ 1, 0, 3 }));
}

//!< 凸包の内部点を削除
void Convex::RemoveInternal(const std::vector<Math::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices, std::vector<Math::Vec3>& Pts)
{
	//!< 内部点を除外
	{
		const auto Range = std::ranges::remove_if(Pts, [&](const auto& Pt) {
			return IsInternal(Pt, Vertices, Indices);
			});
		Pts.erase(std::ranges::cbegin(Range), std::ranges::cend(Range));
	}

	//!< 既存と同一とみなせる点は除外
	{
		const auto Range = std::ranges::remove_if(Pts, [&](const auto& Pt) {
			//!< 同一とみなせる点
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
			//!< 既出の辺かどうかを調べる (真逆も既出として扱う)
			const auto It = std::ranges::find_if(EdgeCounts, [&](const auto& k) {
				return (k.first[0] == j[0] && k.first[1] == j[1]) || (k.first[0] == j[1] && k.first[1] == j[0]);
			});
			if (std::cend(EdgeCounts) == It) {
				//!< 新規の辺として追加
				EdgeCounts.emplace_back(Collision::EdgeIndsCount({ j, 0 }));
			}
			else {
				//!< 既出の辺となったらカウンタをインクリメントして情報を更新
				++It->second;
			}
		});
	});

	//!< ユニークでない辺 (カウンタが 0 より大きい) を削除
	const auto Range = std::ranges::remove_if(EdgeCounts, [](const auto& i) { return i.second > 0; });
	EdgeCounts.erase(std::ranges::cbegin(Range), std::ranges::cend(EdgeCounts));
}

//!< ハイポリを食わせるとかなり時間がかかる上に結局ハイポリの凸包ができるだけなのでコリジョンとして現実的ではない、ローポリを食わせること
void Convex::BuildConvexHull(const std::vector<Math::Vec3>& Pts, std::vector<Math::Vec3>& Vertices, std::vector<Collision::TriInds>& Indices)
{
	LOG(std::data(std::format("Building convex hull...\n")));

	//!< まずは「なるべく」包含するような四面体を作成
	BuildTetrahedron(Pts, Vertices, Indices);

	//!< 内部点の除外 -> 外部点が残る
	auto External = Pts;
	RemoveInternal(Vertices, Indices, External);

	//!< 外部点が無くなるまで繰り返す
	while (!std::empty(External)) {
		LOG(std::data(std::format("Rest vertices = {}\n", size(External))));

		//!< 最遠点を見つける
		const auto ExFarIt = Collision::Distance::Farthest(External, External[0]);

		std::vector<Collision::EdgeIndsCount> DanglingEdges;
		{
			//!< 最遠点を向いていない三角形 (A) と、向いている三角形 (B) に分割
			//!< partition は以下のように返す
			//!<	A ラムダ式が true	: [begin(Indices), begin(Range)]
			//!<	B ラムダ式が false	: [begin(Range), end(Range)]
			const auto Range = std::ranges::partition(Indices, [&](const auto& i) {
				return !Collision::Distance::IsFront(*ExFarIt, Vertices[i[0]], Vertices[i[1]], Vertices[i[2]]);
			});

			//!< A, B の境界となるような辺を収集する (B の中から他の三角形と辺を共有しないユニークな辺のみを収集すれば良い)
			CollectUniqueEdges(std::ranges::cbegin(Range), std::ranges::cend(Range), DanglingEdges);

			//!< (辺は収集済) ここまで来たら B は削除してよい  
			Indices.erase(std::ranges::cbegin(Range), std::ranges::cend(Range));
		}

		//!< 凸包の更新
		{
			//!< 最遠点を頂点として追加する
			Vertices.emplace_back(*ExFarIt);

			//!< 最遠点のインデックス
			const auto FarIndex = static_cast<uint32_t>(std::size(Vertices) - 1);
			//!< 最遠点とユニーク辺からなる三角形群を追加
			std::ranges::transform(DanglingEdges, std::back_inserter(Indices), [&](const auto& i) {
				return Collision::TriInds({ i.first[0], i.first[1], FarIndex });
			});
		}

		//!< 外部点の更新
		{
			//!< ここまで済んだら最遠点は削除してよい
			External.erase(ExFarIt);

			//!< 更新した凸包に対して内部点を削除する
			RemoveInternal(Vertices, Indices, External);
		}
	}
}

Math::Vec3 Convex::Uniform::CalcCenterOfMass(const Collision::AABB& Aabb, const std::vector<Math::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices)
{
	LOG(std::data(std::format("Calculating center of mass (Uniform)...\n")));

	//!< 各軸にサンプリングする個数 (計算に時間がかかる)
	constexpr auto SampleCount = 100;

	auto CenterOfMass = Math::Vec3::Zero();
	auto Sampled = 0;
	const auto Delta = Aabb.GetExtent() / static_cast<float>(SampleCount);
	for (auto x = 0; x < SampleCount; ++x) {
		for (auto y = 0; y < SampleCount; ++y) {
			for (auto z = 0; z < SampleCount; ++z) {
				//!< AABB 内のサンプル点
				const auto Pt = Aabb.Min + Math::Vec3(Delta.X() * x, Delta.Y() * y, Delta.Z() * z);
				if (IsInternal(Pt, Vertices, Indices)) {
					//!< 内部点なら収集
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

	//!< 各軸にサンプリングする個数 (計算に時間がかかる)
	constexpr auto SampleCount = 100;

	auto InertiaTensor = Math::Mat3::Zero();
	auto Sampled = 0;
	const auto Delta = Aabb.GetExtent() / static_cast<float>(SampleCount);
	for (auto x = 0; x < SampleCount; ++x) {
		for (auto y = 0; y < SampleCount; ++y) {
			for (auto z = 0; z < SampleCount; ++z) {
				//!< AABB 内のサンプル点 (重心からの相対)
				const auto Pt = Aabb.Min + Math::Vec3(Delta.X() * x, Delta.Y() * y, Delta.Z() * z) - CenterOfMass;
				if (IsInternal(Pt, Vertices, Indices)) {
					//!< 内部点なら収集する
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
	//!< 真乱数でシードを生成する
	std::random_device SeedGen;
	//!< 疑似乱数 (メルセンヌツイスター) (シード使用)
	std::mt19937 MersenneTwister(SeedGen());
#else
	//!< 疑似乱数 (メルセンヌツイスター) (毎回同じ)
	std::mt19937 MersenneTwister;
#endif
	//!< 分布
	std::uniform_real_distribution<float> Distribution(0.0f, 1.0f);

	//!< サンプリングする個数
	constexpr auto SampleCount = 10000;

	auto CenterOfMass = Math::Vec3::Zero();
	auto Sampled = 0;
	const auto Ext = Aabb.GetExtent();
	for (auto i = 0; i < SampleCount; ++i) {
		//!< AABB 内のサンプル点
		const auto Pt = Aabb.Min + Math::Vec3(Ext.X() * Distribution(MersenneTwister), Ext.Y() * Distribution(MersenneTwister), Ext.Z() * Distribution(MersenneTwister));
		if (IsInternal(Pt, Vertices, Indices)) {
			//!< 内部点なら収集
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

	//!< サンプリングする個数
	constexpr auto SampleCount = 10000;

	auto InertiaTensor = Math::Mat3::Zero();
	auto Sampled = 0;
	const auto Ext = Aabb.GetExtent();
	for (auto i = 0; i < SampleCount; ++i) {
		//!< AABB 内のサンプル点 (重心からの相対)
		const auto Pt = Aabb.Min + Math::Vec3(Ext.X() * Distribution(MersenneTwister), Ext.Y() * Distribution(MersenneTwister), Ext.Z() * Distribution(MersenneTwister)) - CenterOfMass;
		if (IsInternal(Pt, Vertices, Indices)) {
			//!< 内部点なら収集する
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
	//!< 四面体の 4 頂点からなる行列
	const auto M = Math::Mat3(
		Math::Vec3(B.X() - A.X(), C.X() - A.X(), D.X() - A.X()),
		Math::Vec3(B.Y() - A.Y(), C.Y() - A.Y(), D.Y() - A.Y()),
		Math::Vec3(B.Z() - A.Z(), C.Z() - A.Z(), D.Z() - A.Z()));
	const auto Det = M.Determinant();

	auto XX = 0.0f, YY = 0.0f, ZZ = 0.0f, XY = 0.0f, XZ = 0.0f, YZ = 0.0f;
	const std::array Pts = { A, B, C, D };
	for (auto i = 0; i < std::size(Pts); ++i) {
		for (auto j = i; j < std::size(Pts); ++j) {
			//!< 対角線
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
	const auto A = Math::Vec3::Zero(); //!< CenterOfMass - CenterOfMass なので
	std::ranges::for_each(Indices, [&](const auto& i) {
		const auto B = Vertices[i[0]] - CenterOfMass;
		const auto C = Vertices[i[1]] - CenterOfMass;
		const auto D = Vertices[i[2]] - CenterOfMass;
		TotalInertiaTensor += CalcInertiaTensor(A, B, C, D);
		TotalVolume += Collision::Volume::Tetrahedron(A, B, C, D);
	});
	return TotalInertiaTensor / TotalVolume;
}