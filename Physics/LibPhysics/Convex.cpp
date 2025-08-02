#include <random>

#include "Convex.h"
#include "Log.h"

//!< 頂点を「なるべく」包含するような四面体を作成
void Convex::BuildTetrahedron(const std::vector<LinAlg::Vec3>& Mesh, std::vector<LinAlg::Vec3>& Vertices, std::vector<Collision::TriInds>& Indices)
{
	//!< 特定の軸 (ここではX) に一番遠い点
	std::array<LinAlg::Vec3, 4> Pts = { *Collision::Distance::Farthest(Mesh, LinAlg::Vec3::AxisX()) };
	//< 前出の逆向きに一番遠い点
	Pts[1] = *Collision::Distance::Farthest(Mesh, -Pts[0]);
	//!< 前出の 2 点からなる線分に一番遠い点
	Pts[2] = *Collision::Distance::Farthest(Mesh, Pts[0], Pts[1] - Pts[0]);
	//!< 前出の 3 点からなる三角形に一番遠い点
	Pts[3] = *Collision::Distance::Farthest(Mesh, Pts[0], Pts[1], Pts[2]);

	//!< CCW になるように調整
	if (Collision::Distance::IsFrontTriangle(Pts[0], Pts[1], Pts[2], Pts[3])) {
		std::swap(Pts[0], Pts[1]);
	}

	//!< 四面体の頂点
	Vertices.insert(std::end(Vertices), std::begin(Pts), std::end(Pts));

	//!< 四面体のインデックス
	Indices.insert(std::end(Indices), {{0, 1, 2}, {0, 2, 3}, {2, 1, 3}, {1, 0, 3}});
}

//!< 凸包の内部点を削除
void Convex::RemoveInternal(const std::vector<LinAlg::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices, std::vector<LinAlg::Vec3>& Mesh)
{
	//!< 内部点を除外
	{
		const auto Range = std::ranges::remove_if(Mesh,
			[&](const auto& Pt) {
				return IsInternal(Pt, Vertices, Indices);
			});
		Mesh.erase(std::ranges::cbegin(Range), std::ranges::cend(Range));
	}

	//!< 既存と同一とみなせる点は除外
	{
		const auto Range = std::ranges::remove_if(Mesh,
			[&](const auto& Pt) {
				//!< 同一とみなせる点が存在
				return std::ranges::any_of(Vertices, 
					[&](const auto rhs) {
						return rhs.NearlyEqual(Pt);
					});
			});
		Mesh.erase(std::ranges::cbegin(Range), std::ranges::cend(Range));
	}
}

//!< 辺を共有しないユニークな辺を収集
void Convex::CollectUniqueEdges(std::vector<Collision::TriInds>::const_iterator Begin, std::vector<Collision::TriInds>::const_iterator End, std::vector<Collision::EdgeIndsWithCount>& EdgeCounts)
{
	std::for_each(Begin, End, 
		[&](const auto& i) {
			//!< 三角形の三辺
			const std::array Edges = {
				Collision::EdgeInds({ i[0], i[1] }),
				Collision::EdgeInds({ i[1], i[2] }),
				Collision::EdgeInds({ i[2], i[0] }),
			};
			std::ranges::for_each(Edges,
				[&](const auto& j) {
					//!< (真逆も既出として扱い) 既出の辺かどうかを調べる 
					const auto It = std::ranges::find_if(EdgeCounts, 
						[&](const auto& k) {
							return (k.first[0] == j[0] && k.first[1] == j[1]) || (k.first[0] == j[1] && k.first[1] == j[0]);
						});
					if (std::cend(EdgeCounts) == It) {
						//!< 見つからなかったので、新規の辺として追加
						EdgeCounts.emplace_back(Collision::EdgeIndsWithCount({ j, 0 }));
					}
					else {
						//!< 既出の辺なのでカウンタをインクリメント (更新)
						++It->second;
					}
			});
		});

	//!< ユニークでない辺を削除 (カウンタが 0 のものだけ残す) 
	const auto Range = std::ranges::remove_if(EdgeCounts,
		[](const auto& i) {
			return i.second > 0;
		});
	EdgeCounts.erase(std::ranges::cbegin(Range), std::ranges::cend(EdgeCounts));
}

//!< ハイポリを食わせるとかなり時間がかかる上に結局ハイポリの凸包ができるだけなのでコリジョンとして現実的ではない、ローポリを食わせること
void Convex::BuildConvexHull(const std::vector<LinAlg::Vec3>& Mesh, std::vector<LinAlg::Vec3>& Vertices, std::vector<Collision::TriInds>& Indices)
{
	//PERFORMANCE_COUNTER_FUNC();
	PERFORMANCE_COUNTER("BuildConvexHull()");

	LOG(std::data(std::format("Building convex hull...\n")));

	//!< まずは概ね包含するような四面体を作成する
	BuildTetrahedron(Mesh, Vertices, Indices);

	//!< 内部点の除外 -> 外部点が残る
	auto External = Mesh;
	RemoveInternal(Vertices, Indices, External);

	//!< 外部点が無くなるまで繰り返す
	while (!std::empty(External)) {
		LOG(std::data(std::format("Rest vertices = {}\n", std::size(External))));

		//!< (ここでは External[0]の方向に) 最遠点を見つける
		const auto FarIt = Collision::Distance::Farthest(External, External[0]);

		//!< ぶら下がった辺を見つける
		std::vector<Collision::EdgeIndsWithCount> DanglingEdges;
		{
			//!< 最遠点を向いていない三角形 (A) と、向いている三角形 (B) に分割
			//!< partition は以下のように返す
			//!<	A ラムダ式が true		: [begin(Indices), begin(Range)]
			//!<	B ラムダ式が false	: [begin(Range), end(Range)]
			const auto Range = std::ranges::partition(Indices, 
				[&](const auto& i) {
					return !Collision::Distance::IsFrontTriangle(*FarIt, Vertices[i[0]], Vertices[i[1]], Vertices[i[2]]);
				});

			//!< A, B の境界となるような辺を収集する (B の中から他の三角形と辺を共有しないユニークな辺を収集)
			CollectUniqueEdges(std::ranges::cbegin(Range), std::ranges::cend(Range), DanglingEdges);

			//!< (辺は収集済) ここまで来たら B は削除してよい  
			Indices.erase(std::ranges::cbegin(Range), std::ranges::cend(Range));
		}

		//!< 凸包の更新
		{
			//!< 最遠点を頂点として追加する
			Vertices.emplace_back(*FarIt);

			//!< 最遠点 (最後の要素) のインデックス 
			const auto FarIndex = static_cast<uint32_t>(std::size(Vertices) - 1);
			//!< 最遠点とユニーク辺からなる三角形群を追加
			std::ranges::transform(DanglingEdges, std::back_inserter(Indices), 
				[&](const auto& i) {
					return Collision::TriInds({ i.first[0], i.first[1], FarIndex });
				});
		}

		//!< 外部点の更新
		{
			//!< ここまで済んだら最遠点は削除してよい
			External.erase(FarIt);

			//!< 更新した凸包に対して内部点になったものを削除する
			RemoveInternal(Vertices, Indices, External);
		}
	}
}

LinAlg::Vec3 Convex::Uniform::CalcCenterOfMass(const Collision::AABB& Ab, const std::vector<LinAlg::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices)
{
	LOG(std::data(std::format("Calculating center of mass (Uniform)...\n")));

	//!< 各軸にサンプリングする個数 (計算に時間がかかる)
	constexpr auto SampleCount = 100;

	auto CenterOfMass = LinAlg::Vec3::Zero();
	auto Sampled = 0;
	const auto Delta = Ab.GetExtent() / static_cast<float>(SampleCount);
	for (auto x = 0; x < SampleCount; ++x) {
		for (auto y = 0; y < SampleCount; ++y) {
			for (auto z = 0; z < SampleCount; ++z) {
				//!< AABB 内のサンプル点
				const auto Pt = Ab.Min + LinAlg::Vec3(Delta.X() * x, Delta.Y() * y, Delta.Z() * z);
				if (IsInternal(Pt, Vertices, Indices)) {
					//!< 内部点なら収集
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

	//!< 各軸にサンプリングする個数 (計算に時間がかかる)
	constexpr auto SampleCount = 100;

	auto InertiaTensor = LinAlg::Mat3::Zero();
	auto Sampled = 0;
	const auto Delta = Ab.GetExtent() / static_cast<float>(SampleCount);
	for (auto x = 0; x < SampleCount; ++x) {
		for (auto y = 0; y < SampleCount; ++y) {
			for (auto z = 0; z < SampleCount; ++z) {
				//!< AABB 内のサンプル点 (重心からの相対)
				const auto Pt = Ab.Min + LinAlg::Vec3(Delta.X() * x, Delta.Y() * y, Delta.Z() * z) - CenterOfMass;
				if (IsInternal(Pt, Vertices, Indices)) {
					const auto XX = Pt.X() * Pt.X();
					const auto YY = Pt.Y() * Pt.Y();
					const auto ZZ = Pt.Z() * Pt.Z();
					const auto XY = Pt.X() * Pt.Y();
					const auto XZ = Pt.X() * Pt.Z();
					const auto YZ = Pt.Y() * Pt.Z();
					//!< 内部点なら収集する
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
	constexpr auto SampleCount = 100 * 100;

	auto CenterOfMass = LinAlg::Vec3::Zero();
	auto Sampled = 0;
	const auto Ext = Ab.GetExtent();
	for (auto i = 0; i < SampleCount; ++i) {
		//!< AABB 内のサンプル点
		const auto Pt = Ab.Min + LinAlg::Vec3(Ext.X() * Distribution(MersenneTwister), 
			Ext.Y() * Distribution(MersenneTwister), 
			Ext.Z() * Distribution(MersenneTwister));
		if (IsInternal(Pt, Vertices, Indices)) {
			//!< 内部点なら収集
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

	//!< サンプリングする個数
	constexpr auto SampleCount = 100 * 100;

	auto InertiaTensor = LinAlg::Mat3::Zero();
	auto Sampled = 0;
	const auto Ext = Ab.GetExtent();
	for (auto i = 0; i < SampleCount; ++i) {
		//!< AABB 内のサンプル点 (重心からの相対)
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
			//!< 内部点なら収集する
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

	//!< 頂点の平均 (中心) を求め A と表す
	const auto A = std::accumulate(std::cbegin(Vertices), std::cend(Vertices), LinAlg::Vec3::Zero()) / static_cast<float>(std::size(Vertices));
	const auto CenterVolumeSum = std::accumulate(std::cbegin(Indices), std::cend(Indices), std::pair<LinAlg::Vec3, float>(LinAlg::Vec3::Zero(), 0.0f),
		[&](const auto& Acc, const auto& rhs) {
			//!< A を頂点とした四面体
			const auto& B = Vertices[rhs[0]];
			const auto& C = Vertices[rhs[1]];
			const auto& D = Vertices[rhs[2]];

			//!< 四面体の中心に体積の重み付け (中心に体積が凝縮した体で考える)
			const auto Volume = Collision::Volume::Tetrahedron(A, B, C, D);
			const auto Center = (A + B + C + D) * 0.25f;

			return std::pair<LinAlg::Vec3, float>(Acc.first + Center * Volume, Acc.second + Volume);
		});
	return CenterVolumeSum.first / CenterVolumeSum.second;
}
LinAlg::Mat3 Convex::Tetrahedron::CalcInertiaTensor(const LinAlg::Vec3& A, const LinAlg::Vec3& B, const LinAlg::Vec3& C, const LinAlg::Vec3& D)
{
	//!< 四面体の 4 頂点からなる行列
	const auto AB = B - A;
	const auto AC = C - A;
	const auto AD = D - A;
	const auto Det = LinAlg::Mat3(AB, AC, AD).Transpose().Determinant();

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

	return LinAlg::Mat3(
		LinAlg::Vec3(2.0f * (YY + ZZ), -XY, -XZ),
		LinAlg::Vec3(-XY, 2.0f * (XX + ZZ), -YZ),
		LinAlg::Vec3(-XZ, -YZ, 2.0f * (XX + YY))) * Det / 120.0f;
}
LinAlg::Mat3 Convex::Tetrahedron::CalcInertiaTensor(const std::vector<LinAlg::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices, const LinAlg::Vec3& CenterOfMass)
{
	LOG(std::data(std::format("Calculating inertia tensor (Tetrahedron)...\n")));

	const auto A = LinAlg::Vec3::Zero(); //!< CenterOfMass - CenterOfMass なので
	const auto TensorVolumeSum = std::accumulate(std::cbegin(Indices), std::cend(Indices), std::pair<LinAlg::Mat3, float>(LinAlg::Mat3::Zero(), 0.0f),
		[&](const auto& Acc, const auto& rhs) {
			const auto B = Vertices[rhs[0]] - CenterOfMass;
			const auto C = Vertices[rhs[1]] - CenterOfMass;
			const auto D = Vertices[rhs[2]] - CenterOfMass;

			return std::pair<LinAlg::Mat3, float>(Acc.first + CalcInertiaTensor(A, B, C, D), Acc.second + Collision::Volume::Tetrahedron(A, B, C, D));
		});

	return TensorVolumeSum.first / TensorVolumeSum.second;
}