#pragma once

#include "LinAlg.h"

namespace Collision 
{
	struct Contact;
}

namespace Physics
{
	class RigidBody;

	//!< コンストレイント基底
	class ConstraintBase
	{
	public:
		//!< ヤコビ行列のセットアップ
		virtual void PreSolve(const float DeltaSec) {};
		//!< 繰り返しコールすることで収束させる為、関数化している
		virtual void Solve() {};
		virtual void PostSolve() {}

	protected:
		//!< RigidBodyB を必要としないもの (ConstraintMover 系など) があるので、基底では RigidBodyA のみ持たせる
		Physics::RigidBody* RigidBodyA = nullptr;
	};
	//!< 2剛体 A, B と逆質量行列を持つ
	class Constraint : public ConstraintBase
	{
	public:
		//!< 質量行列
		//!< M = (M_A    0    0    0) ... A の質量の逆数が対角成分 (3x3)
		//!<     (     I_A    0    0) ... A の慣性テンソルの逆行列 (3x3)
		//!<     (   0   0  M_B    0) ... B の質量の逆数が対角成分
		//!<     (   0   0    0  I_B) ... B の慣性テンソルの逆行列
		//!< #TODO 疎行列専用処理最適化の余地あり
		using MassMatrix = LinAlg::Mat<3 * 4, 3 * 4>;

		//!< V = (V_A) ... A の速度
		//!<     (W_A) ... A の角速度
		//!<     (V_B) ... B の速度
		//!<     (W_B) ... B の角速度
		using Velocities = LinAlg::Vec<3 * 4>;
	
	protected:
		Physics::RigidBody* RigidBodyB = nullptr;
		MassMatrix InvMass;
	};

	//!< それぞれのアンカー位置を持つ
	class ConstraintAnchor : public Constraint
	{
	public:
		ConstraintAnchor() {}
		ConstraintAnchor(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB) {
			Init(RbA, RbB);
		}
		ConstraintAnchor(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const LinAlg::Vec3& WAnchor) {
			Init(RbA, RbB, WAnchor);
		}

		static MassMatrix CreateInverseMassMatrix(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB);
		static Velocities CreateVelocties(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB);

		inline const MassMatrix& GetInverseMassMatrix() const { return InvMass; }
		Velocities GetVelocties() const { return CreateVelocties(RigidBodyA, RigidBodyB); }
		void ApplyImpulse(const Velocities& Impulse);

		ConstraintAnchor& Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB);
		ConstraintAnchor& Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const LinAlg::Vec3& WAnchor);

	protected:
		LinAlg::Vec3 LAnchorA;
		LinAlg::Vec3 LAnchorB;
	};

	//!< 軸を持つ基底
	class ConstraintAnchorAxis : public ConstraintAnchor
	{
	public:
		using Super = ConstraintAnchor;

		ConstraintAnchorAxis() {}
		ConstraintAnchorAxis(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const LinAlg::Vec3& WAnchor, const LinAlg::Vec3& WAxis) {
			Init(RbA, RbB, WAnchor, WAxis);
		}

		ConstraintAnchorAxis& Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const LinAlg::Vec3& WAnchor, const LinAlg::Vec3& WAxis);

	protected:
		LinAlg::Vec3 LAxisA;

		LinAlg::Quat InvInitRot;
	};

	class ConstraintDistance : public ConstraintAnchor
	{
	public:
		using Super = ConstraintAnchor;

		ConstraintDistance() {}
		ConstraintDistance(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const LinAlg::Vec3& WAnchor) {
			Init(RbA, RbB, WAnchor);
		}

		ConstraintDistance& Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const LinAlg::Vec3& WAnchor) {
			Super::Init(RbA, RbB, WAnchor);
			return *this;
		}

		virtual void PreSolve(const float DeltaSec) override;
		virtual void Solve() override;
		virtual void PostSolve() override;

	protected:
		//!< ヤコビ行列 (n * 12) n == コンストレイント数, 12 == 6 (移動3、回転3) 軸の自由度 * 2 オブジェクト
		//!< 距離 (n == 1)
		LinAlg::Mat<1, 3 * 4> Jacobian;
		LinAlg::Vec<1> CachedLambda;

		//!< 適正な位置へ戻すような力を適用する事で位置ドリフトを修正 (Baumgarte stabilization)
		//!< 一気にやるとシステムにエネルギーを追加しすぎる為、数フレームかけて適用する
		float Baumgarte = 0.0f;
	};

	class ConstraintHinge : public ConstraintAnchorAxis
	{
	public:
		using Super = ConstraintAnchorAxis;
		
		ConstraintHinge() {}
		ConstraintHinge(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const LinAlg::Vec3& WAnchor, const LinAlg::Vec3& WAxis) {
			Init(RbA, RbB, WAnchor, WAxis);
		}

		ConstraintHinge& Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const LinAlg::Vec3& WAnchor, const LinAlg::Vec3& WAxis) {
			Super::Init(RbA, RbB, WAnchor, WAxis);
			return *this;
		}

		virtual void PreSolve(const float DeltaSec) override;
		virtual void Solve() override;
		virtual void PostSolve() override;

	protected:
		//!< 距離、ヒンジ軸に垂直な U, V (n == 3)
		LinAlg::Mat<3, 3 * 4> Jacobian;
		LinAlg::Vec<3> CachedLambda;

		float Baumgarte = 0.0f;
	};

	class ConstraintHingeLimited : public ConstraintAnchorAxis
	{
	public:
		using Super = ConstraintAnchorAxis;
		
		ConstraintHingeLimited() {}
		ConstraintHingeLimited(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const LinAlg::Vec3& WAnchor, const LinAlg::Vec3& WAxis, const float LimAng = 45.0f) {
			Init(RbA, RbB, WAnchor, WAxis, LimAng);
		}

		ConstraintHingeLimited& Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const LinAlg::Vec3& WAnchor, const LinAlg::Vec3& WAxis, const float LimAng = 45.0f) {
			Super::Init(RbA, RbB, WAnchor, WAxis);
			LimitAngle = LimAng;
			return *this;
		}

		virtual void PreSolve(const float DeltaSec) override;
		virtual void Solve() override;
		virtual void PostSolve() override;

	protected:
		//!< 距離、ヒンジ軸に垂直な U, V、角度制限 (n == 4)
		LinAlg::Mat<4, 3 * 4> Jacobian;
		LinAlg::Vec<4> CachedLambda;

		float Baumgarte = 0.0f;

		bool IsAngleViolated;
		float Angle;
		float LimitAngle;
	};

	class ConstraintBallSocket : public ConstraintAnchorAxis
	{
	public:
		using Super = ConstraintAnchorAxis;

		ConstraintBallSocket() {}
		ConstraintBallSocket(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const LinAlg::Vec3& WAnchor, const LinAlg::Vec3& WAxis) {
			Init(RbA, RbB, WAnchor, WAxis);
		}

		ConstraintBallSocket& Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const LinAlg::Vec3& WAnchor, const LinAlg::Vec3& WAxis) {
			Super::Init(RbA, RbB, WAnchor, WAxis);
			return *this;
		}

		virtual void PreSolve(const float DeltaSec) override;
		virtual void Solve() override;
		virtual void PostSolve() override;
	
	protected:
		//!< 距離、軸 (n == 2)
		LinAlg::Mat<2, 3 * 4> Jacobian;
		LinAlg::Vec<2> CachedLambda;

		float Baumgarte = 0.0f;
	};

	class ConstraintBallSocketLimited : public ConstraintAnchorAxis
	{
	public:
		using Super = ConstraintAnchorAxis;

		ConstraintBallSocketLimited() {}
		ConstraintBallSocketLimited(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const LinAlg::Vec3& WAnchor, const LinAlg::Vec3& WAxis, const float LimAngU = 45.0f, const float LimAngV = 45.0f) {
			Init(RbA, RbB, WAnchor, WAxis, LimAngU, LimAngV);
		}

		ConstraintBallSocketLimited& Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const LinAlg::Vec3& WAnchor, const LinAlg::Vec3& WAxis, const float LimAngU = 45.0f, const float LimAngV = 45.0f) {
			Super::Init(RbA, RbB, WAnchor, WAxis);

			LimitAngles = { LimAngU, LimAngV };

			return *this;
		}

		virtual void PreSolve(const float DeltaSec) override;
		virtual void Solve() override;
		virtual void PostSolve() override;

	protected:
		LinAlg::Mat<4, 3 * 4> Jacobian;
		LinAlg::Vec<4> CachedLambda;

		float Baumgarte = 0.0f;

		std::array<bool, 2> IsAngleViolated = { false, false };
		std::array<float, 2> Angles;
		std::array<float, 2> LimitAngles;
	};

	class ConstraintMotor : public ConstraintAnchorAxis
	{
	public:
		using Super = ConstraintAnchorAxis;

		ConstraintMotor() {}
		ConstraintMotor(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const LinAlg::Vec3& WAnchor, const LinAlg::Vec3& WAxis, const float Spd) {
			Init(RbA, RbB, WAnchor, WAxis, Spd);
		}

		ConstraintMotor& Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const LinAlg::Vec3& WAnchor, const LinAlg::Vec3& WAxis, const float Spd) {
			Super::Init(RbA, RbB, WAnchor, WAxis);
			Speed = Spd;
			return *this;
		}

		virtual void PreSolve(const float DeltaSec) override;
		virtual void Solve() override;

	protected:
		//!< 距離、ヒンジ軸に垂直な U, V、ヒンジ軸 (n == 4)
		LinAlg::Mat<4, 3 * 4> Jacobian;

		LinAlg::Vec3 Baumgarte;

		float Speed;
	};

	class ConstraintMover : public ConstraintBase
	{
	public:
		ConstraintMover() {}
		ConstraintMover(const Physics::RigidBody* Rb) { Init(Rb); }

		ConstraintMover& Init(const Physics::RigidBody* Rb) { RigidBodyA = const_cast<Physics::RigidBody*>(Rb); return *this; }
	};
	class ConstraintMoverUpDown : public ConstraintMover
	{
	public:
		using Super = ConstraintMover;
		
		ConstraintMoverUpDown() {}
		ConstraintMoverUpDown(const Physics::RigidBody* Rb) { Init(Rb); }

		virtual void PreSolve(const float DeltaSec) override;
	protected:
		float Timer = 0.0f;
	}; 
	class ConstraintMoverRotateX : public ConstraintMover
	{
	public:
		using Super = ConstraintMover;
		
		ConstraintMoverRotateX() {}
		ConstraintMoverRotateX(const Physics::RigidBody* Rb) { Init(Rb); }
		
		virtual void PreSolve(const float DeltaSec) override;
	};
	class ConstraintMoverRotateY : public ConstraintMover
	{
	public:
		using Super = ConstraintMover;

		ConstraintMoverRotateY() {}
		ConstraintMoverRotateY(const Physics::RigidBody* Rb) { Init(Rb); }

		virtual void PreSolve(const float DeltaSec) override;
	};
	class ConstraintMoverRotateZ : public ConstraintMover
	{
	public:
		using Super = ConstraintMover;

		ConstraintMoverRotateZ() {}
		ConstraintMoverRotateZ(const Physics::RigidBody* Rb) { Init(Rb); }

		virtual void PreSolve(const float DeltaSec) override;
	};
	
	class ConstraintPenetration : public ConstraintAnchor
	{
	public:
		using Super = ConstraintAnchor;

		ConstraintPenetration() {}
		ConstraintPenetration(const Collision::Contact& Ct) { Init(Ct); }

		ConstraintPenetration& Init(const Collision::Contact& Ct);

		virtual void PreSolve(const float DeltaSec) override;
		virtual void Solve() override;

	protected:
		//!< 法線 N、接面の U, V (n == 3)
		LinAlg::Mat<3, 3 * 4> Jacobian;
		LinAlg::Vec<3> CachedLambda;

		float Baumgarte = 0.0f;

		LinAlg::Vec3 LNormal;
		float Friction = 0.0f;
	};

	//!< 2 オブジェクト間の衝突情報
	//!< 新しい衝突点を追加していく、無効になった衝突は削除される
	class Manifold
	{
	public:
		Manifold(const Collision::Contact& Ct);

		void Add(const Collision::Contact& CtOrig);
		void RemoveExpired();

	protected:
		friend class ManifoldCollector;
		static constexpr uint8_t MaxContacts = 4; //!< 覚えておく衝突数

		//!< コンストラクト時にセットした A, B の順序を覚えておく、以降の追加はこの順に従う
		Physics::RigidBody* RigidBodyA = nullptr;
		Physics::RigidBody* RigidBodyB = nullptr;

		using ContactAndConstraint = std::pair<Collision::Contact, ConstraintPenetration>;
		std::vector<ContactAndConstraint> Constraints;
	};
	//!< 2 オブジェクト間の衝突情報 (マニフォールド) を収集
	class ManifoldCollector
	{
	public:
		void Add(const Collision::Contact& Ct);
		void RemoveExpired() {
			//!< 無効になった衝突点をマニフォールドから削除
			for (auto& i : Manifolds) { i.RemoveExpired(); }

			//!< 衝突点を１つも持たなくなったマニフォールドを削除
			const auto Range = std::ranges::remove_if(Manifolds, 
				[](const auto& i) {
					return std::empty(i.Constraints);
				});
			Manifolds.erase(std::cbegin(Range), std::cend(Range));
		}

		void PreSolve(const float DeltaSec);
		void Solve();
		void PostSolve();

	protected:
		std::vector<Manifold> Manifolds;
	};

	//!< Linear Complimentary Problem (LCP)
	//!<	A * x = b 
	//!<	行列 A, ベクトル b が既知の時、未知のベクトル x を求める

	//!< ガウスザイデル法では以下のいずれかの場合に LCP を解くことができる (ここで扱う行列は対角優位なので、ガウスザイデル法が使用可能)
	//!<	[1] 正定値 (positive definite) 
	//!<		v^t * M * v > 0 
	//!<		ベクトル v の転地と v で挟むように掛けたときに結果が正となるような行列 M
	//!<
	//!<	[2] 対角優位 (diagonally dominant) 
	//!<		|xy_ij| >= Sigma_j,j!=i |xy_ij|
	//!<		対角成分の和の絶対値が、非対角成分の和の絶対値以上となるような行列
	template<size_t N>
	static LinAlg::Vec<N> GaussSiedel(const LinAlg::Mat<N, N>& A, const LinAlg::Vec<N>& b, const uint32_t ItCount = 10)
	{
		auto x = LinAlg::Vec<N>();

		for (uint32_t It = 0; It < ItCount; ++It) {
			for (auto i = 0; i < N; ++i) {
				const auto dx = (b[i] - A[i].Dot(x)) / A[i][i];
				if (dx * 0.01f == dx * 0.01f) {
					x[i] += dx;
				}
			}
		}

		return x;
	}
}