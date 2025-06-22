#pragma once

#include <format>

//!< C++23
#include <mdspan>
//!< C++26 #TODO
//#include <linalg>

namespace LinAlg 
{
	class Mat2 
	{
	public:
		Mat2() {}
		Mat2(const Vec2& Row0, const Vec2& Row1) : Rows{ Row0, Row1 } {}
		Mat2(const Mat2& rhs) : Rows{ rhs.Rows[0], rhs.Rows[1] } {}

		inline static Mat2 Identity() { return Mat2(); }
		inline static Mat2 Zero() { return { Vec2::Zero(), Vec2::Zero() }; }

		inline bool NearlyEqual(const Mat2& rhs, const float Epsilon = (std::numeric_limits<float>::epsilon)()) const { 
			return std::ranges::equal(Rows, rhs.Rows, 
				[&](const Vec2& l, const Vec2& r) {
					return l.NearlyEqual(r, Epsilon); 
				});
		}

		inline bool operator==(const Mat2& rhs) const {
			return std::ranges::equal(Rows, rhs.Rows);
		}
		inline Mat2 operator+(const Mat2& rhs) const {
			Mat2 r; std::ranges::transform(Rows, rhs.Rows, std::begin(r.Rows), std::plus()); return r;
		}
		inline Mat2 operator-(const Mat2& rhs) const { 
			Mat2 r; std::ranges::transform(Rows, rhs.Rows, std::begin(r.Rows), std::minus()); return r;
		}
		inline Mat2 operator*(const float rhs) const { 
			Mat2 r; std::ranges::transform(Rows, std::begin(r.Rows), std::bind(std::multiplies(), std::placeholders::_1, rhs)); return r;
		}
		inline Vec2 operator*(const Vec2& rhs) const { return { Rows[0].Dot(rhs), Rows[1].Dot(rhs) }; }
		inline Mat2 operator*(const Mat2& rhs) const {
			//!< ŒŸŽZ—p : glm ‚Ìê‡‚ÍŠ|‚¯‚é‡˜‚ª‹t‚È‚Ì‚Å’ˆÓ
			// glm::make_mat2(static_cast<const float*>(rhs)) * glm::make_mat2(static_cast<const float*>(*this));
			const auto c0 = Vec2({ rhs.Rows[0][0], rhs.Rows[1][0] });
			const auto c1 = Vec2({ rhs.Rows[0][1], rhs.Rows[1][1] });
			return { 
				{ Rows[0].Dot(c0), Rows[0].Dot(c1) }, 
				{ Rows[1].Dot(c0), Rows[1].Dot(c1) } 
			};
		}
		inline Mat2 operator/(const float rhs) const { 
			Mat2 r; std::ranges::transform(Rows, std::begin(r.Rows), std::bind(std::divides(), std::placeholders::_1, rhs)); return r;
		}
		inline Mat2 operator-() const {
			Mat2 r; std::ranges::transform(Rows, std::begin(r.Rows), std::negate()); return r;
		}

		inline const Vec2& operator[](const int i) const { return Rows[i]; }
		inline operator const float* () const { return static_cast<const float*>(Rows[0]); }

		inline Mat2 Transpose() const {
			return { 
				{ Rows[0].X(), Rows[1].X() }, 
				{ Rows[0].Y(), Rows[1].Y() } 
			};
		}
		inline float Determinant() const {
			//!< ŒŸŽZ—p
			// glm::determinant(glm::make_mat2(static_cast<const float *>(*this)));
			return Rows[0][0] * Rows[1][1] - Rows[1][0] * Rows[0][1];
		}
		inline Mat2 Inverse(const float InvDet) const {
			//!< ŒŸŽZ—p
			// glm::inverse(glm::make_mat2(static_cast<const float*>(*this)));
			return Mat2({ { Rows[1][1], -Rows[0][1] }, { -Rows[1][0], Rows[0][0] } }) * InvDet;
		}
		inline Mat2 Inverse() const { Inverse(1.0f / Determinant()); }

		inline Mat2& operator=(const Mat2& rhs) { 
			std::ranges::copy(rhs.Rows, std::begin(Rows));
			return *this; 
		}
		inline const Mat2& operator+=(const Mat2& rhs) {
			std::ranges::transform(Rows, rhs.Rows, std::begin(Rows), std::plus());
			return *this;
		}
		inline const Mat2& operator-=(const Mat2& rhs) {
			std::ranges::transform(Rows, rhs.Rows, std::begin(Rows), std::minus());
			return *this;
		}
		inline const Mat2& operator*=(const float rhs) {
			std::ranges::transform(Rows, std::begin(Rows), std::bind(std::multiplies(), std::placeholders::_1, rhs));
			return *this;
		}
		inline const Mat2& operator/=(const float rhs) {
			std::ranges::transform(Rows, std::begin(Rows), std::bind(std::divides(), std::placeholders::_1, rhs));
			return *this;
		}
		inline Vec2& operator[](const int i) { return Rows[i]; }

		inline Mat2& ToZero() { return (*this = Zero()); }
		inline Mat2& ToIdentity() { return (*this = Identity()); }

		inline std::string ToString() const { 
			return std::format("({:1.4f}, {:1.4f})\n({:1.4f}, {:1.4f})\n", Rows[0].X(), Rows[0].Y(), Rows[1].X(), Rows[1].Y());
		}

	private:
		std::array<Vec2, 2> Rows = { Vec2::AxisX(), Vec2::AxisY() };
	};

	class Mat3 
	{
	public:
		Mat3() {}
		Mat3(const Vec3& Row0, const Vec3& Row1, const Vec3& Row2) : Rows{ Row0, Row1, Row2 } {}
		Mat3(const Mat3& rhs) : Rows{ rhs.Rows[0], rhs.Rows[1], rhs.Rows[2] } {}

		inline static Mat3 Identity() { return Mat3(); }
		inline static Mat3 Zero() { return { Vec3::Zero(), Vec3::Zero(), Vec3::Zero() }; }

		inline bool NearlyEqual(const Mat3& rhs, const float Epsilon = (std::numeric_limits<float>::epsilon)()) const { 
			return std::ranges::equal(Rows, rhs.Rows,
				[&](const Vec3& l, const Vec3& r) { 
					return l.NearlyEqual(r, Epsilon); 
				});
		}

		inline bool operator==(const Mat3& rhs) const { 
			return std::ranges::equal(Rows, rhs.Rows);
		}
		inline Mat3 operator+(const Mat3& rhs) const { 
			Mat3 r; std::ranges::transform(Rows, rhs.Rows, std::begin(r.Rows), std::plus()); return r;
		}
		inline Mat3 operator-(const Mat3& rhs) const { 
			Mat3 r; std::ranges::transform(Rows, rhs.Rows, std::begin(r.Rows), std::minus()); return r;
		}
		inline Mat3 operator*(const float rhs) const {
			Mat3 r; std::ranges::transform(Rows, std::begin(r.Rows), std::bind(std::multiplies(), std::placeholders::_1, rhs)); return r;
		}
		inline Vec3 operator*(const Vec3& rhs) const { return { Rows[0].Dot(rhs), Rows[1].Dot(rhs), Rows[2].Dot(rhs) }; }
		inline Mat3 operator*(const Mat3& rhs) const {
			//!< ŒŸŽZ—p : glm ‚Ìê‡‚ÍŠ|‚¯‚é‡˜‚ª‹t‚È‚Ì‚Å’ˆÓ
			// glm::make_mat3(static_cast<const float*>(rhs)) * glm::make_mat3(static_cast<const float*>(*this));
			const auto c0 = Vec3({ rhs.Rows[0][0],rhs.Rows[1][0], rhs.Rows[2][0] });
			const auto c1 = Vec3({ rhs.Rows[0][1],rhs.Rows[1][1], rhs.Rows[2][1] });
			const auto c2 = Vec3({ rhs.Rows[0][2],rhs.Rows[1][2], rhs.Rows[2][2] });
			return { 
				{ Rows[0].Dot(c0), Rows[0].Dot(c1), Rows[0].Dot(c2) }, 
				{ Rows[1].Dot(c0), Rows[1].Dot(c1), Rows[1].Dot(c2) }, 
				{ Rows[2].Dot(c0), Rows[2].Dot(c1), Rows[2].Dot(c2) } 
			};
		}
		inline Mat3 operator/(const float rhs) const { 
			Mat3 r; std::ranges::transform(Rows, std::begin(r.Rows), std::bind(std::divides(), std::placeholders::_1, rhs)); return r;
		}
		inline Mat3 operator-() const {
			Mat3 r; std::ranges::transform(Rows, std::begin(r.Rows), std::negate()); return r;
		}

		inline const Vec3& operator[](const int i) const { return Rows[i]; }
		inline operator const float* () const { return static_cast<const float*>(Rows[0]); }

		inline Mat3 Transpose() const {
			return {
				{ Rows[0].X(), Rows[1].X(), Rows[2].X() },
				{ Rows[0].Y(), Rows[1].Y(), Rows[2].Y() },
				{ Rows[0].Z(), Rows[1].Z(), Rows[2].Z() }
			};
		}
		inline float Determinant() const {
			//!< ŒŸŽZ—p
			// glm::determinant(glm::make_mat3(static_cast<const float*>(*this)));
			auto Det = 0.0f;
			for (int j = 0; j < 3; ++j) {
				Det += Rows[0][j] * Cofactor(0, j);
			}
			return Det;
		}
		inline Mat3 Inverse(const float InvDet) const {
			//!< ŒŸŽZ—p
			// glm::inverse(glm::make_mat3(static_cast<const float*>(*this)));
			Mat3 M;
			for (int i = 0; i < 3; ++i) {
				for (int j = 0; j < 3; ++j) {
					M[j][i] = Cofactor(i, j);
				}
			}
			return M * InvDet;
		}
		inline Mat3 Inverse() const { return Inverse(1.0f / Determinant()); }
		//!< ¬s—ñ (s‚Ü‚½‚Í—ñ‚ðŽæ‚èœ‚¢‚Ä‚Å‚«‚é¬‚³‚¢³•ûs—ñ)
		inline Mat2 Minor(const int Column, const int Row) const {
			Mat2 M;
			int r = 0;
			for (int j = 0; j < 3; ++j) {
				if (Row == j) { continue; }
				int c = 0;
				for (int i = 0; i < 3; ++i) {
					if (Column == i) { continue; }
					M[c][r] = (*this)[i][j];
					++c;
				}
				++r;
			}
			return M;
		}
		//!< —]ˆöŽq ¬s—ñŽ®‚É -1^(i + j) ‚ðŠ|‚¯‚Ä“¾‚ç‚ê‚é
		inline float Cofactor(const int Column, const int Row) const {
			return std::powf(-1.0f, static_cast<float>(Column + 1 + Row + 1)) * Minor(Column, Row).Determinant();
		}

		inline Mat3& operator=(const Mat2& rhs) {
			Rows[0] = rhs[0]; Rows[1] = rhs[1];
			return *this;
		}
		inline Mat3& operator=(const Mat3& rhs) { 
			std::ranges::copy(rhs.Rows, std::begin(Rows));
			return *this; 
		}
		inline const Mat3& operator+=(const Mat3& rhs) { 
			std::ranges::transform(Rows, rhs.Rows, std::begin(Rows), std::plus());
			return *this; 
		}
		inline const Mat3& operator-=(const Mat3& rhs) {
			std::ranges::transform(Rows, rhs.Rows, std::begin(Rows), std::minus());
			return *this; 
		}
		inline const Mat3& operator*=(const float rhs) { 
			std::ranges::transform(Rows, std::begin(Rows), std::bind(std::multiplies(), std::placeholders::_1, rhs));
			return *this; 
		}
		inline const Mat3& operator/=(const float rhs) { 
			std::ranges::transform(Rows, std::begin(Rows), std::bind(std::divides(), std::placeholders::_1, rhs));
			return *this;
		}
		inline Vec3& operator[](const int i) { return Rows[i]; }

		inline Mat3& ToZero() { return (*this = Zero()); }
		inline Mat3& ToIdentity() { return (*this = Identity()); }

		inline std::string ToString() const { 
			return std::format("({:1.4f}, {:1.4f}, {:1.4f})\n({:1.4f}, {:1.4f}, {:1.4f})\n({:1.4f}, {:1.4f}, {:1.4f})\n", Rows[0].X(), Rows[0].Y(), Rows[0].Z(), Rows[1].X(), Rows[1].Y(), Rows[1].Z(), Rows[2].X(), Rows[2].Y(), Rows[2].Z());
		}

	private:
		std::array<Vec3, 3> Rows = { Vec3::AxisX(), Vec3::AxisY(), Vec3::AxisZ() };
	};

	class Mat4
	{
	public:
		Mat4() {}
		Mat4(const Vec4& Row0, const Vec4& Row1, const Vec4& Row2, const Vec4& Row3) : Rows{ Row0, Row1, Row2, Row3 } {}
		Mat4(const Mat4& rhs) : Rows{ rhs.Rows[0], rhs.Rows[1], rhs.Rows[2], rhs.Rows[3] } {}

		inline static Mat4 Identity() { return Mat4(); }
		inline static Mat4 Zero() { return { Vec4::Zero(), Vec4::Zero(), Vec4::Zero(), Vec4::Zero() }; }

		inline bool NearlyEqual(const Mat4& rhs, const float Epsilon = (std::numeric_limits<float>::epsilon)()) const { 
			return std::ranges::equal(Rows, rhs.Rows, 
				[&](const Vec4& l, const Vec4& r) {
					return l.NearlyEqual(r, Epsilon); 
				});
		}

		inline bool operator==(const Mat4& rhs) const { 
			return std::ranges::equal(Rows, rhs.Rows);
		}
		inline Mat4 operator+(const Mat4& rhs) const { 
			Mat4 r; std::ranges::transform(Rows, rhs.Rows, std::begin(r.Rows), std::plus()); return r;
		}
		inline Mat4 operator-(const Mat4& rhs) const {
			Mat4 r; std::ranges::transform(Rows, rhs.Rows, std::begin(r.Rows), std::minus()); return r;
		}
		inline Mat4 operator*(const float rhs) const { 
			Mat4 r; std::ranges::transform(Rows, std::begin(r.Rows), std::bind(std::multiplies(), std::placeholders::_1, rhs)); return r;
		}
		inline Vec4 operator*(const Vec4& rhs) const { return { Rows[0].Dot(rhs), Rows[1].Dot(rhs), Rows[2].Dot(rhs), Rows[3].Dot(rhs) }; }
		inline Mat4 operator*(const Mat4& rhs) const {
			//!< ŒŸŽZ—p : glm ‚Ìê‡‚ÍŠ|‚¯‚é‡˜‚ª‹t‚È‚Ì‚Å’ˆÓ
			// glm::make_mat4(static_cast<const float*>(rhs)) * glm::make_mat4(static_cast<const float*>(*this));
			const auto c0 = Vec4({ rhs.Rows[0][0],rhs.Rows[1][0], rhs.Rows[2][0], rhs.Rows[3][0] });
			const auto c1 = Vec4({ rhs.Rows[0][1],rhs.Rows[1][1], rhs.Rows[2][1], rhs.Rows[3][1] });
			const auto c2 = Vec4({ rhs.Rows[0][2],rhs.Rows[1][2], rhs.Rows[2][2], rhs.Rows[3][2] });
			const auto c3 = Vec4({ rhs.Rows[0][3],rhs.Rows[1][3], rhs.Rows[2][3], rhs.Rows[3][3] });
			return { 
				{ Rows[0].Dot(c0), Rows[0].Dot(c1), Rows[0].Dot(c2), Rows[0].Dot(c3) }, 
				{ Rows[1].Dot(c0), Rows[1].Dot(c1), Rows[1].Dot(c2), Rows[1].Dot(c3) },
				{ Rows[2].Dot(c0), Rows[2].Dot(c1), Rows[2].Dot(c2), Rows[2].Dot(c3) }, 
				{ Rows[3].Dot(c0), Rows[3].Dot(c1), Rows[3].Dot(c2), Rows[3].Dot(c3) } 
			};
		}
		inline Mat4 operator/(const float rhs) const { 
			Mat4 r; std::ranges::transform(Rows, std::begin(r.Rows), std::bind(std::divides(), std::placeholders::_1, rhs)); return r;
		}
		inline Mat4 operator-() const {
			Mat4 r; std::ranges::transform(Rows, std::begin(r.Rows), std::negate()); return r;
		}

		inline const Vec4& operator[](const int i) const { return Rows[i]; }
		inline operator const float* () const { return static_cast<const float*>(Rows[0]); }

		inline Mat4 Transpose() const {
			return { 
				{ Rows[0].X(), Rows[1].X(), Rows[2].X(), Rows[3].X() }, 
				{ Rows[0].Y(), Rows[1].Y(), Rows[2].Y(), Rows[3].Y() }, 
				{ Rows[0].Z(), Rows[1].Z(), Rows[2].Z(), Rows[3].Z() }, 
				{ Rows[0].W(), Rows[1].W(), Rows[2].W(), Rows[3].W() } 
			};
		}
		inline float Determinant() const {
			//!< ŒŸŽZ—p
			// glm::determinant(glm::make_mat4(static_cast<const float *>(*this)));
			// DirectX::XMMatrixDeterminant(DirectX::XMLoadFloat4x4(*this)).m128_f32[0];
			auto Det = 0.0f;
			for (int j = 0; j < 4; ++j) {
				Det += Rows[0][j] * Cofactor(0, j);
			}
			return Det;
		}
		inline Mat4 Inverse(const float InvDet) const {
			Mat4 M;
			for (int i = 0; i < 4; ++i) {
				for (int j = 0; j < 4; ++j) {
					M[j][i] = Cofactor(i, j);
				}
			}
			return M * InvDet;
		}
		inline Mat4 Inverse() const {
			//!< ŒŸŽZ—p
			// glm::inverse(glm::make_mat4(static_cast<const float*>(*this)));
			// DirectX::XMMatrixInverse(nullptr, DirectX::XMLoadFloat4x4(*this));
			return Inverse(1.0f / Determinant());
		}
		//!< ¬s—ñ (s‚Ü‚½‚Í—ñ‚ðŽæ‚èœ‚¢‚Ä‚Å‚«‚é¬‚³‚¢³•ûs—ñ)
		inline Mat3 Minor(const int Column, const int Row) const {
			Mat3 M;
			int r = 0;
			for (int j = 0; j < 4; ++j) {
				if (Row == j) { continue; }
				int c = 0;
				for (int i = 0; i < 4; ++i) {
					if (Column == i) { continue; }
					M[c][r] = (*this)[i][j];
					++c;
				}
				++r;
			}
			return M;
		}
		//!< —]ˆöŽq ¬s—ñŽ®‚É -1^(i + j) ‚ðŠ|‚¯‚Ä“¾‚ç‚ê‚é
		inline float Cofactor(const int Column, const int Row) const {
			return std::powf(-1.0f, static_cast<float>(Column + 1 + Row + 1)) * Minor(Column, Row).Determinant();
		}

		inline Mat4& operator=(const Mat2& rhs) {
			Rows[0] = rhs[0]; Rows[1] = rhs[1];
			return *this;
		}
		inline Mat4& operator=(const Mat3& rhs) {
			Rows[0] = rhs[0]; Rows[1] = rhs[1]; Rows[2] = rhs[2];
			return *this;
		}
		inline Mat4& operator=(const Mat4& rhs) { 
			std::ranges::copy(rhs.Rows, std::begin(Rows));
			return *this; 
		}
		inline const Mat4& operator+=(const Mat4& rhs) { 
			std::ranges::transform(Rows, rhs.Rows, std::begin(Rows), std::plus());
			return *this; 
		}
		inline const Mat4& operator-=(const Mat4& rhs) { 
			std::ranges::transform(Rows, rhs.Rows, std::begin(Rows), std::minus());
			return *this; 
		}
		inline const Mat4& operator*=(const float rhs) {
			std::ranges::transform(Rows, std::begin(Rows), std::bind(std::multiplies(), std::placeholders::_1, rhs));
			return *this; 
		}
		inline const Mat4& operator/=(const float rhs) { 
			std::ranges::transform(Rows, std::begin(Rows), std::bind(std::divides(), std::placeholders::_1, rhs));
			return *this; 
		}
		inline Vec4& operator[](const int i) { return Rows[i]; }
		inline operator float* () { return static_cast<float*>(Rows[0]); }

		inline Mat4& ToZero() { return (*this = Zero()); }
		inline Mat4& ToIdentity() { return (*this = Identity()); }

		inline std::string ToString() const {
			return std::format("({:1.4f}, {:1.4f}, {:1.4f}, {:1.4f})\n({:1.4f}, {:1.4f}, {:1.4f}, {:1.4f})\n({:1.4f}, {:1.4f}, {:1.4f}, {:1.4f})\n({:1.4f}, {:1.4f}, {:1.4f}, {:1.4f})\n", Rows[0].X(), Rows[0].Y(), Rows[0].Z(), Rows[0].W(), Rows[1].X(), Rows[1].Y(), Rows[1].Z(), Rows[1].W(), Rows[2].X(), Rows[2].Y(), Rows[2].Z(), Rows[2].W(), Rows[3].X(), Rows[3].Y(), Rows[3].Z(), Rows[3].W()); 
		}

	private:
		std::array<Vec4, 4> Rows = { Vec4::AxisX() , Vec4::AxisY(), Vec4::AxisZ(), Vec4::AxisW() };
	};

	template<size_t M, size_t N>
	class Mat
	{
	public:
		template<typename...A> Mat(A...Args) {
			auto i = 0;
			for (const auto& v : std::initializer_list<Vec<N>>{ Args... }) {
#ifdef _DEBUG
				if (i >= M) { break; }
#endif
				Rows[i++] = v;
			}
		}

		inline static Mat Identity() {
			Mat r;
			for (auto i = 0; i < M; ++i) {
				r.Rows[i][i] = 1.0f;
			}
			return r;
		}
		inline static Mat Zero() { return Mat(); }

		inline bool NearlyEqual(const Mat& rhs, const float Epsilon = (std::numeric_limits<float>::epsilon)()) const {
			return std::ranges::equal(Rows, rhs.Rows,
				[&](const Vec4& l, const Vec4& r) { 
					return l.NearlyEqual(r, Epsilon);
				});
		}

		inline bool operator==(const Mat& rhs) const {
			return std::ranges::equal(Rows, rhs.Rows);
		}
		inline Mat operator+(const Mat& rhs) const {
			Mat r; std::ranges::transform(Rows, rhs.Rows, std::begin(r.Rows), std::plus()); return r;
		}
		inline Mat operator-(const Mat& rhs) const {
			Mat r; std::ranges::transform(Rows, rhs.Rows, std::begin(r.Rows), std::minus()); return r;
		}
		inline Mat operator*(const float rhs) const {
			Mat r; std::ranges::transform(Rows, std::begin(r.Rows), std::bind(std::multiplies(), std::placeholders::_1, rhs)); return r;
		}
		template<uint32_t N>
		inline Vec<M> operator*(const Vec<N>& rhs) const {
			Vec<M> r;
			for (auto i = 0; i < M; ++i) {
				r[i] = rhs.Dot(Rows[i]);
			}
			return r;
		}
		template<uint32_t O>
		inline Mat<M, O> operator*(const Mat<N, O>& rhs) const {
			Mat<M, O> r;
			const auto Trs = rhs.Transpose();
			for (auto i = 0; i < M; ++i) {
				for (auto j = 0; j < O; ++j) {
					r[i][j] = Rows[i].Dot(Trs[j]);
				}
			}
			return r;
		}
		inline Mat operator/(const float rhs) const {
			Mat r; std::ranges::transform(Rows, std::begin(r.Rows), std::bind(std::divides(), std::placeholders::_1, rhs)); return r;
		}
		inline Mat operator-() const {
			Mat r; std::ranges::transform(Rows, std::begin(r.Rows), std::negate()); return r;
		}

		inline const Vec<N>& operator[](const int i) const { return Rows[i]; }
		inline operator const float* () const { return static_cast<const float*>(Rows[0]); }

		inline Mat<N, M> Transpose() const {
			Mat<N, M> r;
			for (auto i = 0; i < M; ++i) {
				for (auto j = 0; j < N; ++j) {
					r[j][i] = Rows[i][j];
				}
			}
			return r;
		}

		inline const Mat& operator+=(const Mat& rhs) {
			std::ranges::transform(Rows, rhs.Rows, std::begin(Rows), std::plus());
			return *this;
		}
		inline const Mat& operator-=(const Mat& rhs) {
			std::ranges::transform(Rows, rhs.Rows, std::begin(Rows), std::minus());
			return *this;
		}
		inline const Mat& operator*=(const float rhs) {
			std::ranges::transform(Rows, std::begin(Rows), std::bind(std::multiplies(), std::placeholders::_1, rhs));
			return *this;
		}
		inline const Mat& operator/=(const float rhs) {
			std::ranges::transform(Rows, std::begin(Rows), std::bind(std::divides(), std::placeholders::_1, rhs));
			return *this;
		}
		inline Vec<N>& operator[](const int i) { return Rows[i]; }
		inline operator float* () { return static_cast<float*>(Rows[0]); }

		inline Mat& ToZero() { return (*this = Zero()); }
		inline Mat& ToIdentity() { return (*this = Identity()); }

		inline size_t Size() const { return std::size(Rows); }

	private:
		std::array<Vec<N>, M> Rows;
	};

	static void MdspanTest() {
		//!< s—Dæ Row-Major (ƒfƒtƒHƒ‹ƒg)
		using RM22 = std::mdspan<float, std::extents<size_t, 2, 2>>;
		//!< —ñ—Dæ Column-Major
		using LM22 = std::mdspan<float, std::extents<size_t, 2, 2>, std::layout_left>;
		using RM33 = std::mdspan<float, std::extents<size_t, 3, 3>>;
		using LM33 = std::mdspan<float, std::extents<size_t, 3, 3>, std::layout_left>;
		using RM44 = std::mdspan<float, std::extents<size_t, 4, 4>>;
		using LM44 = std::mdspan<float, std::extents<size_t, 4, 4>, std::layout_left>;

		std::array F9 = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, 9.0f,
		};
		std::mdspan rm33{ std::data(F9), 3, 3 };
		LM33 lm33{ std::data(F9) };

		for (auto i = 0; i < rm33.extent(0); ++i) {
			for (auto j = 0; j < rm33.extent(1); ++j) {
				std::cout << rm33[i, j] << ", ";
			}
			std::cout << std::endl;
		}
		for (auto i = 0; i < lm33.extent(0); ++i) {
			for (auto j = 0; j < lm33.extent(1); ++j) {
				std::cout << lm33[i, j] << ", ";
			}
			std::cout << std::endl;
		}
	}
}

