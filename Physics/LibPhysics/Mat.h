#pragma once

namespace LinAlg 
{
	class Mat2 
	{
	public:
		friend class Mat3;

		Mat2() {}
		Mat2(const Vec2& Row0, const Vec2& Row1) : Rows{ Row0, Row1 } {}
		Mat2(const Mat2& rhs) : Rows{ rhs.Rows[0], rhs.Rows[1] } {}
		Mat2(View2x2 rhs) : View(rhs) {}

	private:
		static const Mat2 _Identity;
		static const Mat2 _Zero;
	public:
		inline static const Mat2& Identity() { return _Identity; }
		inline static const Mat2& Zero() { return _Zero; }

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
#ifdef USE_STD_LINALG
			Mat2 r;
			std::linalg::add(View, rhs.View, r.View);
			return r;
#else
#ifdef USE_OWN_IMPL
			return Mat2(Rows[0] + rhs.Rows[0], Rows[1] + rhs.Rows[1]);
#else
			Mat2 r;
			std::ranges::transform(Rows, rhs.Rows, std::begin(r.Rows), std::plus());
			return r;
#endif
#endif
		}
		inline Mat2 operator-(const Mat2& rhs) const { 
#ifdef USE_OWN_IMPL
			return Mat2(Rows[0] - rhs.Rows[0], Rows[1] - rhs.Rows[1]);
#else
			Mat2 r;
			std::ranges::transform(Rows, rhs.Rows, std::begin(r.Rows), std::minus()); 
			return r;
#endif		
		}
		inline Mat2 operator*(const float rhs) const { 
#ifdef USE_STD_LINALG
			return Mat2(std::linalg::scale(rhs, View));
#else
#ifdef USE_OWN_IMPL
			return Mat2(Rows[0] * rhs, Rows[1] * rhs);
#else
			Mat2 r;
			std::ranges::transform(Rows, std::begin(r.Rows), std::bind(std::multiplies(), std::placeholders::_1, rhs));
			return r;
#endif
#endif
		}
		inline Vec2 operator*(const Vec2& rhs) const { 
#ifdef USE_STD_LINALG
			Vec2 r;
			std::linalg::matrix_vector_product(View, rhs.View, r.View);
			return r;
#else
			return { Rows[0].Dot(rhs), Rows[1].Dot(rhs) };
#endif
		}
		inline Mat2 operator*(const Mat2& rhs) const {
#ifdef USE_STD_LINALG
			Mat2 r;
			std::linalg::matrix_product(View, rhs.View, r.View);
			return r;
#else
			//!< ŒŸŽZ—p : glm ‚Ìê‡‚ÍŠ|‚¯‚é‡˜‚ª‹t‚È‚Ì‚Å’ˆÓ
			// glm::make_mat2(static_cast<const float*>(rhs)) * glm::make_mat2(static_cast<const float*>(*this));
			const auto c0 = Vec2({ rhs.Rows[0][0], rhs.Rows[1][0] });
			const auto c1 = Vec2({ rhs.Rows[0][1], rhs.Rows[1][1] });
			return { 
				{ Rows[0].Dot(c0), Rows[0].Dot(c1) }, 
				{ Rows[1].Dot(c0), Rows[1].Dot(c1) } 
			};
#endif
		}
		inline Mat2 operator/(const float rhs) const { 
#ifdef USE_STD_LINALG
			return Mat2(std::linalg::scale(1.0f / rhs, View));
#else
#ifdef USE_OWN_IMPL
			return Mat2(Rows[0] / rhs, Rows[1] / rhs);
#else
			Mat2 r;
			std::ranges::transform(Rows, std::begin(r.Rows), std::bind(std::divides(), std::placeholders::_1, rhs));
			return r;
#endif
#endif
		}
		inline Mat2 operator-() const {
#ifdef USE_OWN_IMPL
			return Mat2(-Rows[0], -Rows[1]);
#else
			Mat2 r; 
			std::ranges::transform(Rows, std::begin(r.Rows), std::negate()); 
			return r;
#endif
		}

		inline const Vec2& operator[](const int i) const { return Rows[i]; }
		inline operator const float* () const { return static_cast<const float*>(Rows[0]); }

		inline Mat2 Transpose() const {
#ifdef USE_STD_LINALG
			return Mat2(std::linalg::transposed(View));
#else
			return {
				{ Rows[0].X(), Rows[1].X() }, 
				{ Rows[0].Y(), Rows[1].Y() } 
			};
#endif
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
#ifdef USE_STD_LINALG
			std::linalg::copy(rhs.View, View);
#else
			std::ranges::copy(rhs.Rows, std::begin(Rows));
#endif
			return *this;
		}
		inline const Mat2& operator+=(const Mat2& rhs) {
#ifdef USE_STD_LINALG
			std::linalg::add(View, rhs.View, View);
#else
#ifdef USE_OWN_IMPL
			Rows[0] += rhs.Rows[0];
			Rows[1] += rhs.Rows[1];
#else
			std::ranges::transform(Rows, rhs.Rows, std::begin(Rows), std::plus());
#endif
#endif
			return *this;
		}
		inline const Mat2& operator-=(const Mat2& rhs) {
#ifdef USE_OWN_IMPL
			Rows[0] -= rhs.Rows[0];
			Rows[1] -= rhs.Rows[1];
#else
			std::ranges::transform(Rows, rhs.Rows, std::begin(Rows), std::minus());
#endif
			return *this;
		}
		inline const Mat2& operator*=(const float rhs) {
#ifdef USE_STD_LINALG
			std::linalg::scale(rhs, View);
#else
#ifdef USE_OWN_IMPL
			Rows[0] *= rhs;
			Rows[1] *= rhs;
#else
			std::ranges::transform(Rows, std::begin(Rows), std::bind(std::multiplies(), std::placeholders::_1, rhs));
#endif
#endif
			return *this;
		}
		inline const Mat2& operator/=(const float rhs) {
#ifdef USE_STD_LINALG
			std::linalg::scale(1.0f / rhs, View);
#else
#ifdef USE_OWN_IMPL
			Rows[0] /= rhs;
			Rows[1] /= rhs;
#else
			std::ranges::transform(Rows, std::begin(Rows), std::bind(std::divides(), std::placeholders::_1, rhs));
#endif
#endif
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

		Float4 Data = {
			1.0f, 0.0f,
			0.0f, 1.0f
		};
		View2x2 View{ std::data(Data) };
	};

	class Mat3 
	{
	public:
		friend class Mat4;

		Mat3() {}
		Mat3(const Vec3& Row0, const Vec3& Row1, const Vec3& Row2) : Rows{ Row0, Row1, Row2 } {}
		Mat3(const Mat3& rhs) : Rows{ rhs.Rows[0], rhs.Rows[1], rhs.Rows[2] } {}
		Mat3(View3x3 rhs) : View(rhs) {}

	private:
		static const Mat3 _Identity;
		static const Mat3 _Zero;
	public:
		inline static const Mat3& Identity() { return _Identity; }
		inline static const Mat3& Zero() { return _Zero; }

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
#ifdef USE_STD_LINALG
			Mat3 r;
			std::linalg::add(View, rhs.View, r.View);
			return r;
#else
#ifdef USE_OWN_IMPL
			return Mat3(Rows[0] + rhs.Rows[0], Rows[1] + rhs.Rows[1], Rows[2] + rhs.Rows[2]);
#else
			Mat3 r;
			std::ranges::transform(Rows, rhs.Rows, std::begin(r.Rows), std::plus());
			return r;
#endif
#endif
		}
		inline Mat3 operator-(const Mat3& rhs) const { 
#ifdef USE_OWN_IMPL
			return Mat3(Rows[0] - rhs.Rows[0], Rows[1] - rhs.Rows[1], Rows[2] - rhs.Rows[2]);
#else
			Mat3 r; 
			std::ranges::transform(Rows, rhs.Rows, std::begin(r.Rows), std::minus()); 
			return r;
#endif
		}
		inline Mat3 operator*(const float rhs) const {
#ifdef USE_STD_LINALG
			return Mat3(std::linalg::scale(rhs, View));
#else
#ifdef USE_OWN_IMPL
			return Mat3(Rows[0] * rhs, Rows[1] * rhs, Rows[2] * rhs);
#else
			Mat3 r;
			std::ranges::transform(Rows, std::begin(r.Rows), std::bind(std::multiplies(), std::placeholders::_1, rhs));
			return r;
#endif
#endif
		}
		inline Vec3 operator*(const Vec3& rhs) const { 
#ifdef USE_STD_LINALG
			Vec3 r;
			std::linalg::matrix_vector_product(View, rhs.View, r.View);
			return r;
#else
			return { Rows[0].Dot(rhs), Rows[1].Dot(rhs), Rows[2].Dot(rhs) };
#endif
		}
		inline Mat3 operator*(const Mat3& rhs) const {
#ifdef USE_STD_LINALG
			Mat3 r;
			std::linalg::matrix_product(View, rhs.View, r.View);
			return r;
#else
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
#endif
		}
		inline Mat3 operator/(const float rhs) const { 
#ifdef USE_STD_LINALG
			return Mat3(std::linalg::scale(1.0f / rhs, View));
#else
#ifdef USE_OWN_IMPL
			return Mat3(Rows[0] / rhs, Rows[1] / rhs, Rows[2] / rhs);
#else
			Mat3 r;
			std::ranges::transform(Rows, std::begin(r.Rows), std::bind(std::divides(), std::placeholders::_1, rhs));
			return r;
#endif
#endif
		}
		inline Mat3 operator-() const {
#ifdef USE_OWN_IMPL
			return Mat3(-Rows[0], -Rows[1], -Rows[2]);
#else
			Mat3 r; 
			std::ranges::transform(Rows, std::begin(r.Rows), std::negate()); 
			return r;
#endif
		}

		inline const Vec3& operator[](const int i) const { return Rows[i]; }
		inline operator const float* () const { return static_cast<const float*>(Rows[0]); }

		inline Mat3 Transpose() const {
#ifdef USE_STD_LINALG
			return Mat3(std::linalg::transposed(View));
#else
			return {
				{ Rows[0].X(), Rows[1].X(), Rows[2].X() },
				{ Rows[0].Y(), Rows[1].Y(), Rows[2].Y() },
				{ Rows[0].Z(), Rows[1].Z(), Rows[2].Z() }
			};
#endif
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
#ifdef USE_STD_LINALG
			std::linalg::copy(rhs.View, View);
#else
			std::ranges::copy(rhs.Rows, std::begin(Rows));
#endif
			return *this; 
		}
		inline const Mat3& operator+=(const Mat3& rhs) { 
#ifdef USE_STD_LINALG
			std::linalg::add(View, rhs.View, View);
#else
#ifdef USE_OWN_IMPL
			Rows[0] += rhs.Rows[0];
			Rows[1] += rhs.Rows[1];
			Rows[2] += rhs.Rows[2];
#else
			std::ranges::transform(Rows, rhs.Rows, std::begin(Rows), std::plus());
#endif
#endif
			return *this; 
		}
		inline const Mat3& operator-=(const Mat3& rhs) {
#ifdef USE_OWN_IMPL
			Rows[0] -= rhs.Rows[0];
			Rows[1] -= rhs.Rows[1];
			Rows[2] -= rhs.Rows[2];
#else
			std::ranges::transform(Rows, rhs.Rows, std::begin(Rows), std::minus());
#endif
			return *this; 
		}
		inline const Mat3& operator*=(const float rhs) { 
#ifdef USE_STD_LINALG
			std::linalg::scale(rhs, View);
#else
#ifdef USE_OWN_IMPL
			Rows[0] *= rhs;
			Rows[1] *= rhs;
			Rows[2] *= rhs;
#else
			std::ranges::transform(Rows, std::begin(Rows), std::bind(std::multiplies(), std::placeholders::_1, rhs));
#endif
#endif
			return *this; 
		}
		inline const Mat3& operator/=(const float rhs) {
#ifdef USE_STD_LINALG
			std::linalg::scale(1.0f / rhs, View);
#else
#ifdef USE_OWN_IMPL
			Rows[0] /= rhs;
			Rows[1] /= rhs;
			Rows[2] /= rhs;
#else
			std::ranges::transform(Rows, std::begin(Rows), std::bind(std::divides(), std::placeholders::_1, rhs));
#endif
#endif
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

		Float9 Data = {
			1.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 1.0f
		};
		View3x3 View{ std::data(Data) };
	};

	class Mat4
	{
	public:
		Mat4() {}
		Mat4(const Vec4& Row0, const Vec4& Row1, const Vec4& Row2, const Vec4& Row3) : Rows{ Row0, Row1, Row2, Row3 } {}
		Mat4(const Mat4& rhs) : Rows{ rhs.Rows[0], rhs.Rows[1], rhs.Rows[2], rhs.Rows[3] } {}
		Mat4(View4x4 rhs) : View(rhs) {}

	private:
		static const Mat4 _Identity;
		static const Mat4 _Zero;
	public:
		inline static const Mat4& Identity() { return _Identity; }
		inline static const Mat4& Zero() { return _Zero; }

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
#ifdef USE_STD_LINALG
			Mat4 r;
			std::linalg::add(View, rhs.View, r.View);
			return r;
#else
#ifdef USE_OWN_IMPL
			return Mat4(Rows[0] + rhs.Rows[0], Rows[1] + rhs.Rows[1], Rows[2] + rhs.Rows[2], Rows[3] + rhs.Rows[3]);
#else
			Mat4 r;
			std::ranges::transform(Rows, rhs.Rows, std::begin(r.Rows), std::plus());
			return r;
#endif
#endif
		}
		inline Mat4 operator-(const Mat4& rhs) const {
#ifdef USE_OWN_IMPL
			return Mat4(Rows[0] - rhs.Rows[0], Rows[1] - rhs.Rows[1], Rows[2] - rhs.Rows[2], Rows[3] - rhs.Rows[3]);
#else
			Mat4 r; 
			std::ranges::transform(Rows, rhs.Rows, std::begin(r.Rows), std::minus()); 
			return r;
#endif
		}
		inline Mat4 operator*(const float rhs) const { 
#ifdef USE_STD_LINALG
			return Mat4(std::linalg::scale(rhs, View));
#else
#ifdef USE_OWN_IMPL
			return Mat4(Rows[0] * rhs, Rows[1] * rhs, Rows[2] * rhs, Rows[3] * rhs);
#else
			Mat4 r;
			std::ranges::transform(Rows, std::begin(r.Rows), std::bind(std::multiplies(), std::placeholders::_1, rhs)); 
			return r;
#endif
#endif
		}
		inline Vec4 operator*(const Vec4& rhs) const {
#ifdef USE_STD_LINALG
			Vec4 r;
			std::linalg::matrix_vector_product(View, rhs.View, r.View);
			return r;
#else
			return { Rows[0].Dot(rhs), Rows[1].Dot(rhs), Rows[2].Dot(rhs), Rows[3].Dot(rhs) };
#endif
		}
		inline Mat4 operator*(const Mat4& rhs) const {
#ifdef USE_STD_LINALG
			Mat4 r;
			std::linalg::matrix_product(View, rhs.View, r.View);
			return r;
#else
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
#endif
		}
		inline Mat4 operator/(const float rhs) const { 
#ifdef USE_STD_LINALG
			return Mat4(std::linalg::scale(1.0f / rhs, View));
#else
#ifdef USE_OWN_IMPL
			return Mat4(Rows[0] / rhs, Rows[1] / rhs, Rows[2] / rhs, Rows[3] / rhs);
#else
			Mat4 r;
			std::ranges::transform(Rows, std::begin(r.Rows), std::bind(std::divides(), std::placeholders::_1, rhs)); 
			return r;
#endif
#endif
		}
		inline Mat4 operator-() const {
#ifdef USE_OWN_IMPL
			return Mat4(-Rows[0], -Rows[1], -Rows[2], -Rows[3]);
#else
			Mat4 r;
			std::ranges::transform(Rows, std::begin(r.Rows), std::negate()); 
			return r;
#endif
		}

		inline const Vec4& operator[](const int i) const { return Rows[i]; }
		inline operator const float* () const { return static_cast<const float*>(Rows[0]); }

		inline Mat4 Transpose() const {
#ifdef USE_STD_LINALG
			return Mat4(std::linalg::transposed(View));
#else
			return { 
				{ Rows[0].X(), Rows[1].X(), Rows[2].X(), Rows[3].X() }, 
				{ Rows[0].Y(), Rows[1].Y(), Rows[2].Y(), Rows[3].Y() }, 
				{ Rows[0].Z(), Rows[1].Z(), Rows[2].Z(), Rows[3].Z() }, 
				{ Rows[0].W(), Rows[1].W(), Rows[2].W(), Rows[3].W() } 
			};
#endif
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
#ifdef USE_STD_LINALG
			std::linalg::copy(rhs.View, View);
#else
			std::ranges::copy(rhs.Rows, std::begin(Rows));
#endif
			return *this; 
		}
		inline const Mat4& operator+=(const Mat4& rhs) { 
#ifdef USE_STD_LINALG
			std::linalg::add(View, rhs.View, View);
#else
#ifdef USE_OWN_IMPL
			Rows[0] += rhs.Rows[0];
			Rows[1] += rhs.Rows[1];
			Rows[2] += rhs.Rows[2];
			Rows[3] += rhs.Rows[3];
#else
			std::ranges::transform(Rows, rhs.Rows, std::begin(Rows), std::plus());
#endif
#endif
			return *this; 
		}
		inline const Mat4& operator-=(const Mat4& rhs) { 
#ifdef USE_OWN_IMPL
			Rows[0] -= rhs.Rows[0];
			Rows[1] -= rhs.Rows[1];
			Rows[2] -= rhs.Rows[2];
			Rows[3] -= rhs.Rows[3];
#else
			std::ranges::transform(Rows, rhs.Rows, std::begin(Rows), std::minus());
#endif
			return *this; 
		}
		inline const Mat4& operator*=(const float rhs) {
#ifdef USE_STD_LINALG
			std::linalg::scale(rhs, View);
#else
#ifdef USE_OWN_IMPL
			Rows[0] *= rhs;
			Rows[1] *= rhs;
			Rows[2] *= rhs;
			Rows[3] *= rhs;
#else
			std::ranges::transform(Rows, std::begin(Rows), std::bind(std::multiplies(), std::placeholders::_1, rhs));
#endif
#endif
			return *this; 
		}
		inline const Mat4& operator/=(const float rhs) {
#ifdef USE_STD_LINALG
			std::linalg::scale(1.0f / rhs, View);
#else
#ifdef USE_OWN_IMPL
			Rows[0] /= rhs;
			Rows[1] /= rhs;
			Rows[2] /= rhs;
			Rows[3] /= rhs;
#else
			std::ranges::transform(Rows, std::begin(Rows), std::bind(std::divides(), std::placeholders::_1, rhs));
#endif
#endif
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

		Float16 Data = {
			1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f
		};
		View4x4 View{ std::data(Data) };
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
		Mat(std::mdspan<float, std::extents<std::size_t, N, M>> rhs) : View(rhs) {}

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
			Mat r;
#ifdef USE_STD_LINALG
			std::linalg::add(View, rhs.View, r.View);
#else
#ifdef USE_OWN_IMPL
			for (auto i = 0; i < M; ++i) {
				r.Rows[i] = Rows[i] + rhs.Rows[i];
			}
#else
			std::ranges::transform(Rows, rhs.Rows, std::begin(r.Rows), std::plus()); 
#endif
#endif
			return r;
		}
		inline Mat operator-(const Mat& rhs) const {
			Mat r; 
#ifdef USE_OWN_IMPL
			for (auto i = 0; i < M; ++i) {
				r.Rows[i] = Rows[i] - rhs.Rows[i];
			}
#else
			std::ranges::transform(Rows, rhs.Rows, std::begin(r.Rows), std::minus()); 
#endif
			return r;
		}
		inline Mat operator*(const float rhs) const {
#ifdef USE_STD_LINALG
			return Mat(std::linalg::scale(rhs, View));
#else
			Mat r; 
#ifdef USE_OWN_IMPL
			for (auto i = 0; i < M; ++i) {
				r.Rows[i] = Rows[i] * rhs;
			}
#else
			std::ranges::transform(Rows, std::begin(r.Rows), std::bind(std::multiplies(), std::placeholders::_1, rhs)); 
#endif
			return r;
#endif
		}
		template<uint32_t N>
		inline Vec<M> operator*(const Vec<N>& rhs) const {
#ifdef USE_STD_LINALG
			Vec<M> r;
			std::linalg::matrix_vector_product(View, rhs.View, r.View);
			return r;
#else
			Vec<M> r;
			for (auto i = 0; i < M; ++i) {
				r[i] = rhs.Dot(Rows[i]);
			}
			return r;
#endif
		}
		template<uint32_t O>
		inline Mat<M, O> operator*(const Mat<N, O>& rhs) const {
#ifdef USE_STD_LINALG
			Mat<M, O> r;
			std::linalg::matrix_product(View, rhs.View, r.View);
			return r;
#else
			Mat<M, O> r;
			const auto Trs = rhs.Transpose();
			for (auto i = 0; i < M; ++i) {
				for (auto j = 0; j < O; ++j) {
					r[i][j] = Rows[i].Dot(Trs[j]);
				}
			}
			return r;
#endif
		}
		inline Mat operator/(const float rhs) const {
#ifdef USE_STD_LINALG
			return Mat(std::linalg::scale(1.0f / rhs, View));
#else
			Mat r; 
#ifdef USE_OWN_IMPL
			for (auto i = 0; i < M; ++i) {
				r.Rows[i] = Rows[i] / rhs;
			}
#else
			std::ranges::transform(Rows, std::begin(r.Rows), std::bind(std::divides(), std::placeholders::_1, rhs)); 
#endif
			return r;
#endif
		}
		inline Mat operator-() const {
			Mat r; 
#ifdef USE_OWN_IMPL
			for (auto i = 0; i < M; ++i) {
				r.Rows[i] = -Rows[i];
			}
#else
			std::ranges::transform(Rows, std::begin(r.Rows), std::negate()); 
#endif
			return r;
		}

		inline const Vec<N>& operator[](const int i) const { return Rows[i]; }
		inline operator const float* () const { return static_cast<const float*>(Rows[0]); }

		inline Mat<N, M> Transpose() const {
#ifdef USE_STD_LINALG
			return Mat(std::linalg::transposed(View));
#else
			Mat<N, M> r;
			for (auto i = 0; i < M; ++i) {
				for (auto j = 0; j < N; ++j) {
					r[j][i] = Rows[i][j];
				}
			}
			return r;
#endif
		}

		inline const Mat& operator+=(const Mat& rhs) {
#ifdef USE_STD_LINALG
			std::linalg::add(View, rhs.View, View);
#else
#ifdef USE_OWN_IMPL
			for (auto i = 0; i < M; ++i) {
				Rows[i] += rhs.Rows[i];
			}
#else
			std::ranges::transform(Rows, rhs.Rows, std::begin(Rows), std::plus());
#endif
#endif
			return *this;
		}
		inline const Mat& operator-=(const Mat& rhs) {
#ifdef USE_OWN_IMPL
			for (auto i = 0; i < M; ++i) {
				Rows[i] -= rhs.Rows[i];
			}
#else
			std::ranges::transform(Rows, rhs.Rows, std::begin(Rows), std::minus());
#endif
			return *this;
		}
		inline const Mat& operator*=(const float rhs) {
#ifdef USE_STD_LINALG
			std::linalg::scale(rhs, View);
#else
#ifdef USE_OWN_IMPL
			for (auto i = 0; i < M; ++i) {
				Rows[i] *= rhs;
			}
#else
			std::ranges::transform(Rows, std::begin(Rows), std::bind(std::multiplies(), std::placeholders::_1, rhs));
#endif
#endif
			return *this;
		}
		inline const Mat& operator/=(const float rhs) {
#ifdef USE_STD_LINALG
			std::linalg::scale(1.0f / rhs, View);
#else
#ifdef USE_OWN_IMPL
			for (auto i = 0; i < M; ++i) {
				Rows[i] /= rhs;
			}
#else
			std::ranges::transform(Rows, std::begin(Rows), std::bind(std::divides(), std::placeholders::_1, rhs));
#endif
#endif
			return *this;
		}
		inline Vec<N>& operator[](const int i) { return Rows[i]; }
		inline operator float* () { return static_cast<float*>(Rows[0]); }

		inline Mat& ToZero() { return (*this = Zero()); }
		inline Mat& ToIdentity() { return (*this = Identity()); }

		inline size_t Size() const { return std::size(Rows); }

	private:
		std::array<Vec<N>, M> Rows;

		std::array<float, N * M> Data;
		std::mdspan<float, std::extents<std::size_t, N, M>> View{ std::data(Data) };
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

