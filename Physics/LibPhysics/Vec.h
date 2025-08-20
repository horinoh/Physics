#pragma once

namespace LinAlg 
{
	class Vec2
	{
	public:
		friend class Vec3;
		friend class Mat2;

		Vec2() {}
		Vec2(const float x, const float y) : Data({ x, y }) {}
		Vec2(const float rhs) : Data({ rhs, rhs }) {}
		Vec2(View2 rhs) : View(rhs) {}

	private:
		static const Vec2 _Zero;
		static const Vec2 _One;
		static const Vec2 _AxisX;
		static const Vec2 _AxisY;
		static const Vec2 _Epsilon;
		static const Vec2 _Min;
		static const Vec2 _Max;
		static const Vec2 _UnitXY;
	public:
		inline static const Vec2& Zero() { return _Zero; }
		inline static const Vec2& One() { return _One; }
		inline static const Vec2& AxisX() { return _AxisX; }
		inline static const Vec2& AxisY() { return _AxisY; }
		inline static const Vec2& Epsilon() { return _Epsilon; }
		inline static const Vec2& Min() { return _Min; }
		inline static const Vec2& Max() { return _Max; }
		inline static const Vec2& UnitXY() { return _UnitXY; }

		inline static bool NearlyEqual(const Vec2& lhs, const Vec2& rhs, const float Epsilon) {
			return std::ranges::equal(lhs.Data, rhs.Data, 
				[&](const auto l, const auto r) {
					return std::abs(l - r) < Epsilon;
				});
		}

		inline bool IsValid() const {
			return std::ranges::all_of(Data, [](const auto& i) { return !std::isnan(i); });
		}
		inline bool NearlyEqual(const Vec2& rhs, const float Epsilon = (std::numeric_limits<float>::epsilon)()) const {
			return NearlyEqual(*this, rhs, Epsilon); 
		}

		inline bool operator==(const Vec2& rhs) const {
			return std::ranges::equal(Data, rhs.Data);
		}
		inline bool operator!=(const Vec2& rhs) const { return !(*this == rhs); }
		inline Vec2 operator+(const Vec2& rhs) const {
#ifdef USE_STD_LINALG
			Vec2 r;
			std::linalg::add(View, rhs.View, r.View);
			return r;
#else
#ifdef USE_OWN_IMPL
			return Vec2(Data[0] + rhs.Data[0], Data[1] + rhs.Data[1]);
#else
			Vec2 r;
			std::ranges::transform(Data, rhs.Data, std::begin(r.Data), std::plus());
			return r;
#endif
#endif
		}
		inline Vec2 operator-(const Vec2& rhs) const {
#ifdef USE_OWN_IMPL
			return Vec2(Data[0] - rhs.Data[0], Data[1] - rhs.Data[1]);
#else
			Vec2 r;
			std::ranges::transform(Data, rhs.Data, std::begin(r.Data), std::minus()); 
			return r;
#endif
		}
		inline Vec2 operator*(const float rhs) const {
#ifdef USE_STD_LINALG
			return Vec2(std::linalg::scale(rhs, View));
#else
#ifdef USE_OWN_IMPL
			return Vec2(Data[0] * rhs, Data[1] * rhs);
#else
			Vec2 r;
			std::ranges::transform(Data, std::begin(r.Data), std::bind(std::multiplies(), std::placeholders::_1, rhs));
			return r;
#endif
#endif
		}
		inline Vec2 operator/(const float rhs) const {
#ifdef USE_STD_LINALG
			return Vec2(std::linalg::scale(1.0f / rhs, View));
#else
#ifdef USE_OWN_IMPL
			return Vec2(Data[0] / rhs, Data[1] / rhs);
#else
			Vec2 r;
			std::ranges::transform(Data, std::begin(r.Data), std::bind(std::divides(), std::placeholders::_1, rhs));
			return r;
#endif
#endif
		}
		inline Vec2 operator-() const {
#ifdef USE_OWN_IMPL
			return Vec2(-Data[0], -Data[1]);
#else
			Vec2 r;
			std::ranges::transform(Data, std::begin(r.Data), std::negate());
			return r;
#endif
		}

		inline float X() const { return Data[0]; }
		inline float Y() const { return Data[1]; }
		inline float operator[](const int i) const { return Data[i]; }
		inline operator const float* () const { return std::data(Data); }

		inline float Dot(const Vec2& rhs) const {
#ifdef USE_STD_LINALG
			return std::linalg::dot(View, rhs.View)
#else
			return std::inner_product(std::cbegin(Data), std::cend(Data), std::cbegin(rhs.Data), 0.0f);
#endif
		}
		inline float LengthSq() const { 
#ifdef USE_STD_LINALG
			return std::linalg::vector_sum_of_squares(View);
#else
			return Dot(*this);
#endif
		}
		inline float Length() const {
#ifdef USE_STD_LINALG
			return std::linalg::vector_two_norm(View);
#else
			return std::sqrtf(LengthSq());
#endif
		}
		inline Vec2 Normalize() const {
#ifdef USE_STD_LINALG
			return Vec2(std::linalg::scale(1.0f / std::linalg::vector_two_norm(View), View));
#else
			const auto Sq = LengthSq();
			if (Sq > (std::numeric_limits<float>::epsilon)()) {
				return *this / std::sqrtf(Sq);
			}
#ifdef _DEBUG
			//__debugbreak();
#endif
			return Vec2(*this);
#endif
		}

		inline Vec2& operator=(const Vec2& rhs) {
#ifdef USE_STD_LINALG
			std::linalg::copy(rhs.View, View);
#else
			std::ranges::copy(rhs.Data, std::begin(Data));
#endif
			return *this;
		}
		inline const Vec2& operator+=(const Vec2& rhs) {
#ifdef USE_STD_LINALG
			std::linalg::add(View, rhs.View, View);
#else
#ifdef USE_OWN_IMPL
			Data[0] += rhs.Data[0];
			Data[1] += rhs.Data[1];
#else
			std::ranges::transform(Data, rhs.Data, std::begin(Data), std::plus());
#endif
#endif
			return *this;
		}
		inline const Vec2& operator-=(const Vec2& rhs) {
#ifdef USE_OWN_IMPL
			Data[0] -= rhs.Data[0];
			Data[1] -= rhs.Data[1];
#else
			std::ranges::transform(Data, rhs.Data, std::begin(Data), std::minus());
#endif
			return *this;
		}
		inline const Vec2& operator*=(const float rhs) {
#ifdef USE_STD_LINALG
			std::linalg::scale(rhs, View);
#else
#ifdef USE_OWN_IMPL
			Data[0] *= rhs;
			Data[1] *= rhs;
#else
			std::ranges::transform(Data, std::begin(Data), std::bind(std::multiplies(), std::placeholders::_1, rhs));
#endif
#endif
			return *this;
		}
		inline const Vec2& operator/=(const float rhs) {
#ifdef USE_STD_LINALG
			std::linalg::scale(1.0f / rhs, View);
#else
#ifdef USE_OWN_IMPL
			Data[0] /= rhs;
			Data[1] /= rhs;
#else
			std::ranges::transform(Data, std::begin(Data), std::bind(std::divides(), std::placeholders::_1, rhs));
#endif
#endif
			return *this;
		}
		inline float& operator[](const int i) { return Data[i]; }
		inline operator float* () { return std::data(Data); }
		inline operator const Float2& () const { return Data; }
		inline operator Float2& () { return Data; }

		inline Vec2& ToZero() { return (*this = Zero()); }
		inline Vec2& ToNormalize() { return (*this = Normalize()); }
		inline Vec2& Adjust(const float Length) { return (*this = Normalize() * Length); }

		inline std::string ToString() const { return std::format("({:1.4f}, {:1.4f})\n", X(), Y()); }

	private:
		Float2 Data = { 0.0f, 0.0f };
		View2 View{std::data(Data)};
	};

	class Vec3
	{
	public:
		friend class Vec4;
		friend class Mat3;

		Vec3() {}
		Vec3(const float x, const float y, const float z) : Data({x, y, z}) {}
		Vec3(const float rhs) : Data({rhs, rhs, rhs}) {}
		Vec3(const Vec2& rhs, const float z = 0.0f) : Data({ rhs.X(), rhs.Y(), z }) {}
		Vec3(View3 rhs) : View(rhs) {}

	private:
		static const Vec3 _Zero;
		static const Vec3 _One;
		static const Vec3 _AxisX;
		static const Vec3 _AxisY;
		static const Vec3 _AxisZ;
		static const Vec3 _Epsilon;
		static const Vec3 _Min;
		static const Vec3 _Max;
		static const Vec3 _UnitXYZ;
	public:
		inline static const Vec3& Zero() { return _Zero; }
		inline static const Vec3& One() { return _One; }
		inline static const Vec3& AxisX() { return _AxisX; }
		inline static const Vec3& AxisY() { return _AxisY; }
		inline static const Vec3& AxisZ() { return _AxisZ; }
		inline static const Vec3& Epsilon() { return _Epsilon; }
		inline static const Vec3& Min() { return _Min; }
		inline static const Vec3& Max() { return _Max; }
		inline static const Vec3& UnitXYZ() { return _UnitXYZ; }

		inline static bool NearlyEqual(const Vec3& lhs, const Vec3& rhs, const float Epsilon) {
			return std::ranges::equal(lhs.Data, rhs.Data, 
				[&](const auto l, const auto r) { 
					return std::abs(l - r) < Epsilon; 
				});
		}
		inline static Vec3 Normal(const Vec3& A, const Vec3& B, const Vec3& C) { return (B - A).Cross(C - A); }
		inline static Vec3 UnitNormal(const Vec3& A, const Vec3& B, const Vec3& C) { return Normal(A, B, C).Normalize(); }
		
		inline bool IsValid() const {
			return std::ranges::all_of(Data, [](const auto& i) { return !std::isnan(i); });
		}
		inline bool NearlyEqual(const Vec3& rhs, const float Epsilon = (std::numeric_limits<float>::epsilon)()) const { 
			return NearlyEqual(*this, rhs, Epsilon); 
		}

		inline bool operator==(const Vec3& rhs) const {
			return std::ranges::equal(Data, rhs.Data);
		}
		inline bool operator!=(const Vec3& rhs) const { return !(*this == rhs); }
		inline Vec3 operator+(const Vec3& rhs) const { 
#ifdef USE_STD_LINALG
			Vec3 r;
			std::linalg::add(View, rhs.View, r.View);
			return r;
#else
#ifdef USE_OWN_IMPL
			return Vec3(Data[0] + rhs.Data[0], Data[1] + rhs.Data[1], Data[2] + rhs.Data[2]);
#else
			Vec3 r;
			std::ranges::transform(Data, rhs.Data, std::begin(r.Data), std::plus());
			return r;
#endif
#endif
		}
		inline Vec3 operator-(const Vec3& rhs) const {
#ifdef USE_OWN_IMPL
			return Vec3(Data[0] - rhs.Data[0], Data[1] - rhs.Data[1], Data[2] - rhs.Data[2]);
#else	
			Vec3 r; 
			std::ranges::transform(Data, rhs.Data, std::begin(r.Data), std::minus());
			return r;
#endif
		}
		inline Vec3 operator*(const float rhs) const {
#ifdef USE_STD_LINALG
			return Vec3(std::linalg::scale(rhs, View));
#else
#ifdef USE_OWN_IMPL
			return Vec3(Data[0] * rhs, Data[1] * rhs, Data[2] * rhs);
#else
			Vec3 r;
			std::ranges::transform(Data, std::begin(r.Data), std::bind(std::multiplies(), std::placeholders::_1, rhs));
			return r;
#endif
#endif
		}
		inline Vec3 operator/(const float rhs) const { 
#ifdef USE_STD_LINALG
			return Vec3(std::linalg::scale(1.0f / rhs, View));
#else
#ifdef USE_OWN_IMPL
			return Vec3(Data[0] / rhs, Data[1] / rhs, Data[2] / rhs);
#else
			Vec3 r;
			std::ranges::transform(Data, std::begin(r.Data), std::bind(std::divides(), std::placeholders::_1, rhs)); 
			return r;
#endif
#endif
		}
		inline Vec3 operator-() const { 
#ifdef USE_OWN_IMPL
			return Vec3(-Data[0], -Data[1], -Data[2]);
#else
			Vec3 r; 
			std::ranges::transform(Data, std::begin(r.Data), std::negate()); 
			return r;
#endif
		}

		inline float X() const { return Data[0]; }
		inline float Y() const { return Data[1]; }
		inline float Z() const { return Data[2]; }
		inline float operator[](const int i) const { return Data[i]; }
		inline operator const float* () const { return std::data(Data); }

		inline float Dot(const Vec3& rhs) const { 
#ifdef USE_STD_LINALG
			return std::linalg::dot(View, rhs.View)
#else
			return std::inner_product(std::cbegin(Data), std::cend(Data), std::cbegin(rhs.Data), 0.0f);
#endif
		}
		inline float LengthSq() const { 
#ifdef USE_STD_LINALG
			return std::linalg::vector_sum_of_squares(View);
#else
			return Dot(*this);
#endif
		}
		inline float Length() const {
#ifdef USE_STD_LINALG
			return std::linalg::vector_two_norm(View);
#else
			return std::sqrtf(LengthSq());
#endif
		}
		inline Vec3 Normalize() const {
#ifdef USE_STD_LINALG
			return Vec3(std::linalg::scale(1.0f / std::linalg::vector_two_norm(View), View));
#else
			const auto Sq = LengthSq();
			if (Sq > (std::numeric_limits<float>::epsilon)()) { 
				return *this / std::sqrtf(Sq); 
			}
#ifdef _DEBUG
			//__debugbreak();
#endif
			return Vec3(*this);
#endif
		}
		inline Vec3 Cross(const Vec3& rhs) const {
			return Vec3(Y() * rhs.Z() - rhs.Y() * Z(), rhs.X() * Z() - X() * rhs.Z(), X() * rhs.Y() - rhs.X() * Y()); 
		}
		//!< 垂直な 2 軸を取得
		void GetOrtho(Vec3& U, Vec3& V) const {
			const auto N = Normalize();
			const auto W = (N.Z() * N.Z() > 0.9f * 0.9f) ? Vec3::AxisX() : Vec3::AxisZ();
			V = N.Cross(W.Cross(N).Normalize()).Normalize();
			U = V.Cross(N).Normalize();
		}

		inline Vec3& operator=(const Vec2& rhs) { 
			Data[0] = rhs.X(); Data[1] = rhs.Y(); 
			return *this; 
		}
		inline Vec3& operator=(const Vec3& rhs) {
#ifdef USE_STD_LINALG
			std::linalg::copy(rhs.View, View);
#else
			std::ranges::copy(rhs.Data, std::begin(Data));
#endif
			return *this; 
		}
		inline const Vec3& operator+=(const Vec3& rhs) { 
#ifdef USE_STD_LINALG
			std::linalg::add(View, rhs.View, View);
#else
#ifdef USE_OWN_IMPL
			Data[0] += rhs.Data[0];
			Data[1] += rhs.Data[1];
			Data[2] += rhs.Data[2];
#else
			std::ranges::transform(Data, rhs.Data, std::begin(Data), std::plus());
#endif
#endif
			return *this; 
		}
		inline const Vec3& operator-=(const Vec3& rhs) {
#ifdef USE_OWN_IMPL
			Data[0] -= rhs.Data[0];
			Data[1] -= rhs.Data[1];
			Data[2] -= rhs.Data[2];
#else
			std::ranges::transform(Data, rhs.Data, std::begin(Data), std::minus());
#endif
			return *this; 
		}
		inline const Vec3& operator*=(const float rhs) { 
#ifdef USE_STD_LINALG
			std::linalg::scale(rhs, View);
#else
#ifdef USE_OWN_IMPL
			Data[0] *= rhs;
			Data[1] *= rhs;
			Data[2] *= rhs;
#else
			std::ranges::transform(Data, std::begin(Data), std::bind(std::multiplies(), std::placeholders::_1, rhs));
#endif
#endif
			return *this;
		}
		inline const Vec3& operator/=(const float rhs) {
#ifdef USE_STD_LINALG
			std::linalg::scale(1.0f / rhs, View);
#else
#ifdef USE_OWN_IMPL
			Data[0] /= rhs;
			Data[1] /= rhs;
			Data[2] /= rhs;
#else
			std::ranges::transform(Data, std::begin(Data), std::bind(std::divides(), std::placeholders::_1, rhs));
#endif
#endif
			return *this;
		}
		inline float& operator[](const int i) { return Data[i]; }
		inline operator float* () { return std::data(Data); }
		inline operator const Float3& () const { return Data; }
		inline operator Float3& () { return Data; }

		inline Vec3& ToZero() { return (*this = Zero()); }
		inline Vec3& ToNormalized() { return (*this = Normalize()); }
		inline Vec3& Adjust(const float Length) { return (*this = Normalize() * Length); }

		inline std::string ToString() const { return std::format("({:1.4f}, {:1.4f}, {:1.4f})\n", X(), Y(), Z()); }
	
	private:
		Float3 Data = { 0.0f, 0.0f, 0.0f };
		View3 View{ std::data(Data) };
	};

	class Vec4
	{
	public:
		friend class Mat4;

		Vec4() {}
		Vec4(const float x, const float y, const float z, const float w) : Data({x, y, z, w}) {}
		Vec4(const float rhs) : Data({ rhs, rhs, rhs, rhs }) {}
		Vec4(const Vec3& rhs, const float w = 0.0f) : Data({ rhs.X(), rhs.Y(), rhs.Z(), w }) {}
		Vec4(View4 rhs) : View(rhs) {}

	private:
		static const Vec4 _Zero;
		static const Vec4 _One;
		static const Vec4 _AxisX;
		static const Vec4 _AxisY;
		static const Vec4 _AxisZ;
		static const Vec4 _AxisW;
		static const Vec4 _Epsilon;
		static const Vec4 _Min;
		static const Vec4 _Max;
		static const Vec4 _UnitXYZW;
	public:
		inline static const Vec4& Zero() { return _Zero; }
		inline static const Vec4& One() { return _One; }
		inline static const Vec4& AxisX() { return _AxisX; }
		inline static const Vec4& AxisY() { return _AxisY; }
		inline static const Vec4& AxisZ() { return _AxisZ; }
		inline static const Vec4& AxisW() { return _AxisW; }
		inline static const Vec4& Epsilon() { return _Epsilon; }
		inline static const Vec4& Min() { return _Min; }
		inline static const Vec4& Max() { return _Max; }
		inline static const Vec4& UnitXYZ() { return _UnitXYZW; }

		inline static bool NearlyEqual(const Vec4& lhs, const Vec4& rhs, const float Epsilon) {
			return std::ranges::equal(lhs.Data, rhs.Data,
				[&](const auto l, const auto r) {
					return std::abs(l - r) < Epsilon; 
				});
		}

		inline bool IsValid() const {
			return std::ranges::all_of(Data, [](const auto& i) { return !std::isnan(i); });
		}
		inline bool NearlyEqual(const Vec4& rhs, const float Epsilon = (std::numeric_limits<float>::epsilon)()) const { 
			return NearlyEqual(*this, rhs, Epsilon);
		}

		inline bool operator==(const Vec4& rhs) const {
			return std::ranges::equal(Data, rhs.Data);
		}
		inline bool operator!=(const Vec4& rhs) const { return !(*this == rhs); }
		inline Vec4 operator+(const Vec4& rhs) const { 
#ifdef USE_STD_LINALG
			Vec4 r;
			std::linalg::add(View, rhs.View, r.View);
			return r;
#else
#ifdef USE_OWN_IMPL
			return Vec4(Data[0] + rhs.Data[0], Data[1] + rhs.Data[1], Data[2] + rhs.Data[2], Data[3] + rhs.Data[3]);
#else
			Vec4 r;
			std::ranges::transform(Data, rhs.Data, std::begin(r.Data), std::plus());
			return r;
#endif
#endif
		}
		inline Vec4 operator-(const Vec4& rhs) const {
#ifdef USE_OWN_IMPL
			return Vec4(Data[0] - rhs.Data[0], Data[1] - rhs.Data[1], Data[2] - rhs.Data[2], Data[3] - rhs.Data[3]);
#else
			Vec4 r; 
			std::ranges::transform(Data, rhs.Data, std::begin(r.Data), std::minus()); 
			return r;
#endif
		}
		inline Vec4 operator*(const float rhs) const {
#ifdef USE_STD_LINALG
			return Vec4(std::linalg::scale(rhs, View));
#else
#ifdef USE_OWN_IMPL
			return Vec4(Data[0] * rhs, Data[1] * rhs, Data[2] * rhs, Data[3] * rhs);
#else
			Vec4 r;
			std::ranges::transform(Data, std::begin(r.Data), std::bind(std::multiplies(), std::placeholders::_1, rhs));
			return r;
#endif
#endif
		}
		inline Vec4 operator/(const float rhs) const {
#ifdef USE_STD_LINALG
			return Vec4(std::linalg::scale(1.0f / rhs, View));
#else
#ifdef USE_OWN_IMPL
			return Vec4(Data[0] / rhs, Data[1] / rhs, Data[2] / rhs, Data[3] / rhs);
#else
			Vec4 r;
			std::ranges::transform(Data, std::begin(r.Data), std::bind(std::divides(), std::placeholders::_1, rhs));
			return r;
#endif
#endif
		}
		inline Vec4 operator-() const {
#ifdef USE_OWN_IMPL
			return Vec4(-Data[0], -Data[1], -Data[2], -Data[3]);
#else
			Vec4 r; 
			std::ranges::transform(Data, std::begin(r.Data), std::negate());
			return r;
#endif
		}

		inline float X() const { return Data[0]; }
		inline float Y() const { return Data[1]; }
		inline float Z() const { return Data[2]; }
		inline float W() const { return Data[3]; }
		inline float operator[](const int i) const { return Data[i]; }
		inline operator const float* () const { return std::data(Data); }

		inline float Dot(const Vec4& rhs) const { 
#ifdef USE_STD_LINALG
			return std::linalg::dot(View, rhs.View);
#else
			return std::inner_product(std::cbegin(Data), std::cend(Data), std::cbegin(rhs.Data), 0.0f);
#endif
		}
		inline float LengthSq() const { 
#ifdef USE_STD_LINALG
			return std::linalg::vector_sum_of_squares(View);
#else
			return Dot(*this);
#endif
		}
		inline float Length() const {
#ifdef USE_STD_LINALG
			return std::linalg::vector_two_norm(View);
#else
			return std::sqrtf(LengthSq());
#endif
		}
		inline Vec4 Normalize() const {
#ifdef USE_STD_LINALG
			return Vec4(std::linalg::scale(1.0f / std::linalg::vector_two_norm(View), View));
#else
			const auto Sq = LengthSq();
			if (Sq > (std::numeric_limits<float>::epsilon)()) {
				return *this / std::sqrtf(Sq);
			}
#ifdef _DEBUG
			//__debugbreak();
#endif
			return Vec4(*this);
#endif
		}
		inline Vec4& operator=(const Vec2& rhs) { 
			Data[0] = rhs.X(); Data[1] = rhs.Y(); 
			return *this;
		}
		inline Vec4& operator=(const Vec3& rhs) { 
			Data[0] = rhs.X(); Data[1] = rhs.Y(); Data[2] = rhs.Z(); 
			return *this; 
		}
		inline Vec4& operator=(const Vec4& rhs) {
#ifdef USE_STD_LINALG
			std::linalg::copy(rhs.View, View);
#else
			std::ranges::copy(rhs.Data, std::begin(Data));
#endif
			return *this;
		}
		inline const Vec4& operator+=(const Vec4& rhs) { 
#ifdef USE_STD_LINALG
			std::linalg::add(View, rhs.View, View);
#else
#ifdef USE_OWN_IMPL
			Data[0] += rhs.Data[0];
			Data[1] += rhs.Data[1];
			Data[2] += rhs.Data[2];
			Data[3] += rhs.Data[3];
#else
			std::ranges::transform(Data, rhs.Data, std::begin(Data), std::plus());
#endif
#endif
			return *this;
		}
		inline const Vec4& operator-=(const Vec4& rhs) { 
#ifdef USE_OWN_IMPL
			Data[0] -= rhs.Data[0];
			Data[1] -= rhs.Data[1];
			Data[2] -= rhs.Data[2];
			Data[3] -= rhs.Data[3];
#else
			std::ranges::transform(Data, rhs.Data, std::begin(Data), std::minus());
#endif
			return *this;
		}
		inline const Vec4& operator*=(const float rhs) {
#ifdef USE_STD_LINALG
			std::linalg::scale(rhs, View);
#else
#ifdef USE_OWN_IMPL
			Data[0] *= rhs;
			Data[1] *= rhs;
			Data[2] *= rhs;
			Data[3] *= rhs;
#else
			std::ranges::transform(Data, std::begin(Data), std::bind(std::multiplies(), std::placeholders::_1, rhs));
#endif
#endif
			return *this; 
		}
		inline const Vec4& operator/=(const float rhs) { 
#ifdef USE_STD_LINALG
			std::linalg::scale(1.0f / rhs, View);
#else
#ifdef USE_OWN_IMPL
			Data[0] /= rhs;
			Data[1] /= rhs;
			Data[2] /= rhs;
			Data[3] /= rhs;
#else
			std::ranges::transform(Data, std::begin(Data), std::bind(std::divides(), std::placeholders::_1, rhs));
#endif
#endif
			return *this; 
		}
		inline float& operator[](const int i) { return Data[i]; }
		inline operator float* () { return std::data(Data); }
		inline operator const Float4& () const { return Data; }
		inline operator Float4& () { return Data; }

		inline Vec4& ToZero() { return (*this = Zero()); }
		inline Vec4& ToNormalized() { return (*this = Normalize()); }
		inline Vec4& Adjust(const float Length) { return (*this = Normalize() * Length); }

		inline std::string ToString() const { return std::format("({:1.4f}, {:1.4f}, {:1.4f}, {:1.4f})\n", X(), Y(), Z(), W()); }

	private:
		Float4 Data = { 0.0f, 0.0f, 0.0f, 0.0f };
		View4 View{ std::data(Data) };
	};

	template<size_t N>
	class Vec
	{
	public:
		//!< ここでは「疎」を扱う事が多いので、Vec<N> ではデフォルトコンストラクタでゼロクリアとしておく
		Vec() { std::ranges::fill(Data, 0.0f); }
		Vec(const float rhs) { std::ranges::fill(Data, rhs); }
		template<typename...A> Vec(A...Args) {
			auto i = 0;
			for (const auto f : std::initializer_list<float>{ Args... }) {
#ifdef _DEBUG
				if (i >= N) { break; }
#endif
				Data[i++] = f;
			}
		}
		Vec(std::mdspan<float, std::extents<std::size_t, N>> rhs) : View(rhs) {}

		inline static Vec Zero() { return Vec(); }
		inline static Vec One() { return Vec(1.0f); }
		inline static Vec Epsilon() { return Vec((std::numeric_limits<float>::epsilon)()); }
		inline static Vec Min() { return Vec((std::numeric_limits<float>::min)()); }
		inline static Vec Max() { return Vec((std::numeric_limits<float>::max)()); }
		inline static bool NearlyEqual(const Vec lhs, const Vec& rhs, const float Epsilon) {
			return std::ranges::equal(lhs.Data, rhs.Data,
				[&](const auto l, const auto r) { 
					return std::abs(l - r) < Epsilon; 
				});
		}

		inline bool IsValid() const {
			return std::ranges::all_of(Data, [](const auto& i) { return !std::isnan(i); });
		}
		inline bool NearlyEqual(const Vec& rhs, const float Epsilon = (std::numeric_limits<float>::epsilon)()) const {
			return NearlyEqual(*this, rhs, Epsilon); 
		}

		inline bool operator==(const Vec& rhs) const { return std::ranges::equal(Data, rhs.Data); }
		inline bool operator!=(const Vec& rhs) const { return !(*this == rhs); }
		inline Vec operator+(const Vec& rhs) const { 
			Vec r;
#ifdef USE_STD_LINALG
			std::linalg::add(View, rhs.View, r.View);
#else
#ifdef USE_OWN_IMPL
			for (auto i = 0; i < N; ++i) {
				r.Data[i] = Data[i] + rhs.Data[i];
			}
#else
			std::ranges::transform(Data, rhs.Data, std::begin(r.Data), std::plus());
#endif
#endif
			return r;
		}
		inline Vec operator-(const Vec& rhs) const {
			Vec r;
#ifdef USE_OWN_IMPL
			for (auto i = 0; i < N; ++i) {
				r.Data[i] = Data[i] - rhs.Data[i];
			}
#else
			std::ranges::transform(Data, rhs.Data, std::begin(r.Data), std::minus());
#endif
			return r;
		}
		inline Vec operator*(const float rhs) const {
#ifdef USE_STD_LINALG
			return Vec(std::linalg::scale(rhs, View));
#else
			Vec r;
#ifdef USE_OWN_IMPL
			for (auto i = 0; i < N; ++i) {
				r.Data[i] = Data[i] * rhs;
			}
#else
			std::ranges::transform(Data, rhs, std::begin(r.Data), std::multiplies());
#endif
			return r;
#endif
		}
		inline Vec operator/(const float rhs) const {
#ifdef USE_STD_LINALG
			return Vec(std::linalg::scale(1.0f / rhs, View));
#else
			Vec r;
#ifdef USE_OWN_IMPL
			for (auto i = 0; i < N; ++i) {
				r.Data[i] = Data[i] / rhs;
			}
#else
			std::ranges::transform(Data, rhs, std::begin(r.Data), std::divides());
#endif
			return r;
#endif
		}
		inline Vec operator-() const { 
			Vec r; 
#ifdef USE_OWN_IMPL
			for (auto i = 0; i < N; ++i) {
				r.Data[i] = -Data[i];
			}
#else
			std::ranges::transform(Data, std::begin(r.Data), std::negate());
#endif
			return r;
		}

		inline float operator[](const int i) const { return Data[i]; }
		inline operator const float* () const { return std::data(Data); }

		inline float Dot(const Vec& rhs) const { 
#ifdef USE_STD_LINALG
			return std::linalg::dot(View, rhs.View);
#else
			return std::inner_product(std::cbegin(Data), std::cend(Data), std::cbegin(rhs.Data), 0.0f);
#endif
		}
		inline float LengthSq() const {
#ifdef USE_STD_LINALG
			return std::linalg::vector_sum_of_squares(View);
#else
			return Dot(*this);
#endif
		}
		inline float Length() const { 
#ifdef USE_STD_LINALG
			return std::linalg::vector_two_norm(View);
#else
			return std::sqrtf(LengthSq());
#endif
		}
		inline Vec Normalize() const {
#ifdef USE_STD_LINALG
			return Vec(std::linalg::scale(1.0f / std::linalg::vector_two_norm(View), View));
#else
			const auto Sq = LengthSq();
			if (Sq > (std::numeric_limits<float>::epsilon)()) {
				return *this / std::sqrtf(Sq);
			}
#ifdef _DEBUG
			//__debugbreak();
#endif
			return Vec(*this);
#endif
		}

		inline Vec& operator=(const Vec& rhs) {
#ifdef USE_STD_LINALG
			std::linalg::copy(rhs.View, View);
#else
			std::ranges::copy(rhs.Data, std::begin(Data));
#endif
			return *this;
		}
		inline const Vec& operator+=(const Vec& rhs) { 
#ifdef USE_STD_LINALG
			std::linalg::add(View, rhs.View, View);
#else
#ifdef USE_OWN_IMPL
			for (auto i = 0; i < N; ++i) {
				Data[i] += rhs.Data[i];
			}
#else
			std::ranges::transform(Data, rhs.Data, std::begin(Data), std::plus());
#endif
#endif
			return *this;
		}
		inline const Vec& operator-=(const Vec& rhs) {
#ifdef USE_OWN_IMPL
			for (auto i = 0; i < N; ++i) {
				Data[i] -= rhs.Data[i];
			}
#else
			std::ranges::transform(Data, rhs.Data, std::begin(Data), std::divides());
#endif
			return *this;
		}
		inline const Vec& operator*=(const float rhs) { 
#ifdef USE_STD_LINALG
			std::linalg::scale(rhs, View);
#else
#ifdef USE_OWN_IMPL
			for (auto i = 0; i < N; ++i) {
				Data[i] *= rhs;
			}
#else
			std::ranges::transform(Data, std::begin(Data), std::bind(std::multiplies(), std::placeholders::_1, rhs));
#endif
#endif
			return *this; 
		}
		inline const Vec& operator/=(const float rhs) {
#ifdef USE_STD_LINALG
			std::linalg::scale(1.0f / rhs, View);
#else
#ifdef USE_OWN_IMPL
			for (auto i = 0; i < N; ++i) {
				Data[i] /= rhs;
			}
#else
			std::ranges::transform(Data, std::begin(Data), std::bind(std::divides(), std::placeholders::_1, rhs));
#endif
#endif
			return *this; 
		}
		inline float& operator[](const int i) { return Data[i]; }
		inline operator float* () { return std::data(Data); }

		inline Vec& ToZero() { return (*this = Zero()); }
		inline Vec& ToNormalized() { return (*this = Normalize()); }
		inline Vec& Adjust(const float Length) { return (*this = Normalize() * Length); }

		inline size_t Size() const { return std::size(Data); }

	private:
		std::array<float, N> Data;
		std::mdspan<float, std::extents<std::size_t, N>> View{ std::data(Data) };
	};
}
