#pragma once

#include <limits>
#include <numeric>
#include <array>
#include <algorithm>
#include <functional>
#include <format>

namespace Math 
{
	class Vec2
	{
	public:
		Vec2() {}
		Vec2(const float x, const float y) : Comps({x, y}) {}
		Vec2(const float rhs) : Comps({rhs, rhs}) {}

		inline static Vec2 Zero() { return Vec2(0.0f); }
		inline static Vec2 One() { return Vec2(1.0f); }
		inline static Vec2 AxisX() { return { 1.0f, 0.0f }; }
		inline static Vec2 AxisY() { return { 0.0f, 1.0f }; }
		inline static Vec2 Epsilon() { return  Vec2((std::numeric_limits<float>::epsilon)()); }
		inline static Vec2 Min() { return Vec2((std::numeric_limits<float>::min)()); }
		inline static Vec2 Max() { return Vec2((std::numeric_limits<float>::max)()); }
		inline static bool NearlyEqual(const Vec2& lhs, const Vec2& rhs, const float Epsilon) {
			return std::ranges::equal(lhs.Comps, rhs.Comps, [&](const float l, const float r) { return std::abs(l - r) < Epsilon; });
		}

		inline bool NearlyEqual(const Vec2& rhs, const float Epsilon = (std::numeric_limits<float>::epsilon)()) const { return NearlyEqual(*this, rhs, Epsilon); }

		inline bool operator==(const Vec2& rhs) const { 
			return std::ranges::equal(Comps, rhs.Comps);
		}
		inline bool operator!=(const Vec2& rhs) const { return !(*this == rhs); }
		inline Vec2 operator+(const Vec2& rhs) const {
			Vec2 r; std::ranges::transform(Comps, rhs.Comps, std::begin(r.Comps), std::plus()); return r;
		}
		inline Vec2 operator-(const Vec2& rhs) const {
			Vec2 r; std::ranges::transform(Comps, rhs.Comps, std::begin(r.Comps), std::minus()); return r;
		}
		inline Vec2 operator*(const float rhs) const { 
			Vec2 r; std::ranges::transform(Comps, std::begin(r.Comps), std::bind(std::multiplies(), std::placeholders::_1, rhs)); return r;
		}
		inline Vec2 operator/(const float rhs) const { 
			Vec2 r; std::ranges::transform(Comps, std::begin(r.Comps), std::bind(std::divides(), std::placeholders::_1, rhs)); return r;
		}
		inline Vec2 operator-() const {
			Vec2 r; std::ranges::transform(Comps, std::begin(r.Comps), std::negate()); return r;
		}
		
		inline float X() const { return Comps[0]; }
		inline float Y() const { return Comps[1]; }
		inline float operator[](const int i) const { return Comps[i]; }
		inline operator const float* () const { return std::data(Comps); }

		inline float Dot(const Vec2& rhs) const { 
			return std::inner_product(std::cbegin(Comps), std::cend(Comps), std::cbegin(rhs.Comps), 0.0f);
		}
		inline float LengthSq() const { return Dot(*this); }
		inline float Length() const { return sqrtf(LengthSq()); }
		inline Vec2 Normalize([[maybe_unused]] const bool AssertZero = false) const {
			const auto Sq = LengthSq();
			if (Sq > std::numeric_limits<float>::epsilon()) {
				return *this / sqrt(Sq);
			}
#ifdef _DEBUG
			if (AssertZero) { __debugbreak(); }
#endif
			return *this;
		}

		inline Vec2& operator=(const Vec2& rhs) {
			std::ranges::copy(rhs.Comps, std::begin(Comps));
			return *this;
		}
		inline const Vec2& operator+=(const Vec2& rhs) {
			std::ranges::transform(Comps, rhs.Comps, std::begin(Comps), std::plus());
			return *this;
		}
		inline const Vec2& operator-=(const Vec2& rhs) { 
			std::ranges::transform(Comps, rhs.Comps, std::begin(Comps), std::minus());
			return *this;
		}
		inline const Vec2& operator*=(const float rhs) { 
			std::ranges::transform(Comps, std::begin(Comps), std::bind(std::multiplies(), std::placeholders::_1, rhs));
			return *this;
		}
		inline const Vec2& operator/=(const float rhs) { 
			std::ranges::transform(Comps, std::begin(Comps), std::bind(std::divides(), std::placeholders::_1, rhs));
			return *this;
		}
		inline float& operator[](const int i) { return Comps[i]; }
		inline operator float* () { return std::data(Comps); }
		inline operator const Component2& () const { return Comps; }
		inline operator Component2& () { return Comps; }

		inline Vec2& ToZero() { return (*this = Zero()); }
		inline Vec2& ToNormalize() { return (*this = Normalize()); }
		inline Vec2& Adjust(const float Length) { return (*this = Normalize() * Length); }

		inline std::string ToString() const { return std::format("({:1.4f}, {:1.4f})\n", X(), Y()); }

	private:
		Component2 Comps = { 0.0f, 0.0f };
	};

	class Vec3
	{
	public:
		Vec3() {}
		Vec3(const float x, const float y, const float z) : Comps({x, y, z}) {}
		Vec3(const float rhs) : Comps({rhs, rhs, rhs}) {}
		Vec3(const Vec2& rhs, const float z = 0.0f) : Comps({ rhs.X(), rhs.Y(), z }) {}

		inline static Vec3 Zero() { return Vec3(0.0f); }
		inline static Vec3 One() { return Vec3(1.0f); }
		inline static Vec3 AxisX() { return { 1.0f, 0.0f, 0.0f }; }
		inline static Vec3 AxisY() { return { 0.0f, 1.0f, 0.0f }; }
		inline static Vec3 AxisZ() { return { 0.0f, 0.0f, 1.0f }; }
		inline static Vec3 Epsilon() { return Vec3((std::numeric_limits<float>::epsilon)()); }
		inline static Vec3 Min() { return Vec3((std::numeric_limits<float>::min)()); }
		inline static Vec3 Max() { return Vec3((std::numeric_limits<float>::max)()); }
		inline static bool NearlyEqual(const Vec3& lhs, const Vec3& rhs, const float Epsilon) {
			return std::ranges::equal(lhs.Comps, rhs.Comps, [&](const float l, const float r) { return std::abs(l - r) < Epsilon; });
		}
		inline static Vec3 Normal(const Vec3& A, const Vec3& B, const Vec3& C) { return (B - A).Cross(C - A); }
		inline static Vec3 UnitNormal(const Vec3& A, const Vec3& B, const Vec3& C) { return Normal(A, B, C).Normalize(); }
		
		inline bool NearlyEqual(const Vec3& rhs, const float Epsilon = (std::numeric_limits<float>::epsilon)()) const { return NearlyEqual(*this, rhs, Epsilon); }

		inline bool operator==(const Vec3& rhs) const {
			return std::ranges::equal(Comps, rhs.Comps);
		}
		inline bool operator!=(const Vec3& rhs) const { return !(*this == rhs); }
		inline Vec3 operator+(const Vec3& rhs) const { 
			Vec3 r; std::ranges::transform(Comps, rhs.Comps, std::begin(r.Comps), std::plus()); return r;
		}
		inline Vec3 operator-(const Vec3& rhs) const {
			Vec3 r; std::ranges::transform(Comps, rhs.Comps, std::begin(r.Comps), std::minus()); return r;
		}
		inline Vec3 operator*(const float rhs) const {
			Vec3 r; std::ranges::transform(Comps, std::begin(r.Comps), std::bind(std::multiplies(), std::placeholders::_1, rhs)); return r;
		}
		inline Vec3 operator/(const float rhs) const { 
			Vec3 r; std::ranges::transform(Comps, std::begin(r.Comps), std::bind(std::divides(), std::placeholders::_1, rhs)); return r;
		}
		inline Vec3 operator-() const { 
			Vec3 r; std::ranges::transform(Comps, std::begin(r.Comps), std::negate()); return r;
		}

		inline float X() const { return Comps[0]; }
		inline float Y() const { return Comps[1]; }
		inline float Z() const { return Comps[2]; }
		inline float operator[](const int i) const { return Comps[i]; }
		inline operator const float* () const { return std::data(Comps); }

		inline float Dot(const Vec3& rhs) const { 
			return std::inner_product(std::cbegin(Comps), std::cend(Comps), std::cbegin(rhs.Comps), 0.0f);
		}
		inline float LengthSq() const { return Dot(*this); }
		inline float Length() const { return sqrt(LengthSq()); }
		inline Vec3 Normalize([[maybe_unused]] const bool AssertZero = false) const {
			const auto Sq = LengthSq();
			if (Sq > std::numeric_limits<float>::epsilon()) { 
				return *this / sqrt(Sq); 
			}
#ifdef _DEBUG
			if (AssertZero) { __debugbreak(); }
#endif
			return *this;
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
			Comps[0] = rhs.X(); Comps[1] = rhs.Y(); 
			return *this; 
		}
		inline Vec3& operator=(const Vec3& rhs) {
			std::ranges::copy(rhs.Comps, std::begin(Comps));
			return *this; 
		}
		inline const Vec3& operator+=(const Vec3& rhs) { 
			std::ranges::transform(Comps, rhs.Comps, std::begin(Comps), std::plus());
			return *this; 
		}
		inline const Vec3& operator-=(const Vec3& rhs) {
			std::ranges::transform(Comps, rhs.Comps, std::begin(Comps), std::minus());
			return *this; 
		}
		inline const Vec3& operator*=(const float rhs) { 
			std::ranges::transform(Comps, std::begin(Comps), std::bind(std::multiplies(), std::placeholders::_1, rhs));
			return *this;
		}
		inline const Vec3& operator/=(const float rhs) {
			std::ranges::transform(Comps, std::begin(Comps), std::bind(std::divides(), std::placeholders::_1, rhs));
			return *this;
		}
		inline float& operator[](const int i) { return Comps[i]; }
		inline operator float* () { return std::data(Comps); }
		inline operator const Component3& () const { return Comps; }
		inline operator Component3& () { return Comps; }

		inline Vec3& ToZero() { return (*this = Zero()); }
		inline Vec3& ToNormalized() { return (*this = Normalize()); }
		inline Vec3& Adjust(const float Length) { return (*this = Normalize() * Length); }

		inline std::string ToString() const { return std::format("({:1.4f}, {:1.4f}, {:1.4f})\n", X(), Y(), Z()); }
	
	private:
		Component3 Comps = { 0.0f, 0.0f, 0.0f };
	};

	class Vec4
	{
	public:
		Vec4() {}
		Vec4(const float x, const float y, const float z, const float w) : Comps({x, y, z, w}) {}
		Vec4(const float rhs) : Comps({ rhs, rhs, rhs, rhs }) {}
		//Vec4(const Vec3& rhs) : Comps({ rhs.X(), rhs.Y(), rhs.Z(), 0.0f }) {}
		Vec4(const Vec3& rhs, const float w = 0.0f) : Comps({ rhs.X(), rhs.Y(), rhs.Z(), w }) {}

		inline static Vec4 Zero() { return Vec4(0.0f); }
		inline static Vec4 One() { return Vec4(1.0f); }
		inline static Vec4 AxisX() { return { 1.0f, 0.0f, 0.0f, 0.0f }; }
		inline static Vec4 AxisY() { return { 0.0f, 1.0f, 0.0f, 0.0f }; }
		inline static Vec4 AxisZ() { return { 0.0f, 0.0f, 1.0f, 0.0f }; }
		inline static Vec4 AxisW() { return { 0.0f, 0.0f, 0.0f, 1.0f }; }
		inline static Vec4 Epsilon() { return Vec4((std::numeric_limits<float>::epsilon)()); }
		inline static Vec4 Min() { return Vec4((std::numeric_limits<float>::min)()); }
		inline static Vec4 Max() { return Vec4((std::numeric_limits<float>::max)()); }
		inline static bool NearlyEqual(const Vec4& lhs, const Vec4& rhs, const float Epsilon) {
			return std::ranges::equal(lhs.Comps, rhs.Comps, [&](const float l, const float r) { return std::abs(l - r) < Epsilon; });
		}

		inline bool NearlyEqual(const Vec4& rhs, const float Epsilon = (std::numeric_limits<float>::epsilon)()) const { return NearlyEqual(*this, rhs, Epsilon); }

		inline bool operator==(const Vec4& rhs) const {
			return std::ranges::equal(Comps, rhs.Comps);
		}
		inline bool operator!=(const Vec4& rhs) const { return !(*this == rhs); }
		inline Vec4 operator+(const Vec4& rhs) const { 
			Vec4 r; std::ranges::transform(Comps, rhs.Comps, std::begin(r.Comps), std::plus()); return r;
		}
		inline Vec4 operator-(const Vec4& rhs) const {
			Vec4 r; std::ranges::transform(Comps, rhs.Comps, std::begin(r.Comps), std::minus()); return r;
		}
		inline Vec4 operator*(const float rhs) const {
			Vec4 r; std::ranges::transform(Comps, std::begin(r.Comps), std::bind(std::multiplies(), std::placeholders::_1, rhs)); return r;
		}
		inline Vec4 operator/(const float rhs) const {
			Vec4 r; std::ranges::transform(Comps, std::begin(r.Comps), std::bind(std::divides(), std::placeholders::_1, rhs)); return r;
		}
		inline Vec4 operator-() const { 
			Vec4 r; std::ranges::transform(Comps, std::begin(r.Comps), std::negate()); return r;
		}

		inline float X() const { return Comps[0]; }
		inline float Y() const { return Comps[1]; }
		inline float Z() const { return Comps[2]; }
		inline float W() const { return Comps[3]; }
		inline float operator[](const int i) const { return Comps[i]; }
		inline operator const float* () const { return std::data(Comps); }

		inline float Dot(const Vec4& rhs) const { 
			return std::inner_product(std::cbegin(Comps), std::cend(Comps), std::cbegin(rhs.Comps), 0.0f);
		}
		inline float LengthSq() const { return Dot(*this); }
		inline float Length() const { return sqrtf(LengthSq()); }
		inline Vec4 Normalize([[maybe_unused]] const bool AssertZero = false) const {
			const auto Sq = LengthSq();
			if (Sq > std::numeric_limits<float>::epsilon()) {
				return *this / sqrt(Sq);
			}
#ifdef _DEBUG
			if (AssertZero) { __debugbreak(); }
#endif
			return *this;
		}
		inline Vec4& operator=(const Vec2& rhs) { 
			Comps[0] = rhs.X(); Comps[1] = rhs.Y(); 
			return *this;
		}
		inline Vec4& operator=(const Vec3& rhs) { 
			Comps[0] = rhs.X(); Comps[1] = rhs.Y(); Comps[2] = rhs.Z(); 
			return *this; 
		}
		inline Vec4& operator=(const Vec4& rhs) {
			std::ranges::copy(rhs.Comps, std::begin(Comps));
			return *this;
		}
		inline const Vec4& operator+=(const Vec4& rhs) { 
			std::ranges::transform(Comps, rhs.Comps, std::begin(Comps), std::plus());
			return *this;
		}
		inline const Vec4& operator-=(const Vec4& rhs) { 
			std::ranges::transform(Comps, rhs.Comps, std::begin(Comps), std::minus());
			return *this;
		}
		inline const Vec4& operator*=(const float rhs) {
			std::ranges::transform(Comps, std::begin(Comps), std::bind(std::multiplies(), std::placeholders::_1, rhs));
			return *this; 
		}
		inline const Vec4& operator/=(const float rhs) { 
			std::ranges::transform(Comps, std::begin(Comps), std::bind(std::divides(), std::placeholders::_1, rhs));
			return *this; 
		}
		inline float& operator[](const int i) { return Comps[i]; }
		inline operator float* () { return std::data(Comps); }
		inline operator const Component4& () const { return Comps; }
		inline operator Component4& () { return Comps; }

		inline Vec4& ToZero() { return (*this = Zero()); }
		inline Vec4& ToNormalized() { return (*this = Normalize()); }
		inline Vec4& Adjust(const float Length) { return (*this = Normalize() * Length); }

		inline std::string ToString() const { return std::format("({:1.4f}, {:1.4f}, {:1.4f}, {:1.4f})\n", X(), Y(), Z(), W()); }

	private:
		Component4 Comps = { 0.0f, 0.0f, 0.0f, 0.0f };
	};

	template<size_t N>
	class Vec
	{
	public:
		//!< ここでは「疎」を扱う事が多いので、Vec<N> ではデフォルトコンストラクタでゼロクリアとしておく
		Vec() { std::ranges::fill(Comps, 0.0f); }
		Vec(const float rhs) { std::ranges::fill(Comps, rhs); }
		template<typename...A> Vec(A...Args) {
			auto i = 0;
			for (const auto f : std::initializer_list<float>{ Args... }) {
#ifdef _DEBUG
				if (i >= N) { break; }
#endif
				Comps[i++] = f;
			}
		}

		inline static Vec Zero() { return Vec(); }
		inline static Vec One() { return Vec(1.0f); }
		inline static Vec Epsilon() { return Vec((std::numeric_limits<float>::epsilon)()); }
		inline static Vec Min() { return Vec((std::numeric_limits<float>::min)()); }
		inline static Vec Max() { return Vec((std::numeric_limits<float>::max)()); }
		inline static bool NearlyEqual(const Vec lhs, const Vec& rhs, const float Epsilon) {
			return std::ranges::equal(lhs.Comps, rhs.Comps, [&](const float l, const float r) { return std::abs(l - r) < Epsilon; });
		}

		inline bool NearlyEqual(const Vec& rhs, const float Epsilon = (std::numeric_limits<float>::epsilon)()) const { return NearlyEqual(*this, rhs, Epsilon); }

		inline bool operator==(const Vec& rhs) const { return std::ranges::equal(Comps, rhs.Comps); }
		inline bool operator!=(const Vec& rhs) const { return !(*this == rhs); }
		inline Vec operator+(const Vec& rhs) const { 
			Vec r; std::ranges::transform(Comps, rhs.Comps, std::begin(r.Comps), std::plus()); return r;
		}
		inline Vec operator-(const Vec& rhs) const {
			Vec r; std::ranges::transform(Comps, rhs.Comps, std::begin(r.Comps), std::minus()); return r;
		}
		inline Vec operator*(const float rhs) const {
			Vec r; std::ranges::transform(Comps, rhs, std::begin(r.Comps), std::multiplies()); return r;
		}
		inline Vec operator/(const float rhs) const {
			Vec r; std::ranges::transform(Comps, rhs, std::begin(r.Comps), std::divides()); return r;
		}
		inline Vec operator-() const { 
			Vec r; std::ranges::transform(Comps, std::begin(r.Comps), std::negate()); return r;
		}

		inline float operator[](const int i) const { return Comps[i]; }
		inline operator const float* () const { return std::data(Comps); }

		inline float Dot(const Vec& rhs) const { 
			return std::inner_product(std::cbegin(Comps), std::cend(Comps), std::cbegin(rhs.Comps), 0.0f);
		}
		inline float LengthSq() const { return Dot(*this); }
		inline float Length() const { return sqrtf(LengthSq()); }
		inline Vec Normalize() const {
			const auto Sq = LengthSq();
			if (Sq > std::numeric_limits<float>::epsilon()) {
				return *this / sqrt(Sq);
			}
			return *this;
		}

		inline Vec& operator=(const Vec& rhs) {
			std::ranges::copy(rhs.Comps, std::begin(Comps));
			return *this;
		}
		inline const Vec& operator+=(const Vec& rhs) { 
			std::ranges::transform(Comps, rhs.Comps, std::begin(Comps), std::plus());
			return *this;
		}
		inline const Vec& operator-=(const Vec& rhs) {
			std::ranges::transform(Comps, rhs.Comps, std::begin(Comps), std::divides());
			return *this;
		}
		inline const Vec& operator*=(const float rhs) { 
			std::ranges::transform(Comps, std::begin(Comps), std::bind(std::multiplies(), std::placeholders::_1, rhs));
			return *this; 
		}
		inline const Vec& operator/=(const float rhs) {
			std::ranges::transform(Comps, std::begin(Comps), std::bind(std::divides(), std::placeholders::_1, rhs));
			return *this; 
		}
		inline float& operator[](const int i) { return Comps[i]; }
		inline operator float* () { return std::data(Comps); }

		inline Vec& ToZero() { return (*this = Zero()); }
		inline Vec& ToNormalized() { return (*this = Normalize()); }
		inline Vec& Adjust(const float Length) { return (*this = Normalize() * Length); }

		inline size_t Size() const { return std::size(Comps); }

	private:
		std::array<float, N> Comps;
	};
}
