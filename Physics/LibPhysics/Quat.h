#pragma once

#include <format>

namespace Math 
{
	class Quat 
	{
	public:
		Quat() {}
		Quat(const float x, const float y, const float z, const float w) : Comps({x, y, z, w}) {}
		Quat(const Vec3& rhs) : Comps({ rhs.X(), rhs.Y(), rhs.Z(), 0.0f }) {}
		Quat(const Vec3& Axis, const float Radian) {
			const auto HalfRadian = 0.5f * Radian;
			const auto Ax = Axis.Normalize() * sinf(HalfRadian);
			Comps = { Ax.X(),Ax.Y(),Ax.Z(), cosf(HalfRadian) };
		}

		inline static Quat Identity() { return { 0.0f, 0.0f, 0.0f, 1.0f }; }
		inline static Mat4 ToLeftMat4(const Quat& rhs) {
			return {
				{ rhs.W(), -rhs.Z(), rhs.Y(), rhs.X() },
				{ rhs.Z(), rhs.W(), -rhs.X(), rhs.Y() },
				{ -rhs.Y(), rhs.X(), rhs.W(), rhs.Z() },
				{ -rhs.X(), -rhs.Y(), -rhs.Z(), rhs.W() },
			};
		}
		inline static Mat4 ToRightMat4(const Quat& rhs) {
			return {
				{ rhs.W(), rhs.Z(), -rhs.Y(), rhs.X() },
				{ -rhs.Z(), rhs.W(), rhs.X(), rhs.Y() },
				{ rhs.Y(), -rhs.X(), rhs.W(), rhs.Z() },
				{ -rhs.X(), -rhs.Y(), -rhs.Z(), rhs.W() },
			};
		}

		inline bool NearlyEqual(const Quat& rhs, const float Epsilon = (std::numeric_limits<float>::epsilon)()) const {
			return std::ranges::equal(Comps, rhs.Comps,
				[&](const float l, const float r) { 
					return std::abs(l - r) < Epsilon; 
				});
		}

		inline bool operator==(const Quat& rhs) const { 
			return std::ranges::equal(Comps, rhs.Comps);
		}
		inline bool operator!=(const Quat& rhs) const { return !(*this == rhs); }
		inline Quat operator*(const float rhs) const {
			Quat r; std::ranges::transform(Comps, std::begin(r.Comps), std::bind(std::multiplies(), std::placeholders::_1, rhs)); return r;
		}
		inline Quat operator*(const Quat& rhs) const {
			return Quat(X() * rhs.W() + W() * rhs.X() + Y() * rhs.Z() - Z() * rhs.Y(),
				Y() * rhs.W() + W() * rhs.Y() + Z() * rhs.X() - X() * rhs.Z(),
				Z() * rhs.W() + W() * rhs.Z() + X() * rhs.Y() - Y() * rhs.X(),
				W() * rhs.W() - X() * rhs.X() - Y() * rhs.Y() - Z() * rhs.Z());
		}
		inline Quat operator/(const float rhs) const { 
			Quat r; std::ranges::transform(Comps, std::begin(r.Comps), std::bind(std::divides(), std::placeholders::_1, rhs)); return r;
		}

		inline float X() const { return Comps[0]; }
		inline float Y() const { return Comps[1]; }
		inline float Z() const { return Comps[2]; }
		inline float W() const { return Comps[3]; }
		inline Vec3 XYZ() const { return { X(), Y(), Z() }; }
		inline float operator[](const int i) const { return Comps[i]; }
		inline operator const float* () const { return std::data(Comps); }
		inline operator Vec3() const { return { X(), Y(), Z() }; }
		inline operator Mat3() const {
			return {
				Rotate(Vec3::AxisX()),
				Rotate(Vec3::AxisY()),
				Rotate(Vec3::AxisZ()),
			};
		}
		inline Vec3 ToVec3() const { return static_cast<Vec3>(*this); }
		inline Mat3 ToMat3() const { return static_cast<Mat3>(*this); }
	
		//!< A * B = L(A) * B = R(B) * A となるような 4x4 行列
		inline Mat4 ToLeftMat4() const { return ToLeftMat4(*this); }
		inline Mat4 ToRightMat4() const { return ToRightMat4(*this); }

		inline float Dot(const Quat& rhs) const {
			return std::inner_product(std::cbegin(Comps), std::cend(Comps), std::cbegin(rhs.Comps), 0.0f);
		}
		inline float LengthSq() const { return Dot(*this); }
		inline float Length() const { return std::sqrtf(LengthSq()); }
		inline Quat Normalize() const {
			const auto Sq = LengthSq();
			if (Sq > std::numeric_limits<float>::epsilon()) {
				return *this / std::sqrtf(Sq);
			}
			return *this;
		}
		inline Quat Conjugate() const { return Quat(-X(), -Y(), -Z(), W()); }
		inline Quat Inverse() const { return Conjugate() / LengthSq(); }
		inline Vec3 Rotate(const Vec3& rhs) const {
			return *this * Quat(rhs) * Inverse();
		}
		inline Mat3 Rotate(const Mat3& rhs) const {
			return {
				Rotate(rhs[0]),
				Rotate(rhs[1]),
				Rotate(rhs[2])
			};
		}

		inline Quat& operator=(const Quat& rhs) { 
			std::ranges::copy(rhs.Comps, std::begin(Comps));
			return *this;
		}
		inline const Quat& operator*=(const float rhs) {
			std::ranges::transform(Comps, std::begin(Comps), std::bind(std::multiplies(), std::placeholders::_1, rhs));
			return *this; 
		}
		inline const Quat& operator/=(const float rhs) {
			std::ranges::transform(Comps, std::begin(Comps), std::bind(std::divides(), std::placeholders::_1, rhs));
			return *this; 
		}
		inline float& operator[](const int i) { return Comps[i]; }
		inline operator float* () { return std::data(Comps); }
		inline operator const Component4& () const { return Comps; }
		inline operator Component4& () { return Comps; }

		inline Quat& ToIdentity() { return (*this = Identity()); }
		inline Quat& ToNormalized() { return (*this = Normalize()); }
		
		inline std::string ToString() const { return std::format("({:1.4f}, {:1.4f}, {:1.4f}, {:1.4f})\n", X(), Y(), Z(), W()); }

	private:
		Component4 Comps = { 0.0f, 0.0f, 0.0f, 1.0f };
	};
}



