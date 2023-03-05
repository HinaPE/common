#ifndef HINAPE_VECTOR_H
#define HINAPE_VECTOR_H

// Copyright (c) 2023 Xayah Hina
// MPL-2.0 license

#ifdef HINA_EIGEN
#include "Eigen/Eigen"
#endif
#include <type_traits>

#include "math_ext.h"

namespace HinaPE::Math
{
// ============================== Vector4 ==============================
template<typename T>
class Vector3 final
{
public:
	auto x() -> T &;
	auto y() -> T &;
	auto z() -> T &;
	auto x() const -> const T &;
	auto y() const -> const T &;
	auto z() const -> const T &;
	auto at(size_t i) -> T &;
	auto at(size_t i) const -> const T &;
	auto sum() const -> T;
	auto avg() const -> T;
	auto min() const -> T;
	auto max() const -> T;
	auto dot(const Vector3 &v) const -> T;
	auto cross(const Vector3 &v) const -> Vector3;
	auto length() const -> T;
	auto length_squared() -> T;
	void normalize();
	auto normalized() const -> Vector3;
	auto tangential() const -> std::tuple<Vector3, Vector3>;
	auto reciprocal() const -> Vector3;
//	auto data() const -> const T *;
	auto data() -> T *;

public:
	static inline constexpr auto Zero() -> Vector3 { return Vector3(0, 0, 0); }
	static inline constexpr auto One() -> Vector3 { return Vector3(1, 1, 1); }
	static inline constexpr auto UnitX() -> Vector3 { return Vector3(1, 0, 0); }
	static inline constexpr auto UnitY() -> Vector3 { return Vector3(0, 1, 0); }
	static inline constexpr auto UnitZ() -> Vector3 { return Vector3(0, 0, 1); }

public:
	template<typename U>
	auto operator=(const std::initializer_list<U> &lst) -> Vector3 &;
	auto operator=(const Vector3 &v_) -> Vector3 &;
	auto operator[](size_t i) -> T &;
	auto operator[](size_t i) const -> const T &;
	auto operator+=(T v_) -> Vector3 &;
	auto operator+=(const Vector3 &v_) -> Vector3 &;
	auto operator-=(T v_) -> Vector3 &;
	auto operator-=(const Vector3 &v_) -> Vector3 &;
	auto operator*=(T v_) -> Vector3 &;
	auto operator*=(const Vector3 &v_) -> Vector3 &;
	auto operator/=(T v_) -> Vector3 &;
	auto operator/=(const Vector3 &v_) -> Vector3 &;
	auto operator==(const Vector3 &v_) const -> bool;
	auto operator!=(const Vector3 &v_) const -> bool;

public:
#ifdef HINA_EIGEN
	explicit Vector3(Eigen::Matrix<T, 3, 1, Eigen::DontAlign> v_);
	template<typename U>
	Vector3(const std::initializer_list<U> &lst);
	Vector3();
	explicit Vector3(T s);
	Vector3(T x_, T y_, T z_);
	Vector3(const Vector3 &v);
	Vector3(Vector3 &&v) noexcept;
	~Vector3() = default;
	Eigen::Matrix<T, 3, 1, Eigen::DontAlign> _v;
#else
	T x, y, z;
#endif
};

//@formatter:off
template<typename T> auto operator+(const Vector3<T>& a) -> Vector3<T> { return a;}
template<typename T> auto operator-(const Vector3<T>& a) -> Vector3<T> { return {-a.x(), -a.y(), -a.z()}; }
template<typename T> auto operator+(const Vector3<T>& a, T b) -> Vector3<T> { return { a.x() + b, a.y() + b, a.z() + b}; }
template<typename T> auto operator+(T a, const Vector3<T>& b) -> Vector3<T> { return { a + b.x(), a + b.y(), a + b.z()}; }
template<typename T> auto operator+(const Vector3<T>& a, const Vector3<T>& b) -> Vector3<T> { return { a.x() + b.x(), a.y() + b.y(), a.z() + b.z()}; }
template<typename T> auto operator-(const Vector3<T>& a, T b) -> Vector3<T> { return { a.x() - b, a.y() - b, a.z() - b}; }
template<typename T> auto operator-(T a, const Vector3<T>& b) -> Vector3<T> { return { a - b.x(), a - b.y(), a - b.z()}; }
template<typename T> auto operator-(const Vector3<T>& a, const Vector3<T>& b) -> Vector3<T> { return { a.x() - b.x(), a.y() - b.y(), a.z() - b.z()}; }
template<typename T> auto operator*(const Vector3<T>& a, T b) -> Vector3<T> { return { a.x() * b, a.y() * b, a.z() * b}; }
template<typename T> auto operator*(T a, const Vector3<T>& b) -> Vector3<T> { return { a * b.x(), a * b.y(), a * b.z()}; }
template<typename T> auto operator*(const Vector3<T>& a, const Vector3<T>& b) -> Vector3<T> { return { a.x() * b.x(), a.y() * b.y(), a.z() * b.z()}; }
template<typename T> auto operator/(const Vector3<T>& a, T b) -> Vector3<T> { return { a.x() / b, a.y() / b, a.z() / b}; }
template<typename T> auto operator/(T a, const Vector3<T>& b) -> Vector3<T> { return { a / b.x(), a / b.y(), a / b.z()}; }
template<typename T> auto operator/(const Vector3<T>& a, const Vector3<T>& b) -> Vector3<T> { return { a.x() / b.x(), a.y() / b.y(), a.z() / b.z()}; }
template<typename T> auto max(const Vector3<T> &a, const Vector3<T> &b) -> Vector3<T> { return { std::max(a.x(), b.x()), std::max(a.y(), b.y()), std::max(a.z(), b.z()) };}
template<typename T> auto min(const Vector3<T> &a, const Vector3<T> &b) -> Vector3<T>{ return { std::min(a.x(), b.x()), std::min(a.y(), b.y()), std::min(a.z(), b.z()) };}
template<typename T> auto clamp(const Vector3<T> &v, const Vector3<T> &low, const Vector3<T> &high) -> Vector3<T> { return { clamp(v.x(), low.x(), high.x()), clamp(v.y(), low.y(), high.y()), clamp(v.z(), low.z(), high.z()) }; }
template<typename T> auto ceil(const Vector3<T> &a) -> Vector3<T> { return { std::ceil(a.x()), std::ceil(a.y()), std::ceil(a.z()) }; }
template<typename T> auto floor(const Vector3<T> &a) -> Vector3<T> { return { std::floor(a.x()), std::floor(a.y()), std::floor(a.z()) }; }
//@formatter:on


// ============================== Vector2 ==============================
template<typename T>
class Vector2 final
{
public:
	auto x() -> T &;
	auto y() -> T &;
	auto x() const -> const T &;
	auto y() const -> const T &;
	auto at(size_t i) -> T &;
	auto at(size_t i) const -> const T &;
	auto sum() const -> T;
	auto avg() const -> T;
	auto min() const -> T;
	auto max() const -> T;
	auto dot(const Vector2 &v) const -> T;
	auto cross(const Vector2 &v) const -> Vector2;
	auto length() const -> T;
	auto length_squared() -> T;
	void normalize();
	auto normalized() const -> Vector2;

public:
	static inline constexpr auto Zero() -> Vector2 { return Vector2(0, 0); }

public:
	template<typename U>
	auto operator=(const std::initializer_list<U> &lst) -> Vector2 &;
	auto operator=(const Vector2 &v_) -> Vector2 &;
	auto operator[](size_t i) -> T &;
	auto operator[](size_t i) const -> const T &;
	auto operator+=(T v_) -> Vector2 &;
	auto operator+=(const Vector2 &v_) -> Vector2 &;
	auto operator-=(T v_) -> Vector2 &;
	auto operator-=(const Vector2 &v_) -> Vector2 &;
	auto operator*=(T v_) -> Vector2 &;
	auto operator*=(const Vector2 &v_) -> Vector2 &;
	auto operator/=(T v_) -> Vector2 &;
	auto operator/=(const Vector2 &v_) -> Vector2 &;
	auto operator==(const Vector2 &v_) const -> bool;
	auto operator!=(const Vector2 &v_) const -> bool;

public:
#ifdef HINA_EIGEN
	explicit Vector2(Eigen::Matrix<T, 2, 1, Eigen::DontAlign> v_);
	template<typename U>
	Vector2(const std::initializer_list<U> &lst);
	Vector2();
	explicit Vector2(T s);
	Vector2(T x_, T y_);
	Vector2(const Vector2 &v);
	Vector2(Vector2 &&v) noexcept;
	~Vector2() = default;
	Eigen::Matrix<T, 2, 1, Eigen::DontAlign> _v;
#else
	T x, y;
#endif
};

//@formatter:off
template<typename T> auto operator+(const Vector2<T>& a) -> Vector2<T> { return a;}
template<typename T> auto operator-(const Vector2<T>& a) -> Vector2<T> { return {-a.x(), -a.y()}; }
template<typename T> auto operator+(const Vector2<T>& a, T b) -> Vector2<T> { return { a.x() + b, a.y() + b}; }
template<typename T> auto operator+(T a, const Vector2<T>& b) -> Vector2<T> { return { a + b.x(), a + b.y()}; }
template<typename T> auto operator+(const Vector2<T>& a, const Vector2<T>& b) -> Vector2<T> { return { a.x() + b.x(), a.y() + b.y()}; }
template<typename T> auto operator-(const Vector2<T>& a, T b) -> Vector2<T> { return { a.x() - b, a.y() - b}; }
template<typename T> auto operator-(T a, const Vector2<T>& b) -> Vector2<T> { return { a - b.x(), a - b.y()}; }
template<typename T> auto operator-(const Vector2<T>& a, const Vector2<T>& b) -> Vector2<T> { return { a.x() - b.x(), a.y() - b.y()}; }
template<typename T> auto operator*(const Vector2<T>& a, T b) -> Vector2<T> { return { a.x() * b, a.y() * b}; }
template<typename T> auto operator*(T a, const Vector2<T>& b) -> Vector2<T> { return { a * b.x(), a * b.y()}; }
template<typename T> auto operator*(const Vector2<T>& a, const Vector2<T>& b) -> Vector2<T> { return { a.x() * b.x(), a.y() * b.y()}; }
template<typename T> auto operator/(const Vector2<T>& a, T b) -> Vector2<T> { return { a.x() / b, a.y() / b}; }
template<typename T> auto operator/(T a, const Vector2<T>& b) -> Vector2<T> { return { a / b.x(), a / b.y()}; }
template<typename T> auto operator/(const Vector2<T>& a, const Vector2<T>& b) -> Vector2<T> { return { a.x() / b.x(), a.y() / b.y()}; }
template<typename T> auto max(const Vector2<T> &a, const Vector2<T> &b) -> Vector2<T> { return { std::max(a.x(), b.x()), std::max(a.y(), b.y()) };}
template<typename T> auto min(const Vector2<T> &a, const Vector2<T> &b) -> Vector2<T>{ return { std::min(a.x(), b.x()), std::min(a.y(), b.y()) };}
template<typename T> auto clamp(const Vector2<T> &v, const Vector2<T> &low, const Vector2<T> &high) -> Vector2<T> { return { clamp(v.x(), low.x(), high.x()), clamp(v.y(), low.y(), high.y()) }; }
template<typename T> auto ceil(const Vector2<T> &a) -> Vector2<T> { return { std::ceil(a.x()), std::ceil(a.y()) }; }
template<typename T> auto floor(const Vector2<T> &a) -> Vector2<T> { return { std::floor(a.x()), std::floor(a.y()) }; }
//@formatter:on


// ============================== Vector4 ==============================
template<typename T>
class Vector4 final
{
public:
	auto x() -> T &;
	auto y() -> T &;
	auto z() -> T &;
	auto w() -> T &;
	auto x() const -> const T &;
	auto y() const -> const T &;
	auto z() const -> const T &;
	auto w() const -> const T &;
	auto at(size_t i) -> T &;
	auto at(size_t i) const -> const T &;
	auto sum() const -> T;
	auto avg() const -> T;
	auto min() const -> T;
	auto max() const -> T;
	auto dot(const Vector4 &v) const -> T;
	auto cross(const Vector4 &v) const -> Vector4;
	auto length() const -> T;
	auto length_squared() -> T;
	void normalize();
	auto normalized() const -> Vector4;
	auto xyz() const -> Vector3<T>;

public:
	static inline constexpr auto Zero() -> Vector4 { return Vector4(0, 0, 0, 0); }

public:
	template<typename U>
	auto operator=(const std::initializer_list<U> &lst) -> Vector4 &;
	auto operator=(const Vector4 &v_) -> Vector4 &;
	auto operator[](size_t i) -> T &;
	auto operator[](size_t i) const -> const T &;
	auto operator+=(T v_) -> Vector4 &;
	auto operator+=(const Vector4 &v_) -> Vector4 &;
	auto operator-=(T v_) -> Vector4 &;
	auto operator-=(const Vector4 &v_) -> Vector4 &;
	auto operator*=(T v_) -> Vector4 &;
	auto operator*=(const Vector4 &v_) -> Vector4 &;
	auto operator/=(T v_) -> Vector4 &;
	auto operator/=(const Vector4 &v_) -> Vector4 &;
	auto operator==(const Vector4 &v_) const -> bool;
	auto operator!=(const Vector4 &v_) const -> bool;

public:
#ifdef HINA_EIGEN
	constexpr explicit Vector4(Eigen::Matrix<T, 4, 1, Eigen::DontAlign> v_);
	template<typename U>
	Vector4(const std::initializer_list<U> &lst);
	constexpr Vector4();
	constexpr explicit Vector4(T s);
	constexpr Vector4(T x_, T y_, T z_, T w_);
	constexpr Vector4(const Vector4 &v);
	constexpr Vector4(Vector4 &&v) noexcept;
	~Vector4() = default;
	Eigen::Matrix<T, 4, 1, Eigen::DontAlign> _v;
#else
	T x, y, z;
#endif
};

//@formatter:off
template<typename T> auto operator+(const Vector4<T>& a) -> Vector4<T> { return a;}
template<typename T> auto operator-(const Vector4<T>& a) -> Vector4<T> { return {-a.x(), -a.y(), -a.z(), a.w()}; }
template<typename T> auto operator+(const Vector4<T>& a, T b) -> Vector4<T> { return { a.x() + b, a.y() + b, a.z() + b, a.w() + b}; }
template<typename T> auto operator+(T a, const Vector4<T>& b) -> Vector4<T> { return { a + b.x(), a + b.y(), a + b.z()}; }
template<typename T> auto operator+(const Vector4<T>& a, const Vector4<T>& b) -> Vector4<T> { return { a.x() + b.x(), a.y() + b.y(), a.z() + b.z(), a.w() + b.w()}; }
template<typename T> auto operator-(const Vector4<T>& a, T b) -> Vector4<T> { return { a.x() - b, a.y() - b, a.z() - b, a.w() - b}; }
template<typename T> auto operator-(T a, const Vector4<T>& b) -> Vector4<T> { return { a - b.x(), a - b.y(), a - b.z(), a - b.w()}; }
template<typename T> auto operator-(const Vector4<T>& a, const Vector4<T>& b) -> Vector4<T> { return { a.x() - b.x(), a.y() - b.y(), a.z() - b.z(), a.w() - b.w()}; }
template<typename T> auto operator*(const Vector4<T>& a, T b) -> Vector4<T> { return { a.x() * b, a.y() * b, a.z() * b, a.w() * b}; }
template<typename T> auto operator*(T a, const Vector4<T>& b) -> Vector4<T> { return { a * b.x(), a * b.y(), a * b.z(), a * b.w()}; }
template<typename T> auto operator*(const Vector4<T>& a, const Vector4<T>& b) -> Vector4<T> { return { a.x() * b.x(), a.y() * b.y(), a.z() * b.z(), a.w() * b.w()}; }
template<typename T> auto operator/(const Vector4<T>& a, T b) -> Vector4<T> { return { a.x() / b, a.y() / b, a.z() / b, a.w() / b}; }
template<typename T> auto operator/(T a, const Vector4<T>& b) -> Vector4<T> { return { a / b.x(), a / b.y(), a / b.z(), a / b.w()}; }
template<typename T> auto operator/(const Vector4<T>& a, const Vector4<T>& b) -> Vector4<T> { return { a.x() / b.x(), a.y() / b.y(), a.z() / b.z(), a.w() / b.w()}; }
template<typename T> auto max(const Vector4<T> &a, const Vector4<T> &b) -> Vector4<T> { return { std::max(a.x(), b.x()), std::max(a.y(), b.y()), std::max(a.z(), b.z()), std::max(a.w(), b.w()) };}
template<typename T> auto min(const Vector4<T> &a, const Vector4<T> &b) -> Vector4<T>{ return { std::min(a.x(), b.x()), std::min(a.y(), b.y()), std::min(a.z(), b.z()), std::min(a.w(), b.w()) };}
template<typename T> auto clamp(const Vector4<T> &v, const Vector4<T> &low, const Vector4<T> &high) -> Vector4<T> { return { clamp(v.x(), low.x(), high.x()), clamp(v.y(), low.y(), high.y()), clamp(v.z(), low.z(), high.z()), clamp(v.w(), low.w(), high.w()) }; }
template<typename T> auto ceil(const Vector4<T> &a) -> Vector4<T> { return { std::ceil(a.x()), std::ceil(a.y()), std::ceil(a.z()), std::ceil(a.w()) }; }
template<typename T> auto floor(const Vector4<T> &a) -> Vector4<T> { return { std::floor(a.x()), std::floor(a.y()), std::floor(a.z()), std::floor(a.w()) }; }
//@formatter:on

}
#ifdef HINA_EIGEN
#include "impl/vector_impl_eigen.h"
#else
#endif

#ifdef HINAPE_DOUBLE
using mVector3 = HinaPE::Math::Vector3<double>;
using mVector2 = HinaPE::Math::Vector2<double>;
using mVector4 = HinaPE::Math::Vector4<double>;
#else
using mVector3 = HinaPE::Math::Vector3<float>;
using mVector2 = HinaPE::Math::Vector2<float>;
using mVector4 = HinaPE::Math::Vector4<float>;
#endif
using mVector3i = HinaPE::Math::Vector3<int>;
using mVector3u = HinaPE::Math::Vector3<size_t>;
using mVector2i = HinaPE::Math::Vector2<int>;
using mVector2u = HinaPE::Math::Vector2<size_t>;
using mVector4i = HinaPE::Math::Vector4<int>;
using mVector4u = HinaPE::Math::Vector4<size_t>;

namespace HinaPE::Color
{
//@formatter:off
static mVector3 MIKU      = mVector3(0.2227f, 0.7698f, 0.7307f);
static mVector3 RED       = mVector3(1.0f   , 0.0f   , 0.0f);
static mVector3 GREEN     = mVector3(0.0f   , 1.0f   , 0.0f);
static mVector3 BLUE      = mVector3(0.0f   , 0.0f   , 1.0f);
static mVector3 YELLOW    = mVector3(1.0f   , 1.0f   , 0.0f);
static mVector3 CYAN      = mVector3(0.0f   , 1.0f   , 1.0f);
static mVector3 MAGENTA   = mVector3(1.0f   , 0.0f   , 1.0f);
static mVector3 WHITE     = mVector3(1.0f   , 1.0f   , 1.0f);
static mVector3 BLACK     = mVector3(0.0f   , 0.0f   , 0.0f);
static mVector3 GRAY      = mVector3(0.5f   , 0.5f   , 0.5f);
static mVector3 ORANGE    = mVector3(1.0f   , 0.5f   , 0.0f);
static mVector3 PURPLE    = mVector3(0.5f   , 0.0f   , 0.5f);
static mVector3 BROWN     = mVector3(0.5f   , 0.25f  , 0.0f);
static mVector3 PINK      = mVector3(1.0f   , 0.75f  , 0.8f);
static mVector3 NO_COLORS = mVector3(0.0f   , 0.0f   , 0.0f);
//@formatter:on
}

namespace HinaPE::Math
{
template<typename T>
inline auto uniform_sample_cone(T u1, T u2, const mVector3 &axis, T angle) -> mVector3
{
	T cos_angle_2 = std::cos(degrees_to_radians(angle) / 2);
	T y = 1 - (1 - cos_angle_2) * u1;
	T r = std::sqrt(std::max<T>(0, 1 - y * y));
	T phi = Constant::TwoPI * u2;
	T x = r * std::cos(phi);
	T z = r * std::sin(phi);
	auto ts = axis.tangential();
	return std::get<0>(ts) * x + axis * y + std::get<1>(ts) * z;
}
}
#endif //HINAPE_VECTOR_H
