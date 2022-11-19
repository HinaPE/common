#ifndef HINAPE_DETAIL_QUATERNION_INL_H_
#define HINAPE_DETAIL_QUATERNION_INL_H_

#include "quaternion.h"

#include <limits>

namespace HinaPE
{
// Constructors
template<typename T>
inline Quaternion<T>::Quaternion() { setIdentity(); }
template<typename T>
inline Quaternion<T>::Quaternion(T newW, T newX, T newY, T newZ) { set(newW, newX, newY, newZ); }
template<typename T>
Quaternion<T>::Quaternion(T roll, T pitch, T yaw) { set(roll, pitch, yaw); }
template<typename T>
inline Quaternion<T>::Quaternion(const std::initializer_list<T> &lst) { set(lst); }
template<typename T>
inline Quaternion<T>::Quaternion(const Vector3<T> &axis, T angle) { set(axis, angle); }
template<typename T>
inline Quaternion<T>::Quaternion(const Vector3<T> &from, const Vector3<T> &to) { set(from, to); }
template<typename T>
inline Quaternion<T>::Quaternion(const Vector3<T> &rotationBasis0, const Vector3<T> &rotationBasis1, const Vector3<T> &rotationBasis2) { set(rotationBasis0, rotationBasis1, rotationBasis2); }
template<typename T>
inline Quaternion<T>::Quaternion(const Matrix3x3<T> &matrix) { set(matrix); }
template<typename T>
inline Quaternion<T>::Quaternion(const Quaternion &other) { set(other); }

// Basic setters
template<typename T>
inline void Quaternion<T>::set(const Quaternion &other) { set(other.w, other.x, other.y, other.z); }

template<typename T>
inline void Quaternion<T>::set(T newW, T newX, T newY, T newZ)
{
    w = newW;
    x = newX;
    y = newY;
    z = newZ;
}

template<typename T>
void Quaternion<T>::set(T roll, T pitch, T yaw)
{
    // ref: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_angles_to_quaternion_conversion
    auto degree_roll = to_degrees(roll);
    auto degree_pitch = to_degrees(pitch);
    auto degree_yaw = to_degrees(yaw);

    double cr = std::cos(degree_roll * static_cast<T>(0.5));
    double sr = std::sin(degree_roll * static_cast<T>(0.5));
    double cp = std::cos(degree_pitch * static_cast<T>(0.5));
    double sp = std::sin(degree_pitch * static_cast<T>(0.5));
    double cy = std::cos(degree_yaw * static_cast<T>(0.5));
    double sy = std::sin(degree_yaw * static_cast<T>(0.5));

    w = cr * cp * cy + sr * sp * sy;
    x = sr * cp * cy - cr * sp * sy;
    y = cr * sp * cy + sr * cp * sy;
    z = cr * cp * sy - sr * sp * cy;
}

template<typename T>
inline void Quaternion<T>::set(const std::initializer_list<T> &lst)
{
    assert(lst.size() == 4);

    auto inputElem = lst.begin();
    w = *inputElem;
    x = *(++inputElem);
    y = *(++inputElem);
    z = *(++inputElem);
}

template<typename T>
inline void Quaternion<T>::set(const Vector3<T> &axis, T angle)
{
    static const T eps = std::numeric_limits<T>::epsilon();

    T axisLengthSquared = axis.lengthSquared();

    if (axisLengthSquared < eps)
    {
        setIdentity();
    } else
    {
        Vector3<T> normalizedAxis = axis.normalized();
        T s = std::sin(angle / 2);

        x = normalizedAxis.x * s;
        y = normalizedAxis.y * s;
        z = normalizedAxis.z * s;
        w = std::cos(angle / 2);
    }
}

template<typename T>
inline void Quaternion<T>::set(const Vector3<T> &from, const Vector3<T> &to)
{
    static const T eps = std::numeric_limits<T>::epsilon();

    Vector3<T> axis = from.cross(to);

    T fromLengthSquared = from.lengthSquared();
    T toLengthSquared = to.lengthSquared();

    if (fromLengthSquared < eps || toLengthSquared < eps)
    {
        setIdentity();
    } else
    {
        T axisLengthSquared = axis.lengthSquared();

        // In case two vectors are exactly the opposite, pick orthogonal vector
        // for axis.
        if (axisLengthSquared < eps)
        {
            axis = std::get<0>(from.tangential());
        }

        set(from.dot(to), axis.x, axis.y, axis.z);
        w += l2Norm();

        normalize();
    }
}

template<typename T>
inline void Quaternion<T>::set(const Vector3<T> &rotationBasis0, const Vector3<T> &rotationBasis1, const Vector3<T> &rotationBasis2)
{
    Matrix3x3<T> matrix3;

    matrix3.setColumn(0, rotationBasis0.normalized());
    matrix3.setColumn(1, rotationBasis1.normalized());
    matrix3.setColumn(2, rotationBasis2.normalized());

    set(matrix3);
}

template<typename T>
inline void Quaternion<T>::set(const Matrix3x3<T> &m)
{
    static const T eps = std::numeric_limits<T>::epsilon();
    static const T quater = static_cast<T>(0.25);

    T onePlusTrace = m.trace() + 1;

    if (onePlusTrace > eps)
    {
        T S = std::sqrt(onePlusTrace) * 2;
        w = quater * S;
        x = (m(2, 1) - m(1, 2)) / S;
        y = (m(0, 2) - m(2, 0)) / S;
        z = (m(1, 0) - m(0, 1)) / S;
    } else if (m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2))
    {
        T S = std::sqrt(1 + m(0, 0) - m(1, 1) - m(2, 2)) * 2;
        w = (m(2, 1) - m(1, 2)) / S;
        x = quater * S;
        y = (m(0, 1) + m(1, 0)) / S;
        z = (m(0, 2) + m(2, 0)) / S;
    } else if (m(1, 1) > m(2, 2))
    {
        T S = std::sqrt(1 + m(1, 1) - m(0, 0) - m(2, 2)) * 2;
        w = (m(0, 2) - m(2, 0)) / S;
        x = (m(0, 1) + m(1, 0)) / S;
        y = quater * S;
        z = (m(1, 2) + m(2, 1)) / S;
    } else
    {
        T S = std::sqrt(1 + m(2, 2) - m(0, 0) - m(1, 1)) * 2;
        w = (m(1, 0) - m(0, 1)) / S;
        x = (m(0, 2) + m(2, 0)) / S;
        y = (m(1, 2) + m(2, 1)) / S;
        z = quater * S;
    }
}

// Basic getters
template<typename T>
template<typename U>
auto Quaternion<T>::castTo() const -> Quaternion<U> { return Quaternion<U>(static_cast<U>(w), static_cast<U>(x), static_cast<U>(y), static_cast<U>(z)); }

//! Returns normalized quaternion.
template<typename T>
auto Quaternion<T>::normalized() const -> Quaternion<T>
{
    Quaternion q(*this);
    q.normalize();
    return q;
}

// Binary operator methods - new instance = this instance (+) input
template<typename T>
inline auto Quaternion<T>::mul(const Vector3<T> &v) const -> Vector3<T>
{
    T _2xx = 2 * x * x;
    T _2yy = 2 * y * y;
    T _2zz = 2 * z * z;
    T _2xy = 2 * x * y;
    T _2xz = 2 * x * z;
    T _2xw = 2 * x * w;
    T _2yz = 2 * y * z;
    T _2yw = 2 * y * w;
    T _2zw = 2 * z * w;

    return Vector3<T>((1 - _2yy - _2zz) * v.x + (_2xy - _2zw) * v.y + (_2xz + _2yw) * v.z, (_2xy + _2zw) * v.x + (1 - _2zz - _2xx) * v.y + (_2yz - _2xw) * v.z, (_2xz - _2yw) * v.x + (_2yz + _2xw) * v.y + (1 - _2yy - _2xx) * v.z);
}

template<typename T>
inline auto Quaternion<T>::mul(const Quaternion &other) const -> Quaternion<T>
{
    return Quaternion(w * other.w - x * other.x - y * other.y - z * other.z, w * other.x + x * other.w + y * other.z - z * other.y, w * other.y - x * other.z + y * other.w + z * other.x, w * other.z + x * other.y - y * other.x + z * other.w);
}

template<typename T>
inline auto Quaternion<T>::dot(const Quaternion<T> &other) -> T { return w * other.w + x * other.x + y * other.y + z * other.z; }

// Binary operator methods - new instance = input (+) this instance
template<typename T>
inline auto Quaternion<T>::rmul(const Quaternion &other) const -> Quaternion<T>
{
    return Quaternion(other.w * w - other.x * x - other.y * y - other.z * z, other.w * x + other.x * w + other.y * z - other.z * y, other.w * y - other.x * z + other.y * w + other.z * x, other.w * z + other.x * y - other.y * x + other.z * w);
}

// Augmented operator methods - this instance (+)= input
template<typename T>
inline void Quaternion<T>::imul(const Quaternion &other)
{
    *this = mul(other);
}

// Modifiers
template<typename T>
inline void Quaternion<T>::setIdentity()
{
    set(1, 0, 0, 0);
}

template<typename T>
inline void Quaternion<T>::rotate(T angleInRadians)
{
    Vector3<T> axis;
    T currentAngle;

    getAxisAngle(&axis, &currentAngle);

    currentAngle += angleInRadians;

    set(axis, currentAngle);
}

template<typename T>
inline void Quaternion<T>::normalize()
{
    T norm = l2Norm();

    if (norm > 0)
    {
        w /= norm;
        x /= norm;
        y /= norm;
        z /= norm;
    }
}

// Complex getters
template<typename T>
inline auto Quaternion<T>::axis() const -> Vector3<T>
{
    Vector3<T> result(x, y, z);
    result.normalize();

    if (2 * std::acos(w) < pi<T>())
    {
        return result;
    } else
    {
        return -result;
    }
}

template<typename T>
inline auto Quaternion<T>::angle() const -> T
{
    T result = 2 * std::acos(w);

    if (result < pi<T>())
    {
        return result;
    } else
    {
        // Wrap around
        return 2 * pi<T>() - result;
    }
}

template<typename T>
inline void Quaternion<T>::getAxisAngle(Vector3<T> *axis, T *angle) const
{
    axis->set(x, y, z);
    axis->normalize();
    *angle = 2 * std::acos(w);

    if (*angle > pi<T>())
    {
        // Wrap around
        (*axis) = -(*axis);
        *angle = 2 * pi<T>() - (*angle);
    }
}

template<typename T>
inline auto Quaternion<T>::inverse() const -> Quaternion<T>
{
    T denom = w * w + x * x + y * y + z * z;
    return Quaternion(w / denom, -x / denom, -y / denom, -z / denom);
}

template<typename T>
inline auto Quaternion<T>::matrix3() const -> Matrix3x3<T>
{
    T _2xx = 2 * x * x;
    T _2yy = 2 * y * y;
    T _2zz = 2 * z * z;
    T _2xy = 2 * x * y;
    T _2xz = 2 * x * z;
    T _2xw = 2 * x * w;
    T _2yz = 2 * y * z;
    T _2yw = 2 * y * w;
    T _2zw = 2 * z * w;

    Matrix3x3<T> m(1 - _2yy - _2zz, _2xy - _2zw, _2xz + _2yw, _2xy + _2zw, 1 - _2zz - _2xx, _2yz - _2xw, _2xz - _2yw, _2yz + _2xw, 1 - _2yy - _2xx);

    return m;
}

template<typename T>
inline auto Quaternion<T>::matrix4() const -> Matrix4x4<T>
{
    T _2xx = 2 * x * x;
    T _2yy = 2 * y * y;
    T _2zz = 2 * z * z;
    T _2xy = 2 * x * y;
    T _2xz = 2 * x * z;
    T _2xw = 2 * x * w;
    T _2yz = 2 * y * z;
    T _2yw = 2 * y * w;
    T _2zw = 2 * z * w;

    Matrix4x4<T> m(1 - _2yy - _2zz, _2xy - _2zw, _2xz + _2yw, 0, _2xy + _2zw, 1 - _2zz - _2xx, _2yz - _2xw, 0, _2xz - _2yw, _2yz + _2xw, 1 - _2yy - _2xx, 0, 0, 0, 0, 1);

    return m;
}

template<typename T>
inline auto Quaternion<T>::l2Norm() const -> T
{
    return std::sqrt(w * w + x * x + y * y + z * z);
}

template<typename T>
auto Quaternion<T>::euler() const -> Vector3<T>
{
    // ref: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_code_2
    T roll = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    T pitch = std::asin(2 * (w * y - z * x));
    T yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

    return Vector3<T>(to_degrees(roll), to_degrees(pitch), to_degrees(yaw));
}

// Setter operators
template<typename T>
inline auto Quaternion<T>::operator=(const Quaternion &other) -> Quaternion<T> &
{
    set(other);
    return *this;
}

template<typename T>
inline auto Quaternion<T>::operator*=(const Quaternion &other) -> Quaternion<T> &
{
    imul(other);
    return *this;
}

// Getter operators
template<typename T>
inline auto Quaternion<T>::operator[](size_t i) -> T &
{
    return (&w)[i];
}

template<typename T>
inline auto Quaternion<T>::operator[](size_t i) const -> const T &
{
    return (&w)[i];
}

template<typename T>
auto Quaternion<T>::operator==(const Quaternion &other) const -> bool
{
    return w == other.w && x == other.x && y == other.y && z == other.z;
}

template<typename T>
auto Quaternion<T>::operator!=(const Quaternion &other) const -> bool
{
    return w != other.w || x != other.x || y != other.y || z != other.z;
}

template<typename T>
auto Quaternion<T>::makeIdentity() -> Quaternion<T>
{
    return Quaternion();
}

template<typename T>
inline auto slerp(const Quaternion<T> &a, const Quaternion<T> &b, T t) -> Quaternion<T>
{
    static const double threshold = 0.01;
    static const T eps = std::numeric_limits<T>::epsilon();

    T cosHalfAngle = dot(a, b);
    T weightA, weightB;

    // For better accuracy, return lerp result when a and b are close enough.
    if (1.0 - std::fabs(cosHalfAngle) < threshold)
    {
        weightA = 1.0 - t;
        weightB = t;
    } else
    {
        T halfAngle = std::acos(cosHalfAngle);
        T sinHalfAngle = std::sqrt(1 - cosHalfAngle * cosHalfAngle);

        // In case of angle ~ 180, pick middle value.
        // If not, perform slerp.
        if (std::fabs(sinHalfAngle) < eps)
        {
            weightA = static_cast<T>(0.5);
            weightB = static_cast<T>(0.5);
        } else
        {
            weightA = std::sin((1 - t) * halfAngle) / sinHalfAngle;
            weightB = std::sin(t * halfAngle) / sinHalfAngle;
        }
    }

    return Quaternion<T>(weightA * a.w + weightB * b.w, weightA * a.x + weightB * b.x, weightA * a.y + weightB * b.y, weightA * a.z + weightB * b.z);
}

// Operator overloadings
template<typename T>
inline auto operator*(const Quaternion<T> &q, const Vector<T, 3> &v) -> Vector<T, 3>
{
    return q.mul(v);
}

template<typename T>
inline auto operator*(const Quaternion<T> &a, const Quaternion<T> &b) -> Quaternion<T>
{
    return a.mul(b);
}

}  // namespace HinaPE

#endif  // HINAPE_DETAIL_QUATERNION_INL_H_
