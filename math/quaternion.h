#ifndef HINAPE_QUATERNION_H_
#define HINAPE_QUATERNION_H_

#include "matrix4x4.h"

namespace HinaPE
{

#define PI_F 3.14159265358979323846264338327950288f

template<typename T>
auto to_radians(T v) -> T
{
    return v * (PI_F / 180.0f);
}

template<typename T>
auto to_degrees(T v) -> T
{
    return v * (180.0f / PI_F);
}

//!
//! \brief Quaternion class defined as q = w + xi + yj + zk.
//!
template<typename T>
class Quaternion
{
public:
    static_assert(std::is_floating_point<T>::value, "Quaternion only can be instantiated with floating point types");

    //! Real part.
    T w;

    //!< Imaginary part (i).
    T x;

    //!< Imaginary part (j).
    T y;

    //!< Imaginary part (k).
    T z;

    // MARK: Constructors

    //! Make an identity quaternion.
    Quaternion();

    //! Constructs a quaternion with given elements.
    Quaternion(T newW, T newX, T newY, T newZ);

    //! Constructs a quaternion with given euler angles.
    Quaternion(T roll, T pitch, T yaw);

    //! Constructs a quaternion with given elements.
    Quaternion(const std::initializer_list<T> &lst);

    //! Constructs a quaternion with given rotation axis and angle.
    Quaternion(const Vector3<T> &axis, T angle);

    //! Constructs a quaternion with from and to vectors.
    Quaternion(const Vector3<T> &from, const Vector3<T> &to);

    //! Constructs a quaternion with three basis vectors.
    Quaternion(const Vector3<T> &rotationBasis0, const Vector3<T> &rotationBasis1, const Vector3<T> &rotationBasis2);

    //! Constructs a quaternion with 3x3 rotational matrix.
    explicit Quaternion(const Matrix3x3<T> &matrix);

    //! Copy constructor.
    Quaternion(const Quaternion &other);


    // MARK: Basic setters

    //! Sets the quaternion with other quaternion.
    void set(const Quaternion &other);

    //! Sets the quaternion with given elements.
    void set(T newW, T newX, T newY, T newZ);

    //! Sets the quaternion with given euler angles.
    void set(T roll, T pitch, T yaw);

    //! Sets the quaternion with given elements.
    void set(const std::initializer_list<T> &lst);

    //! Sets the quaternion with given rotation axis and angle.
    void set(const Vector3<T> &axis, T angle);

    //! Sets the quaternion with from and to vectors.
    void set(const Vector3<T> &from, const Vector3<T> &to);

    //! Sets quaternion with three basis vectors.
    void set(const Vector3<T> &rotationBasis0, const Vector3<T> &rotationBasis1, const Vector3<T> &rotationBasis2);

    //! Sets the quaternion with 3x3 rotational matrix.
    void set(const Matrix3x3<T> &matrix);

    // MARK: Basic getters

    //! Returns quaternion with other base type.
    template<typename U>
    auto castTo() const -> Quaternion<U>;

    //! Returns normalized quaternion.
    auto normalized() const -> Quaternion;


    // MARK: Binary operator methods - new instance = this instance (+) input

    //! Returns this quaternion * vector.
    auto mul(const Vector3<T> &v) const -> Vector3<T>;

    //! Returns this quaternion * other quaternion.
    auto mul(const Quaternion &other) const -> Quaternion;

    //! Computes the dot product with other quaternion.
    auto dot(const Quaternion<T> &other) -> T;


    // MARK: Binary operator methods - new instance = input (+) this instance

    //! Returns other quaternion * this quaternion.
    auto rmul(const Quaternion &other) const -> Quaternion;

    // MARK: Augmented operator methods - this instance (+)= input

    //! Returns this quaternion *= other quaternion.
    void imul(const Quaternion &other);


    // MARK: Modifiers

    //! Makes this quaternion identity.
    void setIdentity();

    //! Rotate this quaternion with given angle in radians.
    void rotate(T angleInRadians);

    //! Normalizes the quaternion.
    void normalize();


    // MARK: Complex getters

    //! Returns the rotational axis.
    auto axis() const -> Vector3<T>;

    //! Returns the rotational angle.
    auto angle() const -> T;

    //! Returns the axis and angle.
    void getAxisAngle(Vector3<T> *axis, T *angle) const;

    //! Returns the inverse quaternion.
    auto inverse() const -> Quaternion;

    //! Converts to the 3x3 rotation matrix.
    auto matrix3() const -> Matrix3x3<T>;

    //! Converts to the 4x4 rotation matrix.
    auto matrix4() const -> Matrix4x4<T>;

    //! Returns L2 norm of this quaternion.
    auto l2Norm() const -> T;

    auto euler() const -> Vector3<T>;


    // MARK: Setter operators

    //! Assigns other quaternion.
    auto operator=(const Quaternion &other) -> Quaternion &;

    //! Returns this quaternion *= other quaternion.
    auto operator*=(const Quaternion &other) -> Quaternion &;


    // MARK: Getter operators

    //! Returns the reference to the i-th element.
    auto operator[](size_t i) -> T &;

    //! Returns the const reference to the i-th element.
    auto operator[](size_t i) const -> const T &;

    //! Returns true if equal.
    auto operator==(const Quaternion &other) const -> bool;

    //! Returns true if not equal.
    auto operator!=(const Quaternion &other) const -> bool;


    // MARK: Builders

    //! Returns identity matrix.
    static auto makeIdentity() -> Quaternion;
};

//! Computes spherical linear interpolation.
template<typename T>
auto slerp(const Quaternion<T> &a, const Quaternion<T> &b, T t) -> Quaternion<T>;

//! Returns quaternion q * vector v.
template<typename T>
auto operator*(const Quaternion<T> &q, const Vector<T, 3> &v) -> Vector<T, 3>;

//! Returns quaternion a times quaternion b.
template<typename T>
auto operator*(const Quaternion<T> &a, const Quaternion<T> &b) -> Quaternion<T>;

//! Float-type quaternion.
using QuaternionF = Quaternion<float>;

//! Double-type quaternion.
using QuaternionD = Quaternion<double>;

}  // namespace HinaPE

#include "quaternion-inl.h"

#endif  // HINAPE_QUATERNION_H_
