#ifndef HINAPE_TRANSFORM3_H_
#define HINAPE_TRANSFORM3_H_

#include "bounding_box3.h"
#include "ray3.h"

#include "math/quaternion.h"
#include "math/vector3.h"

namespace HinaPE
{

//!
//! \brief Represents 3-D rigid body transform.
//!
class Transform3
{
public:
    //! Constructs identity transform.
    Transform3();

    //! Constructs a transform with translation and orientation.
    Transform3(const Vector3D &translation, const QuaternionD &orientation);

    //! Returns the translation.
    const Vector3D &translation() const;

    //! Sets the traslation.
    void setTranslation(const Vector3D &translation);

    //! Returns the orientation.
    const QuaternionD &orientation() const;

    //! Sets the orientation.
    void setOrientation(const QuaternionD &orientation);

    //! Transforms a point in world coordinate to the local frame.
    Vector3D toLocal(const Vector3D &pointInWorld) const;

    //! Transforms a direction in world coordinate to the local frame.
    Vector3D toLocalDirection(const Vector3D &dirInWorld) const;

    //! Transforms a ray in world coordinate to the local frame.
    Ray3D toLocal(const Ray3D &rayInWorld) const;

    //! Transforms a bounding box in world coordinate to the local frame.
    BoundingBox3D toLocal(const BoundingBox3D &bboxInWorld) const;

    //! Transforms a point in local space to the world coordinate.
    Vector3D toWorld(const Vector3D &pointInLocal) const;

    //! Transforms a direction in local space to the world coordinate.
    Vector3D toWorldDirection(const Vector3D &dirInLocal) const;

    //! Transforms a ray in local space to the world coordinate.
    Ray3D toWorld(const Ray3D &rayInLocal) const;

    //! Transforms a bounding box in local space to the world coordinate.
    BoundingBox3D toWorld(const BoundingBox3D &bboxInLocal) const;

private:
    Vector3D _translation;
    QuaternionD _orientation;
    Matrix3x3D _orientationMat3;
    Matrix3x3D _inverseOrientationMat3;
};

}  // namespace HinaPE

#include "transform3-inl.h"

#endif  // HINAPE_TRANSFORM3_H_
