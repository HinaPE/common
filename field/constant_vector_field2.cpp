#include "constant_vector_field2.h"

#include "pch.h"

using namespace HinaPE;

ConstantVectorField2::ConstantVectorField2(const Vector2D &value) : _value(value)
{
}

Vector2D ConstantVectorField2::sample(const Vector2D &x) const
{
    UNUSED_VARIABLE(x);

    return _value;
}

std::function<Vector2D(const Vector2D &)> ConstantVectorField2::sampler() const
{
    return [this](const Vector2D &) -> Vector2D
    {
        return _value;
    };
}

ConstantVectorField2::Builder ConstantVectorField2::builder()
{
    return Builder();
}

ConstantVectorField2::Builder &ConstantVectorField2::Builder::withValue(const Vector2D &value)
{
    _value = value;
    return *this;
}

ConstantVectorField2 ConstantVectorField2::Builder::build() const
{
    return ConstantVectorField2(_value);
}

ConstantVectorField2Ptr ConstantVectorField2::Builder::makeShared() const
{
    return std::shared_ptr<ConstantVectorField2>(new ConstantVectorField2(_value), [](ConstantVectorField2 *obj)
    {
        delete obj;
    });
}
