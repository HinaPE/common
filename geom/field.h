#ifndef HINAPE_FIELD_H
#define HINAPE_FIELD_H

// Copyright (c) 2023 Xayah Hina
// MPL-2.0 license

#include "math/vector.h"

namespace HinaPE::Geom
{
class Field3 {};
class ScalarField3 : public Field3
{
public:
	virtual auto sample(const mVector3 &x) const -> real = 0;
	virtual auto sampler() const -> std::function<real(const mVector3 &)> = 0;
	virtual auto gradient(const mVector3 &x) const -> mVector3 = 0;
	virtual auto laplacian(const mVector3 &x) const -> real = 0;
};
class ConstantScalarField3 : public ScalarField3
{
public:
	auto sample(const mVector3 &x) const -> real final { return _value; }
	auto gradient(const mVector3 &x) const -> mVector3 final { return mVector3::Zero(); }
	auto laplacian(const mVector3 &x) const -> real final { return Constant::Zero; }
	auto sampler() const -> std::function<real(const mVector3 &)> final { return [this](const mVector3 &) { return _value; }; }
	explicit ConstantScalarField3(real value) : _value(value) {}

private:
	real _value = Constant::Zero;
};
class CustomScalarField3 : public ScalarField3
{
public:
	auto sample(const mVector3 &x) const -> real final { return _custom_function(x); }
	auto gradient(const mVector3 &x) const -> mVector3 final { return _custom_gradient_function(x); }
	auto laplacian(const mVector3 &x) const -> real final { return _custom_laplacian_function(x); }
	auto sampler() const -> std::function<real(const mVector3 &)> final { return _custom_function; }

private:
	std::function<real(const mVector3 &)> _custom_function;
	std::function<mVector3(const mVector3 &)> _custom_gradient_function;
	std::function<real(const mVector3 &)> _custom_laplacian_function;
	real _resolution = 1e-3;
};
class VectorField3 : public Field3
{
public:
	virtual auto sample(const mVector3 &x) const -> mVector3 = 0;
	virtual auto sampler() const -> std::function<mVector3(const mVector3 &)> = 0;
	virtual auto divergence(const mVector3 &x) const -> real = 0;
	virtual auto curl(const mVector3 &x) const -> mVector3 = 0;
};
class ConstantVectorField3 : public VectorField3
{
public:
	auto sample(const mVector3 &x) const -> mVector3 final { return _value; }
	auto divergence(const mVector3 &x) const -> real final { return Constant::Zero; }
	auto curl(const mVector3 &x) const -> mVector3 final { return mVector3::Zero(); }
	auto sampler() const -> std::function<mVector3(const mVector3 &)> final { return [this](const mVector3 &) -> mVector3 { return _value; }; }
	explicit ConstantVectorField3(mVector3 value) : _value(std::move(value)) {}

private:
	mVector3 _value = mVector3::Zero();
};
class CustomVectorField3 : public VectorField3
{
public:
	auto sample(const mVector3 &x) const -> mVector3 final { return _custom_function(x); }
	auto sampler() const -> std::function<mVector3(const mVector3 &)> final { return _custom_function; }
	auto divergence(const mVector3 &x) const -> real final { return _custom_divergence_function(x); }
	auto curl(const mVector3 &x) const -> mVector3 final { return _custom_curl_function(x); }

private:
	std::function<mVector3(const mVector3 &)> _custom_function;
	std::function<real(const mVector3 &)> _custom_divergence_function;
	std::function<mVector3(const mVector3 &)> _custom_curl_function;
	real _resolution = 1e-3;
};

//@formatter:off
using Field3Ptr               = std::shared_ptr<Field3>;
using ScalarField3Ptr         = std::shared_ptr<ScalarField3>;
using VectorField3Ptr         = std::shared_ptr<VectorField3>;
using ConstantScalarField3Ptr = std::shared_ptr<ConstantScalarField3>;
using ConstantVectorField3Ptr = std::shared_ptr<ConstantVectorField3>;
using CustomVectorField3Ptr   = std::shared_ptr<CustomVectorField3>;
//@formatter:on
}
#endif //HINAPE_FIELD_H
