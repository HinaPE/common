#ifndef HINAPE_ANIMATION_H
#define HINAPE_ANIMATION_H

// Copyright (c) 2023 Xayah Hina
// MPL-2.0 license

#include <chrono>
#include <memory>
#include <cassert>

namespace HinaPE::Animation
{
struct Frame final
{
public:
	inline auto time() const -> float { return static_cast<float>(_index) * _time_step; }
	inline void advance() { ++_index; }
	inline void advance(int delta) { _index += delta; }

public:
	int _index = 0;
	float _time_step = 1.0f / 60.0f;
};
using FramePtr = std::shared_ptr<Frame>;

class Animation
{
public:
	virtual void update(const Frame &frame) final
	{
		on_update(frame);
	}
	virtual void VALID_CHECK() {}

protected:
	virtual void on_update(const Frame &frame) = 0;
};
using AnimationPtr = std::shared_ptr<Animation>;
}

#endif //HINAPE_ANIMATION_H
