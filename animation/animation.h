#ifndef HINAPE_ANIMATION_H_
#define HINAPE_ANIMATION_H_

#include "macros.h"

#include <memory>

namespace HinaPE
{

//!
//! \brief Representation of an animation frame.
//!
//! This struct holds current animation frame index and frame interval in
//! seconds.
//!
struct Frame final
{
    //! Frame index.
    int index = 0;

    //! Time interval in seconds between two adjacent frames.
    double timeIntervalInSeconds = 1.0 / 60.0;

    //! Constructs Frame instance with 1/60 seconds time interval.
    Frame();

    //! Constructs Frame instance with given time interval.
    Frame(int newIndex, double newTimeIntervalInSeconds);

    //! Returns the elapsed time in seconds.
    auto timeInSeconds() const -> double;

    //! Advances single frame.
    void advance();

    //! Advances multiple frames.
    //! \param delta Number of frames to advance.
    void advance(int delta);

    //! Advances single frame (prefix).
    auto operator++() -> Frame &;

    //! Advances single frame (postfix).
    auto operator++(int) -> Frame;
};

//!
//! \brief Abstract base class for animation-related class.
//!
//! This class represents the animation logic in very abstract level.
//! Generally animation is a function of time and/or its previous state.
//! This base class provides a virtual function update() which can be
//! overriden by its sub-classes to implement their own state update logic.
//!
class Animation
{
public:
    Animation();

    virtual ~Animation();

    //!
    //! \brief Updates animation state for given \p frame.
    //!
    //! This function updates animation state by calling Animation::onUpdate
    //! function.
    //!
    void update(const Frame &frame);

protected:
    //!
    //! \brief The implementation of this function should update the animation
    //!     state for given Frame instance \p frame.
    //!
    //! This function is called from Animation::update when state of this class
    //! instance needs to be updated. Thus, the inherited class should overrride
    //! this function and implement its logic for updating the animation state.
    //!
    virtual void onUpdate(const Frame &frame) = 0;
};

//! Shared pointer for the Animation type.
using AnimationPtr = std::shared_ptr<Animation>;

}  // namespace HinaPE

#endif  // HINAPE_ANIMATION_H_
