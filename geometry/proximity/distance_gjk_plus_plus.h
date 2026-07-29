#pragma once

#include "fcl/narrowphase/collision_object.h"
#include "fcl/narrowphase/distance_request.h"
#include "drake/geometry/proximity/distance_to_shape_callback.h"
#include "drake/math/rigid_transform.h"
#include "drake/common/autodiff.h"

namespace drake {
namespace geometry {
namespace internal {

// The interception function that calculates distance using GJK++
template <typename T>
void ComputeGjkPlusPlus(
    const fcl::CollisionObjectd& a, const math::RigidTransform<T>& X_WA,
    const fcl::CollisionObjectd& b, const math::RigidTransform<T>& X_WB,
    const fcl::DistanceRequestd& request,
    SignedDistancePair<T>* result);

} // namespace internal
} // namespace geometry
} // namespace drake