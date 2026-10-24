#pragma once

#include <memory>
#include <unordered_map>
#include <utility>

#include <coal/collision_data.h>
#include <coal/collision_object.h>
#include <fcl/fcl.h>

#include "drake/common/drake_export.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace internal DRAKE_NO_EXPORT {

/* Maps an fcl shape to its Coal twin, built when the geometry is registered.
 A pair with either shape missing from the map is computed by fcl. */
using CoalShapeMap =
    std::unordered_map<const fcl::CollisionGeometryd*,
                       std::shared_ptr<const coal::CollisionGeometry>>;

/* What a proximity callback needs to compute a pair with Coal. A null `shapes`
 (the default) sends every pair to fcl. */
struct CoalSettings {
  const CoalShapeMap* shapes{nullptr};
  coal::GJKVariant gjk_variant{coal::GJKVariant::DefaultGJK};
  coal::GJKConvergenceCriterion gjk_convergence_criterion{
      coal::GJKConvergenceCriterion::Default};
};

/* Builds a box with full side lengths `size` as an 8-vertex coal::Convex.
 coal::Box is avoided: its distance and penetration errors are ~1e-11, versus
 ~1e-15 for the same box as a Convex. */
std::shared_ptr<const coal::CollisionGeometry> MakeCoalBox(
    const Eigen::Vector3d& size);

/* Returns the Coal twins of the geometries of `a` and `b`, or
 {nullptr, nullptr} if either twin is missing. */
std::pair<const coal::CollisionGeometry*, const coal::CollisionGeometry*>
FindCoalShapes(const CoalShapeMap* shapes, const fcl::CollisionObjectd& a,
               const fcl::CollisionObjectd& b);

/* Converts an fcl pose to a Coal pose. */
coal::Transform3s ToCoalTransform(const fcl::Transform3d& X);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
