#pragma once

#include <memory>
#include <unordered_map>

#include <coal/collision_data.h>
#include <coal/collision_object.h>
#include <coal/shape/convex.h>
#include <fcl/fcl.h>

#include "drake/common/drake_export.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace internal DRAKE_NO_EXPORT {

/* Maps an fcl convex geometry to the equivalent Coal geometry, built at
 registration time and sharing the same vertex array. Absent entries simply
 mean the pair falls back to fcl. */
using CoalShapeMap =
    std::unordered_map<const fcl::CollisionGeometryd*,
                       std::shared_ptr<const coal::CollisionGeometry>>;

struct CoalSettings {
  const CoalShapeMap* shapes{nullptr};
  coal::GJKVariant gjk_variant{coal::GJKVariant::DefaultGJK};
  coal::GJKConvergenceCriterion gjk_convergence_criterion{
      coal::GJKConvergenceCriterion::Default};
};

std::shared_ptr<const coal::CollisionGeometry> MakeCoalBox(
    const Eigen::Vector3d& size);

}  // namespace internal
}  // namespace geometry
}  // namespace drake