#include "drake/geometry/proximity/collisions_exist_callback.h"

#include "drake/geometry/proximity/proximity_utilities.h"

#include <coal/collision.h>

namespace drake {
namespace geometry {
namespace internal {
namespace has_collisions {

CallbackData::CallbackData(const CollisionFilter* collision_filter_in,
                           const CoalSettings& coal_in)
    : collision_filter(*collision_filter_in), coal(coal_in) {
  DRAKE_DEMAND(collision_filter_in != nullptr);
  request.num_max_contacts = 1;
  request.enable_contact = false;
  // NOTE: As of 5/1/2018 the GJK implementation of Libccd appears to be
  // superior to FCL's "independent" implementation. Furthermore, libccd
  // appears to behave badly if its gjk tolerance is much tighter than
  // 2e-12. Until this changes, we explicitly specify these parameters rather
  // than relying on FCL's defaults.
  request.gjk_tolerance = 2e-12;
  request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
}

bool Callback(fcl::CollisionObjectd* object_A_ptr,
              fcl::CollisionObjectd* object_B_ptr, void* callback_data) {
  auto& data = *static_cast<CallbackData*>(callback_data);

  const EncodedData encoding_a(*object_A_ptr);
  const EncodedData encoding_b(*object_B_ptr);

  const bool can_collide =
      data.collision_filter.CanCollideWith(encoding_a.id(), encoding_b.id());
  if (!can_collide) return false;

  const auto [coal_A, coal_B] =
      FindCoalShapes(data.coal.shapes, *object_A_ptr, *object_B_ptr);
  if (coal_A != nullptr) {
    coal::CollisionRequest coal_request;
    coal_request.enable_contact = false;
    coal_request.num_max_contacts = 1;
    coal_request.gjk_tolerance = data.request.gjk_tolerance;
    coal_request.distance_upper_bound =
        coal_request.collision_distance_threshold;
    coal_request.gjk_variant = data.coal.gjk_variant;
    coal_request.gjk_convergence_criterion =
        data.coal.gjk_convergence_criterion;
    coal::CollisionResult coal_result;
    coal::collide(coal_A, ToCoalTransform(object_A_ptr->getTransform()), coal_B,
                  ToCoalTransform(object_B_ptr->getTransform()), coal_request,
                  coal_result);
    data.collisions_exist = coal_result.isCollision();
    return data.collisions_exist;
  }

  // Unpack the callback data.
  const fcl::CollisionRequestd& request = data.request;

  // This callback only works for a single contact, this confirms a request
  // hasn't been made for more contacts.
  DRAKE_ASSERT(request.num_max_contacts == 1);
  fcl::CollisionResultd result;

  // Perform nearphase collision detection.
  fcl::collide(object_A_ptr, object_B_ptr, request, result);

  data.collisions_exist = result.isCollision();
  return data.collisions_exist;
}

}  // namespace has_collisions
}  // namespace internal
}  // namespace geometry
}  // namespace drake
