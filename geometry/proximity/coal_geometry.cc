#include "drake/geometry/proximity/coal_geometry.h"

#include <vector>

#include <coal/shape/convex.h>

namespace drake {
namespace geometry {
namespace internal {


std::shared_ptr<const coal::CollisionGeometry> MakeCoalBox(
    const Eigen::Vector3d& size) {
  // coal::Box loses precision under EPA; an 8-vertex coal::Convex does not.
  const Eigen::Vector3d half = size / 2;
  auto vertices = std::make_shared<std::vector<coal::Vec3s>>();
  for (double x : {-1, 1}) {
    for (double y : {-1, 1}) {
      for (double z : {-1, 1}) {
        vertices->push_back(half.cwiseProduct(Eigen::Vector3d{x, y, z}));
      }
    }
  }
  auto triangles = std::make_shared<std::vector<coal::Triangle>>(
      std::vector<coal::Triangle>{{0, 4, 5}, {0, 5, 1}, {5, 7, 3}, {5, 3, 1},
                                  {3, 7, 6}, {3, 6, 2}, {0, 2, 6}, {0, 6, 4},
                                  {4, 6, 7}, {4, 7, 5}, {1, 3, 2}, {1, 2, 0}});
  return std::make_shared<coal::Convex<coal::Triangle>>(vertices, 8, triangles,
                                                        12);
}

std::pair<const coal::CollisionGeometry*, const coal::CollisionGeometry*>
FindCoalShapes(const CoalShapeMap* shapes, const fcl::CollisionObjectd& a,
               const fcl::CollisionObjectd& b) {
  if (shapes == nullptr) return {nullptr, nullptr};
  const auto it_a = shapes->find(a.collisionGeometry().get());
  const auto it_b = shapes->find(b.collisionGeometry().get());
  if (it_a == shapes->end() || it_b == shapes->end()) return {nullptr, nullptr};
  return {it_a->second.get(), it_b->second.get()};
}

coal::Transform3s ToCoalTransform(const fcl::Transform3d& X) {
  return coal::Transform3s(Eigen::Matrix3d(X.linear()),
                           Eigen::Vector3d(X.translation()));
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake