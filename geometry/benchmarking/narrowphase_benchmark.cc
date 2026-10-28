#include <cmath>
#include <random>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <benchmark/benchmark.h>
#include <fmt/format.h>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity_engine.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Matrix3Xd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using math::RigidTransformd;
using math::RotationMatrixd;

enum Method { kFcl, kCoalDefault, kCoalPolyak, kCoalNesterov };
enum Query { kDistance, kCollision, kPenetration };

constexpr int kNumPoses = 16;
constexpr double kAgreementTolerance = 1e-6;

const char* MethodName(Method m) {
  switch (m) {
    case kFcl:
      return "fcl";
    case kCoalDefault:
      return "coal_default";
    case kCoalPolyak:
      return "coal_polyak";
    case kCoalNesterov:
      return "coal_nesterov";
  }
  return "";
}

const char* QueryName(Query q) {
  switch (q) {
    case kDistance:
      return "distance";
    case kCollision:
      return "collision";
    case kPenetration:
      return "penetration";
  }
  return "";
}

CoalGjkOptions OptionsFor(Method m) {
  CoalGjkOptions options;
  if (m == kFcl) options.enabled = false;
  if (m == kCoalPolyak) options.variant = CoalGjkVariant::kPolyak;
  if (m == kCoalNesterov) options.variant = CoalGjkVariant::kNesterov;
  return options;
}

int NumVertices(int n_lat) {
  return 2 + 2 * n_lat * n_lat;
}

Matrix3Xd UvSpherePoints(int n_lat) {
  const int n_lon = 2 * n_lat;
  Matrix3Xd p(3, NumVertices(n_lat));
  p.col(0) = Vector3d(0, 0, 1);
  p.col(1) = Vector3d(0, 0, -1);
  int k = 2;
  for (int i = 1; i <= n_lat; ++i) {
    const double theta = M_PI * i / (n_lat + 1);
    for (int j = 0; j < n_lon; ++j) {
      const double phi = 2 * M_PI * j / n_lon;
      p.col(k++) = Vector3d(std::sin(theta) * std::cos(phi),
                            std::sin(theta) * std::sin(phi), std::cos(theta));
    }
  }
  return p;
}

using Poses = std::unordered_map<GeometryId, RigidTransformd>;

struct Scene {
  std::vector<ProximityEngine<double>> engines;
  std::vector<Poses> poses;
};

Scene MakeScene(int n_lat, Query query, const CoalGjkOptions& options) {
  const Convex hull(UvSpherePoints(n_lat), "uv_sphere");
  const double center_distance = (query == kPenetration) ? 1.5 : 2.5;
  std::mt19937 gen(0);
  std::normal_distribution<double> normal;
  auto random_rotation = [&]() {
    return RotationMatrixd(
        Quaterniond(normal(gen), normal(gen), normal(gen), normal(gen))
            .normalized());
  };
  Scene scene;
  for (int i = 0; i < kNumPoses; ++i) {
    const GeometryId id_A = GeometryId::get_new_id();
    const GeometryId id_B = GeometryId::get_new_id();
    const Vector3d dir =
        Vector3d(normal(gen), normal(gen), normal(gen)).normalized();
    Poses X_WGs{{id_A, RigidTransformd(random_rotation(), Vector3d::Zero())},
                {id_B, RigidTransformd(random_rotation(),
                                       center_distance * dir)}};
    ProximityEngine<double> engine;
    engine.set_coal_gjk_options(options);
    engine.AddDynamicGeometry(hull, X_WGs.at(id_A), id_A);
    engine.AddDynamicGeometry(hull, X_WGs.at(id_B), id_B);
    engine.UpdateWorldPoses(X_WGs);
    scene.engines.push_back(std::move(engine));
    scene.poses.push_back(std::move(X_WGs));
  }
  return scene;
}

double Evaluate(const ProximityEngine<double>& engine, const Poses& X_WGs,
                Query query) {
  switch (query) {
    case kDistance:
      return engine.ComputeSignedDistancePairwiseClosestPoints(X_WGs, 10.0)
          .at(0)
          .distance;
    case kCollision:
      return engine.HasCollisions() ? 1.0 : 0.0;
    case kPenetration: {
      const auto pairs = engine.ComputePointPairPenetration(X_WGs);
      return pairs.empty() ? 0.0 : pairs[0].depth;
    }
  }
  return 0.0;
}

Matrix3Xd MinkowskiPoints(int n_lat, const Poses& X_WGs) {
  const Matrix3Xd p = UvSpherePoints(n_lat);
  auto it = X_WGs.begin();
  const RigidTransformd X_A = (it++)->second;
  const RigidTransformd X_B = it->second;
  Matrix3Xd diff(3, p.cols() * p.cols());
  int k = 0;
  for (int i = 0; i < p.cols(); ++i) {
    for (int j = 0; j < p.cols(); ++j) {
      diff.col(k++) = X_A * Vector3d(p.col(i)) - X_B * Vector3d(p.col(j));
    }
  }
  return diff;
}

double ExactDepth(int n_lat, const Poses& X_WGs) {
  const Convex minkowski(MinkowskiPoints(n_lat, X_WGs), "minkowski");
  const auto& hull = minkowski.GetConvexHull();
  double depth = std::numeric_limits<double>::infinity();
  for (int f = 0; f < hull.num_faces(); ++f) {
    const Vector3d& v = hull.vertex(hull.element(f).vertex(0));
    depth = std::min(depth, hull.face_normal(f).dot(v));
  }
  return depth;
}

double OriginToSegment(const Vector3d& a, const Vector3d& b) {
  const Vector3d ab = b - a;
  const double t = std::clamp(-a.dot(ab) / ab.squaredNorm(), 0.0, 1.0);
  return (a + t * ab).norm();
}

double ExactDistance(int n_lat, const Poses& X_WGs) {
  const Convex minkowski(MinkowskiPoints(n_lat, X_WGs), "minkowski");
  const auto& hull = minkowski.GetConvexHull();
  double distance = std::numeric_limits<double>::infinity();
  for (int f = 0; f < hull.num_faces(); ++f) {
    const auto& face = hull.element(f);
    const Vector3d& n = hull.face_normal(f);
    const int m = face.num_vertices();
    bool inside = true;
    for (int i = 0; i < m; ++i) {
      const Vector3d& a = hull.vertex(face.vertex(i));
      const Vector3d& b = hull.vertex(face.vertex((i + 1) % m));
      distance = std::min(distance, OriginToSegment(a, b));
      if ((b - a).cross(-a).dot(n) < 0) inside = false;
    }
    if (inside) {
      distance = std::min(
          distance, std::abs(n.dot(hull.vertex(face.vertex(0)))));
    }
  }
  return distance;
}

double CachedExact(Query query, int n_lat, int pose, const Poses& X_WGs) {
  static std::map<std::tuple<int, int, int>, double> cache;
  const auto key = std::make_tuple(static_cast<int>(query), n_lat, pose);
  auto it = cache.find(key);
  if (it == cache.end()) {
    const double exact = (query == kPenetration) ? ExactDepth(n_lat, X_WGs)
                                                 : ExactDistance(n_lat, X_WGs);
    it = cache.emplace(key, exact).first;
  }
  return it->second;
}

void BM_Narrowphase(benchmark::State& state) {
  const auto method = static_cast<Method>(state.range(0));
  const auto query = static_cast<Query>(state.range(1));
  const int n_lat = static_cast<int>(state.range(2));

  const Scene scene = MakeScene(n_lat, query, OptionsFor(method));
  double max_error = std::numeric_limits<double>::quiet_NaN();
  if (query == kPenetration || query == kDistance) {
    max_error = 0.0;
    for (int i = 0; i < kNumPoses; ++i) {
      const double value = Evaluate(scene.engines[i], scene.poses[i], query);
      max_error = std::max(max_error,
          std::abs(value - CachedExact(query, n_lat, i, scene.poses[i])));
    }
  } else {
    const Scene reference = MakeScene(n_lat, query, OptionsFor(kFcl));
    for (int i = 0; i < kNumPoses; ++i) {
      const double value = Evaluate(scene.engines[i], scene.poses[i], query);
      const double expected =
          Evaluate(reference.engines[i], reference.poses[i], query);
      if (std::abs(value - expected) > kAgreementTolerance) {
        state.SkipWithError(fmt::format("pose {}: got {:.17g}, FCL {:.17g}",
                                        i, value, expected));
        return;
      }
    }
  }

  for (auto _ : state) {
    for (int i = 0; i < kNumPoses; ++i) {
      benchmark::DoNotOptimize(
          Evaluate(scene.engines[i], scene.poses[i], query));
    }
  }

  state.SetItemsProcessed(state.iterations() * kNumPoses);
  state.counters["vertices"] = NumVertices(n_lat);
  state.counters["max_error"] = max_error;
  state.SetLabel(std::string(MethodName(method)) + "/" + QueryName(query));
}

BENCHMARK(BM_Narrowphase)
    ->ArgsProduct({{kFcl, kCoalDefault, kCoalPolyak, kCoalNesterov},
                   {kDistance, kCollision, kPenetration},
                   {2, 4, 8, 16, 32}})
    ->Unit(benchmark::kMicrosecond);

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake

BENCHMARK_MAIN();