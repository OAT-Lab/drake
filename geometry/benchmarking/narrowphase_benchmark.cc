#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <utility>
#include <vector>

#include <benchmark/benchmark.h>
#include <coal/shape/convex.h>
#include <fcl/fcl.h>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/coal_geometry.h"
#include "drake/geometry/proximity/distance_to_shape_callback.h"
#include "drake/geometry/proximity/polygon_to_triangle_mesh.h"
#include "drake/geometry/proximity/proximity_utilities.h"
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

enum Method { kFcl, kCoal, kCoalPolyak, kCoalNesterov };

constexpr int kNumPoses = 16;
constexpr std::array<double, 5> kGaps = {0.5, 1e-1, 1e-2, 1e-3, 1e-4};
constexpr int kNumGaps = static_cast<int>(kGaps.size());
constexpr double kAspect = 1.0;
constexpr double kStartDistance = 2.5;

constexpr double kTimedTolerance = 1e-6;
constexpr double kReferenceTolerance = 1e-12;
constexpr double kReferenceAgreement = 1e-10;
constexpr int kReferenceMaxIterations = 100000;

const char* MethodName(Method m) {
  switch (m) {
    case kFcl:
      return "fcl";
    case kCoal:
      return "coal_default";
    case kCoalPolyak:
      return "coal_polyak";
    case kCoalNesterov:
      return "coal_nesterov";
  }
  return "";
}

int NumVertices(int n_lat) {
  return 2 + 2 * n_lat * n_lat;
}

Matrix3Xd HullPoints(int n_lat) {
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
  p.row(1) *= kAspect;
  p.row(2) *= kAspect;
  return p;
}

struct Hull {
  std::shared_ptr<fcl::CollisionGeometryd> fcl_shape;
  CoalShapeMap coal_shapes;
};

Hull MakeHull(int n_lat) {
  const Convex convex(HullPoints(n_lat), "hull");
  const PolygonSurfaceMesh<double>& mesh = convex.GetConvexHull();

  auto vertices = std::make_shared<std::vector<Vector3d>>();
  for (int v = 0; v < mesh.num_vertices(); ++v) {
    vertices->push_back(mesh.vertex(v));
  }
  auto faces = std::make_shared<std::vector<int>>(mesh.face_data());

  const TriangleSurfaceMesh<double> triangulated =
      MakeTriangleFromPolygonMesh(mesh);
  auto triangles = std::make_shared<std::vector<coal::Triangle>>();
  for (int t = 0; t < triangulated.num_triangles(); ++t) {
    const SurfaceTriangle& tri = triangulated.element(t);
    triangles->emplace_back(
        static_cast<coal::Triangle::index_type>(tri.vertex(0)),
        static_cast<coal::Triangle::index_type>(tri.vertex(1)),
        static_cast<coal::Triangle::index_type>(tri.vertex(2)));
  }

  Hull hull;
  hull.fcl_shape = std::make_shared<fcl::Convexd>(
      vertices, mesh.num_elements(), faces);
  hull.coal_shapes[hull.fcl_shape.get()] =
      std::make_shared<coal::Convex<coal::Triangle>>(
          vertices, static_cast<unsigned int>(vertices->size()), triangles,
          static_cast<unsigned int>(triangles->size()));
  return hull;
}

std::unique_ptr<fcl::CollisionObjectd> PlaceShape(const Hull& hull,
                                                  const RigidTransformd& X_WG) {
  auto object = std::make_unique<fcl::CollisionObjectd>(hull.fcl_shape);
  object->setTransform(X_WG.GetAsIsometry3());
  object->computeAABB();
  EncodedData(GeometryId::get_new_id(), true).write_to(object.get());
  return object;
}

struct ShapePair {
  std::unique_ptr<fcl::CollisionObjectd> a;
  std::unique_ptr<fcl::CollisionObjectd> b;
  RigidTransformd X_WA;
  RigidTransformd X_WB;
};

ShapePair MakeShapePair(const Hull& hull, const RigidTransformd& X_WA,
                        const RigidTransformd& X_WB) {
  return {PlaceShape(hull, X_WA), PlaceShape(hull, X_WB), X_WA, X_WB};
}

CoalSettings SettingsFor(Method m, const Hull& hull) {
  CoalSettings settings;
  if (m == kFcl) return settings;
  settings.shapes = &hull.coal_shapes;
  if (m == kCoalPolyak) {
    settings.gjk_variant = coal::GJKVariant::PolyakAcceleration;
  }
  if (m == kCoalNesterov) {
    settings.gjk_variant = coal::GJKVariant::NesterovAcceleration;
  }
  return settings;
}

fcl::DistanceRequestd MakeRequest(double tolerance) {
  fcl::DistanceRequestd request;
  request.enable_nearest_points = true;
  request.enable_signed_distance = true;
  request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
  request.distance_tolerance = tolerance;
  return request;
}

SignedDistancePair<double> Narrowphase(const ShapePair& pair,
                                       const fcl::DistanceRequestd& request,
                                       const CoalSettings& settings) {
  SignedDistancePair<double> result;
  shape_distance::ComputeNarrowPhaseDistance<double>(
      *pair.a, pair.X_WA, *pair.b, pair.X_WB, request, &result, settings);
  return result;
}

struct Scene {
  Hull hull;
  std::vector<ShapePair> pairs;
  std::vector<double> reference;
};

Scene MakeScene(int n_lat, double gap) {
  Scene scene;
  scene.hull = MakeHull(n_lat);

  const fcl::DistanceRequestd accurate = MakeRequest(kReferenceTolerance);
  const CoalSettings fcl_accurate = SettingsFor(kFcl, scene.hull);
  CoalSettings coal_accurate = SettingsFor(kCoal, scene.hull);
  coal_accurate.gjk_max_iterations = kReferenceMaxIterations;

  std::mt19937 gen(0);
  std::normal_distribution<double> normal;
  auto random_rotation = [&]() {
    return RotationMatrixd(
        Quaterniond(normal(gen), normal(gen), normal(gen), normal(gen))
            .normalized());
  };

  for (int i = 0; i < kNumPoses; ++i) {
    const Vector3d direction =
        Vector3d(normal(gen), normal(gen), normal(gen)).normalized();
    const RigidTransformd X_WA(random_rotation(), Vector3d::Zero());
    const RotationMatrixd R_WB = random_rotation();

    const RigidTransformd X_WB_start(R_WB, kStartDistance * direction);
    const SignedDistancePair<double> start = Narrowphase(
        MakeShapePair(scene.hull, X_WA, X_WB_start), accurate, coal_accurate);
    const Vector3d p_WB = X_WB_start.translation() +
                          (start.distance - gap) * start.nhat_BA_W;
    ShapePair pair =
        MakeShapePair(scene.hull, X_WA, RigidTransformd(R_WB, p_WB));

    const double d_fcl = Narrowphase(pair, accurate, fcl_accurate).distance;
    const double d_coal = Narrowphase(pair, accurate, coal_accurate).distance;
    scene.reference.push_back(std::abs(d_fcl - d_coal) <= kReferenceAgreement
                                  ? 0.5 * (d_fcl + d_coal)
                                  : std::numeric_limits<double>::quiet_NaN());
    scene.pairs.push_back(std::move(pair));
  }
  return scene;
}

const Scene& CachedScene(int n_lat, int gap_index) {
  static std::map<std::pair<int, int>, Scene> cache;
  const auto key = std::make_pair(n_lat, gap_index);
  auto it = cache.find(key);
  if (it == cache.end()) {
    it = cache.emplace(key, MakeScene(n_lat, kGaps[gap_index])).first;
  }
  return it->second;
}

void BM_Narrowphase(benchmark::State& state) {
  const int pose = static_cast<int>(state.range(0));
  const int n_lat = static_cast<int>(state.range(1));
  const int gap_index = static_cast<int>(state.range(2));
  const auto method = static_cast<Method>(state.range(3));

  const Scene& scene = CachedScene(n_lat, gap_index);
  const ShapePair& pair = scene.pairs[pose];
  const fcl::DistanceRequestd request = MakeRequest(kTimedTolerance);
  const CoalSettings settings = SettingsFor(method, scene.hull);

  const double distance = Narrowphase(pair, request, settings).distance;
  const double gjk_iterations = static_cast<double>(LastGjkIterations());
  const double reference = scene.reference[pose];

  for (auto _ : state) {
    benchmark::DoNotOptimize(Narrowphase(pair, request, settings));
  }

  state.counters["vertices"] = NumVertices(n_lat);
  state.counters["pose"] = pose;
  state.counters["gap"] = kGaps[gap_index];
  state.counters["aspect"] = kAspect;
  state.counters["reference"] = reference;
  state.counters["gap_miss"] = std::abs(reference - kGaps[gap_index]);
  state.counters["error"] = std::abs(distance - reference);
  state.counters["no_consensus"] = std::isnan(reference) ? 1 : 0;
  state.counters["gjk_iterations"] = gjk_iterations;
  state.SetLabel(MethodName(method));
}

BENCHMARK(BM_Narrowphase)
    ->ArgsProduct({benchmark::CreateDenseRange(0, kNumPoses - 1, 1),
                   {8, 16, 32, 64},
                   benchmark::CreateDenseRange(0, kNumGaps - 1, 1),
                   {kFcl, kCoal, kCoalPolyak, kCoalNesterov}})
    ->Unit(benchmark::kMicrosecond);

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake

BENCHMARK_MAIN();