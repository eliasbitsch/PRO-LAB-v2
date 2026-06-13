#pragma once
// Likelihood Field for laser-vs-map measurement updates (Probabilistic
// Robotics §6.4 / §8.3.5).
//
// Builds a precomputed Euclidean distance transform of an OccupancyGrid:
// for every cell, the metric distance (in metres) to the nearest occupied
// cell. A particle's beam endpoint then evaluates to a Gaussian density
// over that distance - fast O(beams) per particle.
//
// Why a 2-pass chamfer DT instead of an exact algorithm: the map is built
// once and queried millions of times per scan-update, so a simple,
// dependency-free implementation is more important than the last fraction
// of metric accuracy. The (1, sqrt(2)) chamfer is within ~5% of true
// Euclidean and is plenty for likelihood weighting.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace pro_lab_filters {

class LikelihoodField {
public:
  LikelihoodField() = default;

  // Build from raw OccupancyGrid fields. `data` is row-major, length w*h,
  // with values in [-1, 100] (ROS convention). Cells with `data[i] >=
  // occupied_threshold` are treated as obstacles; everything else is free
  // (including unknowns, which is conservative - we'd rather under-weight
  // far cells than over-weight them).
  void build(int w, int h, double resolution,
             double origin_x, double origin_y,
             const std::int8_t * data, int occupied_threshold = 50) {
    w_ = w;
    h_ = h;
    res_ = resolution;
    origin_x_ = origin_x;
    origin_y_ = origin_y;
    dist_.assign(static_cast<std::size_t>(w * h),
                 std::numeric_limits<float>::infinity());

    for (int i = 0; i < w * h; ++i) {
      if (data[i] >= occupied_threshold) {
        dist_[i] = 0.0f;
      }
    }

    // 2-pass chamfer Euclidean DT (cell units). We multiply by resolution
    // at the end to get metres.
    const float SQRT2 = 1.41421356f;

    auto idx = [&](int x, int y) { return y * w + x; };

    // Forward pass
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        float & d = dist_[idx(x, y)];
        if (y > 0) {
          d = std::min(d, dist_[idx(x, y - 1)] + 1.0f);
          if (x > 0)     d = std::min(d, dist_[idx(x - 1, y - 1)] + SQRT2);
          if (x < w - 1) d = std::min(d, dist_[idx(x + 1, y - 1)] + SQRT2);
        }
        if (x > 0)        d = std::min(d, dist_[idx(x - 1, y)] + 1.0f);
      }
    }
    // Backward pass
    for (int y = h - 1; y >= 0; --y) {
      for (int x = w - 1; x >= 0; --x) {
        float & d = dist_[idx(x, y)];
        if (y < h - 1) {
          d = std::min(d, dist_[idx(x, y + 1)] + 1.0f);
          if (x > 0)     d = std::min(d, dist_[idx(x - 1, y + 1)] + SQRT2);
          if (x < w - 1) d = std::min(d, dist_[idx(x + 1, y + 1)] + SQRT2);
        }
        if (x < w - 1)   d = std::min(d, dist_[idx(x + 1, y)] + 1.0f);
      }
    }
    // Cells to metres
    for (auto & d : dist_) {
      d *= static_cast<float>(res_);
    }
    valid_ = true;
  }

  bool valid() const { return valid_; }

  // Distance (m) to nearest obstacle at world coordinate (x, y). Returns
  // a "max" distance when off-map so beams that fall outside contribute
  // a near-zero likelihood - equivalent to "this particle is making us
  // shoot rays into oblivion, treat it as bad".
  float distanceMeters(double x, double y) const {
    if (!valid_) {
      return 0.0f;
    }
    int cx = static_cast<int>(std::floor((x - origin_x_) / res_));
    int cy = static_cast<int>(std::floor((y - origin_y_) / res_));
    if (cx < 0 || cy < 0 || cx >= w_ || cy >= h_) {
      return out_of_map_dist_m_;
    }
    return dist_[cy * w_ + cx];
  }

  void setOutOfMapDistanceMeters(float d) { out_of_map_dist_m_ = d; }

private:
  bool                  valid_ {false};
  int                   w_ {0};
  int                   h_ {0};
  double                res_ {0.0};
  double                origin_x_ {0.0};
  double                origin_y_ {0.0};
  std::vector<float>    dist_;
  float                 out_of_map_dist_m_ {2.0f};   // saturate beyond this
};

}  // namespace pro_lab_filters
