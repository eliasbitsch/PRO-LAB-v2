#pragma once
// Particle Filter (pure, no ROS).
// State per particle: [x, y, yaw]. Control: [v, omega]. Measurements: IMU yaw, full pose.
//
// Two init modes:
//   - GAUSSIAN (default): particles ~ N(mean, diag(spread)^2)
//   - UNIFORM: particles ~ U(center ± half_extent), yaw ~ U(-half_extent_yaw, +half_extent_yaw)
//     Used for the "kidnapped robot" wrong-init scenario where the filter
//     has no a-priori pose and must globally localize.
#include "pro_lab_filters/LikelihoodField.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <unordered_set>
#include <vector>

namespace pro_lab_filters {

inline double pf_wrap_pi(double a) { return std::atan2(std::sin(a), std::cos(a)); }

enum class PFInitMode { Gaussian, Uniform };

class PF {
public:
  using Vec3 = Eigen::Vector3d;

  PF() = default;

  // Gaussian init around `mean` with per-axis std-dev `spread`.
  void init(const Vec3& mean, const Vec3& spread, std::size_t num_particles,
            double motion_noise_v, double motion_noise_w,
            std::uint64_t seed = 42) {
    initCommon(num_particles, motion_noise_v, motion_noise_w, seed);
    std::normal_distribution<double> nx(mean(0), spread(0));
    std::normal_distribution<double> ny(mean(1), spread(1));
    std::normal_distribution<double> nt(mean(2), spread(2));
    for (auto& p : particles_) { p << nx(rng_), ny(rng_), nt(rng_); }
    init_mode_ = PFInitMode::Gaussian;
  }

  // Uniform init: particles ~ U(center - half_extent, center + half_extent).
  // Use a large half_extent (e.g. half the map size) for global localisation.
  void initUniform(const Vec3& center, const Vec3& half_extent,
                   std::size_t num_particles,
                   double motion_noise_v, double motion_noise_w,
                   std::uint64_t seed = 42) {
    initCommon(num_particles, motion_noise_v, motion_noise_w, seed);
    std::uniform_real_distribution<double> ux(center(0) - half_extent(0),
                                              center(0) + half_extent(0));
    std::uniform_real_distribution<double> uy(center(1) - half_extent(1),
                                              center(1) + half_extent(1));
    std::uniform_real_distribution<double> ut(-half_extent(2), half_extent(2));
    for (auto& p : particles_) { p << ux(rng_), uy(rng_), pf_wrap_pi(center(2) + ut(rng_)); }
    init_mode_ = PFInitMode::Uniform;
  }

  // Velocity motion model with alpha1..6 noise (Probabilistic Robotics
  // §5.3, Table 5.3). Falls back to the simpler nv_/nw_ Gaussian when
  // setMotionAlphas() has not been called.
  //
  // Noise variances scale with the magnitude of v and w:
  //   v_noise² = α1·v² + α2·w²
  //   w_noise² = α3·v² + α4·w²
  //   γ_noise² = α5·v² + α6·w²       (final yaw "drift", separate from w)
  // This makes the cloud spread proportionally to commanded motion — the
  // robot is more uncertain about a fast turn than about creeping forward.
  void predict(double v, double w, double dt) {
    if (use_alphas_) {
      const double v2 = v * v;
      const double w2 = w * w;
      const double sigma_v_sq = alpha1_ * v2 + alpha2_ * w2;
      const double sigma_w_sq = alpha3_ * v2 + alpha4_ * w2;
      const double sigma_g_sq = alpha5_ * v2 + alpha6_ * w2;
      std::normal_distribution<double> n01(0.0, 1.0);
      const double sv = std::sqrt(std::max(sigma_v_sq, 1e-9));
      const double sw = std::sqrt(std::max(sigma_w_sq, 1e-9));
      const double sg = std::sqrt(std::max(sigma_g_sq, 1e-9));
      for (auto& p : particles_) {
        const double v_hat     = v + sv * n01(rng_);
        const double w_hat     = w + sw * n01(rng_);
        const double gamma_hat =     sg * n01(rng_);
        const double theta = p(2);
        if (std::abs(w_hat) > 1e-4) {
          // Curved-arc integration (exact for constant v, w).
          const double r = v_hat / w_hat;
          p(0) += -r * std::sin(theta) + r * std::sin(theta + w_hat * dt);
          p(1) +=  r * std::cos(theta) - r * std::cos(theta + w_hat * dt);
        } else {
          // Straight-line limit (avoids divide-by-zero).
          p(0) += v_hat * std::cos(theta) * dt;
          p(1) += v_hat * std::sin(theta) * dt;
        }
        p(2) = pf_wrap_pi(theta + w_hat * dt + gamma_hat * dt);
      }
    } else {
      std::normal_distribution<double> nv(0.0, nv_);
      std::normal_distribution<double> nw(0.0, nw_);
      for (auto& p : particles_) {
        const double vn = v + nv(rng_);
        const double wn = w + nw(rng_);
        const double theta = p(2);
        p(0) += vn * std::cos(theta) * dt;
        p(1) += vn * std::sin(theta) * dt;
        p(2) = pf_wrap_pi(theta + wn * dt);
      }
    }
  }

  // Enable Thrun §5.3 velocity motion model. Disable by passing alpha1==0.
  // AMCL TB4 default: alpha1..5 = 0.2.
  void setMotionAlphas(double a1, double a2, double a3,
                       double a4, double a5, double a6 = 0.2) {
    alpha1_ = a1; alpha2_ = a2; alpha3_ = a3;
    alpha4_ = a4; alpha5_ = a5; alpha6_ = a6;
    use_alphas_ = (a1 + a2 + a3 + a4 + a5 + a6) > 0.0;
  }

  void updateImuYaw(double yaw_meas, double r_yaw) {
    double sum = 0.0;
    std::vector<double> logw(N_);
    double maxlog = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < N_; ++i) {
      double d = pf_wrap_pi(yaw_meas - particles_[i](2));
      logw[i] = -0.5 * d * d / r_yaw;
      if (logw[i] > maxlog) maxlog = logw[i];
    }
    for (std::size_t i = 0; i < N_; ++i) {
      weights_[i] *= std::exp(logw[i] - maxlog);
      sum += weights_[i];
    }
    normalizeOrResetWeights(sum);
    maybeResample();
  }

  void updatePose(double mx, double my, double myaw, const Vec3& r_diag) {
    double sum = 0.0;
    std::vector<double> logw(N_);
    double maxlog = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < N_; ++i) {
      const auto& p = particles_[i];
      double dx = mx - p(0);
      double dy = my - p(1);
      double dt = pf_wrap_pi(myaw - p(2));
      logw[i] = -0.5 * (dx * dx / r_diag(0) + dy * dy / r_diag(1) +
                        dt * dt / r_diag(2));
      if (logw[i] > maxlog) maxlog = logw[i];
    }
    for (std::size_t i = 0; i < N_; ++i) {
      weights_[i] *= std::exp(logw[i] - maxlog);
      sum += weights_[i];
    }
    normalizeOrResetWeights(sum);
    maybeResample();
  }

  // Likelihood-field scan update (Probabilistic Robotics §6.4 / §8.3.5).
  //
  // beams_local: subsampled 2D ray endpoints expressed in the *robot*
  //   (base) frame — i.e. lidar TF already applied. Empty / max-range /
  //   too-short beams should be filtered out by the caller.
  // sigma_hit:  metres. Std-dev of the gaussian on distance-to-obstacle.
  //             Typical 0.1–0.3 m for an indoor lidar.
  // w_rand:     small mixing weight for "random" beams (1 / range_max),
  //             keeps the filter from dying when no particle agrees with
  //             the scan (e.g. right after a kidnap).
  void updateScan(const std::vector<Eigen::Vector2d>& beams_local,
                  const LikelihoodField& lf,
                  double sigma_hit = 0.2,
                  double w_rand    = 0.05) {
    if (beams_local.empty() || !lf.valid()) {
      return;
    }
    const double inv_two_sig2 = 1.0 / (2.0 * sigma_hit * sigma_hit);
    const std::size_t M = beams_local.size();
    const double n_beams_d = static_cast<double>(M);

    // Beam skip (AMCL §5.3 / params beam_skip_distance, beam_skip_threshold,
    // beam_skip_error_threshold). For each beam, compute the fraction of
    // particles whose endpoint sits within `beam_skip_dist_` of an obstacle.
    // Beams that almost no particle accepts are likely from a dynamic
    // obstacle (person walking, moved chair) and get dropped — *unless*
    // most beams look bad, in which case the robot really is in a novel
    // place and we should trust the scan.
    std::vector<bool> use_beam(M, true);
    if (do_beam_skip_ && N_ > 0) {
      std::vector<int> agree(M, 0);
      for (std::size_t i = 0; i < N_; ++i) {
        const double px  = particles_[i](0);
        const double py  = particles_[i](1);
        const double cs  = std::cos(particles_[i](2));
        const double sn  = std::sin(particles_[i](2));
        for (std::size_t k = 0; k < M; ++k) {
          const auto & b = beams_local[k];
          const double wx = px + cs * b.x() - sn * b.y();
          const double wy = py + sn * b.x() + cs * b.y();
          if (lf.distanceMeters(wx, wy) <= beam_skip_dist_) {
            agree[k]++;
          }
        }
      }
      std::size_t skipped = 0;
      for (std::size_t k = 0; k < M; ++k) {
        if (static_cast<double>(agree[k]) / N_ < beam_skip_thresh_) {
          use_beam[k] = false;
          skipped++;
        }
      }
      // If >beam_skip_error_thresh fraction would be skipped, scan is
      // probably novel (e.g. just after a kidnap) — keep all beams.
      if (static_cast<double>(skipped) / M > beam_skip_err_thresh_) {
        std::fill(use_beam.begin(), use_beam.end(), true);
      }
    }

    std::vector<double> logw(N_, 0.0);
    double maxlog  = -std::numeric_limits<double>::infinity();
    std::size_t kept = 0;
    for (bool b : use_beam) {
      if (b) ++kept;
    }
    const double n_beams = static_cast<double>(std::max<std::size_t>(kept, 1));

    for (std::size_t i = 0; i < N_; ++i) {
      const double px  = particles_[i](0);
      const double py  = particles_[i](1);
      const double yaw = particles_[i](2);
      const double cs  = std::cos(yaw);
      const double sn  = std::sin(yaw);

      double sum_loglik = 0.0;
      for (std::size_t k = 0; k < M; ++k) {
        if (!use_beam[k]) continue;
        const auto & b = beams_local[k];
        const double wx = px + cs * b.x() - sn * b.y();
        const double wy = py + sn * b.x() + cs * b.y();
        const float  d  = lf.distanceMeters(wx, wy);
        const double p_hit  = std::exp(-double(d) * double(d) * inv_two_sig2);
        const double p_beam = (1.0 - w_rand) * p_hit + w_rand;
        sum_loglik += std::log(std::max(p_beam, 1e-12));
      }
      logw[i] = sum_loglik;
      if (logw[i] > maxlog) {
        maxlog = logw[i];
      }
    }
    (void)n_beams_d;

    double sum = 0.0;
    for (std::size_t i = 0; i < N_; ++i) {
      const double w = std::exp(logw[i] - maxlog);
      weights_[i] *= w;
      sum += weights_[i];
    }

    // Augmented-MCL random-particle injection — OPT-IN (kidnap recovery only).
    // It tracks the short/long-term average per-beam geometric-mean likelihood
    // and injects uniform particles across the whole map when the scan stops
    // matching. Disabled by default: during normal driving the likelihood
    // fluctuates (the robot sees new parts of the map), which would otherwise
    // raise inject_prob_ and scatter particles across the large warehouse,
    // dragging the weighted-mean estimate away from truth (PF "divergence"
    // with a deceptively healthy ESS, since resampling resets weights).
    if (aug_mcl_enabled_) {
      double w_avg = 0.0;
      for (std::size_t i = 0; i < N_; ++i)
        w_avg += std::exp(logw[i] / std::max(1.0, n_beams));
      w_avg /= static_cast<double>(N_);
      if (w_slow_ == 0.0) { w_slow_ = w_avg; }
      if (w_fast_ == 0.0) { w_fast_ = w_avg; }
      w_slow_ += alpha_slow_ * (w_avg - w_slow_);
      w_fast_ += alpha_fast_ * (w_avg - w_fast_);
      inject_prob_ = std::clamp(1.0 - w_fast_ / std::max(w_slow_, 1e-12),
                                0.0, 0.30);
    }

    normalizeOrResetWeights(sum);
    maybeResample();
  }

  Vec3 estimate() const {
    double x = 0.0, y = 0.0, sx = 0.0, cx = 0.0;
    for (std::size_t i = 0; i < N_; ++i) {
      x += weights_[i] * particles_[i](0);
      y += weights_[i] * particles_[i](1);
      sx += weights_[i] * std::sin(particles_[i](2));
      cx += weights_[i] * std::cos(particles_[i](2));
    }
    return Vec3(x, y, std::atan2(sx, cx));
  }

  Vec3 variance() const {
    Vec3 mean = estimate();
    double vx = 0.0, vy = 0.0, vyaw = 0.0;
    for (std::size_t i = 0; i < N_; ++i) {
      double dx = particles_[i](0) - mean(0);
      double dy = particles_[i](1) - mean(1);
      double dt = pf_wrap_pi(particles_[i](2) - mean(2));
      vx += weights_[i] * dx * dx;
      vy += weights_[i] * dy * dy;
      vyaw += weights_[i] * dt * dt;
    }
    return Vec3(vx, vy, vyaw);
  }

  // Full XY covariance including cross-term so RViz can render the actual
  // particle-cloud ellipse instead of an axis-aligned circle.
  Eigen::Matrix2d covarianceXY() const {
    Vec3 mean = estimate();
    Eigen::Matrix2d C = Eigen::Matrix2d::Zero();
    for (std::size_t i = 0; i < N_; ++i) {
      double dx = particles_[i](0) - mean(0);
      double dy = particles_[i](1) - mean(1);
      C(0, 0) += weights_[i] * dx * dx;
      C(1, 1) += weights_[i] * dy * dy;
      C(0, 1) += weights_[i] * dx * dy;
    }
    C(1, 0) = C(0, 1);
    return C;
  }

  // Effective Sample Size: 1 / sum(w_i^2). Drops as particles concentrate;
  // hits ~1 (all weight on one particle) for a degenerate filter.
  double ess() const {
    double sq = 0.0;
    for (double w : weights_) sq += w * w;
    return sq > 0.0 ? 1.0 / sq : 0.0;
  }

  std::size_t size() const { return N_; }
  PFInitMode initMode() const { return init_mode_; }
  const std::vector<Vec3>& particles() const { return particles_; }
  const std::vector<double>& weights() const { return weights_; }

  // Map bounds (world coords) for Augmented-MCL random-particle injection.
  // Without these set, the kidnap-recovery extension is disabled.
  void setMapBounds(double xmin, double ymin, double xmax, double ymax) {
    map_xmin_ = xmin; map_ymin_ = ymin; map_xmax_ = xmax; map_ymax_ = ymax;
    have_map_bounds_ = true;
  }
  void setAugmentedMcl(double alpha_slow, double alpha_fast) {
    alpha_slow_ = alpha_slow;
    alpha_fast_ = alpha_fast;
  }
  // Enable random-particle injection for global/kidnap recovery. OFF by
  // default — see updateScan(): on during normal motion it scatters particles
  // across the map and the estimate diverges.
  void enableAugmentedMcl(bool on) { aug_mcl_enabled_ = on; }
  double injectProbability() const { return inject_prob_; }

  void setBeamSkip(bool on, double dist = 0.5, double thresh = 0.3,
                   double err_thresh = 0.9) {
    do_beam_skip_      = on;
    beam_skip_dist_    = dist;
    beam_skip_thresh_  = thresh;
    beam_skip_err_thresh_ = err_thresh;
  }

  // KLD bounds — particle count adapts within [n_min, n_max].
  // epsilon: max allowed K–L error (typical 0.05)
  // z: 1-δ quantile of standard normal (δ=0.01 → z ≈ 2.33)
  // bin_xy / bin_yaw: histogram bin sizes for "did we cover this region"
  void setKld(std::size_t n_min, std::size_t n_max,
              double epsilon = 0.05, double z = 2.33,
              double bin_xy = 0.5, double bin_yaw = 0.17) {
    kld_n_min_ = n_min;
    kld_n_max_ = n_max;
    kld_eps_   = epsilon;
    kld_z_     = z;
    kld_bin_xy_  = bin_xy;
    kld_bin_yaw_ = bin_yaw;
    use_kld_ = true;
  }

private:
  void initCommon(std::size_t num_particles,
                  double motion_noise_v, double motion_noise_w,
                  std::uint64_t seed) {
    N_ = num_particles;
    nv_ = motion_noise_v;
    nw_ = motion_noise_w;
    rng_.seed(seed);
    particles_.assign(N_, Vec3::Zero());
    weights_.assign(N_, 1.0 / static_cast<double>(N_));
  }

  void normalizeOrResetWeights(double sum) {
    if (sum < 1e-12) {
      std::fill(weights_.begin(), weights_.end(), 1.0 / N_);
    } else {
      for (auto& w : weights_) w /= sum;
    }
  }

  void maybeResample() {
    // Standard trigger: low effective sample size (= weights have
    // collapsed to a few particles).
    // Augmented-MCL trigger: inject_prob > 0 means short-term avg
    // likelihood is below long-term avg → resample anyway, so the
    // injection step actually runs and the filter can recover.
    // Threshold 0.02: in our pillar-grid warehouse the scan likelihood
    // only drops mildly after a kidnap (translational aliasing — a pose
    // shifted by the 7.5 m grid period explains the scan almost equally
    // well), so inject_prob rarely exceeds ~0.05. Trigger early and let
    // the injected particles + IMU yaw + landmark-free scan evidence
    // fight it out.
    if (ess() < N_ / 2.0 || inject_prob_ > 0.02) {
      resample();
    }
  }

  void resample() {
    std::uniform_real_distribution<double> U(0.0, 1.0);
    std::uniform_real_distribution<double> Ux(map_xmin_, map_xmax_);
    std::uniform_real_distribution<double> Uy(map_ymin_, map_ymax_);
    std::uniform_real_distribution<double> Uyaw(-M_PI, M_PI);

    if (use_kld_) {
      resampleKld(U, Ux, Uy, Uyaw);
    } else {
      resampleFixed(U, Ux, Uy, Uyaw);
    }
    // Don't zero w_slow_/w_fast_ here — that would prevent the running
    // averages from ever diverging enough for inject_prob_ to grow.
  }

private:
  void resampleFixed(std::uniform_real_distribution<double> & U,
                     std::uniform_real_distribution<double> & Ux,
                     std::uniform_real_distribution<double> & Uy,
                     std::uniform_real_distribution<double> & Uyaw) {
    // Systematic resampling, optionally with augmented-MCL injection of
    // uniform random particles from the map.
    double r = U(rng_) / N_;
    std::vector<Vec3> new_p(N_);
    double c = weights_[0];
    std::size_t i = 0;
    for (std::size_t m = 0; m < N_; ++m) {
      if (have_map_bounds_ && U(rng_) < inject_prob_) {
        new_p[m] << Ux(rng_), Uy(rng_), Uyaw(rng_);
      } else {
        double u = r + static_cast<double>(m) / N_;
        while (u > c && i + 1 < N_) { ++i; c += weights_[i]; }
        new_p[m] = particles_[i];
      }
    }
    particles_ = std::move(new_p);
    std::fill(weights_.begin(), weights_.end(), 1.0 / N_);
  }

  // KLD-sampling (Probabilistic Robotics §4.3.4 / Fox 2003): keep drawing
  // particles until we've covered enough histogram bins to bound the
  // K–L divergence between sampled & true posterior at ≤ epsilon. When
  // the posterior is concentrated (well-localised), few unique bins → few
  // particles needed. When spread out (after kidnap), many bins → many
  // particles. Bound: N >= ceil((k-1)/(2ε) · (1 − 2/(9(k-1)) +
  //                              sqrt(2/(9(k-1)))·z)^3)
  void resampleKld(std::uniform_real_distribution<double> & U,
                   std::uniform_real_distribution<double> & Ux,
                   std::uniform_real_distribution<double> & Uy,
                   std::uniform_real_distribution<double> & Uyaw) {
    std::vector<Vec3> new_p;
    new_p.reserve(kld_n_max_);

    // Hashed set of (xb, yb, yawb) integer bin coordinates.
    std::unordered_set<std::int64_t> seen_bins;
    seen_bins.reserve(kld_n_max_);

    auto bin_id = [this](const Vec3 & p) {
      const std::int64_t bx = static_cast<std::int64_t>(
          std::floor(p(0) / kld_bin_xy_));
      const std::int64_t by = static_cast<std::int64_t>(
          std::floor(p(1) / kld_bin_xy_));
      const std::int64_t bt = static_cast<std::int64_t>(
          std::floor(p(2) / kld_bin_yaw_));
      // Pack into 64 bits: 21+21+22, with offsets to handle negatives.
      const std::int64_t bx_o = bx + (1ll << 20);
      const std::int64_t by_o = by + (1ll << 20);
      const std::int64_t bt_o = bt + (1ll << 21);
      return (bx_o & ((1ll << 21) - 1))
           | ((by_o & ((1ll << 21) - 1)) << 21)
           | ((bt_o & ((1ll << 22) - 1)) << 42);
    };

    // Build a CDF from current weights for sampling (alias method overkill
    // for our N).
    std::vector<double> cdf(N_);
    cdf[0] = weights_[0];
    for (std::size_t i = 1; i < N_; ++i) {
      cdf[i] = cdf[i - 1] + weights_[i];
    }

    std::size_t target_n = kld_n_min_;
    while (new_p.size() < target_n && new_p.size() < kld_n_max_) {
      Vec3 sample;
      if (have_map_bounds_ && U(rng_) < inject_prob_) {
        sample << Ux(rng_), Uy(rng_), Uyaw(rng_);
      } else {
        const double u = U(rng_) * cdf.back();
        const auto it  = std::lower_bound(cdf.begin(), cdf.end(), u);
        const std::size_t idx = std::min<std::size_t>(
            std::distance(cdf.begin(), it), N_ - 1);
        sample = particles_[idx];
      }
      new_p.push_back(sample);

      const std::int64_t b = bin_id(sample);
      if (seen_bins.insert(b).second) {
        const std::size_t k = seen_bins.size();
        if (k > 1) {
          // Wilson-Hilferty: target N for given # unique bins.
          const double k_minus_1 = static_cast<double>(k - 1);
          const double a = 2.0 / (9.0 * k_minus_1);
          const double inner = 1.0 - a + std::sqrt(a) * kld_z_;
          const double n_kld = (k_minus_1 / (2.0 * kld_eps_))
                             * inner * inner * inner;
          target_n = std::max(kld_n_min_,
                              static_cast<std::size_t>(std::ceil(n_kld)));
        }
      }
    }

    particles_ = std::move(new_p);
    N_ = particles_.size();
    weights_.assign(N_, 1.0 / static_cast<double>(N_));
  }

public:

  std::size_t N_ = 0;
  double nv_ = 0.05, nw_ = 0.05;
  std::vector<Vec3> particles_;
  std::vector<double> weights_;
  std::mt19937_64 rng_;
  PFInitMode init_mode_ = PFInitMode::Gaussian;

  // Augmented-MCL state
  bool   aug_mcl_enabled_ = false;
  double alpha_slow_ = 0.001;
  double alpha_fast_ = 0.10;
  double w_slow_     = 0.0;
  double w_fast_     = 0.0;
  double inject_prob_= 0.0;
  bool   have_map_bounds_ = false;
  double map_xmin_ = 0.0, map_ymin_ = 0.0, map_xmax_ = 0.0, map_ymax_ = 0.0;

  // Velocity motion model alpha1..6 (Thrun §5.3 Table 5.3)
  bool   use_alphas_ = false;
  double alpha1_ = 0.2, alpha2_ = 0.2, alpha3_ = 0.2;
  double alpha4_ = 0.2, alpha5_ = 0.2, alpha6_ = 0.2;

  // Beam skip (AMCL §5.3 / params beam_skip_*)
  bool   do_beam_skip_      = false;
  double beam_skip_dist_    = 0.5;
  double beam_skip_thresh_  = 0.3;
  double beam_skip_err_thresh_ = 0.9;

  // KLD-sampling params
  bool   use_kld_      = false;
  std::size_t kld_n_min_ = 100;
  std::size_t kld_n_max_ = 5000;
  double kld_eps_      = 0.05;
  double kld_z_        = 2.33;
  double kld_bin_xy_   = 0.5;
  double kld_bin_yaw_  = 0.17;
};

}  // namespace pro_lab_filters
