#include <block_map/traversability.h>
#include <iostream>

namespace traversability {

// ─────────────────────────────────────────────────────────────────────────────
// updateVoxelStats
// Welford online algorithm: accumulate mean and M2 (second central moment).
// ─────────────────────────────────────────────────────────────────────────────
void updateVoxelStats(Grid_Block& gb, int idx, const Vector3f& q_local)
{
  int&     n  = gb.n_pts_[idx];
  Vector3f& mu = gb.mean_[idx];
  Matrix3f& M2 = gb.M2_[idx];

  n++;
  Vector3f delta  = q_local - mu;
  mu += delta / float(n);
  Vector3f delta2 = q_local - mu;
  M2 += delta * delta2.transpose();   // symmetric outer-product accumulation

  gb.dirty_[idx] = 1;
}

// ─────────────────────────────────────────────────────────────────────────────
// solveNormalAndRoughness
// PCA on covariance matrix: eigenvector of min eigenvalue → surface normal;
// λ_min / trace → normalised roughness ∈ [0,1].
// ─────────────────────────────────────────────────────────────────────────────
bool solveNormalAndRoughness(const Grid_Block& gb, int idx,
                             Vector3f& n_out, float& rough_out)
{
  const int n = gb.n_pts_[idx];
  if (n < 2) return false;

  Matrix3f cov = gb.M2_[idx] / float(n - 1);
  if (!cov.allFinite()) return false;

  Eigen::SelfAdjointEigenSolver<Matrix3f> es(cov, Eigen::ComputeEigenvectors);
  if (es.info() != Eigen::Success) return false;

  // Eigenvalues are in ascending order; col(0) corresponds to λ_min.
  n_out = es.eigenvectors().col(0);
  if (!n_out.allFinite() || n_out.norm() < 1e-8f) return false;
  n_out.normalize();

  const auto& L = es.eigenvalues();
  float trace = std::max(1e-8f, L.sum());
  rough_out = std::max(0.f, std::min(1.f, L(0) / trace));
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// isOccupied
// ─────────────────────────────────────────────────────────────────────────────
bool isOccupied(const Grid_Block& gb, int idx, float occ_thr_log)
{
  return gb.odds_log_[idx] >= occ_thr_log;
}

// ─────────────────────────────────────────────────────────────────────────────
// markSurfaceVoxels
// An occupied voxel is "surface" if at least one neighbour is not occupied.
// ─────────────────────────────────────────────────────────────────────────────
int markSurfaceVoxels(Grid_Block& gb,
                      float occ_thr_log,
                      bool  use26,
                      int   min_pts_thr)
{
  const int Sx = gb.block_size_.x();
  const int Sy = gb.block_size_.y();
  const int Sz = gb.block_size_.z();

  auto I = [&](int x, int y, int z){ return z * (Sx * Sy) + y * Sx + x; };

  // 6-connectivity offsets
  static const int K6       = 6;
  static const int DX6[6]  = {+1,-1, 0, 0, 0, 0};
  static const int DY6[6]  = { 0, 0,+1,-1, 0, 0};
  static const int DZ6[6]  = { 0, 0, 0, 0,+1,-1};

  // 26-connectivity offsets
  static const int K26      = 26;
  static const int DX26[26] = {
    -1, 0, 1,  -1, 0, 1,  -1, 0, 1,   // dz = -1
    -1, 0, 1,  -1,      1,   -1, 0, 1,   // dz =  0 (no centre)
    -1, 0, 1,  -1, 0, 1,  -1, 0, 1    // dz = +1
  };
  static const int DY26[26] = {
    -1,-1,-1,   0, 0, 0,   1, 1, 1,
    -1,-1,-1,   0,      0,   1, 1, 1,
    -1,-1,-1,   0, 0, 0,   1, 1, 1
  };
  static const int DZ26[26] = {
    -1,-1,-1, -1,-1,-1, -1,-1,-1,
     0, 0, 0,  0,      0,  0, 0, 0,
     1, 1, 1,  1, 1, 1,  1, 1, 1
  };

  const int  K  = use26 ? K26  : K6;
  const int* DX = use26 ? DX26 : DX6;
  const int* DY = use26 ? DY26 : DY6;
  const int* DZ = use26 ? DZ26 : DZ6;

  std::fill(gb.is_surface_.begin(), gb.is_surface_.end(), 0);

  int marked = 0;
  for (int z = 0; z < Sz; ++z)
  for (int y = 0; y < Sy; ++y)
  for (int x = 0; x < Sx; ++x) {
    const int idx = I(x, y, z);

    if (!isOccupied(gb, idx, occ_thr_log)) continue;
    if (min_pts_thr > 0 && gb.n_pts_[idx] < min_pts_thr) continue;

    bool surface = false;
    for (int k = 0; k < K; ++k) {
      const int xx = x + DX[k];
      const int yy = y + DY[k];
      const int zz = z + DZ[k];
      if (xx < 0 || yy < 0 || zz < 0 || xx >= Sx || yy >= Sy || zz >= Sz) continue;
      if (!isOccupied(gb, I(xx, yy, zz), occ_thr_log)) { surface = true; break; }
    }

    if (surface) { gb.is_surface_[idx] = 1; ++marked; }
  }
  return marked;
}

// ─────────────────────────────────────────────────────────────────────────────
// processDirtyVoxels
// Full traversability pipeline for one Grid_Block:
//   1) PCA → normal & roughness for dirty voxels
//   2) Surface voxel marking
//   3) (Optional) normal field smoothing  [currently disabled]
//   4) Slope from normals
//   5) Traversability score ∈ [0,1]  (1 = flat/traversable)
// ─────────────────────────────────────────────────────────────────────────────
void processDirtyVoxels(Grid_Block&                    gb,
                        float                          voxel_size,
                        const Vector3f&                gravity_dir_unit,
                        const NormalSmoothParams&       np,
                        const TraversabilityScoreParams& tsp,
                        float                          occ_thr_log)
{
  const int Nx = gb.block_size_.x();
  const int Ny = gb.block_size_.y();
  const int Nz = gb.block_size_.z();
  auto I = [=](int x, int y, int z){ return z * (Nx * Ny) + y * Nx + x; };

  const int min_pts = gb.innerPointsNumThr;  // set from tsp.min_pts_thr at Awake time

  // 1) PCA: update normal & roughness for every dirty voxel
  for (int z = 0; z < Nz; ++z)
  for (int y = 0; y < Ny; ++y)
  for (int x = 0; x < Nx; ++x) {
    int idx = I(x, y, z);
    if (!gb.dirty_[idx]) continue;
    gb.dirty_[idx] = 0;
    if (gb.n_pts_[idx] < min_pts || gb.n_pts_[idx] > min_pts * 100) continue;

    Vector3f n; float r;
    if (solveNormalAndRoughness(gb, idx, n, r)) {
      n = (n.dot(gravity_dir_unit) >= 0.f) ? n : -n;
      gb.normal_[idx]    = n;
      gb.roughness_[idx] = r;
    } else {
      gb.roughness_[idx] = -1.f;
    }
  }

  // 2) Surface voxel marking
  markSurfaceVoxels(gb, occ_thr_log, true, min_pts);

  // 3) Normal field smoothing (controlled by np.enabled)
  if (np.enabled) {
    for (int it = 0; it < np.iterations; ++it)
      smoothNormalsOnce(gb, np, voxel_size, gravity_dir_unit);
  }

  // 4) Slope from (smoothed) normals
  for (int z = 0; z < Nz; ++z)
  for (int y = 0; y < Ny; ++y)
  for (int x = 0; x < Nx; ++x) {
    int idx = I(x, y, z);

    if (!gb.is_surface_[idx] || gb.n_pts_[idx] < min_pts) {
      gb.slope_rad_[idx] = -1.f;
      continue;
    }

    Vector3f n = gb.normal_[idx];
    if (!n.allFinite() || n.squaredNorm() < 1e-8f) {
      gb.slope_rad_[idx] = -1.f;
      continue;
    }
    n.normalize();
    float c = std::fabs(n.dot(gravity_dir_unit));
    c = std::min(1.f, std::max(0.f, c));
    gb.slope_rad_[idx] = std::acos(c);
  }

  // 5) Traversability score  (0 = impassable, 1 = flat)
  const float slope_max_rad = std::max(1e-6f, tsp.slope_max_deg * float(M_PI) / 180.f);
  const float rough_max     = std::max(1e-6f, tsp.rough_max);

  for (int z = 0; z < Nz; ++z)
  for (int y = 0; y < Ny; ++y)
  for (int x = 0; x < Nx; ++x) {
    const int idx = I(x, y, z);

    if (!gb.is_surface_[idx] || gb.n_pts_[idx] < min_pts) {
      gb.score_[idx] = -1.f;
      continue;
    }

    const float slope = gb.slope_rad_[idx];
    const float rough = gb.roughness_[idx];
    if (slope < 0.f || !std::isfinite(slope) ||
        rough < 0.f || !std::isfinite(rough)) {
      gb.score_[idx] = -1.f;
      continue;
    }

    float slope_norm = std::min(1.f, slope / slope_max_rad);
    float rough_norm = std::min(1.f, rough / rough_max);

    float hazard = tsp.w_slope * slope_norm + tsp.w_rough * rough_norm;
    gb.score_[idx] = 1.f - std::min(1.f, std::max(0.f, hazard));
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// orientTo
// ─────────────────────────────────────────────────────────────────────────────
Vector3f orientTo(const Vector3f& n, const Vector3f& ref)
{
  return (n.dot(ref) >= 0.f) ? n : -n;
}

// ─────────────────────────────────────────────────────────────────────────────
// smoothNormalsOnce
// Bilateral filter over the normal field.  Double-buffered to avoid order bias.
// ─────────────────────────────────────────────────────────────────────────────
void smoothNormalsOnce(Grid_Block&              gb,
                       const NormalSmoothParams& np,
                       float                    voxel_size,
                       const Vector3f&          gravity_dir_unit)
{
  const int Sx = gb.block_size_.x();
  const int Sy = gb.block_size_.y();
  const int Sz = gb.block_size_.z();
  auto I = [&](int x, int y, int z){ return z * (Sx * Sy) + y * Sx + x; };

  // Double buffer: read from normal_, write into newN
  std::vector<Vector3f> newN = gb.normal_;

  const float sigma_s2       = std::max(1e-8f, np.sigma_s * np.sigma_s);
  const float sigma_ang_rad  = std::max(1e-6f, np.sigma_ang_deg * float(M_PI) / 180.f);
  const float two_sig_ang2   = 2.f * sigma_ang_rad * sigma_ang_rad;

  for (int z = 0; z < Sz; ++z)
  for (int y = 0; y < Sy; ++y)
  for (int x = 0; x < Sx; ++x) {
    const int idx = I(x, y, z);

    if (!gb.is_surface_[idx] || gb.n_pts_[idx] < gb.innerPointsNumThr) continue;

    Vector3f ni = gb.normal_[idx];
    if (!ni.allFinite() || ni.squaredNorm() < 1e-8f) continue;
    ni.normalize();

    Vector3f sum  = Vector3f::Zero();
    float    wsum = 0.f;

    for (int dz = -np.half_z; dz <= np.half_z; ++dz) {
      const int zz = z + dz; if (zz < 0 || zz >= Sz) continue;
      for (int dy = -np.radius_xy; dy <= np.radius_xy; ++dy) {
        const int yy = y + dy; if (yy < 0 || yy >= Sy) continue;
        for (int dx = -np.radius_xy; dx <= np.radius_xy; ++dx) {
          const int xx = x + dx; if (xx < 0 || xx >= Sx) continue;
          if (dx == 0 && dy == 0 && dz == 0) continue;

          const int j = I(xx, yy, zz);
          if (!gb.is_surface_[j] || gb.n_pts_[j] < gb.innerPointsNumThr) continue;

          Vector3f nj = gb.normal_[j];
          if (!nj.allFinite() || nj.squaredNorm() < 1e-8f) continue;
          nj.normalize();

          // Align sign to gravity direction for global consistency
          nj = (nj.dot(gravity_dir_unit) >= 0.f) ? nj : -nj;

          // Spatial weight
          const float ds2 = (dx*dx + dy*dy + dz*dz) * voxel_size * voxel_size;
          const float ws  = std::exp(-ds2 / (2.f * sigma_s2));

          // Angular similarity weight
          float c   = std::max(-1.f, std::min(1.f, ni.dot(nj)));
          float ang = std::acos(c);
          const float wn = std::exp(-(ang*ang) / two_sig_ang2);

          // Roughness and point-count weights (optional)
          const float wr = (gb.roughness_[j] >= 0.f)
                           ? std::exp(-gb.roughness_[j] / std::max(1e-6f, np.sigma_r))
                           : 1.f;
          const float wc = std::min(1.f, gb.n_pts_[j] / float(std::max(1, np.n_ref)));

          const float w = ws * wn * wr * wc;
          sum  += w * nj;
          wsum += w;
        }
      }
    }

    if (wsum > 1e-6f) {
      Vector3f nsm = sum / wsum;
      if (nsm.squaredNorm() > 1e-8f) {
        nsm.normalize();
        nsm = (nsm.dot(gravity_dir_unit) >= 0.f) ? nsm : -nsm;
        newN[idx] = nsm;
      }
    }
  }

  gb.normal_.swap(newN);
}

} // namespace traversability
