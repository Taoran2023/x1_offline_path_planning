#ifndef TRAVERSABILITY_H_
#define TRAVERSABILITY_H_

#include <block_map/mapping_struct.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>

using namespace Eigen;
using namespace BlockMapStruct;

struct NormalSmoothParams {
  bool  enabled       = true;
  int   iterations    = 1;
  int   radius_xy     = 1;    // neighborhood radius (xy)
  int   half_z        = 2;    // neighborhood half-thickness (z)
  float sigma_s       = 0.06f;  // spatial scale [m]
  float sigma_ang_deg = 20.f;   // angular Gaussian scale [deg]
  float sigma_r       = 0.02f;  // roughness scale
  int   n_ref         = 5;      // reference point count
};

struct TraversabilityScoreParams {
  int   min_pts_thr   = 5;    // min points per voxel (used by PCA / surface / score)
  float slope_max_deg = 60.f; // slope normalization upper bound [deg]; score→0 beyond this angle
  float rough_max     = 0.10f;// roughness normalization upper bound (λ_min/trace ∈ [0,0.33])
  float w_slope       = 0.9f; // slope weight in hazard
  float w_rough       = 0.1f; // roughness weight in hazard
};

namespace traversability {

/**
 * @brief Welford online update of per-voxel point statistics (mean, M2 covariance).
 *        Sets dirty flag on update.
 */
void updateVoxelStats(Grid_Block& gb, int idx, const Vector3f& q_local);

/**
 * @brief PCA on accumulated M2 covariance to estimate surface normal and roughness.
 * @return false if not enough points or numerical failure.
 */
bool solveNormalAndRoughness(const Grid_Block& gb, int idx,
                             Vector3f& n_out, float& rough_out);

/**
 * @brief Check whether a voxel is occupied by log-odds threshold.
 */
bool isOccupied(const Grid_Block& gb, int idx, float occ_thr_log);

/**
 * @brief Mark surface voxels: occupied voxels with at least one non-occupied neighbour.
 * @param use26      true → 26-connectivity, false → 6-connectivity
 * @param min_pts_thr minimum point count to consider a voxel; ≤0 disables check
 * @return number of voxels marked as surface
 */
int markSurfaceVoxels(Grid_Block& gb,
                      float occ_thr_log,
                      bool  use26      = false,
                      int   min_pts_thr = 0);

/**
 * @brief Main traversability pipeline for one block:
 *        1) PCA → normal & roughness for dirty voxels
 *        2) Surface voxel marking
 *        3) Normal smoothing (enabled via np.enabled)
 *        4) Slope from smoothed normals
 *        5) Traversability score  (0 = impassable, 1 = flat)
 */
void processDirtyVoxels(Grid_Block&                    gb,
                        float                          voxel_size,
                        const Vector3f&                gravity_dir_unit,
                        const NormalSmoothParams&       np,
                        const TraversabilityScoreParams& tsp,
                        float                          occ_thr_log);

/**
 * @brief Orient normal n to the same hemisphere as ref.
 */
Vector3f orientTo(const Vector3f& n, const Vector3f& ref);

/**
 * @brief One pass of bilateral normal smoothing (double-buffered).
 */
void smoothNormalsOnce(Grid_Block&              gb,
                       const NormalSmoothParams& np,
                       float                    voxel_size,
                       const Vector3f&          gravity_dir_unit);

} // namespace traversability

#endif // TRAVERSABILITY_H_
