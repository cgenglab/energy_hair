//
// Created by Nobuyuki Umetani on 2021-10-01.
//

#ifndef HAIR_H_
#define HAIR_H_

#include "delfem2/vec2.h"
#include "delfem2/vec3.h"
#include "delfem2/point_on_surface_mesh.h"
#include "delfem2/ls_pentadiagonal.h"
#include "delfem2/geo3_v23m34q.h"
#include "delfem2/eigen/implicit_rbf_approximation.h"

class Stroke {
 public:
  Stroke()
      : rbf( [](double r) { return std::exp(-r); }, true ) {};

 public:
  std::vector<delfem2::CVec2f> xys{};
  ImplicitRbfApproximation rbf;
};


/**
 * @param vtx_xyz 3D polyline of a hair strand
 * @param camra_stroke mapping from camera index to stroke
 */
class Hair {
 public:
  void Initialize(
      delfem2::PointOnSurfaceMesh<double> sample,
      double length_edge_ini,
      const std::vector<double>& vtx_xyz_head,
      const std::vector<unsigned int>& tri_vtx_xyz_head,
      const std::vector<double>& vtx_nrm_head,
      const std::vector<unsigned int>& tri_vtx_nrm_head) {
    root_on_mesh = sample;
    root_pos = root_on_mesh.PositionOnMeshTri3(
        vtx_xyz_head, tri_vtx_xyz_head);
    root_normal = root_on_mesh.UnitNormalOnMeshTri3(
        vtx_nrm_head, tri_vtx_nrm_head);
    root_normal.normalize();
    for (int idiv = 0; idiv < 10; ++idiv) {
      delfem2::CVec3f p1 = (root_pos + root_normal * idiv * length_edge_ini).cast<float>();
      this->vtx_xyz.push_back(p1.x);
      this->vtx_xyz.push_back(p1.y);
      this->vtx_xyz.push_back(p1.z);
    }
  }
 public:
  bool is_converged = false;
  std::vector<float> vtx_xyz{};
  std::map<unsigned int, Stroke> camera_stroke{};
  delfem2::LinearSystemSolver_BlockPentaDiagonal<3> linsys;
  delfem2::PointOnSurfaceMeshd root_on_mesh;
  delfem2::CVec3d root_pos, root_normal;
  delfem2::CQuatd root_rotation; // initialize with unit quaternion

  // below: unnecessary
  std::vector<double> vtx_res{};
};


/**
 *
 * @param mvp_matrix
 * @param hairs
 * @param screen_position
 * @return idx_hair, param, screen distance
 */
std::tuple<unsigned int, float, float> PickHair(
  const delfem2::CMat4f &mvp_matrix,
  const std::vector<Hair> &hairs,
  const delfem2::CVec2f &screen_position) {
  float dist_best = -1;
  float param_best;
  unsigned int idx_hair_best = UINT_MAX;
  for (unsigned int ih = 0; ih < hairs.size(); ++ih) {
    const std::vector<float> &xyzs = hairs[ih].vtx_xyz;
    const unsigned int nxyz = xyzs.size() / 3;
    if (nxyz < 2) { continue; }
    for (unsigned int i = 0; i < nxyz - 1; ++i) {
      unsigned int j = i + 1;
      delfem2::CVec3f p0 = mvp_matrix.MultVec3_Homography(xyzs.data() + i * 3);
      delfem2::CVec3f p1 = mvp_matrix.MultVec3_Homography(xyzs.data() + j * 3);
      float param0;
      delfem2::CVec2f p2 = delfem2::Nearest_Edge_Point(
          param0,
          screen_position,
          delfem2::CVec2f(p0.data()),
          delfem2::CVec2f(p1.data()));
      const float dist = (p2 - screen_position).norm();
      if (dist_best >= 0 && dist > dist_best) { continue; }
      dist_best = dist;
      idx_hair_best = ih;
      param_best = i + param0;
    }
  }
  return {idx_hair_best, param_best, dist_best};
}

void RelaxAndPropagateHairRootOrientation(
    std::vector<Hair> &hairs)
{
  for(auto & hair : hairs){
    if( hair.camera_stroke.empty() ) { continue; }
    delfem2::CVec3f p0(hair.vtx_xyz.data());
    delfem2::CVec3f p1(hair.vtx_xyz.data() + 3);
    delfem2::CVec3f n1 = hair.root_rotation.RotateVector(hair.root_normal.data());
    delfem2::CVec3f n2 = (p1-p0).normalized();
    delfem2::CVec3f n3 = (n2 + hair.root_normal.cast<float>()).normalized();
    delfem2::CMat3f R0 = delfem2::Mat3_MinimumRotation(n1,n3);
    delfem2::CQuatf q0 = R0.GetQuaternion();
    hair.root_rotation = q0.cast<double>() * hair.root_rotation;
  }
  for(unsigned int ih=0;ih<hairs.size();++ih){
    delfem2::CVec3d pi = hairs[ih].root_pos;
    double dist_best = -1;
    unsigned int idx_best = UINT_MAX;
    for(unsigned int jh=0;jh<hairs.size();++jh) {
      if( jh == ih ){ continue; }
      if( hairs[jh].camera_stroke.empty() ){ continue; }
      delfem2::CVec3d pj = hairs[jh].root_pos;
      double dist0 = (pi-pj).norm();
      if( idx_best != UINT_MAX && dist0 > dist_best ){ continue; }
      idx_best = jh;
      dist_best = dist0;
    }
    if( idx_best == UINT_MAX){ continue; }
    hairs[ih].root_rotation = hairs[idx_best].root_rotation;
  }
}

#endif // HAIR_H_
