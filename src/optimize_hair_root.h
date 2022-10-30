//
// Created by Nobuyuki Umetani on 2021-10-15.
//

#ifndef OPTIMIZE_HAIR_ROOT_H_
#define OPTIMIZE_HAIR_ROOT_H_

#include <map>
#include <Eigen/Core>
#include <Eigen/LU>

#include "delfem2/vec2.h"
#include "delfem2/srchuni_v3.h"

#include "camera_cofing.h"
#include "camera_projection.h"
#include "hair.h"

void OptimizeHairRoot(
    std::vector<float> &vtx_xyz,
    const CameraConfig &cam_config,
    //
    const std::map<unsigned int, Stroke> &camera_has_polyline,
    const std::vector<double> &vtx_xyz_head,
    const std::vector<unsigned int> &tri_vtx_head) {
  namespace dfm2 = delfem2;
  assert(vtx_xyz.size()>=3);

  Eigen::MatrixXd matrix_lm = Eigen::MatrixXd::Zero(3, 3);
  Eigen::VectorXd resvec_lm = Eigen::VectorXd::Zero(3);

  {  // root vtx
    for (unsigned int icam = 0; icam < cam_config.aCamera.size(); ++icam) {
      using V2 = dfm2::CVec2f;
      if (camera_has_polyline.find(icam) == camera_has_polyline.end()) { continue; }
      const std::vector<V2> &polyline = camera_has_polyline.find(icam)->second.xys;
      if (polyline.empty()) { continue; }
      //
      const auto[mat_modelview, mat_projection] = CameraTransformationMVP(cam_config, icam);
      const dfm2::CMat4f mat_modelviewprojection = mat_projection * mat_modelview;
      // compute where the vertex of hair ends up on the screen [-1,+1,-1,+1]
      V2 target_screen_position = polyline[0];
      // compute the sensitivity how the current screen position moves when 3D hair vertex move
      float c[2], dc[2][3];
      dfm2::CdC_ScreenCoordinate<float>(
          c, dc,
          mat_modelviewprojection.data(), vtx_xyz.data());
      c[0] -= target_screen_position.x;
      c[1] -= target_screen_position.y;
      for (int idim = 0; idim < 3; ++idim) {
        for (int jdim = 0; jdim < 3; ++jdim) {
          matrix_lm(idim, jdim) += dc[0][idim] * dc[0][jdim] + dc[1][idim] * dc[1][jdim];
        }
        resvec_lm(idim) += c[0] * dc[0][idim] + c[1] * dc[1][idim];
      }
    }
  }
  {
    double stiff = 0.1;
    auto near_surface_mesh = delfem2::Nearest_Point_MeshTri3D(
        dfm2::CVec3d(vtx_xyz.data()),
        vtx_xyz_head, tri_vtx_head);
    dfm2::CVec3d trg = near_surface_mesh.PositionOnMeshTri3(vtx_xyz_head, tri_vtx_head);
    auto diff = dfm2::CVec3d(vtx_xyz.data()) - trg;
    resvec_lm(0) += diff.x * stiff;
    resvec_lm(1) += diff.y * stiff;
    resvec_lm(2) += diff.z * stiff;
    for (int i = 0; i < 3; ++i) {
      matrix_lm(i, i) += stiff;
    }
  }

  for (unsigned int i = 0; i < 3; ++i) {
    matrix_lm(i, i) += 1.0e-5;
  }

  Eigen::FullPivLU<Eigen::MatrixXd> lu(matrix_lm);
  Eigen::VectorXd solvec_lm = lu.solve(resvec_lm);
  vtx_xyz[0] -= static_cast<float>(solvec_lm(0));
  vtx_xyz[1] -= static_cast<float>(solvec_lm(1));
  vtx_xyz[2] -= static_cast<float>(solvec_lm(2));
}


#endif //OPTIMIZE_HAIR_ROOT_H_
