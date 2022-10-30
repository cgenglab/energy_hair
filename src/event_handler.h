//
// Created by Nobuyuki Umetani on 2022/01/05.
//

#ifndef EVENT_HANDLER_H_
#define EVENT_HANDLER_H_

#include "delfem2/polyline_elastic_rod2.h"

#include "object_redo_undo.h"

void EventHandler(
    std::vector<Hair> &hairs,
    MyViewer &viewer,
    delfem2::LinearSystemSolver_BlockPentaDiagonal<2> &linsys_for_stroke,
    const SimulationParam &sim_param,
    const delfem2::AdaptiveDistanceField3 &adf,
    const HeadMesh &head) {
  namespace dfm2 = delfem2;
  // sketch smooth
  while (viewer.action_sketch_smooth) {  // use while to avoid nested "if" statement
    viewer.action_sketch_smooth = false;
    if (viewer.idx_hair >= hairs.size()) { break; }
    auto &camst = hairs[viewer.idx_hair].camera_stroke;
    if (camst.find(viewer.idx_camera) == camst.end()) { break; }
    const auto[mat_modelview, mat_projection] = CameraTransformationMVP(
        viewer.cam_config, viewer.idx_camera);
    const delfem2::CMat4f mat_mvp = mat_projection * mat_modelview;
    PhysicallySmooth_Polyline(
        camst[viewer.idx_camera].xys,
        linsys_for_stroke,
        hairs[viewer.idx_hair].vtx_xyz,
        mat_mvp,
        viewer.stroke_edge_length,
        0.001);
    hairs[viewer.idx_hair].is_converged = false;
    camst[viewer.idx_camera].rbf.SetPolyline2(camst[viewer.idx_camera].xys, 0.001);
  }
  // sketch drag
  while (viewer.action_sketch_drag) {  // use while to avoid nested "if" statement
    viewer.action_sketch_drag = false;
    if (viewer.idx_hair >= hairs.size()) { break; }
    auto &camst = hairs[viewer.idx_hair].camera_stroke;
    if (camst.find(viewer.idx_camera) == camst.end()) { break; }
    DragPolylineElastic_Rod2(
        camst[viewer.idx_camera].xys, linsys_for_stroke,
        viewer.idx_sketchvtx, {viewer.nav.mouse_x, viewer.nav.mouse_y},
        viewer.stroke_edge_length, 0.001);
    hairs[viewer.idx_hair].is_converged = false;
  }
  while (viewer.action_hair_drag) {
    viewer.action_hair_drag = false;
    if (viewer.idx_hair >= hairs.size()) { break; }
    if (viewer.idx_camera >= viewer.cam_config.aCamera.size()) { break; }
    if (viewer.ipart_hair == 1 ) {
      const auto[mmv, mp] = CameraTransformationMVP(viewer.cam_config, viewer.idx_camera);
      const dfm2::CMat4f mat_mvp_inv = (mp * mmv).Inverse();
      std::map<double, dfm2::PointOnSurfaceMesh<double>> mapDepthPES;
      {
        const float scrx = viewer.nav.mouse_x;
        const float scry = viewer.nav.mouse_y;
        const dfm2::CVec3f ps = mat_mvp_inv.MultVec3_Homography(dfm2::CVec3f{scrx, scry, +1.f}.data());
        const dfm2::CVec3f pe = mat_mvp_inv.MultVec3_Homography(dfm2::CVec3f{scrx, scry, -1.f}.data());
        dfm2::IntersectionRay_MeshTri3(
            mapDepthPES,
            ps.cast<double>(),
            (pe - ps).cast<double>(),
            head.tri_vtx_xyz, head.vtx_xyz,
            0.001);
      }
      if (mapDepthPES.empty()) { break; }
      Hair &hair = hairs[viewer.idx_hair];
      hair.root_on_mesh = mapDepthPES.begin()->second;
      hair.root_normal = hair.root_on_mesh.UnitNormalOnMeshTri3(head.vtx_nrm, head.tri_vtx_xyz);
      delfem2::CVec3d pos_new = hair.root_on_mesh.PositionOnMeshTri3(head.vtx_xyz, head.tri_vtx_xyz);
      delfem2::CVec3d pos_dlt = pos_new - hair.root_pos;
      hair.root_pos = pos_new;
      for (unsigned int ivtx = 0; ivtx < hair.vtx_xyz.size() / 3; ++ivtx) {
        hair.vtx_xyz[ivtx * 3 + 0] += pos_dlt.x;
        hair.vtx_xyz[ivtx * 3 + 1] += pos_dlt.y;
        hair.vtx_xyz[ivtx * 3 + 2] += pos_dlt.z;
      }
    }
  }
  // deform hair
  for (auto &hair: hairs) {
    if (!viewer.is_animation) { continue; }
    double upd = DeformHair(
        hair,
        sim_param, adf, viewer.cam_config);
    hair.is_converged = (upd < 1.0e-5);
  }
  // automatic adjust length
  if (viewer.idx_hair < hairs.size()) {  // hair selected
    if (hairs[viewer.idx_hair].is_converged) {
      hairs[viewer.idx_hair].is_converged = false;
      AdjustNumberOfSegmentsInHair(
          hairs[viewer.idx_hair],
          viewer.cam_config);
    }
  }
  RelaxAndPropagateHairRootOrientation(hairs);
}

#endif EVENT_HANDLER_H_
