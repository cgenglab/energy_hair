//
// Created by Nobuyuki Umetani on 2021-10-01.
//

#ifndef GENERATE_INITIAL_HAIR_H_
#define GENERATE_INITIAL_HAIR_H_

#include "delfem2/geo_polyline.h"
#include "delfem2/srchuni_v3.h"
#include "delfem2/vec2.h"
#include "delfem2/mat4.h"

// this file does not refer to other header files

delfem2::PointOnSurfaceMesh<double> GenerateInitialHair(
    std::vector<float> &vtx_xyz_hair,
    double edge_length,
    const std::vector<delfem2::CVec2f> &stroke,
    const delfem2::CMat4f &mat_mvp,
    const std::vector<double> &vtx_xyz_head,
    const std::vector<unsigned int> &tri_vtx_head){
  namespace dfm2 = delfem2;
  assert(stroke.size()>=4);
  const dfm2::CMat4f mat_mvp_inv = mat_mvp.Inverse();
  std::map<double, dfm2::PointOnSurfaceMesh<double>> mapDepthPES;
  {
    const dfm2::CVec3f ps = mat_mvp_inv.MultVec3_Homography(dfm2::CVec3f{stroke[0].x, stroke[0].y, +1.f}.data());
    const dfm2::CVec3f pe = mat_mvp_inv.MultVec3_Homography(dfm2::CVec3f{stroke[0].x, stroke[0].y, -1.f}.data());
    dfm2::IntersectionRay_MeshTri3(
        mapDepthPES,
        ps.cast<double>(),
        (pe - ps).cast<double>(),
        tri_vtx_head, vtx_xyz_head,
        0.001);
  }
  if( mapDepthPES.empty() ){ return dfm2::PointOnSurfaceMesh<double>{}; }
  //
  const dfm2::PointOnSurfaceMesh<double> pes = mapDepthPES.begin()->second;
  const dfm2::CVec3d hit_point = pes.PositionOnMeshTri3(vtx_xyz_head, tri_vtx_head);
  const dfm2::CVec3f hit_point_device = mat_mvp.MultVec3_Homography(hit_point.cast<float>().data());
  assert( fabs(hit_point_device.x - stroke[0].x) < 1.0e-3 );
  assert( fabs(hit_point_device.y - stroke[0].y) < 1.0e-3 );
  const float device_depth = hit_point_device.z;
  std::vector<dfm2::CVec3f> hair0;
  for(auto stroke_point : stroke){
    dfm2::CVec3f p0 = mat_mvp_inv.MultVec3_Homography(
        dfm2::CVec3f{stroke_point.x, stroke_point.y, device_depth}.data());
    hair0.push_back(p0);
  }
  std::vector<dfm2::CVec3f> hair1 = dfm2::Polyline_Resample_Polyline(hair0, edge_length);
  vtx_xyz_hair.clear();
  for(const auto& p : hair1){
    vtx_xyz_hair.push_back(p.x);
    vtx_xyz_hair.push_back(p.y);
    vtx_xyz_hair.push_back(p.z);
  }
  return pes;
}


#endif // GENERATE_INITIAL_HAIR_H_
