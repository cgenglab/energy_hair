//
// Created by Nobuyuki Umetani on 2021-09-23.
//

#ifndef OPTIMIZE_LENGTH_H_
#define OPTIMIZE_LENGTH_H_

#include "delfem2/geo_polyline.h"

void OptimizeLength(
    std::vector<float> &vtx_xyz,
    const CameraConfig &cam_config,
    const std::map<unsigned int,Stroke> &camera_stroke) {
  namespace dfm2 = delfem2;
  const unsigned int nvtx = vtx_xyz.size() / 3;
  if (nvtx < 2) { return; }
  for (unsigned int icam = 0; icam < cam_config.aCamera.size(); ++icam) {
    using V2 = dfm2::CVec2f;
    if (camera_stroke.find(icam) == camera_stroke.end()) { continue; }
    const std::vector<V2> &polyline = camera_stroke.find(icam)->second.xys;
    if (polyline.empty()) { continue; }
    //
    const auto[mat_modelview, mat_projection] = CameraTransformationMVP(cam_config, icam);
    const dfm2::CMat4f mat_mvp = mat_projection * mat_modelview;

    float polyline_length = dfm2::Length_Polyline<V2>(polyline);

    float al[2];
    for(unsigned int iivtx=0;iivtx<2;++iivtx) {
      unsigned int ivtx = nvtx-2+iivtx;
      const V2 scr = dfm2::Vec2_Mat4Vec3_Homography(
          mat_mvp.data(), vtx_xyz.data() + ivtx * 3);
      float alen = dfm2::ArcLengthPointInPolyline(polyline, scr);
      al[iivtx] = alen;
    }
    if( 2*al[1]-al[0] < 0.95*polyline_length ){
      float xnew = vtx_xyz[(nvtx-1)*3+0]*2 - vtx_xyz[(nvtx-2)*3+0];
      float ynew = vtx_xyz[(nvtx-1)*3+1]*2 - vtx_xyz[(nvtx-2)*3+1];
      float znew = vtx_xyz[(nvtx-1)*3+2]*2 - vtx_xyz[(nvtx-2)*3+2];
      vtx_xyz.push_back(xnew);
      vtx_xyz.push_back(ynew);
      vtx_xyz.push_back(znew);
    }
    break;
  }
}

#endif //OPTIMIZE_LENGTH_H_
