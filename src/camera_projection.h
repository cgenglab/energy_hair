//
// Created by Nobuyuki Umetani on 2021-10-11.
//

#ifndef CAMERA_PROJECTION_H_
#define CAMERA_PROJECTION_H_

#include "delfem2/cvcamera.h"
#include "delfem2/cam_projection.h"
#include "delfem2/cam_modelview.h"
#include "delfem2/glfw/viewer3.h"

#include "camera_cofing.h"

std::pair<delfem2::CMat4f,delfem2::CMat4f>
CameraTransformationMVP(
    const CameraConfig& cam_config,
    unsigned int idx_camera) {
  assert(idx_camera < cam_config.aCamera.size());
  const CameraConfig::Camera &cam = cam_config.aCamera[idx_camera];
  assert(cam.id_sensor>=0 && cam.id_sensor < static_cast<int>(cam_config.aSensor.size()));
  const CameraConfig::Sensor &snsr = cam_config.aSensor[cam.id_sensor];
  const auto width0 = static_cast<float>(snsr.width);
  const auto height0 = static_cast<float>(snsr.height);
  const delfem2::CMat4f glb2img(
      delfem2::Mat4_CameraInternal_MetashapePinhole(
          snsr.f, snsr.cx, snsr.cy, width0, height0).data());
  const delfem2::CMat4f img2scr(
      delfem2::Mat4_Image2Screen(
          width0, height0, -0.1).data());
  delfem2::CMat4f zinv(
      1, 0, 0, 0,
      0, -1, 0, 0,
      0, 0, -1, 0,
      0, 0, 0, 1);
  return {zinv*cam.transform.Inverse(), img2scr * glb2img * zinv};
//  return {cam.transform.Inverse(), img2scr * glb2img};
}

class Projection_FrameCamera : public delfem2::Projection {
 public:
  Projection_FrameCamera(
      int width0, int height0,
      float f, float cx, float cy,
      float depth) {
    const auto wf = static_cast<float>(width0);
    const auto hf = static_cast<float>(height0);
    const delfem2::CMat4f mT = delfem2::CMat4f::Translation({0.f, 0.f, depth});
    const delfem2::CMat4f glb2img =
        delfem2::Mat4_CameraInternal_MetashapePinhole(f, cx, cy, wf, hf);
    const delfem2::CMat4f img2scr =
        delfem2::Mat4_Image2Screen(wf, hf, -1);
    const delfem2::CMat4f zyinv = delfem2::CMat4f::ScaleXYZ(1,-1,-1);
    const delfem2::CMat4f zinv = delfem2::CMat4f::ScaleXYZ(1,1,-1);
    mM = zinv * img2scr * glb2img * zyinv * mT;
  }

  [[nodiscard]] std::array<float, 16> GetMatrix([[maybe_unused]] float asp) const override {
    std::array<float, 16> m{};
    mM.CopyTo(m.data());
    return m;
  }
 public:
  delfem2::CMat4f mM;
};

void SetCameraToViewer(
    delfem2::glfw::CViewer3& viewer,
    unsigned int idx_camera,
    const CameraConfig& cam_config)
{
  assert(idx_camera<cam_config.aCamera.size());
  const CameraConfig::Camera &cam = cam_config.aCamera[idx_camera];
  assert(cam.id_sensor>=0 && cam.id_sensor < static_cast<int>(cam_config.aSensor.size()));
  const CameraConfig::Sensor &snsr = cam_config.aSensor[cam.id_sensor];
  static auto camera_width = viewer.width = snsr.width / 8;
  static auto camera_height = viewer.height = snsr.height / 8;

  ::glfwSetWindowSizeCallback(
      viewer.window, [](GLFWwindow* window, int width, int height) {
      camera_width = width;
      camera_height = height;
      });
  ::glfwSetWindowSize(
      viewer.window,
      static_cast<int>(camera_width),
      static_cast<int>(camera_height));
  ::glfwSetWindowTitle(
      viewer.window,
      (std::string("camera:") + std::to_string(idx_camera)).c_str());

  const auto[mat_modelview, mat_projection]
  = CameraTransformationMVP(cam_config, idx_camera);

  viewer.scale = 1.0;

  float view_height = cam_config.region_size.norm();
  float depth = view_height * snsr.f / static_cast<float>(snsr.height);

  {  // set modelview matrix
    auto *pmv = dynamic_cast<delfem2::ModelView_Trackball *>(viewer.view_rotation.get());
    const delfem2::CMat3f m3 = mat_modelview.GetMat3();
    const delfem2::CVec3f cnt = cam_config.region_center;
    const delfem2::CVec3f tmp1 = m3.MatVec(cnt.data());
    const auto q1 = m3.GetQuaternion();
    delfem2::Copy_Quat(pmv->quaternion, q1.data());
    pmv->anchor[0] = cnt.x;
    pmv->anchor[1] = cnt.y;
    pmv->anchor[2] = cnt.z;
    viewer.trans[0] = tmp1[0] + mat_modelview(0, 3);
    viewer.trans[1] = tmp1[1] + mat_modelview(1, 3);
    viewer.trans[2] = tmp1[2] + mat_modelview(2, 3) + depth;
  }

  { // set projection matrix
    const CameraConfig::Camera &cam0 = cam_config.aCamera[idx_camera];
    const CameraConfig::Sensor &snsr0 = cam_config.aSensor[cam0.id_sensor];
    const auto width0 = static_cast<float>(snsr0.width);
    const auto height0 = static_cast<float>(snsr0.height);
    viewer.projection = std::make_unique<Projection_FrameCamera>(
        width0, height0,
        snsr0.f, snsr0.cx, snsr0.cy,
        -depth);
  }
}

unsigned int SnapCamera(
  const delfem2::CMat4f& model_view_matrix,
  const CameraConfig& cam_config){
  delfem2::CMat4f mv0 = model_view_matrix; // GetModelViewMatrix();
  delfem2::CVec3f dir0 = mv0.Inverse().MultVec3(std::array<float, 3>{0, 0, 1}.data());
  const float phi0 = std::asin(dir0.y); // [-pi/2, +pi/2]
  const float theta0 = std::atan2(dir0.x, dir0.z);  // [-pi,+pi]
  unsigned int icam_best = UINT_MAX;
  double dist_best = -1;
  for (unsigned int icam = 0; icam < cam_config.aCamera.size(); ++icam) {
    {  // skip inactive camera
      const int idx_sensor = cam_config.aCamera[icam].id_sensor;
      if(idx_sensor < 0 || idx_sensor >= static_cast<int>(cam_config.aSensor.size())){
        continue;
      }
    }
    auto[mv1, mp] = CameraTransformationMVP(cam_config, icam);
    delfem2::CVec3f dir1 = mv1.Inverse().MultVec3(std::array<float, 3>{0, 0, 1}.data());
    const float phi1 = std::asin(dir1.y); // [-pi/2, +pi/2]
    const float theta1 = std::atan2(dir1.x, dir1.z);  // [-pi,+pi]
    double dt0 = theta0 - theta1;
    double dt1 = theta0 - theta1 + 2 * M_PI;
    double dt2 = theta0 - theta1 - 2 * M_PI;
    double dist0 = (phi0 - phi1) * (phi0 - phi1) + dt0 * dt0;
    double dist1 = (phi0 - phi1) * (phi0 - phi1) + dt1 * dt1;
    double dist2 = (phi0 - phi1) * (phi0 - phi1) + dt2 * dt2;
    if (dist0 < dist_best || dist_best < 0) {
      icam_best = icam;
      dist_best = dist0;
    }
    if (dist1 < dist_best || dist_best < 0) {
      icam_best = icam;
      dist_best = dist1;
    }
    if (dist2 < dist_best || dist_best < 0) {
      icam_best = icam;
      dist_best = dist2;
    }
  }
  return icam_best;
}


#endif //CAMERA_PROJECTION_H_
