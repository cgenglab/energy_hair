//
// Created by Nobuyuki Umetani on 2021-09-12.
//

#ifndef VIEWERSKETCHOPTIMIZATION_H
#define VIEWERSKETCHOPTIMIZATION_H

#include <map>
#include <sstream>
#include <utility>
#include <vector>
#include <filesystem>
#include <array>
#include <climits>

#include "delfem2/cvcamera.h"
#include "delfem2/geo3_v23m34q.h"
#include "delfem2/vec2.h"
#include "delfem2/mshmisc.h"
#include "delfem2/geo_polyline.h"
#include "delfem2/geo_tri.h"
#include "delfem2/glfw/viewer3.h"
#include "delfem2/openglstb/img2tex.h"

// this file does not refer head.h
#include "camera_projection.h"
#include "hair.h"

class MyViewer :
    public delfem2::glfw::CViewer3 {
 public:
  explicit MyViewer(
      std::vector<Hair> &hairs,
      std::filesystem::path path_xml,
      std::string image_folder_name) :
      hairs(hairs), cam_config_path_xml(std::move(path_xml)),
      image_folder_name(std::move(image_folder_name)) {
    cam_config.ReadXML(cam_config_path_xml.string().c_str());
    for (idx_camera = 0; idx_camera < cam_config.aCamera.size(); ++idx_camera) {
      if (cam_config.aCamera[idx_camera].id_sensor == -1) { continue; }
      break;
    }
    bgcolor[0] = 0.8;
    ResetOperation();
  }

  void OpenWindow() override {
    CViewer3::OpenWindow();
  }

  void key_press(int key, [[maybe_unused]] int mods) override {
    if (key == GLFW_KEY_A) {
      is_animation = !is_animation;
    }
    if (key == GLFW_KEY_T) {
      is_draw_texture = !is_draw_texture;
    }
    if (key == GLFW_KEY_W) {
      is_draw_wireframe = !is_draw_wireframe;
    }
    if (key == GLFW_KEY_I) {
      action_initialize = true;
    }
    if (key == GLFW_KEY_SPACE) {  //
      if (idx_hair >= hairs.size()) { return; }
      const auto &cs = hairs[idx_hair].camera_stroke;
      if (cs.empty()) { return; }
      if (cs.find(idx_camera) == cs.end()) {
        SetCamera(cs.begin()->first);
        return;
      }
      auto itr = cs.find(idx_camera);
      auto itr1 = ++itr;
      if (itr1 != cs.end()) {
        SetCamera(itr1->first);
        return;
      }
      SetCamera(cs.begin()->first);
      return;
    }
    // switch character
    if (key == GLFW_KEY_1) {
      next_character = 0;
    } else if (key == GLFW_KEY_2) {
      next_character = 1;
    } else if (key == GLFW_KEY_3) {
      next_character = 2;
    } else if (key == GLFW_KEY_4) {
      next_character = 3;
    }
    // gui operation
    if (nav.imodifier == GLFW_MOD_CONTROL) {
      if (key == GLFW_KEY_O) {
        is_open_operation = true;
        return;
      } else if (key == GLFW_KEY_S) {
        is_save_operation = true;
        return;
      } else if (key == GLFW_KEY_E) {
        is_export_operation = true;
        return;
      } else if (key == GLFW_KEY_Z) {
        is_undo_operation = true;
        return;
      } else if (key == GLFW_KEY_Y) {
        is_redo_operation = true;
        return;
      }
    }
  }

  void mouse_press(
      [[maybe_unused]] const float src[3],
      [[maybe_unused]] const float dir[3]) override {
    namespace dfm2 = delfem2;
#ifdef IMGUI_VERSION
    if (ImGui::GetIO().WantCaptureMouse) { return; }
#endif
    if (idx_camera >= cam_config.aCamera.size()) { return; }
    // need selected camera below
    if (nav.imodifier == 0) { // no modifier: pick new hair
      const delfem2::CMat4f mp = this->GetProjectionMatrix();
      const delfem2::CMat4f mmv = this->GetModelViewMatrix();
      const delfem2::CVec2f q0 = (mp * mmv).MultVec3_Homography(src).data(); // mouse position
      if (idx_hair < hairs.size()) { // check currently selected strand
        const auto p0 = delfem2::CVec3f(hairs[idx_hair].vtx_xyz.data());
        const delfem2::CVec2f q1 = (mp * mmv).MultVec3_Homography(p0.data()).data(); // mouse position
        if ((q0 - q1).norm() < 0.03) {
          this->ipart_hair = 1;
          return;
        }
      }
      { // pick new strand
        const auto[idx_hair0, param_hair0, dist_hair0] = PickHair(mp * mmv, hairs, q0);
        this->idx_hair = UINT_MAX;
        if (idx_hair0 >= hairs.size() || dist_hair0 > 0.03) { return; }
        this->idx_hair = idx_hair0; // new pick
      }
      const auto p0 = delfem2::CVec3f(hairs[idx_hair].vtx_xyz.data());
      const delfem2::CVec2f q1 = (mp * mmv).MultVec3_Homography(p0.data()).data(); // mouse position
      if ((q0 - q1).norm() < 0.03) { this->ipart_hair = 1; }
      else { this->ipart_hair = 0; }
      return;
    }
    if (idx_hair >= hairs.size()) { return; }
    // need selected hair below
    if (nav.imodifier == GLFW_MOD_CONTROL) {  // edit sketch
      if (nav.ibutton == GLFW_MOUSE_BUTTON_LEFT) {  // new sketch
        std::cout << "sketch edit: begin sketch" << std::endl;
        hairs[idx_hair].camera_stroke[idx_camera].xys.clear();
        hairs[idx_hair].camera_stroke[idx_camera].xys.emplace_back(
            this->nav.mouse_x, this->nav.mouse_y);
        hairs[idx_hair].is_converged = false;
      } else if (nav.ibutton == GLFW_MOUSE_BUTTON_MIDDLE) {
        idx_sketchvtx = UINT_MAX;
      } else if (nav.ibutton == GLFW_MOUSE_BUTTON_RIGHT) {  // drag sketch
        idx_sketchvtx = UINT_MAX;
        const auto &xys = hairs[idx_hair].camera_stroke[idx_camera].xys;
        if (xys.empty()) {
          return;
        }
        const auto scr0 = delfem2::CVec2d(nav.mouse_x, nav.mouse_y).cast<float>();
        const unsigned int ivtx0 = delfem2::FindNearestPointInPoints(xys, scr0);
        if ((scr0 - xys[ivtx0]).norm() < 0.1) {
          idx_sketchvtx = ivtx0;
        }
      }
    }
  }

  void mouse_drag(
      [[maybe_unused]] const float src0[3],
      [[maybe_unused]] const float src1[3],
      [[maybe_unused]] const float dir[3]) override {
    if (idx_camera >= cam_config.aCamera.size()) { return; }
    if (idx_hair >= hairs.size()) { return; }
    if (nav.imodifier == 0) { // no modifier
      if (nav.ibutton == GLFW_MOUSE_BUTTON_LEFT) { // drag sketch
        action_hair_drag = true;
      }
    }
    // below: sketch edit
    if (hairs[idx_hair].camera_stroke.find(idx_camera) == hairs[idx_hair].camera_stroke.end()) { return; }
    else if (nav.imodifier == GLFW_MOD_CONTROL) {  // during sketch edit
      if (nav.ibutton == GLFW_MOUSE_BUTTON_LEFT) { // new sketch
        auto &xys = hairs[idx_hair].camera_stroke[idx_camera].xys;
        assert(!xys.empty());
        delfem2::CVec2f p0 = xys[xys.size() - 1];
        delfem2::CVec2f p1 = delfem2::CVec2d(this->nav.mouse_x, this->nav.mouse_y).cast<float>();
        if ((p0 - p1).norm() < stroke_edge_length * 0.5) { return; }
        hairs[idx_hair].camera_stroke[idx_camera].xys.emplace_back(p1);
        hairs[idx_hair].is_converged = false;
      } else if (nav.ibutton == GLFW_MOUSE_BUTTON_MIDDLE) {  // smooth sketch
        action_sketch_smooth = false;
        idx_sketchvtx = UINT_MAX;
        const auto &xys = hairs[idx_hair].camera_stroke[idx_camera].xys;
        if (xys.empty()) {
          return;
        }
        const auto scr0 = delfem2::CVec2d(nav.mouse_x, nav.mouse_y).cast<float>();
        const unsigned int ivtx0 = delfem2::FindNearestPointInPoints(xys, scr0);
        if ((scr0 - xys[ivtx0]).norm() > 0.1) { return; }
        idx_sketchvtx = ivtx0;
        action_sketch_smooth = true;
      } else if (nav.ibutton == GLFW_MOUSE_BUTTON_RIGHT) {  // drag sketch
        auto &xys = hairs[idx_hair].camera_stroke[idx_camera].xys;
        if (idx_sketchvtx >= xys.size()) { return; }
        action_sketch_drag = true;
      }
    }
  }

  void mouse_release() override {
#ifdef IMGUI_VERSION
    if (ImGui::GetIO().WantCaptureMouse) { return; }
#endif
    if (nav.imodifier == GLFW_MOD_ALT || nav.imodifier == GLFW_MOD_SHIFT) {
      const unsigned int icam_best = SnapCamera(
          this->GetModelViewMatrix(),
          this->cam_config);
      SetCamera(icam_best);
    }
    if (idx_camera >= cam_config.aCamera.size()) { return; }
    if (idx_hair >= hairs.size()) { return; }
    action_release_mouse = true;
    if (nav.imodifier == GLFW_MOD_CONTROL) { // end sketch edit
      if (nav.ibutton == GLFW_MOUSE_BUTTON_LEFT) { // new sketch
        if (hairs[idx_hair].camera_stroke[idx_camera].xys.size() <= 2) {
          hairs[idx_hair].camera_stroke.erase(idx_camera);
        } else {
          auto &stroke = hairs[idx_hair].camera_stroke[idx_camera];
          stroke.xys = delfem2::Polyline_Resample_Polyline(
              stroke.xys, static_cast<float>(stroke_edge_length));
          hairs[idx_hair].is_converged = false;
        }
      } else if (nav.ibutton == GLFW_MOUSE_BUTTON_MIDDLE) {  // smooth sketch
      } else if (nav.ibutton == GLFW_MOUSE_BUTTON_RIGHT) {  // drag sketch
        // pick sketch
      }
    }
  }

  void mouse_wheel(double) override {
    idx_camera = UINT_MAX;
    ::glfwSetWindowTitle(this->window, "free view");
  }

  void CursorPosition(double xpos, double ypos) override {
#ifdef IMGUI_VERSION
    if (ImGui::GetIO().WantCaptureMouse) { return; }
#endif
    ::delfem2::glfw::CViewer3::CursorPosition(xpos, ypos);
    if (this->nav.ibutton == GLFW_MOUSE_BUTTON_LEFT &&
        (nav.imodifier == GLFW_MOD_ALT || nav.imodifier == GLFW_MOD_SHIFT)) {
      idx_camera = UINT_MAX;
      ::glfwSetWindowTitle(this->window, "free view");
    }
  }

  void SetCamera(unsigned int idx_camera_) {
    this->idx_camera = idx_camera_ % cam_config.aCamera.size();
    SetCameraToViewer(
        *this,
        idx_camera, cam_config);
    // -----------------------
    if (idx_camera != idx_camera_texture) {  // set background image
      std::filesystem::path path_img = cam_config_path_xml.parent_path()
          / image_folder_name
          / (cam_config.aCamera[idx_camera].label + ".png");
      if (std::filesystem::exists(path_img)) {
        idx_camera_texture = idx_camera;
        if (!glIsTexture(id_texture)) { glGenTextures(1, &id_texture); }
        glBindTexture(GL_TEXTURE_2D, id_texture);
        auto imgshape = delfem2::openglstb::LoadImageFileSetToTexture(
            path_img.string().c_str());
        std::cout << cam_config.aCamera[idx_camera].label << " ";
        std::cout << std::get<0>(imgshape) << " ";
        std::cout << std::get<1>(imgshape) << " ";
        std::cout << std::get<2>(imgshape) << std::endl;
      }
    }
  }

  void ResetOperation() {
    action_initialize = false;
    action_sketch_smooth = false;
    action_sketch_drag = false;
    action_hair_drag = false;
    is_open_operation = false;
    is_undo_operation = false;
    is_redo_operation = false;
    is_save_operation = false;
    is_export_operation = false;
  }

 public:
  // switch and actions
  bool is_animation = true;
  bool is_draw_texture = true;
  bool is_draw_wireframe = false;
  bool action_initialize = false;
  bool action_sketch_smooth = false;
  bool action_sketch_drag = false;
  bool action_hair_drag = false;
  bool action_release_mouse = false;

  // parameters for shortcut-key operation 
  bool is_open_operation = false;
  bool is_undo_operation = false;
  bool is_redo_operation = false;
  bool is_save_operation = false;
  bool is_export_operation = false;

  // hair
  unsigned int idx_hair = UINT_MAX;  // index of current hair
  unsigned int ipart_hair = 0; // 0: no_part, 1: root, 2: body, 3: tip
  std::vector<Hair> &hairs;  // array of hair

  // camera
  std::filesystem::path cam_config_path_xml;
  CameraConfig cam_config{};  // camera configuration
  unsigned int idx_camera = 0;  // index of current camera

  // sketch
  unsigned int idx_sketchvtx = UINT_MAX;
  const double stroke_edge_length = 0.03;

  // background image
  unsigned int id_texture = 0;  // id of texture for background image
  std::string image_folder_name;  // path for the background image
  unsigned int idx_camera_texture = UINT_MAX;  // index of camera which current texture has the image

  // command
  unsigned int next_character = -1;
};

#endif //VIEWERSKETCHOPTIMIZATION_H
