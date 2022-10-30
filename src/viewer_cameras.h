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
#include "delfem2/msh_io_obj.h"
#include "delfem2/geo_polyline.h"
#include "delfem2/geo_tri.h"
#include "delfem2/glfw/viewer3.h"
#include "delfem2/openglstb/img2tex.h"

// this file does not refer head.h
#include "camera_projection.h"
#include "hair.h"
#include "camera_control.h"

class MyViewer :
    public delfem2::glfw::CViewer3 {
 public:
  explicit MyViewer(
      std::vector<Hair> &hairs,
      std::filesystem::path  path_xml,
      std::string image_folder_name) :
      hairs(hairs), camconfig_path_xml(std::move(path_xml)),
      image_folder_name(std::move(image_folder_name)) {
    cam_config.ReadXML(camconfig_path_xml.string().c_str());
    this->idx_camera = 0;
    bgcolor[0] = 0.8;
  }

  bool LoadCameraControl(const std::filesystem::path& path_xml) {
    return cam_control.ReadXML(path_xml.string());
  }

  void OpenWindow() override {
    CViewer3::OpenWindow();
  }

  void key_press(int key, [[maybe_unused]] int mods) override {
    if (cam_control.valid_camera_control) {
      if (key == GLFW_KEY_UP || key == GLFW_KEY_DOWN || key == GLFW_KEY_RIGHT || key == GLFW_KEY_LEFT) {
          auto current_idx_camera = cam_control.GetCurrentIndex();
          auto next_idx_camera = current_idx_camera;
          if (key == GLFW_KEY_UP) {
              next_idx_camera = cam_control.MoveUp();
          }
          if (key == GLFW_KEY_DOWN) {
              next_idx_camera = cam_control.MoveDown();
          }
          if (key == GLFW_KEY_LEFT) {
              next_idx_camera = cam_control.MoveLeft();
          }
          if (key == GLFW_KEY_RIGHT) {
              next_idx_camera = cam_control.MoveRight();
          }
          // ---------------
          if (next_idx_camera != current_idx_camera) {
            SetCamera(next_idx_camera);
          }
      }
    } else {
      if (key == GLFW_KEY_UP || key == GLFW_KEY_DOWN) {
        if (key == GLFW_KEY_UP) {
            SetCamera(idx_camera + 1);
        }
        if (key == GLFW_KEY_DOWN) {
            const auto num_camera = static_cast<unsigned int>(cam_config.aCamera.size());
            SetCamera(idx_camera + num_camera - 1);
        }
      }
    }
    if (key == GLFW_KEY_O ) {
      is_animation = !is_animation;
    }
    if (key == GLFW_KEY_T ) {
      is_draw_texture = !is_draw_texture;
    }
    if (key == GLFW_KEY_W) {
      is_wireframe = !is_wireframe;
    }
    if( key == GLFW_KEY_D ){
      mode_edit = MODE_DELETE;
    }
    if ( key == GLFW_KEY_S) {
        is_save_operation = true;
    }
    if (key == GLFW_KEY_Z) {
        is_undo_operation = true;
    }
    if (key == GLFW_KEY_Y) {
        is_redo_operation = true;
    }
    if (key == GLFW_KEY_O) {
        is_open_operation = true;
    }
    if (key == GLFW_KEY_E) {
        is_export_operation = true;
    }
  }

  void mouse_press(
      [[maybe_unused]] const float src[3],
      [[maybe_unused]] const float dir[3]) override {
    namespace dfm2 = delfem2;
#ifdef IMGUI_VERSION
    if(  ImGui::GetIO().WantCaptureMouse ){ return; }
#endif
    if( idx_camera >= cam_config.aCamera.size() ){ return; }
    {
      const delfem2::CMat4f mp = this->GetProjectionMatrix();
      const delfem2::CMat4f mmv = this->GetModelViewMatrix();
      const delfem2::CVec2f q0 = (mp * mmv).MultVec3_Homography(src).data();
      const auto [idx_hair0, param_hair0, dist0] = PickHair(mp * mmv, hairs, q0);
      if( idx_hair0 != UINT_MAX && dist0 < 0.003 ){
        std::cout << idx_hair0 << " " << dist0 << std::endl;
        idx_hair = idx_hair0;
        if( mode_edit == MODE_DELETE ){
          hairs.erase(hairs.begin()+idx_hair);
          idx_hair = UINT_MAX;
          return;
        }
        else {
          mode_edit = MODE_DEFORM;
          return;
        }
      }
    }
    mode_edit = MODE_NEW;
    idx_hair = hairs.size();
    hairs.resize(hairs.size()+1);
    hairs[idx_hair].camera_stroke.insert( std::make_pair(idx_camera,Stroke()) );
  }

  void mouse_drag(
      [[maybe_unused]] const float src0[3],
      [[maybe_unused]] const float src1[3],
      [[maybe_unused]] const float dir[3]) override {
    if( idx_camera >= cam_config.aCamera.size() ){ return; }
    if( idx_hair >= hairs.size() ){ return; }
    if( mode_edit == MODE_NEW ) {
      auto &camera_sroke = hairs[idx_hair].camera_stroke;
      if (camera_sroke.find(idx_camera) == camera_sroke.end()) { return; }
      camera_sroke[idx_camera].xys.emplace_back(
          this->nav.mouse_x, this->nav.mouse_y);
    }
  }

  void mouse_release() override {
#ifdef IMGUI_VERSION
    if(  ImGui::GetIO().WantCaptureMouse ){ return; }
#endif
    if ( (nav.imodifier == GLFW_MOD_ALT || nav.imodifier == GLFW_MOD_SHIFT) && cam_control.has_camera_position && is_auto_camera_detection) {
      // find nearest camera view
      float* mat4_modelview = GetModelViewMatrix().data();
      assert(mat4_modelview);
      auto find_nearest_camera_id_x = UINT_MAX;
      auto find_nearest_camera_id_y = UINT_MAX;
      auto find_nearest_camera_id = UINT_MAX;
      auto nearest_distance_camera = 1.0e+10;
      for (auto ny = 0; ny < cam_control.height; ny++) {
        for (auto nx = 0; nx < cam_control.width; nx++) {
          float tmp_distance = 0.0f;
          auto idx = nx + ny * cam_control.width;
          tmp_distance += (cam_positions[idx].x - mat4_modelview[3]) * (cam_positions[idx].x - mat4_modelview[3]);
          tmp_distance += (cam_positions[idx].y - mat4_modelview[7]) * (cam_positions[idx].y - mat4_modelview[7]);
          tmp_distance += (cam_positions[idx].z - mat4_modelview[11]) * (cam_positions[idx].z - mat4_modelview[11]);
          if (nearest_distance_camera > tmp_distance) {
              nearest_distance_camera = tmp_distance;
              find_nearest_camera_id_x = nx;
              find_nearest_camera_id_y = ny;
              find_nearest_camera_id = cam_control.fileindex_array[nx][ny];
          }
        }
      }
      SetCamera(find_nearest_camera_id);
      cam_control.current_x = find_nearest_camera_id_x;
      cam_control.current_y = find_nearest_camera_id_y;
      return;
    }
    if (idx_camera >= cam_config.aCamera.size()){ return; }
    if ( idx_hair >= hairs.size()){ return; }
    if( mode_edit == MODE_NEW ) {
      if (hairs[idx_hair].camera_stroke[idx_camera].xys.size() < 2) {
        idx_hair -= 1;
        hairs.resize(hairs.size() - 1);
      } else {
        is_initial_hair = true;
        auto &stroke = hairs[idx_hair].camera_stroke[idx_camera];
        // stroke.rbf.SetPolyline2(stroke.xys, 1.e-3);
      }
    }
  }

  void CursorPosition(double xpos, double ypos) override {
#ifdef IMGUI_VERSION
    if(  ImGui::GetIO().WantCaptureMouse ){ return; }
#endif
    ::delfem2::glfw::CViewer3::CursorPosition(xpos, ypos);
    if (this->nav.ibutton == GLFW_MOUSE_BUTTON_LEFT &&
      (nav.imodifier == GLFW_MOD_ALT || nav.imodifier == GLFW_MOD_SHIFT) ){
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
    if( idx_camera != idx_camera_texture ){
      std::cout << camconfig_path_xml.parent_path() << std::endl;
      std::filesystem::path path_img = camconfig_path_xml.parent_path()
          / image_folder_name
          / (cam_config.aCamera[idx_camera].label + ".png");
      std::cout << path_img << std::endl;
      if( std::filesystem::exists(path_img) ){
        std::cout << "file exists" << std::endl;
        idx_camera_texture = idx_camera;
        if(!glIsTexture(id_texture)){ glGenTextures(1,&id_texture); }
        glBindTexture(GL_TEXTURE_2D, id_texture);
        delfem2::openglstb::LoadImageFileSetToTexture(
            path_img.string().c_str());
      }
      else{
        std::cout << "file not exists" << std::endl;
      }
    }

  }

  void CalculateCameraPositions(bool write_file) {
    if (cam_control.width * cam_control.height == 0) {
      // todo : show error message
      return;
    }

    std::ofstream ofs;
    if (write_file) {
      ofs.open("./camera_control.xml");
      ofs << "<?xml version=\"1.0\"?>" << std::endl;
      ofs << "<photo_array width=\""<< cam_control.width << "\" height=\"" << cam_control.height << "\">" << std::endl;
    }

    cam_positions.resize(static_cast<uint64_t>(cam_control.width) * static_cast<uint64_t>(cam_control.height));

    for (auto nx = 0; nx < cam_control.width; nx++) {
      
      if (write_file) {
        ofs << "  <photo id =\"" << nx << "\" ";
      }
      
      for (auto ny = 0; ny < cam_control.height; ny++) {

        if (cam_control.has_camera_position && !write_file) {
          cam_positions[nx + ny * static_cast<uint64_t>(cam_control.width)].x = cam_control.camera_position_array[(ny + nx * cam_control.height) * 3];
          cam_positions[nx + ny * static_cast<uint64_t>(cam_control.width)].y = cam_control.camera_position_array[(ny + nx * cam_control.height) * 3 + 1];
          cam_positions[nx + ny * static_cast<uint64_t>(cam_control.width)].z = cam_control.camera_position_array[(ny + nx * cam_control.height) * 3 + 2];
        } else {
          SetCamera(cam_control.fileindex_array[nx][ny]);
          float* mat4_modelview = GetModelViewMatrix().data();
          assert(mat4_modelview);
          cam_positions[nx + ny * static_cast<uint64_t>(cam_control.width)].x = mat4_modelview[3];
          cam_positions[nx + ny * static_cast<uint64_t>(cam_control.width)].y = mat4_modelview[7];
          cam_positions[nx + ny * static_cast<uint64_t>(cam_control.width)].z = mat4_modelview[11];

          if (write_file) {
            if (cam_control.valid_camera_control) {
              ofs << "height_" << ny << "_idx=\"" << cam_control.fileindex_array[nx][ny] << "\" ";
            }
            ofs << "position_" << ny << "=\"" << mat4_modelview[3] << " " << mat4_modelview[7] << " " << mat4_modelview[11] << "\" ";
            ofs << "height_" << ny << "_name=\"" << cam_config.aCamera[idx_camera].label << "\" ";
          }
        }
      }

      if (write_file) {
          ofs << "/>" << std::endl;
      }
    }

    if (write_file) {
      ofs << "</photo_array>" << std::endl;
      ofs.close();
    }
    is_cam_positions = true;
  }

  void ResetOperation() {
    is_open_operation = false;
    is_undo_operation = false;
    is_redo_operation = false;
    is_save_operation = false;
    is_export_operation = false;
  }

 public:
  CameraConfig cam_config{};
  CameraControl cam_control{};
  unsigned int idx_camera = 0;
  unsigned int idx_hair = -1;
  bool is_animation = false;
  bool is_draw_texture = true;
  bool is_wireframe = false;
  std::vector<Hair> &hairs;
  enum {
    MODE_NEW,
    MODE_DEFORM,
    MODE_DELETE,
  };
  int mode_edit = MODE_NEW;
  bool is_initial_hair = false;

  // parameters for shortcut-key operation 
  bool is_open_operation = false;
  bool is_undo_operation = false;
  bool is_redo_operation = false;
  bool is_save_operation = false;
  bool is_export_operation = false;
  bool is_auto_camera_detection = false;

  // rotation information for each camera
  bool is_cam_positions = false;
  std::vector<delfem2::CVec3f> cam_positions;

  std::filesystem::path camconfig_path_xml;
  unsigned int id_texture = 0;
  std::string image_folder_name;
  unsigned int idx_camera_texture = UINT_MAX;
};

#endif //VIEWERSKETCHOPTIMIZATION_H
