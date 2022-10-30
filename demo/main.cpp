/*
* Copyright (c) 2019 Nobuyuki Umetani
*
* This source code is licensed under the MIT license found in the
* LICENSE file in the root directory of this source tree.
*/

#include <vector>
#if defined(_WIN32) // windows
#  define NOMINMAX   // to remove min,max macro
#  include <windows.h>
#endif
#define GL_SILENCE_DEPRECATION
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "ImGuiFileBrowser.h"
#include "delfem2/cvcamera.h"
#include "delfem2/srchuni_v3.h"
#include "delfem2/view_array2d.h"
#include "delfem2/glfw/util.h"
#include "delfem2/opengl/new/drawer_mshtex.h"
#include "delfem2/opengl/new/drawer_polyline.h"

#define STB_IMAGE_IMPLEMENTATION
#include "head_collider.h"
#include "hair_elastic.h"
#include "headmesh.h"
#include "viewer_cameras2.h"
#include "save_scaled_head_hair.h"
#include "event_handler.h"
#include "hair_filelist_db.h"
#include "object_redo_undo.h"
#include "demo_utility.h"
#include "hair_io_state.h"
#include "hair_root.h"

int Design(
    const std::filesystem::path &path_xml,
    const std::filesystem::path &path_obj,
    const std::filesystem::path &path_tex,
    const std::filesystem::path &path_hair_root,
    const std::filesystem::path &path_head_collider,
    const std::string &image_folder_name,
    float length_edge_ini) {
  namespace dfm2 = delfem2;
  dfm2::opengl::Drawer_RectangleTex drawer_background;
  dfm2::opengl::Drawer_MeshTex drawer_head;
  dfm2::opengl::Drawer_Polyline drawer_hair;
  //
  SimulationParam sim_param;
  sim_param.length_edge_ini = length_edge_ini;
  const SimulationParam sim_param_initial = sim_param;
  //
  HeadMesh head;
  head.Init(path_obj, path_tex);
  MakeTriFlagForWrapBaseMesh( // for base mesh only
      head.tri_flg_hairroot,
      head.group_elem_index, head.vtx_tex, head.tri_vtx_tex);
  //
  std::vector<Hair> hairs;
  RedoUndoOperator<Hair> redo_undo_operator(hairs);
  DemoUtility demo_utility(sim_param, sim_param_initial);
  /*
  ReadHairRoot(
      hairs,
      path_hair_root,
      head.vtx_xyz, head.vtx_nrm, head.tri_vtx_xyz);
      */
  {
    std::vector<std::tuple<unsigned int, double, double> > samples;
    SampleHairRoot(
        samples,
        100, head.tri_flg_hairroot, head.vtx_xyz, head.tri_vtx_xyz, head.tri_adjtri_xyz);
    std::cout << "number of hairs: " << samples.size() << std::endl;
    hairs.clear();
    for(auto sample : samples){
      hairs.resize(hairs.size()+1);
      auto& hair = hairs[hairs.size()-1];
      hair.root_on_mesh.itri = std::get<0>(sample);
      hair.root_on_mesh.r0 = std::get<1>(sample);
      hair.root_on_mesh.r1 = std::get<2>(sample);
      hair.root_pos = hair.root_on_mesh.PositionOnMeshTri3(head.vtx_xyz, head.tri_vtx_xyz);
      hair.root_normal = hair.root_on_mesh.UnitNormalOnMeshTri3(head.vtx_nrm, head.tri_vtx_nrm);
      hair.root_normal.normalize();
      for (int idiv = 0; idiv < 10; ++idiv) {
        delfem2::CVec3f p1 = (hair.root_pos + hair.root_normal * idiv * length_edge_ini).cast<float>();
        hair.vtx_xyz.push_back(p1.x);
        hair.vtx_xyz.push_back(p1.y);
        hair.vtx_xyz.push_back(p1.z);
      }
    }
  }
  delfem2::AdaptiveDistanceField3 adf;
  MakeAdaptiveDistanceFieldForHead(
      adf,
      head.vtx_xyz, head.tri_vtx_xyz,
      path_head_collider);
  //
  delfem2::LinearSystemSolver_BlockPentaDiagonal<2> linsys_for_stroke;
  MyViewer viewer(
      hairs,
      path_xml,
      image_folder_name);

  int action_next = -1;
  bool action_save = false;
  bool action_export = false;
  bool action_undo = false;
  bool action_add_hair = false;
  viewer.keypress_callbacks.emplace_back([&action_save,
          &action_export,
          &action_next,
          &action_undo,
          &action_add_hair](int key, int) {
      if (key == GLFW_KEY_1) { action_next = 0; }
      else if (key == GLFW_KEY_2) { action_next = 1; }
      else if (key == GLFW_KEY_3) { action_next = 2; }
      else if (key == GLFW_KEY_4) { action_next = 3; }
      else if (key == GLFW_KEY_5) { action_next = 4; }
      if (key == GLFW_KEY_S) { action_save = true; }
      if (key == GLFW_KEY_E) { action_export = true; }
      if (key == GLFW_KEY_Z) { action_undo = true; }
      if (key == GLFW_KEY_RIGHT_BRACKET) { action_add_hair = true; }
  });
  // -------------------------
  // below: opengl starts here
  delfem2::glfw::InitGLNew();
  viewer.OpenWindow();
  if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress)) {
    std::cout << "Failed to initialize GLAD" << std::endl;
    return -1;
  }
  viewer.SetCamera(viewer.idx_camera);
  head.InitGL();
  drawer_hair.InitGL();
  drawer_background.InitGL();
  drawer_head.InitGL();
  //
  { // make unified vtx & tex index for head
    std::vector<double> uni_xyz0, uni_tex0;
    std::vector<unsigned int> tri_uni, uni_xyz, uni_tex;
    delfem2::UnifySeparateIndexing_PosTex(
        uni_xyz0, uni_tex0,
        tri_uni, uni_xyz, uni_tex,
        head.vtx_xyz, head.vtx_tex,
        head.tri_vtx_xyz, head.tri_vtx_tex);
    drawer_head.SetCoords(uni_xyz0, 3);
    drawer_head.SetElement(tri_uni, GL_TRIANGLES);
    drawer_head.SetTexUV(uni_tex0);
  }
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui_ImplGlfw_InitForOpenGL(viewer.window, true);// Setup Platform/Renderer bindings
  ImGui::StyleColorsDark(); // Setup Dear ImGui style

  bool show_open_dialog = false;
  bool show_save_dialog = false;
  bool show_export_dialog = false;
  bool quit_application_flag = false;

  //
  ImGui_ImplOpenGL3_Init("#version 150");
  while (!glfwWindowShouldClose(viewer.window)) {
    EventHandler(
        hairs, viewer, linsys_for_stroke,
        sim_param, adf, head);
    // ----------------------
    glfwPollEvents();
    glClearColor(0.45f, 0.55f, 0.60f, 1.00f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // feed inputs to dear imgui, start new frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    { // render your GUI
      ImGui::Begin("Hair Designer");
      ImGui::SliderFloat("bend stiffness", &sim_param.stiff_bend, 0.f, 0.005f);
      ImGui::SliderFloat("offset contact", &sim_param.offset_contact, 0, 0.02);

      demo_utility.PostImGui(viewer, hairs, head);

      if (viewer.next_character != -1)
        break;

      ImGui::End();
    }

    ::glBindTexture(GL_TEXTURE_2D, head.texture.id_tex);
    drawer_head.Draw(
        viewer.GetProjectionMatrix().data(),
        viewer.GetModelViewMatrix().data());

    ::glEnable(GL_DEPTH_TEST);
    if (viewer.idx_camera_texture == viewer.idx_camera && viewer.idx_camera != UINT_MAX) {
      // draw only depth
      drawer_head.Draw(
          viewer.GetProjectionMatrix().data(),
          viewer.GetModelViewMatrix().data());
      // draw 2d plane
      ::glBindTexture(GL_TEXTURE_2D, viewer.id_texture);
      drawer_background.Draw(
          dfm2::CMat4f::Identity().data(),
          dfm2::CMat4f::Identity().data());
      ::glEnable(GL_DEPTH_TEST);
    }
    for (unsigned int ih=0;ih<hairs.size();++ih) {
      drawer_hair.radius_sphere = 0.003;
      drawer_hair.radius_cylinder = 0.003;
      drawer_hair.sphere.color = {0.5, 0, 0.5, 1};
      drawer_hair.cylinder.color = {0.5, 0, 0.5, 1};
      if( !hairs[ih].camera_stroke.empty() ){
        drawer_hair.sphere.color = {1.0, 0, 1.0, 1};
        drawer_hair.cylinder.color = {1.0, 0, 1.0, 1};
      }
      if( ih == viewer.idx_hair ) {
        drawer_hair.sphere.color = {0, 1, 1, 1};
        drawer_hair.cylinder.color = {0, 1, 1, 1};
      }
      drawer_hair.Draw(
          ViewAsArray2D{hairs[ih].vtx_xyz.data(),
                        hairs[ih].vtx_xyz.size() / 3, 3},
          3,
          viewer.GetProjectionMatrix().data(),
          viewer.GetModelViewMatrix().data());
    }
    while ( viewer.idx_hair < hairs.size() ){
      if( viewer.idx_camera >= viewer.cam_config.aCamera.size() ){ break; }
      auto& camst = hairs[viewer.idx_hair].camera_stroke;
      if( camst.find(viewer.idx_camera) == camst.end() ){ break; }
      const std::vector<dfm2::CVec2f> &xys =camst[viewer.idx_camera].xys;
      drawer_hair.sphere.color = {1.0, 0, 0.0, 1};
      drawer_hair.cylinder.color = {1.0, 0, 0.0, 1};
      ::glDisable(GL_DEPTH_TEST);
      drawer_hair.Draw(
          xys,
          2,
          delfem2::CMat4f::Identity().data(),
          delfem2::CMat4f::Identity().data());
      ::glEnable(GL_DEPTH_TEST);
      break;
    }
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    viewer.SwapBuffers();
  }

  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();;

  glfwDestroyWindow(viewer.window);
  glfwTerminate();

  return viewer.next_character;
}

int main() {

  int current_person = -1;
  {
    for (int np = 0; np < db::NumPerson; np++) {
      bool person_detect_flag = true;
      auto test = std::filesystem::path(PATH_SOURCE_DIR) / ".." / "data" / db::PersonDbList[np].second / db::PathCamera;
      person_detect_flag &= std::filesystem::exists(std::filesystem::path(PATH_SOURCE_DIR) / ".." / "data" / db::PersonDbList[np].second / db::PathCamera);
      person_detect_flag &= std::filesystem::exists(std::filesystem::path(PATH_SOURCE_DIR) / ".." / "data" / db::PersonDbList[np].second / db::PathObj);
      person_detect_flag &= std::filesystem::exists(std::filesystem::path(PATH_SOURCE_DIR) / ".." / "data" / db::PersonDbList[np].second / db::PathTexture);
      person_detect_flag &= std::filesystem::exists(std::filesystem::path(PATH_SOURCE_DIR) / ".." / "data" / db::PersonDbList[np].second / db::PathHairRoot);
      person_detect_flag &= std::filesystem::exists(std::filesystem::path(PATH_SOURCE_DIR) / ".." / "data" / db::PersonDbList[np].second / db::PathHeadCollider);
      person_detect_flag &= std::filesystem::exists(std::filesystem::path(PATH_SOURCE_DIR) / ".." / "data" / db::PersonDbList[np].second / db::ImageFolderName);
      if (current_person == -1 && person_detect_flag) {
          current_person = np;
      }
      if (person_detect_flag)
        std::cout << db::PersonDbList[np].second << " dataset is detected.\n";
      db::PersonDbList[np].first = person_detect_flag;
    }
  }

  while(current_person != -1){
    const auto path_dir = std::filesystem::path(PATH_SOURCE_DIR) / ".." / "data" / db::PersonDbList[current_person].second;
    current_person = Design(
        path_dir / db::PathCamera,
        path_dir / db::PathObj,
        path_dir / db::PathTexture,
        path_dir / db::PathHairRoot,
        path_dir / db::PathHeadCollider,
        db::ImageFolderName,
        0.03f);
    if (!db::PersonDbList[current_person].first) {
      std::cout << db::PersonDbList[current_person].second << " dataset is not exsisting.\n";
      current_person = -1;
    }
  }
}


