//
// Created by Nobuyuki Umetani on 2021-09-19.
//

#ifndef HEADMESH_H
#define HEADMESH_H

#include <filesystem>
#include <vector>
#define GL_SILENCE_DEPRECATION

#include "delfem2/opengl/tex.h"
#include "delfem2/mshmisc.h"
#include "delfem2/msh_io_obj.h"
#include "delfem2/mshuni.h"
#include "delfem2/openglstb/img2tex.h"

class HeadMesh{
 public:
  void Init(
      const std::filesystem::path &path_obj,
      const std::filesystem::path &path_tex){
    {
      std::string fname_mtl;
      std::vector<unsigned int> tri_vtx_index;
      delfem2::Read_WavefrontObjWithMaterialMixedElem(
          fname_mtl,
          vtx_xyz,vtx_tex,vtx_nrm,
          tri_vtx_index,
          tri_vtx_xyz,tri_vtx_tex,tri_vtx_nrm,
          group_names, group_elem_index,
          path_obj);
      std::cout << vtx_nrm.size()/3 << std::endl;
      std::cout << group_names.size() << " " << group_elem_index.size() << std::endl;
    }

    if( vtx_nrm.size() != vtx_xyz.size() ){
      vtx_nrm.resize(vtx_xyz.size());
      delfem2::Normal_MeshTri3D(
          vtx_nrm.data(),
          vtx_xyz.data(),
          vtx_xyz.size() / 3,
          tri_vtx_xyz.data(),
          tri_vtx_xyz.size() / 3);
      tri_vtx_nrm = tri_vtx_xyz;
    }

    delfem2::ElSuEl_MeshElem(
        tri_adjtri_xyz,
        tri_vtx_xyz.data(), tri_vtx_xyz.size() / 3,
        delfem2::MESHELEM_TRI, vtx_xyz.size() / 3);

    // load texture of head
    delfem2::openglstb::SetRgbToTex(
        texture,
        path_tex.string(),
        true);
  }
  void InitGL(){
    texture.InitGL();
  }
  void SaveObj(
      const std::filesystem::path& path,
      const double trans[3],
      double scale) const {
    std::vector<double> vtx_xyz1 = vtx_xyz;
    delfem2::Translate_Points3(
        vtx_xyz1,
        trans[0], trans[1], trans[2] );
    delfem2::Scale_Points(
        vtx_xyz1.data(),
        vtx_xyz1.size()/3, 3,
        scale );
    delfem2::Write_WavefrontObj(
        path.string(),
        vtx_xyz1, vtx_tex, vtx_nrm,
        tri_vtx_xyz, tri_vtx_tex, tri_vtx_nrm,
        group_names, group_elem_index);
  }
 public:
  std::vector<double> vtx_xyz;
  std::vector<double> vtx_tex;
  std::vector<double> vtx_nrm;

  std::vector<std::string> group_names;
  std::vector<unsigned int> group_elem_index;

  std::vector<unsigned int> tri_vtx_xyz;
  std::vector<unsigned int> tri_vtx_tex;
  std::vector<unsigned int> tri_vtx_nrm;
  delfem2::opengl::CTexRGB texture;

  std::vector<unsigned int> tri_adjtri_xyz;
  std::vector<unsigned int> tri_flg_hairroot;  // 0: no hair on this tri 1: hair will be sampled on this tri
};

#endif //HEADMESH_H
