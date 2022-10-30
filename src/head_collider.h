//
// Created by Nobuyuki Umetani on 2021/12/05.
//

#ifndef HEAD_COLLIDER_H_
#define HEAD_COLLIDER_H_

#include <filesystem>

#include "delfem2/isrf_adf.h"
#include "delfem2/srch_v3bvhmshtopo.h"
#include "delfem2/srchbv3sphere.h"
#include "delfem2/points.h"

#include "head_collider_io.h"

void MakeAdaptiveDistanceFieldForHead(
  delfem2::AdaptiveDistanceField3 &adf,
  const std::vector<double> &vtx_xyz,
  const std::vector<unsigned int> &tri_vtx,
  const std::filesystem::path xml_path)
{
  namespace dfm2 = delfem2;
  class CMesh : public delfem2::Input_AdaptiveDistanceField3 {
   public:
    [[nodiscard]] double sdf(double x, double y, double z) const override {
      dfm2::CVec3d n0;
      double sdf0 = obj.SignedDistanceFunction(
        n0,
        dfm2::CVec3d(x, y, z),
        vtx_xyz, tri_vtx, vtx_nrm);
      return sdf0;
    }
   public:
    std::vector<double> vtx_xyz, vtx_nrm;
    std::vector<unsigned int> tri_vtx;
    dfm2::CBVH_MeshTri3D<dfm2::CBV3d_Sphere, double> obj;
  };
  CMesh mesh;
  {
    std::vector<unsigned int> tri_adjtri;
    dfm2::ElSuEl_MeshElem(
      tri_adjtri,
      tri_vtx.data(), tri_vtx.size() / 3,
      dfm2::MESHELEM_TRI, vtx_xyz.size() / 3);
    std::vector<unsigned int> edge_vtx;
    for (unsigned int it = 0; it < tri_adjtri.size() / 3; ++it) {
      for (int ie = 0; ie < 3; ++ie) {
        if (tri_adjtri[it * 3 + ie] != UINT_MAX) { continue; }
        edge_vtx.push_back(tri_vtx[it * 3 + (ie + 1) % 3]);
        edge_vtx.push_back(tri_vtx[it * 3 + (ie + 2) % 3]);
      }
    }
    double len;
    double cg[3];
    dfm2::CG_MeshLine3(len, cg, vtx_xyz, edge_vtx);
    // std::cout << cg[0] << " " << cg[1] << " " << cg[2] << std::endl;
    mesh.vtx_xyz = vtx_xyz;
    mesh.tri_vtx = tri_vtx;
    unsigned int ivtx0 = vtx_xyz.size() / 3;
    for (unsigned int ie = 0; ie < edge_vtx.size() / 2; ++ie) {
      mesh.tri_vtx.push_back(ivtx0);
      mesh.tri_vtx.push_back(edge_vtx[ie * 2 + 1]);
      mesh.tri_vtx.push_back(edge_vtx[ie * 2 + 0]);
    }
    mesh.vtx_xyz.push_back(cg[0]);
    mesh.vtx_xyz.push_back(cg[1]);
    mesh.vtx_xyz.push_back(cg[2]);
  }
  {
    mesh.obj.Init(
      mesh.vtx_xyz.data(), mesh.vtx_xyz.size() / 3,
      mesh.tri_vtx.data(), mesh.tri_vtx.size() / 3,
      0.0);
    mesh.vtx_nrm.resize(mesh.vtx_xyz.size());
    delfem2::Normal_MeshTri3D(
      mesh.vtx_nrm.data(),
      mesh.vtx_xyz.data(), mesh.vtx_xyz.size() / 3,
      mesh.tri_vtx.data(), mesh.tri_vtx.size() / 3);
  }
  double min0[3], max0[3];
  dfm2::BoundingBox3_Points3(
    min0,max0,
    mesh.vtx_xyz.data(),mesh.vtx_xyz.size()/3);
  double bb[6] = {min0[0], max0[0], min0[1], max0[1], min0[2], max0[2]};
  if ( std::filesystem::exists(xml_path) ) {
    std::cout << "loading ADF" << std::endl;
    if (!LoadHairCollider(adf, xml_path.string())) {
      std::cout << "loading ADF fails...." << std::endl;
      std::cout << "building ADF please wait...." << std::endl;
      adf.SetUp(mesh, bb);
    }    
  } else {
    std::cout << "building ADF please wait...." << std::endl;
    adf.SetUp(mesh, bb);
    SaveHairCollider(adf, xml_path.string() );
  }
  std::cout << "ADF Oct-tree Node Size : " << adf.aNode.size() << std::endl;
}

#endif //HEAD_COLLIDER_H_
