//
// Created by Nobuyuki Umetani on 2022/01/06.
//

#ifndef HAIR_ROOT_H_
#define HAIR_ROOT_H_

#include <functional>

#include "delfem2/sampler_trimesh.h"
#include "delfem2/mshmisc.h"

#include "hair.h"

void ReadHairRoot(
    std::vector<Hair> &hairs,
    const std::filesystem::path &path_hair_root,
    const std::vector<double> &vtx_xyz_head,
    const std::vector<double> &vtx_nrm_head,
    const std::vector<unsigned int> &tri_vtx_head) {
  std::ifstream fin;
  fin.open(path_hair_root);
  int nhair;
  fin >> nhair;
  hairs.resize(nhair);
  for (int ih = 0; ih < nhair; ++ih) {
    auto &hair = hairs[ih];
    {
      int it;
      double r0, r1;
      fin >> it >> r0 >> r1;
      hair.root_on_mesh.itri = it;
      hair.root_on_mesh.r0 = r0;
      hair.root_on_mesh.r1 = r1;
    }
    hair.root_pos = hair.root_on_mesh.PositionOnMeshTri3(vtx_xyz_head, tri_vtx_head);
    hair.root_normal = hair.root_on_mesh.UnitNormalOnMeshTri3(vtx_nrm_head, tri_vtx_head);
    hair.root_normal.normalize();
    for (int idiv = 0; idiv < 10; ++idiv) {
      delfem2::CVec3f p1 = (hair.root_pos + hair.root_normal * idiv * 0.05).cast<float>();
      hair.vtx_xyz.push_back(p1.x);
      hair.vtx_xyz.push_back(p1.y);
      hair.vtx_xyz.push_back(p1.z);
    }
  }
}

void MakeTriFlagForWrapBaseMesh(
    std::vector<unsigned int> &tri_flg_hairroot,
    const std::vector<unsigned int> &group_elem_index,
    const std::vector<double> &vtx_tex,
    const std::vector<unsigned int> &tri_vtx_tex)  {
  namespace dfm2 = delfem2;
  tri_flg_hairroot.assign(tri_vtx_tex.size()/3, 0);
  std::vector<unsigned int> tri_adjtri_tex;
  dfm2::ElSuEl_MeshElem(
      tri_adjtri_tex,
      tri_vtx_tex.data(), tri_vtx_tex.size() / 3,
      dfm2::MESHELEM_TRI, vtx_tex.size() / 2);
  unsigned int igroup = 4;
  for(unsigned int it=group_elem_index[igroup];it<group_elem_index[igroup+1];++it){
    tri_flg_hairroot[it] = 1;
  }
  int num_group;
  std::vector<unsigned int> tri_flg2;
  dfm2::MakeGroupElem(num_group, tri_flg2, tri_vtx_tex, tri_adjtri_tex, 3, 3);
  for(unsigned int it=0; it<tri_flg_hairroot.size(); ++it){
    if( tri_flg_hairroot[it] == 0 ){ continue; }
    if( tri_flg2[it] == 3 ){ tri_flg_hairroot[it] = 0; }
    if( tri_flg2[it] == 4 ){ tri_flg_hairroot[it] = 0; }
  }
  std::cout << "num_group: " << num_group << std::endl;
}

void SampleHairRoot(
    std::vector<std::tuple<unsigned int, double, double> > &samples,
    unsigned int nadd,
    const std::vector<unsigned int> &tri_flg,
    const std::vector<double> &vtx_xyz,
    const std::vector<unsigned int> &tri_vtx_xyz,
    const std::vector<unsigned int> &tri_adjtri){
  namespace dfm2 = delfem2;
  unsigned int nsmpl = samples.size() + nadd;
  const double area0 = delfem2::Area_MeshTri3(
      vtx_xyz, tri_vtx_xyz,
      [&tri_flg](unsigned int it){ return tri_flg[it]==1; });
  double rad0 = std::sqrt(area0 / nsmpl) * 1.0;
  std::multimap<unsigned int, unsigned int> el_smpl;
  for(unsigned int ismpl=0;ismpl<samples.size();++ismpl){
    unsigned int itri = std::get<0>(samples[ismpl]);
    assert(itri<tri_vtx_xyz.size()/3);
    el_smpl.insert({itri, ismpl});
  }
  dfm2::RandomSamplingOnMeshTri3Selective mapper(
      vtx_xyz, tri_vtx_xyz,
      [&tri_flg](unsigned int itri) { return tri_flg[itri] == 1; });
  mapper.rndeng.seed(0);
  unsigned int count_fail = 0;
  for (unsigned int ismpl = 0; ismpl < 3000; ++ismpl) {
    const auto smpli = mapper.Sample();
    const bool is_near = dfm2::IsTherePointOnMeshInsideSphere(
        smpli, rad0, samples, el_smpl,
        vtx_xyz, tri_vtx_xyz, tri_adjtri);
    if (is_near) { count_fail++; }
    else {
      count_fail = 0;
      el_smpl.insert({std::get<0>(smpli), samples.size()});
      samples.push_back(smpli);
      if( samples.size() >= nsmpl ){ return; }
    }
    if( count_fail >= 100 ){
      rad0 *= 0.99;
      std::cout << "shrink rad: " << rad0 << " " << samples.size() << " " << nsmpl << " " << ismpl << std::endl;
    }
  }
}

#endif //HAIR_ROOT_H_
