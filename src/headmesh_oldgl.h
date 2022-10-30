//
// Created by Nobuyuki Umetani on 2021-10-06.
//

#ifndef HEADMESH_OLDGL_H_
#define HEADMESH_OLDGL_H_

#include "delfem2/opengl/old/mshuni.h"

void DrawHead_Texture(
  const HeadMesh &head) {
  ::glEnable(GL_DEPTH_TEST);
  ::glDisable(GL_CULL_FACE);
//    ::glCullFace(GL_BACK);
  ::glDisable(GL_LIGHTING);
  ::glColor3d(1, 1, 1);
  ::glEnable(GL_TEXTURE_2D);
  ::glBindTexture(GL_TEXTURE_2D, head.texture.id_tex);
  ::glPolygonMode(GL_FRONT, GL_FILL);
  delfem2::opengl::DrawMeshTri3D_FaceNorm_TexVtx(
    head.vtx_xyz,
    head.tri_vtx_xyz,
    head.vtx_tex,
    head.tri_vtx_tex);
}

void DrawHead_Lambert(
  const HeadMesh &head) {
  ::glEnable(GL_DEPTH_TEST);
  ::glDisable(GL_TEXTURE_2D);
  ::glEnable(GL_LIGHTING);
  ::glPolygonMode(GL_FRONT, GL_FILL);
  delfem2::opengl::DrawMeshTri3D_FaceNorm(
    head.vtx_xyz,
    head.tri_vtx_xyz,
    head.vtx_nrm);
}

void DrawHead_Wireframe(
  const HeadMesh &head) {
  ::glEnable(GL_DEPTH_TEST);
  ::glDisable(GL_TEXTURE_2D);
  ::glDisable(GL_LIGHTING);
  ::glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  delfem2::opengl::DrawMeshTri3D_FaceNorm(
    head.vtx_xyz,
    head.tri_vtx_xyz,
    head.vtx_nrm);
  ::glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

#endif //HEADMESH_OLDGL_H_
