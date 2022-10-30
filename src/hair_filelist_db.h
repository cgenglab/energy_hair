//
// Created by Shinichi Kinuwaki on 2022-01-04.
//

#pragma once

namespace db {

const int NumPerson = 4;
const std::filesystem::path PathCamera = "camera.xml";
const std::filesystem::path PathObj = "wrap_mesh.obj";
const std::filesystem::path PathTexture = "wrap_texture.jpg";
const std::filesystem::path PathHairRoot = "hair_root.txt";
const std::filesystem::path PathHeadCollider = "head_collider.bin";
const std::string ImageFolderName = "image_uncalibrated_rotated_resized";

std::pair<bool, std::string> PersonDbList[NumPerson] = {
  std::make_pair(false, "2021_0915_S"),
  std::make_pair(false, "2021_0906_M"),
  std::make_pair(false, "2021_0908_M"),
//  std::make_pair(false, "2021_0909_N"),
  std::make_pair(false, "2021_1018_A"),
};

};