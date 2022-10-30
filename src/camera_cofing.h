//
// Created by Nobuyuki Umetani on 2021-08-20.
//

#ifndef CAMERA_COFING_H_
#define CAMERA_COFING_H_

#include <sstream>
#include <optional>

#include "pugixml.hpp"
#include "delfem2/mat4.h"
#include "delfem2/vec3.h"
#include "delfem2/mat3.h"

std::vector<float> SplitTextIntoVectorFloat(const char *pc) {
  std::stringstream ss(pc);
  std::vector<float> vec;
  std::string buf_split;
  while (std::getline(ss, buf_split, ' ')) {
    vec.push_back(std::stof(buf_split));
  }
  return vec;
}

class CameraConfig {
 public:
  class Camera {
   public:
    Camera() : has_transform(false) {}
   public:
    int id_sensor = -1;
    bool has_transform;
    delfem2::CMat4f transform;
    std::string label;
  };
  class Sensor {
   public:
    float f;
    float cx, cy;
    float k1, k2, k3;
    float p1, p2;
    unsigned int width, height;
  };
 public:
  std::vector<Sensor> aSensor;
  std::vector<Camera> aCamera;
  delfem2::CVec3f region_center;
  delfem2::CVec3f region_size;
  delfem2::CMat3f region_R;
 public:
  bool ReadXML(const char *path) {
    pugi::xml_document file;
    const auto res = file.load_file(path);
    if( !res ){
      return false;
    }
    {  // region
      const auto &region = file.child("document").child("chunk").child("region");
      region_center = delfem2::CVec3f(
          SplitTextIntoVectorFloat(region.child_value("center")).data());
      region_size = delfem2::CVec3f(
          SplitTextIntoVectorFloat(region.child_value("size")).data());
      region_R = delfem2::CMat3f(
          SplitTextIntoVectorFloat(region.child_value("R")).data());
    }
    for (const pugi::xml_node sensor: file.child("document").child("chunk").child("sensors")) {
      unsigned int id = sensor.attribute("id").as_int();
      if (aSensor.size() < id + 1) {
        aSensor.resize(id + 1);
      }
      const pugi::xml_node calibration = sensor.child("calibration");
      if( calibration.empty() ){ continue; }  // this node does not have <calibration> tag
      aSensor[id].width = calibration.child("resolution").attribute("width").as_uint();
      aSensor[id].height = calibration.child("resolution").attribute("height").as_uint();
      aSensor[id].f = std::stof(calibration.child_value("f"));
      {  // cx
        const pugi::char_t *sval = calibration.child_value("cx");
        aSensor[id].cx = (*sval=='\0') ? 0.f : std::stof(sval);
      }
      {  // cy
        const pugi::char_t *sval = calibration.child_value("cy");
        aSensor[id].cy = (*sval=='\0') ? 0.f : std::stof(sval);
      }
      aSensor[id].k1 = std::stof(calibration.child_value("k1"));
      aSensor[id].k2 = std::stof(calibration.child_value("k2"));
      {  // k3
        const pugi::char_t *sval = calibration.child_value("k3");
        aSensor[id].k3 = (*sval=='\0') ? 0.f : std::stof(sval);
      }
      aSensor[id].p1 = std::stof(calibration.child_value("p1"));
      aSensor[id].p2 = std::stof(calibration.child_value("p2"));
    }
    for (pugi::xml_node camera: file.child("document").child("chunk").child("cameras")) {
      unsigned int id = camera.attribute("id").as_int();
      if (aCamera.size() < id + 1) {
        aCamera.resize(id + 1);
      }
      aCamera[id].label = camera.attribute("label").as_string();
      aCamera[id].id_sensor = camera.attribute("sensor_id").as_int();
      if (camera.child("transform").empty()) {
        continue;
      }
      std::vector<float> vec = SplitTextIntoVectorFloat(camera.child_value("transform"));
      assert(vec.size() == 16);
      aCamera[id].transform = delfem2::CMat4f(vec.data());
      aCamera[id].has_transform = true;
    }
    return true;
  }
};

#endif
