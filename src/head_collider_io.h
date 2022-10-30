//
// Created by Shinichi Kinuwaki on 2021-12-29.
//

//#pragma once

#include <fstream>
// #include <pugixml.hpp>

constexpr int HairColliderSize = 128;
static_assert(sizeof(delfem2::AdaptiveDistanceField3::CNode) == HairColliderSize);

void SaveHairCollider(
    const delfem2::AdaptiveDistanceField3& adf,
    const std::string& bin_path) {

    std::ofstream fout;
    fout.open(bin_path.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
    if (!fout) {
      std::cerr << "Error!--> file cannot open for save:" << bin_path << std::endl;
      return;
    }

    fout.write((char*)&adf.dist_min, sizeof(double) * 1);
    fout.write((char*)&adf.dist_max, sizeof(double) * 1);

    size_t num_anode = adf.aNode.size();
    fout.write((char*)&num_anode, sizeof(size_t) * 1);

    size_t anode_size = sizeof(delfem2::AdaptiveDistanceField3::CNode);
    fout.write((char*)&anode_size, sizeof(size_t) * 1);

    fout.write((char*)&adf.aNode[0], sizeof(delfem2::AdaptiveDistanceField3::CNode) * num_anode);

    fout.close();
}


bool LoadHairCollider(
    delfem2::AdaptiveDistanceField3& adf,
    const std::string& bin_path) {

    std::ifstream fin;
    fin.open(bin_path.c_str(), std::ios::in | std::ios::binary);
    if (!fin) {
      std::cerr << "Error!--> file cannot open for load:" << bin_path << std::endl;
      return false;
    }

    fin.read((char*)&adf.dist_min, sizeof(double) * 1);
    fin.read((char*)&adf.dist_max, sizeof(double) * 1);

    size_t num_anode;
    fin.read((char*)&num_anode, sizeof(size_t) * 1);

    size_t anode_size;
    fin.read((char*)&anode_size, sizeof(size_t) * 1);

    if (num_anode == 0 || anode_size != HairColliderSize ) {
      std::cout << num_anode << " " << anode_size << " " << HairColliderSize << std::endl;
        fin.close();
        return false;
    }

    adf.aNode.resize(num_anode);
    fin.read((char*)&adf.aNode[0], sizeof(delfem2::AdaptiveDistanceField3::CNode) * num_anode);

    fin.close();

    return true;
}