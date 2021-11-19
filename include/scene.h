#ifndef _SCENE_H
#define _SCENE_H
#include <embree3/rtcore.h>

#include <string>
#include <unordered_map>
#include <vector>

#include "core.h"
#include "material.h"

class Material {
 private:
  const Vec3 kd;
  const Vec3 ks;
  const Vec3 ke;

  Material(const Vec3& kd, const Vec3& ks, const Vec3& ke)
      : kd(kd), ks(ks), ke(ke) {}
};

class Scene {
 private:
  // mesh
  std::vector<float> vertices;
  std::vector<unsigned int> indices;
  std::vector<float> normals;
  std::vector<float> texcoords;
  std::vector<float> tangents;

  // key: meshID, value: offset of vertices, indices, normals, ...
  std::unordered_map<unsigned int, unsigned int> meshOffsets;

  // material
  std::vector<Material> materials;
  // key: meshID, value: offset of materials
  std::unordered_map<unsigned int, unsigned int> materialIDs;

  // embree
  RTCDevice device;
  RTCScene scene;

 public:
  void addModel(const std::string& filepath) {}

  void build() const {}

  bool intersect(const Ray& ray, IntersectInfo& info) const { return false; }
};

#endif