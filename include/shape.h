#include <embree3/rtcore.h>

#include <vector>

#include "core.h"

class Shape {
 public:
  virtual bool intersect(const Ray& ray, IntersectInfo& info) const = 0;
  virtual AABB bounds() const = 0;
};

class Mesh : public Shape {
 private:
  std::vector<float> vertices;
  std::vector<unsigned int> indices;
  std::vector<float> normals;
  std::vector<float> texcoords;
  std::vector<float> tangents;

  // embree
  RTCDevice device;
  RTCScene scene;

 public:
  Mesh(const std::vector<float>& vertices, std::vector<unsigned int>& indices,
       std::vector<float>& normals, std::vector<float>& texcoords,
       std::vector<float>& tangents)
      : vertices(vertices),
        indices(indices),
        normals(normals),
        texcoords(texcoords),
        tangents(tangents) {}

  bool intersect(const Ray& ray, IntersectInfo& info) const override {
    return false;
  }

  AABB bounds() const override { return AABB(Vec3(), Vec3()); }
};