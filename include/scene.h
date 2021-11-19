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

  bool intersect(const Ray& ray, IntersectInfo& info) const {
    RTCRayHit rayhit;
    rayhit.ray.org_x = ray.origin[0];
    rayhit.ray.org_y = ray.origin[1];
    rayhit.ray.org_z = ray.origin[2];
    rayhit.ray.dir_x = ray.direction[0];
    rayhit.ray.dir_y = ray.direction[1];
    rayhit.ray.dir_z = ray.direction[2];
    rayhit.ray.tnear = ray.tmin;
    rayhit.ray.tfar = ray.tmax;
    rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;

    RTCIntersectContext context;
    rtcInitIntersectContext(&context);

    rtcIntersect1(scene, &context, &rayhit);

    if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
      info.t = rayhit.ray.tfar;
      info.surfaceInfo.position = ray(info.t);

      // compute shading normal

      return true;
    } else {
      return false;
    }
  }
};

#endif