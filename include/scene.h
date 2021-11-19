#ifndef _SCENE_H
#define _SCENE_H
#include <embree3/rtcore.h>

#include <string>
#include <unordered_map>
#include <vector>

#include "core.h"
#include "material.h"

struct MeshInfo {
  unsigned int nVertices;        // number of vertices
  unsigned int nFaces;           // number of faces
  unsigned int normalsOffset;    // offset of normals array
  unsigned int texcoordsOffset;  // offset of texcoords array
  unsigned int tangentsOffset;   // offset of tangents array
  unsigned int materialsOffset;  // offset of materials array
};

class Scene {
 private:
  // embree
  RTCDevice device;
  RTCScene scene;

  // triangles
  std::vector<float> vertices;
  std::vector<unsigned int> indices;
  std::vector<float> normals;
  std::vector<float> texcoords;
  std::vector<float> tangents;

  // mesh info
  std::vector<MeshInfo> meshInfos;

 public:
  void addModel(const std::string& filepath) {}

  unsigned int nVertices() const {
    unsigned int ret = 0;
    for (const auto& mi : meshInfos) {
      ret += mi.nVertices;
    }
    return ret;
  }

  unsigned int nFaces() const {
    unsigned int ret = 0;
    for (const auto& mi : meshInfos) {
      ret += mi.nFaces;
    }
    return ret;
  }

  unsigned int nMeshes() const { return meshInfos.size(); }

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