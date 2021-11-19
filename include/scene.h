#ifndef _SCENE_H
#define _SCENE_H
#include <embree3/rtcore.h>
#include <spdlog/spdlog.h>

#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

#include "core.h"
#include "material.h"
#include "tiny_obj_loader.h"

struct Vertex {
  float vertex[3];
  float normal[3];
  float texcoords[2];
  bool hasNormal;
  bool hasTexcoords;

  Vertex() {
    vertex[0] = vertex[1] = vertex[2] = 0;
    normal[0] = normal[1] = normal[2] = 0;
    texcoords[0] = texcoords[1] = texcoords[2] = 0;
    hasNormal = false;
    hasTexcoords = false;
  }

  bool operator==(const Vertex& v) const {
    return vertex[0] == v.vertex[0] && vertex[1] == v.vertex[1] &&
           vertex[2] == v.vertex[2] && normal[0] == v.normal[0] &&
           normal[1] == v.normal[1] && normal[2] == v.normal[2] &&
           texcoords[0] == v.texcoords[0] && texcoords[1] == v.texcoords[1] &&
           hasNormal == v.hasNormal && hasTexcoords == v.hasTexcoords;
  }

  // NOTE: used for unordered_map's key
  std::string toString() const {
    return std::to_string(vertex[0]) + "," + std::to_string(vertex[1]) + ", " +
           std::to_string(vertex[2]) + "," + std::to_string(normal[0]) + "," +
           std::to_string(normal[1]) + "," + std::to_string(normal[2]) + "," +
           std::to_string(texcoords[0]) + "," + std::to_string(texcoords[1]) +
           "," + std::to_string(hasNormal) + "," + std::to_string(hasTexcoords);
  }
};

struct MeshInfo {
  uint32_t nVertices;        // number of vertices
  uint32_t nFaces;           // number of faces
  uint32_t normalsOffset;    // offset of normals array
  uint32_t texcoordsOffset;  // offset of texcoords array
  uint32_t tangentsOffset;   // offset of tangents array
  uint32_t materialsOffset;  // offset of materials array
};

class Scene {
 private:
  // embree
  RTCDevice device;
  RTCScene scene;

  // triangles
  std::vector<float> vertices;
  std::vector<uint32_t> indices;
  std::vector<float> normals;
  std::vector<float> texcoords;
  std::vector<float> tangents;
  std::vector<uint32_t> materialIDs;

  // mesh info
  std::vector<MeshInfo> meshInfos;

  // materials
  std::vector<Material> materials;

 public:
  void addModel(const std::filesystem::path& filepath) {
    tinyobj::ObjReaderConfig reader_config;
    reader_config.mtl_search_path = "./";

    tinyobj::ObjReader reader;
    if (!reader.ParseFromFile(filepath.generic_string(), reader_config)) {
      if (!reader.Error().empty()) {
        spdlog::error("[Scene] failed to load {} : {}",
                      filepath.generic_string(), reader.Error());
      }
      return;
    }

    if (!reader.Warning().empty()) {
      spdlog::warn("[Scene] {}", reader.Warning());
    }

    const auto& attrib = reader.GetAttrib();
    const auto& shapes = reader.GetShapes();
    const auto& materials = reader.GetMaterials();

    // key: vertex string, value: start index of vertices
    std::unordered_map<std::string, uint32_t> uniqueVertices;

    // loop over shapes
    for (size_t s = 0; s < shapes.size(); ++s) {
      size_t index_offset = 0;
      // loop over faces
      for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); ++f) {
        const size_t fv =
            static_cast<size_t>(shapes[s].mesh.num_face_vertices[f]);

        for (size_t v = 0; v < fv; ++v) {
          Vertex vertex;

          const tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
          const tinyobj::real_t vx =
              attrib.vertices[3 * static_cast<size_t>(idx.vertex_index) + 0];
          const tinyobj::real_t vy =
              attrib.vertices[3 * static_cast<size_t>(idx.vertex_index) + 1];
          const tinyobj::real_t vz =
              attrib.vertices[3 * static_cast<size_t>(idx.vertex_index) + 2];

          vertex.vertex[0] = vx;
          vertex.vertex[1] = vx;
          vertex.vertex[2] = vx;

          if (idx.normal_index >= 0) {
            const tinyobj::real_t nx =
                attrib.normals[3 * static_cast<size_t>(idx.normal_index) + 0];
            const tinyobj::real_t ny =
                attrib.normals[3 * static_cast<size_t>(idx.normal_index) + 1];
            const tinyobj::real_t nz =
                attrib.normals[3 * static_cast<size_t>(idx.normal_index) + 2];

            vertex.hasNormal = true;
            vertex.normal[0] = nx;
            vertex.normal[1] = ny;
            vertex.normal[2] = nz;
          }
          // face normal
          else {
          }

          if (idx.texcoord_index >= 0) {
            const tinyobj::real_t tx =
                attrib
                    .texcoords[3 * static_cast<size_t>(idx.texcoord_index) + 0];
            const tinyobj::real_t ty =
                attrib
                    .texcoords[3 * static_cast<size_t>(idx.texcoord_index) + 1];

            vertex.hasTexcoords = true;
            vertex.texcoords[0] = tx;
            vertex.texcoords[1] = ty;
          }
          // barycentric coordinate
          else {
          }

          // tangents

          // https://vulkan-tutorial.com/Loading_models
          // See Vertex deduplication section
          const std::string key = vertex.toString();
          if (uniqueVertices.count(key) == 0) {
            uniqueVertices[key] = static_cast<uint32_t>(vertices.size());

            vertices.push_back(vertex.vertex[0]);
            vertices.push_back(vertex.vertex[1]);
            vertices.push_back(vertex.vertex[2]);

            normals.push_back(vertex.normal[0]);
            normals.push_back(vertex.normal[1]);
            normals.push_back(vertex.normal[2]);

            texcoords.push_back(vertex.texcoords[0]);
            texcoords.push_back(vertex.texcoords[1]);
          }
        }
        index_offset += fv;

        // per face material
        shapes[s].mesh.material_ids[f];
      }
    }
  }

  uint32_t nVertices() const {
    uint32_t ret = 0;
    for (const auto& mi : meshInfos) {
      ret += mi.nVertices;
    }
    return ret;
  }

  uint32_t nFaces() const {
    uint32_t ret = 0;
    for (const auto& mi : meshInfos) {
      ret += mi.nFaces;
    }
    return ret;
  }

  uint32_t nMeshes() const { return meshInfos.size(); }

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