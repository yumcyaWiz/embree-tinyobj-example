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

  Vertex() {
    vertex[0] = vertex[1] = vertex[2] = 0;
    normal[0] = normal[1] = normal[2] = 0;
    texcoords[0] = texcoords[1] = texcoords[2] = 0;
  }

  bool operator==(const Vertex& v) const {
    return vertex[0] == v.vertex[0] && vertex[1] == v.vertex[1] &&
           vertex[2] == v.vertex[2] && normal[0] == v.normal[0] &&
           normal[1] == v.normal[1] && normal[2] == v.normal[2] &&
           texcoords[0] == v.texcoords[0] && texcoords[1] == v.texcoords[1];
  }

  // NOTE: used for unordered_map's key
  std::string toString() const {
    return std::to_string(vertex[0]) + "," + std::to_string(vertex[1]) + ", " +
           std::to_string(vertex[2]) + "," + std::to_string(normal[0]) + "," +
           std::to_string(normal[1]) + "," + std::to_string(normal[2]) + "," +
           std::to_string(texcoords[0]) + "," + std::to_string(texcoords[1]);
  }
};

class Scene {
 private:
  // embree
  RTCDevice device;
  RTCScene scene;

  // triangles
  // NOTE: size of normals, texcoords == size of vertices
  std::vector<float> vertices;
  std::vector<uint32_t> indices;
  std::vector<float> normals;
  std::vector<float> texcoords;

  // materials
  std::vector<Material> materials;

  // create material from tinyobj material
  static Material createMaterial(const tinyobj::material_t& material) {
    const Vec3 kd =
        Vec3(material.diffuse[0], material.diffuse[1], material.diffuse[2]);
    const Vec3 ks =
        Vec3(material.specular[0], material.specular[1], material.specular[2]);
    const Vec3 ke =
        Vec3(material.emission[0], material.emission[1], material.emission[2]);
    return Material(kd, ks, ke);
  }

 public:
  void loadModel(const std::filesystem::path& filepath) {
    spdlog::info("[Scene] loading: {}", filepath.generic_string());

    tinyobj::ObjReaderConfig reader_config;
    reader_config.mtl_search_path = "./";
    reader_config.triangulate = true;

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

        std::vector<Vec3> vertices;
        std::vector<Vec3> normals;
        std::vector<Vec2> texcoords;

        // loop over vertices
        // get vertices, normals, texcoords of a triangle
        for (size_t v = 0; v < fv; ++v) {
          const tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];

          const tinyobj::real_t vx =
              attrib.vertices[3 * static_cast<size_t>(idx.vertex_index) + 0];
          const tinyobj::real_t vy =
              attrib.vertices[3 * static_cast<size_t>(idx.vertex_index) + 1];
          const tinyobj::real_t vz =
              attrib.vertices[3 * static_cast<size_t>(idx.vertex_index) + 2];
          vertices.push_back(Vec3(vx, vy, vz));

          if (idx.normal_index >= 0) {
            const tinyobj::real_t nx =
                attrib.normals[3 * static_cast<size_t>(idx.normal_index) + 0];
            const tinyobj::real_t ny =
                attrib.normals[3 * static_cast<size_t>(idx.normal_index) + 1];
            const tinyobj::real_t nz =
                attrib.normals[3 * static_cast<size_t>(idx.normal_index) + 2];

            normals.push_back(Vec3(nx, ny, nz));
          }

          if (idx.texcoord_index >= 0) {
            const tinyobj::real_t tx =
                attrib
                    .texcoords[3 * static_cast<size_t>(idx.texcoord_index) + 0];
            const tinyobj::real_t ty =
                attrib
                    .texcoords[3 * static_cast<size_t>(idx.texcoord_index) + 1];

            texcoords.push_back(Vec2(tx, ty));
          }
        }

        // if normals is empty, add face normal
        if (normals.size() == 0) {
          const Vec3 v1 = normalize(vertices[1] - vertices[0]);
          const Vec3 v2 = normalize(vertices[1] - vertices[0]);
          const Vec3 n = normalize(cross(v1, v2));
          normals.push_back(n);
          normals.push_back(n);
          normals.push_back(n);
        }

        // if texcoords is empty, add barycentric coords
        if (texcoords.size() == 0) {
          texcoords.push_back(Vec2(0, 0));
          texcoords.push_back(Vec2(1, 0));
          texcoords.push_back(Vec2(0, 1));
        }

        // populate vertices, indices, normals, texcoords array
        for (int i = 0; i < 3; ++i) {
          Vertex vertex;
          vertex.vertex[0] = vertices[i][0];
          vertex.vertex[1] = vertices[i][1];
          vertex.vertex[2] = vertices[i][2];

          vertex.normal[0] = normals[i][0];
          vertex.normal[1] = normals[i][1];
          vertex.normal[2] = normals[i][2];

          vertex.texcoords[0] = texcoords[i][0];
          vertex.texcoords[1] = texcoords[i][1];

          // remove dupulicate
          // https://vulkan-tutorial.com/Loading_models
          // See Vertex deduplication section
          const std::string key = vertex.toString();
          if (uniqueVertices.count(key) == 0) {
            uniqueVertices[key] = static_cast<uint32_t>(vertices.size());

            this->vertices.push_back(vertex.vertex[0]);
            this->vertices.push_back(vertex.vertex[1]);
            this->vertices.push_back(vertex.vertex[2]);

            this->normals.push_back(vertex.normal[0]);
            this->normals.push_back(vertex.normal[1]);
            this->normals.push_back(vertex.normal[2]);

            this->texcoords.push_back(vertex.texcoords[0]);
            this->texcoords.push_back(vertex.texcoords[1]);
          }

          this->indices.push_back(uniqueVertices[key]);
        }
        index_offset += fv;

        // add material
        // TODO: remove duplicate
        const int materialID = shapes[s].mesh.material_ids[f];
        const tinyobj::material_t& m = materials[materialID];
        this->materials.push_back(createMaterial(m));
      }
    }

    spdlog::info("[Scene] vertices: {}", nVertices());
    spdlog::info("[Scene] faces: {}", nFaces());
    spdlog::info("[Scene] materials: {}", nMaterials());
  }

  uint32_t nVertices() const { return indices.size(); }
  uint32_t nFaces() const { return indices.size() / 3; }
  uint32_t nMaterials() const { return materials.size(); }

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