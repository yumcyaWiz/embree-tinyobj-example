#ifndef _SCENE_H
#define _SCENE_H
#include <embree3/rtcore.h>
#include <spdlog/spdlog.h>

#include <filesystem>
#include <string>
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
  // triangles
  // NOTE: size of normals, texcoords == size of vertices
  std::vector<float> vertices;
  std::vector<uint32_t> indices;
  std::vector<float> normals;
  std::vector<float> texcoords;

  // materials
  std::vector<Material> materials;

  // embree
  RTCDevice device;
  RTCScene scene;

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

  // return vertex normal
  Vec3 getVertexNormal(uint32_t vertexID) const {
    return Vec3(normals[3 * vertexID + 0], normals[3 * vertexID + 1],
                normals[3 * vertexID + 2]);
  }

  // return vertex indices of specified face
  // TODO: use Vec3i
  struct VertexIndices {
    uint32_t v1idx;
    uint32_t v2idx;
    uint32_t v3idx;
  };
  VertexIndices getIndices(uint32_t faceID) const {
    VertexIndices ret;
    ret.v1idx = indices[3 * faceID + 0];
    ret.v2idx = indices[3 * faceID + 1];
    ret.v3idx = indices[3 * faceID + 2];
    return ret;
  }

  // compute normal of specified face
  Vec3 getFaceNormal(uint32_t faceID, const Vec2& uv) const {
    const VertexIndices vidx = getIndices(faceID);
    const Vec3 n1 = getVertexNormal(vidx.v1idx);
    const Vec3 n2 = getVertexNormal(vidx.v2idx);
    const Vec3 n3 = getVertexNormal(vidx.v3idx);
    return n1 * (1.0f - uv[0] - uv[1]) + n2 * uv[0] + n3 * uv[1];
  }

  void clear() {
    vertices.clear();
    indices.clear();
    normals.clear();
    texcoords.clear();
    materials.clear();
  }

 public:
  // load obj file
  // TODO: remove vertex duplication
  void loadModel(const std::filesystem::path& filepath) {
    clear();

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

        // if normals is empty, add geometric normal
        if (normals.size() == 0) {
          const Vec3 v1 = normalize(vertices[1] - vertices[0]);
          const Vec3 v2 = normalize(vertices[2] - vertices[0]);
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

        for (int i = 0; i < 3; ++i) {
          this->vertices.push_back(vertices[i][0]);
          this->vertices.push_back(vertices[i][1]);
          this->vertices.push_back(vertices[i][2]);

          this->normals.push_back(normals[i][0]);
          this->normals.push_back(normals[i][1]);
          this->normals.push_back(normals[i][2]);

          this->texcoords.push_back(texcoords[i][0]);
          this->texcoords.push_back(texcoords[i][1]);

          this->indices.push_back(this->indices.size());
        }

        // add material
        // TODO: remove duplicate
        const int materialID = shapes[s].mesh.material_ids[f];
        const tinyobj::material_t& m = materials[materialID];
        this->materials.push_back(createMaterial(m));

        index_offset += fv;
      }
    }

    spdlog::info("[Scene] vertices: {}", nVertices());
    spdlog::info("[Scene] faces: {}", nFaces());
    spdlog::info("[Scene] materials: {}", nMaterials());
  }

  uint32_t nVertices() const { return vertices.size() / 3; }
  uint32_t nFaces() const { return indices.size() / 3; }
  uint32_t nMaterials() const { return materials.size(); }

  // setup embree
  void build() {
    device = rtcNewDevice(NULL);
    scene = rtcNewScene(device);

    RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

    // set vertices
    float* vb = (float*)rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0,
                                                RTC_FORMAT_FLOAT3,
                                                3 * sizeof(float), nVertices());
    for (int i = 0; i < vertices.size(); ++i) {
      vb[i] = vertices[i];
    }

    // set indices
    unsigned* ib = (unsigned*)rtcSetNewGeometryBuffer(
        geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3 * sizeof(unsigned),
        nFaces());
    for (int i = 0; i < indices.size(); ++i) {
      ib[i] = indices[i];
    }

    rtcCommitGeometry(geom);
    rtcAttachGeometry(scene, geom);
    rtcReleaseGeometry(geom);
    rtcCommitScene(scene);
  }

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
      info.primID = rayhit.hit.primID;

      info.surfaceInfo.position = ray(info.t);
      info.surfaceInfo.barycentric = Vec2(rayhit.hit.u, rayhit.hit.v);
      info.surfaceInfo.normal =
          getFaceNormal(rayhit.hit.primID, Vec2(rayhit.hit.u, rayhit.hit.v));
      orthonormalBasis(info.surfaceInfo.normal, info.surfaceInfo.dpdu,
                       info.surfaceInfo.dpdv);

      info.material = &materials[rayhit.hit.primID];
      return true;
    } else {
      return false;
    }
  }
};

#endif