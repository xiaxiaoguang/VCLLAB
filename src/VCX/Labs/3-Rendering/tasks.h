#pragma once

#include <numeric>
#include <spdlog/spdlog.h>

#include "Engine/Scene.h"
#include "Labs/3-Rendering/Ray.h"

namespace VCX::Labs::Rendering {

    constexpr float EPS1 = 1e-2f; // distance to prevent self-intersection
    constexpr float EPS2 = 1e-8f; // angle for parallel judgement
    constexpr float EPS3 = 1e-4f; // relative distance to enlarge kdtree

    glm::vec4 GetTexture(Engine::Texture2D<Engine::Formats::RGBA8> const & texture, glm::vec2 const & uvCoord);

    glm::vec4 GetAlbedo(Engine::Material const & material, glm::vec2 const & uvCoord);

    struct Intersection {
        float t, u, v; // ray parameter t, barycentric coordinates (u, v)
    };

    bool IntersectTriangle(Intersection & output, Ray const & ray, glm::vec3 const & p1, glm::vec3 const & p2, glm::vec3 const & p3);

    struct RayHit {
        bool              IntersectState;
        Engine::BlendMode IntersectMode;
        Engine::ReflectionType IntersectRel;
        glm::vec3         IntersectPosition;
        glm::vec3         IntersectNormal;
        glm::vec3         IntersectColor;
        glm::vec3         IntersectEmission;
        glm::vec4         IntersectAlbedo;   // [Albedo   (vec3), Alpha     (float)]
        glm::vec4         IntersectMetaSpec; // [Specular (vec3), Shininess (float)]
    };

    struct TrivialRayIntersector {
        Engine::Scene const * InternalScene = nullptr;

        TrivialRayIntersector() = default;

        void InitScene(Engine::Scene const * scene) {
            InternalScene = scene;
        }

        RayHit IntersectRay(Ray const & ray) const {
            RayHit result;
            
            if (! InternalScene) {
                spdlog::warn("VCX::Labs::Rendering::RayIntersector::IntersectRay(..): uninitialized intersector.");
                result.IntersectState = false;
                return result;
            }

            int          modelIdx, meshIdx;
            Intersection its;
            float        tmin     = 1e7, umin, vmin;
            int          maxmodel = InternalScene->Models.size();
            // printf("Now Intersect and Maxmodel is %d\n",maxmodel);
            for (int i = 0; i < maxmodel; ++i) {
                auto const & model  = InternalScene->Models[i];
                // 选择一个model
                int          maxidx = model.Mesh.Indices.size();
                // 然后看看最大的下标是啥
                for (int j = 0; j < maxidx; j += 3) {
                    std::uint32_t const * face = model.Mesh.Indices.data() + j;
                    // face是对应的面
                    glm::vec3 const &     p1   = model.Mesh.Positions[face[0]];
                    glm::vec3 const &     p2   = model.Mesh.Positions[face[1]];
                    glm::vec3 const &     p3   = model.Mesh.Positions[face[2]];
                    // 三个面,如果都没交上就自闭
                    if (! IntersectTriangle(its, ray, p1, p2, p3)) continue;
                    if (its.t < EPS1 || its.t > tmin) continue;
                    tmin = its.t, umin = its.u, vmin = its.v, modelIdx = i, meshIdx = j;
                }
            }

            if (tmin == 1e7) {
                result.IntersectState = false;
                return result;
            }

            auto const &          model     = InternalScene->Models[modelIdx];
            auto const &          normals   = model.Mesh.IsNormalAvailable() ? model.Mesh.Normals : model.Mesh.ComputeNormals();
            auto const &          texcoords = model.Mesh.IsTexCoordAvailable() ? model.Mesh.TexCoords : model.Mesh.GetEmptyTexCoords();

            std::uint32_t const * face      = model.Mesh.Indices.data() + meshIdx;
            glm::vec3 const &     p1        = model.Mesh.Positions[face[0]];
            glm::vec3 const &     p2        = model.Mesh.Positions[face[1]];
            glm::vec3 const &     p3        = model.Mesh.Positions[face[2]];
            glm::vec3 const &     n1        = normals[face[0]];
            glm::vec3 const &     n2        = normals[face[1]];
            glm::vec3 const &     n3        = normals[face[2]];
            glm::vec2 const &     uv1       = texcoords[face[0]];
            glm::vec2 const &     uv2       = texcoords[face[1]];
            glm::vec2 const &     uv3       = texcoords[face[2]];

            result.IntersectState           = true;
            auto const & material           = InternalScene->Materials[model.MaterialIndex];
            result.IntersectMode            = material.Blend;
            
            result.IntersectPosition        = (1.0f - umin - vmin) * p1 + umin * p2 + vmin * p3;
            result.IntersectNormal          = (1.0f - umin - vmin) * n1 + umin * n2 + vmin * n3;
            glm::vec2 uvCoord               = (1.0f - umin - vmin) * uv1 + umin * uv2 + vmin * uv3;

            result.IntersectAlbedo          = GetAlbedo(material, uvCoord);
            result.IntersectEmission        = glm::vec3();
            result.IntersectRel             = material.Rel;

            // if(material.Rel == Engine::ReflectionType::PhysicalSpecular){
            //     puts("Yes");
            //     exit(0);
            // }

            result.IntersectMetaSpec        = GetTexture(material.MetaSpec, uvCoord);

            if(material.Blend == Engine::BlendMode::Transparent){
                // result.IntersectAlbedo   = glm::vec4();
                // auto a = result.IntersectMetaSpec.w / 0.003922;
                auto a = InternalScene->Lights[0].Intensity;
                // printf("%f %f %f %f \n",result.IntersectMetaSpec[0],result.IntersectMetaSpec[1],result.IntersectMetaSpec[2],result.IntersectMetaSpec[3]);
                // exit(0);
                result.IntersectRel    = Engine::ReflectionType::PhysicalDiffusion;
                result.IntersectAlbedo = glm::vec4(0,0,0,0);
                result.IntersectMetaSpec = glm::vec4(a,0);
            }
            return result;
        }
    };

    /* Optional: write your own accelerated intersector here */

    using RayIntersector = TrivialRayIntersector;

    glm::vec3 RayTrace(const RayIntersector & intersector, Ray ray, unsigned short * Xi, int mxd);

} // namespace VCX::Labs::Rendering
