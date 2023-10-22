#include <unordered_map>

#include <glm/gtc/matrix_inverse.hpp>
#include <spdlog/spdlog.h>

#include "Labs/2-GeometryProcessing/DCEL.hpp"
#include "Labs/2-GeometryProcessing/tasks.h"

namespace VCX::Labs::GeometryProcessing {

#include "Labs/2-GeometryProcessing/marching_cubes_table.h"

    /******************* 1. Mesh Subdivision *****************/
    void SubdivisionMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, std::uint32_t numIterations) {
        Engine::SurfaceMesh curr_mesh = input;
        // We do subdivison iteratively.
        for (std::uint32_t it = 0; it < numIterations; ++it) {
            // During each iteration, we first move curr_mesh into prev_mesh.
            Engine::SurfaceMesh prev_mesh;
            prev_mesh.Swap(curr_mesh);
            // Then we create doubly connected edge list.
            DCEL G(prev_mesh);
            if (! G.IsManifold()) {
                spdlog::warn("VCX::Labs::GeometryProcessing::SubdivisionMesh(..): Non-manifold mesh.");
                return;
            }
            // Note that here curr_mesh has already been empty.
            // We reserve memory first for efficiency.
            curr_mesh.Positions.reserve(prev_mesh.Positions.size() * 3 / 2);
            curr_mesh.Indices.reserve(prev_mesh.Indices.size() * 4);
            // Then we iteratively update currently existing vertices.
            for (std::size_t i = 0; i < prev_mesh.Positions.size(); ++i) {
                // Update the currently existing vetex v from prev_mesh.Positions.
                // Then add the updated vertex into curr_mesh.Positions.
                auto v           = G.Vertex(i);
                auto neighbors   = v->Neighbors();
                // your code here:
            }
            // We create an array to store indices of the newly generated vertices.
            // Note: newIndices[i][j] is the index of vertex generated on the "opposite edge" of j-th
            //       vertex in the i-th triangle.
            std::vector<std::array<std::uint32_t, 3U>> newIndices(prev_mesh.Indices.size() / 3, { ~0U, ~0U, ~0U });
            // Iteratively process each halfedge.
            for (auto e : G.Edges()) {
                // newIndices[face index][vertex index] = index of the newly generated vertex
                newIndices[G.IndexOf(e->Face())][e->EdgeLabel()] = curr_mesh.Positions.size();
                auto eTwin                                       = e->TwinEdgeOr(nullptr);
                // eTwin stores the twin halfedge.
                if (! eTwin) {
                    // When there is no twin halfedge (so, e is a boundary edge):
                    // your code here: generate the new vertex and add it into curr_mesh.Positions.
                } else {
                    // When the twin halfedge exists, we should also record:
                    //     newIndices[face index][vertex index] = index of the newly generated vertex
                    // Because G.Edges() will only traverse once for two halfedges,
                    //     we have to record twice.
                    newIndices[G.IndexOf(eTwin->Face())][e->TwinEdge()->EdgeLabel()] = curr_mesh.Positions.size();
                    // your code here: generate the new vertex and add it into curr_mesh.Positions.
                }
            }

            // Here we've already build all the vertices.
            // Next, it's time to reconstruct face indices.
            for (std::size_t i = 0; i < prev_mesh.Indices.size(); i += 3U) {
                // For each face F in prev_mesh, we should create 4 sub-faces.
                // v0,v1,v2 are indices of vertices in F.
                // m0,m1,m2 are generated vertices on the edges of F.
                auto v0           = prev_mesh.Indices[i + 0U];
                auto v1           = prev_mesh.Indices[i + 1U];
                auto v2           = prev_mesh.Indices[i + 2U];
                auto [m0, m1, m2] = newIndices[i / 3U];
                // Note: m0 is on the opposite edge (v1-v2) to v0.
                // Please keep the correct indices order (consistent with order v0-v1-v2)
                //     when inserting new face indices.
                // toInsert[i][j] stores the j-th vertex index of the i-th sub-face.
                std::uint32_t toInsert[4][3] = {
                    // your code here:
                };
                // Do insertion.
                curr_mesh.Indices.insert(
                    curr_mesh.Indices.end(),
                    reinterpret_cast<std::uint32_t *>(toInsert),
                    reinterpret_cast<std::uint32_t *>(toInsert) + 12U
                );
            }

            if (curr_mesh.Positions.size() == 0) {
                spdlog::warn("VCX::Labs::GeometryProcessing::SubdivisionMesh(..): Empty mesh.");
                output = input;
                return;
            }
        }
        // Update output.
        output.Swap(curr_mesh);
    }

    /******************* 2. Mesh Parameterization *****************/
    void Parameterization(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, const std::uint32_t numIterations) {
        // Copy.
        output = input;
        // Reset output.TexCoords.
        output.TexCoords.resize(input.Positions.size(), glm::vec2 { 0 });

        // Build DCEL.
        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::Parameterization(..): non-manifold mesh.");
            return;
        }

        // Set boundary UVs for boundary vertices.
        // your code here: directly edit output.TexCoords

        // Solve equation via Gauss-Seidel Iterative Method.
        for (int k = 0; k < numIterations; ++k) {
            // your code here:
        }
    }

    /******************* 3. Mesh Simplification *****************/
    void SimplifyMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, float simplification_ratio) {

        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Non-manifold mesh.");
            return;
        }
        // We only allow watertight mesh.
        if (! G.IsWatertight()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Non-watertight mesh.");
            return;
        }

        // Copy.
        output = input;

        // Compute Q matrix of the face f.
        auto UpdateQ {
            [&G, &output] (DCEL::Triangle const * f) -> glm::mat4 {
                glm::mat4 Q;
                // your code here:
                return Q;
            }
        };

        // The struct to record constraction info.
        struct ConstractionPair {
            DCEL::HalfEdge const * edge;            // which edge to constract; if $edge == nullptr$, it means this pair is no longer valid
            glm::vec4              targetPosition;  // the targetPosition $v$ for vertex $edge->From()$ to move to
            float                  cost;            // the cost $v.T * Qbar * v$
        };

        // Given an edge (v1->v2), the positions of its two endpoints (p1, p2) and the Q matrix (Q1+Q2),
        //     return the ConstractionPair struct.
        static constexpr auto MakePair {
            [] (DCEL::HalfEdge const * edge,
                glm::vec3 const & p1,
                glm::vec3 const & p2,
                glm::mat4 const & Q
            ) -> ConstractionPair {
                // your code here:
                return {};
            }
        };

        // pair_map: map EdgeIdx to index of $pairs$
        // pairs:    store ConstractionPair
        // Qv:       $Qv[idx]$ is the Q matrix of vertex with index $idx$
        // Qf:       $Qf[idx]$ is the Q matrix of face with index $idx$
        std::unordered_map<DCEL::EdgeIdx, std::size_t> pair_map; 
        std::vector<ConstractionPair>                  pairs; 
        std::vector<glm::mat4>                         Qv(G.NumOfVertices(), glm::mat4(0));
        std::vector<glm::mat4>                         Qf(G.NumOfFaces(),    glm::mat4(0));

        // Initially, we compute Q matrix for each faces and it accumulates at each vertex.
        for (auto f : G.Faces()) {
            auto Q                 = UpdateQ(f);
            Qv[f->VertexIndex(0)] += Q;
            Qv[f->VertexIndex(1)] += Q;
            Qv[f->VertexIndex(2)] += Q;
            Qf[G.IndexOf(f)]       = Q;
        }

        pair_map.reserve(G.NumOfFaces() * 3);
        pairs.reserve(G.NumOfFaces() * 3 / 2);

        // Initially, we make pairs from all the constractable edges.
        for (auto e : G.Edges()) {
            if (! G.IsConstractable(e)) continue;
            auto v1                            = e->From();
            auto v2                            = e->To();
            auto pair                          = MakePair(e, input.Positions[v1], input.Positions[v2], Qv[v1] + Qv[v2]);
            pair_map[G.IndexOf(e)]             = pairs.size();
            pair_map[G.IndexOf(e->TwinEdge())] = pairs.size();
            pairs.emplace_back(pair);
        }

        // Loop until the number of vertices is less than $simplification_ratio * initial_size$.
        while (G.NumOfVertices() > simplification_ratio * Qv.size()) {
            // Find the constractable pair with minimal cost.
            std::size_t min_idx = ~0;
            for (std::size_t i = 1; i < pairs.size(); ++i) {
                if (! pairs[i].edge) continue;
                if (!~min_idx || pairs[i].cost < pairs[min_idx].cost) {
                    if (G.IsConstractable(pairs[i].edge)) min_idx = i;
                    else pairs[i].edge = nullptr;
                }
            }
            if (!~min_idx) break;

            // top:    the constractable pair with minimal cost
            // v1:     the reserved vertex
            // v2:     the removed vertex
            // result: the constract result
            // ring:   the edge ring of vertex v1
            ConstractionPair & top    = pairs[min_idx];
            auto               v1     = top.edge->From();
            auto               v2     = top.edge->To();
            auto               result = G.Constract(top.edge);
            auto               ring   = G.Vertex(v1)->Ring();

            top.edge             = nullptr;            // The constraction has already been done, so the pair is no longer valid. Mark it as invalid.
            output.Positions[v1] = top.targetPosition; // Update the positions.

            // We do something to repair $pair_map$ and $pairs$ because some edges and vertices no longer exist.
            for (int i = 0; i < 2; ++i) {
                DCEL::EdgeIdx removed           = G.IndexOf(result.removed_edges[i].first);
                DCEL::EdgeIdx collapsed         = G.IndexOf(result.collapsed_edges[i].second);
                pairs[pair_map[removed]].edge   = result.collapsed_edges[i].first;
                pairs[pair_map[collapsed]].edge = nullptr;
                pair_map[collapsed]             = pair_map[G.IndexOf(result.collapsed_edges[i].first)];
            }

            // For the two wing vertices, each of them lose one incident face.
            // So, we update the Q matrix.
            Qv[result.removed_faces[0].first] -= Qf[G.IndexOf(result.removed_faces[0].second)];
            Qv[result.removed_faces[1].first] -= Qf[G.IndexOf(result.removed_faces[1].second)];

            // For the vertex v1, Q matrix should be recomputed.
            // And as the position of v1 changed, all the vertices which are on the ring of v1 should update their Q matrix as well.
            Qv[v1] = glm::mat4(0);
            for (auto e : ring) {
                // your code here:
                //     1. Compute the new Q matrix for $e->Face()$.
                //     2. According to the difference between the old Q (in $Qf$) and the new Q (computed in step 1),
                //        update Q matrix of each vertex on the ring (update $Qv$).
                //     3. Update Q matrix of vertex v1 as well (update $Qv$).
                //     4. Update $Qf$.
            }

            // Finally, as the Q matrix changed, we should update the relative $ConstractionPair$ in $pairs$.
            // Any pair with the Q matrix of its endpoints changed, should be remade by $MakePair$.
            // your code here:

        }

        // In the end, we check if the result mesh is watertight and manifold.
        if (! G.DebugWatertightManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Result is not watertight manifold.");
        }

        auto exported = G.ExportMesh();
        output.Indices.swap(exported.Indices);
    }

    /******************* 4. Mesh Smoothing *****************/
    void SmoothMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, std::uint32_t numIterations, float lambda, bool useUniformWeight) {
        // Define function to compute cotangent value of the angle v1-vAngle-v2
        static constexpr auto GetCotangent {
            [] (glm::vec3 vAngle, glm::vec3 v1, glm::vec3 v2) -> float {
                // your code here:
                return 0.0;
            }
        };

        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SmoothMesh(..): Non-manifold mesh.");
            return;
        }
        // We only allow watertight mesh.
        if (! G.IsWatertight()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SmoothMesh(..): Non-watertight mesh.");
            return;
        }

        Engine::SurfaceMesh prev_mesh;
        prev_mesh.Positions = input.Positions;
        for (std::uint32_t iter = 0; iter < numIterations; ++iter) {
            Engine::SurfaceMesh curr_mesh = prev_mesh;
            for (std::size_t i = 0; i < input.Positions.size(); ++i) {
                // your code here: curr_mesh.Positions[i] = ...
            }
            // Move curr_mesh to prev_mesh.
            prev_mesh.Swap(curr_mesh);
        }
        // Move prev_mesh to output.
        output.Swap(prev_mesh);
        // Copy indices from input.
        output.Indices = input.Indices;
    }

    /******************* 5. Marching Cubes *****************/
    void MarchingCubes(Engine::SurfaceMesh & output, const std::function<float(const glm::vec3 &)> & sdf, const glm::vec3 & grid_min, const float dx, const int n) {
        // your code here:
    }
} // namespace VCX::Labs::GeometryProcessing
