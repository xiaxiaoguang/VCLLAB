#include <unordered_map>

#include <glm/gtc/matrix_inverse.hpp>
#include <spdlog/spdlog.h>

#include "Labs/2-GeometryProcessing/DCEL.hpp"
#include "Labs/2-GeometryProcessing/tasks.h"

namespace VCX::Labs::GeometryProcessing {

#include "Labs/2-GeometryProcessing/marching_cubes_table.h"

    /******************* 1. Mesh Subdivision *****************/
    glm::vec3 operator+(const glm::vec3 & a,const glm::vec3 & b){
        return glm::vec3(a[0]+b[0],a[1]+b[1],a[2]+b[2]);
    }

    glm::vec3 operator-(const glm::vec3 & a,const glm::vec3 & b){
        return glm::vec3(a[0]-b[0],a[1]-b[1],a[2]-b[2]);
    }

    glm::vec3 operator*(const glm::vec3 & a,const double b){
        return glm::vec3(a[0]*b,a[1]*b,a[2]*b);
    }

    glm::vec3 operator/(const glm::vec3 & a,const double & b){
        return glm::vec3(a[0]/b,a[1]/b,a[2]/b);
    }

    glm::vec2 operator+(const glm::vec2 & a,const glm::vec2 & b){
        return glm::vec2(a[0]+b[0],a[1]+b[1]);
    }

    glm::vec2 operator+(const glm::vec2 & a,const int & b){
        return glm::vec2(a[0]+b,a[1]+b);
    }

    glm::vec2 operator-(const glm::vec2 & a,const glm::vec2 & b){
        return glm::vec2(a[0]-b[0],a[1]-b[1]);
    }

    glm::vec2 operator-(const int & a,const glm::vec2 & b){
        return glm::vec2(a-b[0],a-b[1]);
    }

    glm::vec2 operator*(const glm::vec2 & a,const double b){
        return glm::vec2(a[0]*b,a[1]*b);
    }

    glm::vec2 operator/(const glm::vec2 & a,const double & b){
        return glm::vec2(a[0]/b,a[1]/b);
    }

    glm::vec4 operator/(const glm::vec4 &a,const double &b){
        return glm::vec4(a[0]/b,a[1]/b,a[2]/b,a[3]/b);
    }

    inline void outnode(const glm::vec3 & a){
        printf("(%lf,%lf,%lf)\n",a[0],a[1],a[2]);
        return ;
    }

    void SubdivisionMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, std::uint32_t numIterations) {
        Engine::SurfaceMesh curr_mesh = input;
        // We do subdivison iteratively.
        for (std::uint32_t it = 0; it < numIterations; ++it) {
            // During each iteration, we first move curr_mesh into prev_mesh.
            Engine::SurfaceMesh prev_mesh;
            prev_mesh.Swap(curr_mesh);// 在这里交换来保证迭代不改变同时有curr_mesh的信息
            DCEL G(prev_mesh);
            if (! G.IsManifold()) {
                spdlog::warn("VCX::Labs::GeometryProcessing::SubdivisionMesh(..): Non-manifold mesh.");
                return;
            }
            // printf("%d %d %d?\n ",prev_mesh.Positions[0][0],prev_mesh.Positions[0][1],prev_mesh.Positions[0][2]);
            // Note that here curr_mesh has already been empty.
            // We reserve memory first for efficiency.
            curr_mesh.Positions.reserve(prev_mesh.Positions.size() + prev_mesh.Indices.size());//分配空间
            curr_mesh.Indices.reserve(prev_mesh.Indices.size() * 4);
            // Then we iteratively update currently existing vertices.
            for (std::size_t i = 0; i < prev_mesh.Positions.size(); ++i) {
                // Update the currently existing vetex v from prev_mesh.Positions.
                // Then add the updated vertex into curr_mesh.Positions.
                auto v           = G.Vertex(i);
                auto neighbors   = v->Neighbors(); // v是节点集合，然后neighbors是邻居节点信息。都是数组变量
                int n = neighbors.size();
                double u = 3 / (8.0 * (n == 3 ? 2 : n));
                auto nv = prev_mesh.Positions[i] * (1-n*u);

                for(auto to : neighbors)nv = nv + prev_mesh.Positions[to] * u;
                curr_mesh.Positions.emplace_back(nv);
            }
            // We create an array to store indices of the newly generated vertices.
            // Note: newIndices[i][j] is the index of vertex generated on the "opposite edge" of j-th
            //       vertex in the i-th triangle.
            std::vector<std::array<std::uint32_t, 3U>> newIndices(prev_mesh.Indices.size() / 3, { ~0U, ~0U, ~0U });
            // Iteratively process each halfedge.
            int cnt=0;
            for (auto e : G.Edges()) {
                cnt++;
                // newIndices[face index][vertex index] = index of the newly generated vertex
                newIndices[G.IndexOf(e->Face())][e->EdgeLabel()] = curr_mesh.Positions.size();
                auto eTwin                                       = e->TwinEdgeOr(nullptr);
                // eTwin stores the twin halfedge.
                if (! eTwin) {
                    // When there is no twin halfedge (so, e is a boundary edge):
                    // your code here: generate the new vertex and add it into curr_mesh.Positions.
                    auto x = prev_mesh.Positions[e->OppositeVertex()];
                    auto v = prev_mesh.Positions[e->To()];
                    auto u = prev_mesh.Positions[e->From()];
                    auto nv =( x * 2 + v * 3 + u * 3 ) / 8.0;
                    curr_mesh.Positions.emplace_back(nv);
                } else {
                    // When the twin halfedge exists, we should also record:
                    //     newIndices[face index][vertex index] = index of the newly generated vertex
                    // Because G.Edges() will only traverse once for two halfedges,
                    //     we have to record twice.
                    newIndices[G.IndexOf(eTwin->Face())][e->TwinEdge()->EdgeLabel()] = curr_mesh.Positions.size();
                    // your code here: generate the new vertex and add it into curr_mesh.Positions.
                    auto x = prev_mesh.Positions[e->OppositeVertex()];
                    auto y = prev_mesh.Positions[e->TwinOppositeVertex()];
                    auto v = prev_mesh.Positions[e->To()];
                    auto u = prev_mesh.Positions[e->From()];
                    auto nv=( x + y + v * 3 + u * 3 ) / 8.0;
                    curr_mesh.Positions.emplace_back(nv);
                }
            }
            // printf("%d %d\n",cnt,prev_mesh.Indices.size());
            // for(auto p : curr_mesh.Positions){
            //     outnode(p);
            // }
            // puts("");
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
                // printf("%d %d %d %d %d %d\n",v0,v1,v2,m0,m1,m2);
                // outnode(prev_mesh.Positions[v0]);
                // outnode(prev_mesh.Positions[v1]);
                // outnode(prev_mesh.Positions[v2]);
                // outnode(curr_mesh.Positions[m0]);
                // outnode(curr_mesh.Positions[m1]);
                // outnode(curr_mesh.Positions[m2]);
                // Note: m0 is on the opposite edge (v1-v2) to v0.
                // Please keep the correct indices order (consistent with order v0-v1-v2)
                //     when inserting new face indices.
                // toInsert[i][j] stores the j-th vertex index of the i-th sub-face.
                std::uint32_t toInsert[4][3] = { // 按照逆时针顺序来确定每个节点，每个平面具体从哪个点开始无关
                    // your code here:
                    {m2,v1,m0},
                    {m0,v2,m1},
                    {m1,m2,m0},
                    {v0,m2,m1},
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
        // printf("%d %d %d\n",output.TexCoords.size(),output.Positions.size(),input.TexCoords.size());

        std::vector<int> to,que;
        to.resize(output.Positions.size());
        for(int i=0;i<to.size();++i)to[i]=0;
        int rc=-1;
        for (auto e : G.Edges()){
            auto eTwin = e->TwinEdgeOr(nullptr);
            if(!eTwin){
                auto x = e->To();
                auto y = e->From();
                if(!to[x])to[x]=y;
                else to[y]=x;//连一圈
                if(rc==-1)rc=x;
            }
        }
        output.TexCoords[rc]={0,0};
        que.emplace_back(rc);
        for(int i=to[rc];i!=rc;i=to[i])que.emplace_back(i);
        int s = que.size();
        int p = s/4;
        double dp = 1.0/p;
        double res = 1.0/(s-p-p-p);
        for(int i=0;i<s;++i){
            auto &u=output.TexCoords[que[i]];
            // y=0,x=0-1
            // x=1,y=0-1
            // y=1,x=1-0
            // x=0,y=1-0
            if(i >= s-p){
                u = {0.0,(s-i)*dp};
            }else if(i >= s-p-p){
                u = {(s-p-i)*dp,1.0};
            }else if(i >= s-p-p-p){
                u={1.0,(i-s+3*p)*dp};
            }else {
                u={res*i,0.0};
            }
        }
        // Solve equation via Gauss-Seidel Iterative Method.
        for (int k = 0; k < numIterations; ++k) {
            // your code here:
            for(int id=0;id<output.Positions.size();++id){
                if(to[id])continue;
                auto v = G.Vertex(id);
                auto neighbors = v->Neighbors();
                double n = 1.0/neighbors.size();
                glm::vec2 nv = {0,0};
                for(auto v : neighbors)
                    nv = nv + output.TexCoords[v] * n;
                output.TexCoords[id]=nv;
            }
        }
        return ;
    }
    const double eps = 1e-5;
    const double inf = 1e5;
    using point = glm::vec3;
    using CostMatrix = glm::mat4;
    using std::max;
    using std::min;
    struct planes{
        double A,B,C,D;
        planes(double a=0,double b=0,double c=0,double d=0):A(a),B(b),C(c),D(d){};
        double operator[] (int x){
            if(x==0)return A;
            if(x==1)return B;
            if(x==2)return C;
            if(x==3)return D;
            throw;
        }
        planes operator * (const double &x){
            return planes(A*x,B*x,C*x,D*x);
        }
        planes getv(const point &a){
            return a[0]*A + a[1]*B+ a[2]*C +D;
        }
    };
    void outMat(const CostMatrix &A){
        for(int i=0;i<4;++i,puts("]")){
            putchar('[');
            for(int j=0;j<4;++j){
                printf("%.6f,",A[i][j]);
            }
        }
        return ;
    }    
    void outplanes(const planes& a){
        printf("%lf %lf %lf %lf\n",a.A,a.B,a.C,a.D);
        return ;
    }
    planes calc_planes(point a,point b,point c){
            planes tmp;
            //     a = (p2.y - p1.y)*(p3.z - p1.z) - (p2.z - p1.z)*(p3.y - p1.y);
            //     b = (p2.z - p1.z)*(p3.x - p1.x) - (p2.x - p1.x)*(p3.z - p1.z);
            //     c = (p2.x - p1.x)*(p3.y - p1.y) - (p2.y - p1.y)*(p3.x - p1.x);
            tmp.A=((b[1]-a[1])*(c[2]-a[2])-(b[2]-a[2])*(c[1]-a[1]));
            tmp.B=((b[2]-a[2])*(c[0]-a[0])-(b[0]-a[0])*(c[2]-a[2]));
            tmp.C=((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]));
            tmp.D=(-(tmp.A*a[0]+tmp.B*a[1]+tmp.C*a[2]));//D = -(A * x1 + B * y1 + C * z1)
            double tmpM = max(max(fabs(tmp.A),fabs(tmp.B)),max(fabs(tmp.C),fabs(tmp.D)));
            tmp.A/=tmpM;
            tmp.B/=tmpM;
            tmp.C/=tmpM;
            tmp.D/=tmpM;
            // printf("%lf %lf %lf %lf\n",tmp.A,tmp.B,tmp.C,tmp.D);
            // printf("%lf %lf %lf\n",tmp.getv(a),tmp.getv(b),tmp.getv(c));
            // exit(0);
            // assert(tmp.getv(a)+tmp.getv(b)+tmp.getv(c)>eps);
            return tmp;
    }    
    CostMatrix selfmul_planes(planes a,CostMatrix &ret){
        for(int i=0;i<4;++i){
            for(int j=0;j<4;++j){
                ret[i][j]=a[i]*a[j];
            }
        }
        return ret;
    }
    void init(CostMatrix &a){
        for(int i=0;i<4;++i)for(int j=0;j<4;++j)a[i][j]=0;
        for(int i=0;i<4;++i)a[i][i]=1;
    }

    using std::swap;
    int getInverse(CostMatrix &a){
        CostMatrix b;
        init(b);
        for(int i=0;i<4;++i){
            bool flg=0;
            for(int j=i;j<4;++j){
                if(a[j][i]>eps){
                    if(i!=j)
                        for(int k=0;k<4;++k){
                            swap(a[j][k],a[i][k]);
                            swap(b[j][k],b[i][k]);
                        }
                    flg=1;
                    break;
                }
            }
            if(!flg){
                return 0;
            }
            double tmp = a[i][i];
            for(int j=0;j<4;++j){
                a[i][j]/=tmp;
                b[i][j]/=tmp;
            }
            for(int j=0;j<4;++j){
                if(i==j)continue;
                tmp=a[j][i];
                for(int k=0;k<4;++k){
                    a[j][k] -= tmp*a[i][k];
                    b[j][k] -= tmp*b[i][k];
                }
            }
        }
        a=b;
        return 1;
    }
    double realmul(CostMatrix A,glm::vec4 B){
        double ret=0;
        for(int i=0;i<4;++i){
            for(int j=0;j<4;++j){
                ret+=B[i]*A[i][j]*B[j];
            }
        }
        return ret;
    }
    double _calct(CostMatrix A,glm::vec4 B,glm::vec4 C){
        // double tmp=max(min(min(fabs(B[0]),fabs(B[1])),fabs(B[2])),(float)1e-4);
        // B=B/tmp;
        // tmp       =max(min(min(fabs(C[0]),fabs(C[1])),min(fabs(C[2]),C[3])),(float)1e-4);
        // C=C/tmp;
        // printf("%lf\n",tmp);
        // outnode(C);
        double x=0;
        double x2=0;
        double b=0;
        for(int i=0;i<4;++i){
            for(int j=0;j<4;++j){
                x2  += (double)B[i]*A[i][j]*B[j];
                x   += (double)B[i]*C[j]*A[i][j]+C[i]*B[j]*A[i][j];
                b   += (double)C[i]*A[i][j]*C[j];
            }
        }
        // outMat(A);
        // outnode(B);
        // outnode(C);
        // printf("! %.10lf %.10lf %.10lf\n",x2,x,b);
        // puts("---");
        // if(x2<eps)return 0;
        b = -x/(2*x2);
        printf("result : %lf\n",b);
        // if(fabs(x2)<1e-8)return -x*inf;
        return min(b,1.0);
        // if(b<=2 && b>=-2)return b;
        // else return 0.5;
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
        output = input;
        auto UpdateQ {
            [&G, &output] (DCEL::Triangle const * f) -> glm::mat4 {
                glm::mat4 Q;
                auto g1 = output.Positions[f->VertexIndex(0)];
                auto g2 = output.Positions[f->VertexIndex(1)];
                auto g3 = output.Positions[f->VertexIndex(2)];
                planes b = calc_planes(g1,g2,g3);
                selfmul_planes(b,Q);//自己乘自己实现
                return Q;
            }
        };

        // The struct to record constraction info.
        struct ConstractionPair {
            DCEL::HalfEdge const * edge;            // which edge to constract; if $edge == nullptr$, it means this pair is no longer valid
            glm::vec4              targetPosition;  // the targetPosition $v$ for vertex $edge->From()$ to move to
            float                  cost;            // the cost $v.T * Qbar * v$
            ConstractionPair(){};
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
                ConstractionPair tmp;
                tmp.edge=edge;
                auto tQ = Q;
                tQ[3][0]=tQ[3][1]=tQ[3][2]=0;tQ[3][3]=1;
                int c = getInverse(tQ);
                if(c==0){
                    // do{
                    tQ=Q;
                    // tQ[3][0]=tQ[3][1]=tQ[3][2]=0;
                    // tQ[3][3]=0;
                    //     for(int i=0;i<4;++i){
                    //         for(int j=0;j<4;++j){
                    //             tQ[i][j]+=(rand()-rand())/(RAND_MAX*100.0);
                    //         }
                    //     }
                    //     c = getInverse(tQ);
                    // }
                    // while(c==0);
                    double t = _calct(tQ,{p1[0]-p2[0],p1[1]-p2[1],p1[2]-p2[2],0},{p2[0],p2[1],p2[2],1});
                    tmp.targetPosition = {p1[0]*t+p2[0]*(1-t),p1[1]*t+p2[1]*(1-t),p1[2]*t+p2[2]*(1-t),1};
                }else tmp.targetPosition = {tQ[0][3],tQ[1][3],tQ[2][3],1};
                tmp.cost=realmul(Q,tmp.targetPosition);
                // outnode(p1);
                // outnode(p2);
                // outMat(Q);
                // puts("----");
                // outnode(tmp.targetPosition);
                // outMat(tQ);
                // puts("****");
                // if(tmp.targetPosition[0]>1.0 || tmp.targetPosition[1] > 1.0 || tmp.targetPosition[2]>1.0) exit(0);
                // printf("%lf\n",tmp.cost);
                // printf("%lf\n",realmul(Q,{p1[0],p1[1],p1[2],1}));
                // printf("%f %f %f\n",tmp.targetPosition[0],tmp.targetPosition[1],tmp.targetPosition[2]);
                // exit(0);
                return tmp;
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
        std::vector<bool> vis;
        vis.resize(input.Positions.size());
        for(int i=0;i<vis.size();++i)vis[i]=0;
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
        // puts("qwq!");


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
                auto face = e->Face();
                auto fid = G.IndexOf(face);
                auto tmpQ = UpdateQ(face);
                auto nQ = tmpQ - Qf[fid];
                Qv[e->To()]  +=nQ;
                Qv[e->From()]+=nQ;
                Qv[v1]       +=tmpQ;
                vis[e->From()]=1;
                vis[e->To()]  =1;
                Qf[fid]      = tmpQ;
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
            for(int i=1;i<pairs.size();++i){
                if(pairs[i].edge){
                    auto v1 = pairs[i].edge->From();
                    auto v2 = pairs[i].edge->To();
                    if(!vis[v1] && !vis[v2])continue;
                    auto pair = MakePair(pairs[i].edge,output.Positions[v1],output.Positions[v2],Qv[v1]+Qv[v2]);
                    pairs[i]=pair;
                }
            }
            for(int i=0;i<output.Positions.size();++i)vis[i]=0;
        }

        // In the end, we check if the result mesh is watertight and manifold.
        if (! G.DebugWatertightManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Result is not watertight manifold.");
        }

        auto exported = G.ExportMesh();
        output.Indices.swap(exported.Indices);
    }
    double operator*(const point &a,const point &b){
        return (double)a[0]*b[0]+(double)a[1]*b[1]+(double)a[2]*b[2];
    }
    /******************* 4. Mesh Smoothing *****************/
    void SmoothMesh(Engine::SurfaceMesh const & input, 
                    Engine::SurfaceMesh & output, 
                    std::uint32_t numIterations, 
                    float lambda, bool useUniformWeight) {
        // Define function to compute cotangent value of the angle v1-vAngle-v2
        static constexpr auto GetCotangent {
            [] (glm::vec3 vAngle, glm::vec3 v1, glm::vec3 v2) -> float {
                v1=v1-vAngle;
                v2=v2-vAngle;//方向向量
                double cosv = (v1*v2)/sqrt((v1*v1)*(v2*v2));
                double sinv = sqrt(1-cosv*cosv);
                if(fabs(sinv)<1e-3)return 1e4;
                return sqrt(fabs(cosv/sinv));//根号余切
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
            for (std::size_t i = 0; i < curr_mesh.Positions.size(); ++i) {
                // your code here: curr_mesh.Positions[i] = ...
                auto &u = curr_mesh.Positions[i];
                auto _v = point(0,0,0);
                double sum = 0.0;
                auto tgv = G.Vertex(i);
                auto nei = tgv->Faces();
                for(auto fa : nei){
                    int id=fa->LabelOfVertex(i);
                    for(int k=0;k<3;++k){
                        if(k==id)continue;
                        auto p=fa->Edge(k);
                        if(p->From()==i){//我们只在起点处统计,而且只统计不是0的边
                            double wij = 0.0;
                            auto v=curr_mesh.Positions[p->To()];
                            if(useUniformWeight)wij=1.0;
                            else if((p->TwinEdgeOr(nullptr))){
                                auto vangle = curr_mesh.Positions[p->OppositeVertex()];
                                wij = GetCotangent(vangle,u,v);
                                auto vangle2 = curr_mesh.Positions[p->TwinEdge()->OppositeVertex()];
                                wij += GetCotangent(vangle2,u,v);
                            }
                            _v=_v+v*wij;
                            sum=sum+wij;
                        }
                    }
                }
                _v=_v/sum;
                u=(1-lambda)*u+lambda*_v;
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
    inline int sgn(double x){
        if(fabs(x)<eps)return 0;
        else if(x<0)return -1;
        else return 1;
    }

    inline int getV(point x,float a,const std::function<float(const glm::vec3 &)> &f){
        const double dx[] = {a,0,a,0,a,0,a,0};
        const double dy[] = {a,a,0,0,a,a,0,0};
        const double dz[] = {a,a,a,a,0,0,0,0};
        int ret=0;
        for(int i=0;i<8;++i)
            ret=ret<<1|(sgn(f({x[0]+dx[i],x[1]+dy[i],x[2]+dz[i]}))>0);
        return ret;
    }
    //                 0         1      2        3       4       5       6      7       8         9     10        11
    const point st[]={{0,0,0},{0,1,0},{0,0,1},{0,1,1},{0,0,0},{0,0,1},{1,0,0},{1,0,1},{0,0,0},{1,0,0},{0,1,0},{1,1,0}};
    const point ed[]={{1,0,0},{1,1,0},{1,0,1},{1,1,1},{0,1,0},{0,1,1},{1,1,0},{1,1,1},{0,0,1},{1,0,1},{0,1,1},{1,1,1}};
    const point unit[]={{1,0,0},{0,1,0},{0,0,1}};
    void MarchingCubes(Engine::SurfaceMesh & output, 
    const std::function<float(const glm::vec3 &)> & sdf, 
    const glm::vec3 & grid_min, const float dx, const int n) {
        std::vector<int> nod;
        nod.resize(12);
        for(int i=0;i<n;++i){
            double nx=i*dx+grid_min[0];
            for(int j=0;j<n;++j){
                double ny=j*dx+grid_min[1];
                for(int k=0;k<n;++k){
                    double nz=k*dx+grid_min[2];
                    auto nw = point(nx,ny,nz);
                    int a = getV(nw,dx,sdf);
                    int edgev=c_EdgeStateTable[a];
                    auto ordv=c_EdgeOrdsTable[a];
                    for(int _=0;_<12;++_)nod[_]=0;
                    for(int w=0;w<12;++w){
                        if(edgev>>w&1){
                            auto u = nw + st[w]*dx;
                            auto v = nw + ed[w]*dx;
                            auto a = fabs(sdf(u)),b = fabs(sdf(v));
                            auto np = u*(b/(a+b))+v*(a/(a+b));
                            nod[w]=output.Positions.size();
                            output.Positions.emplace_back(np);
                            // 这种就是狗屎
                        }
                    }
                    for(int i=0;ordv[i]!=-1;i+=3){
                        output.Indices.emplace_back(nod[ordv[i]]);
                        output.Indices.emplace_back(nod[ordv[i+1]]);
                        output.Indices.emplace_back(nod[ordv[i+2]]);
                    }
                }
            }
        }
        return ;
    }
} // namespace VCX::Labs::GeometryProcessing
