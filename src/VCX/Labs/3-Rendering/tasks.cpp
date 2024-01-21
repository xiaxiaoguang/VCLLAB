#include "Labs/3-Rendering/tasks.h"
#include <iostream>

namespace VCX::Labs::Rendering {

    glm::vec4 GetTexture(Engine::Texture2D<Engine::Formats::RGBA8> const & texture, glm::vec2 const & uvCoord) {
        if (texture.GetSizeX() == 1 || texture.GetSizeY() == 1) return texture.At(0, 0);
        glm::vec2 uv      = glm::fract(uvCoord);
        uv.x              = uv.x * texture.GetSizeX() - .5f;
        uv.y              = uv.y * texture.GetSizeY() - .5f;
        std::size_t xmin  = std::size_t(glm::floor(uv.x) + texture.GetSizeX()) % texture.GetSizeX();
        std::size_t ymin  = std::size_t(glm::floor(uv.y) + texture.GetSizeY()) % texture.GetSizeY();
        std::size_t xmax  = (xmin + 1) % texture.GetSizeX();
        std::size_t ymax  = (ymin + 1) % texture.GetSizeY();
        float       xfrac = glm::fract(uv.x), yfrac = glm::fract(uv.y);
        return glm::mix(glm::mix(texture.At(xmin, ymin), texture.At(xmin, ymax), yfrac), glm::mix(texture.At(xmax, ymin), texture.At(xmax, ymax), yfrac), xfrac);
    }

    glm::vec4 GetAlbedo(Engine::Material const & material, glm::vec2 const & uvCoord) {
        glm::vec4 albedo       = GetTexture(material.Albedo, uvCoord);
        glm::vec3 diffuseColor = albedo;
        return glm::vec4(glm::pow(diffuseColor, glm::vec3(2.2)), albedo.w);
    }
    using point = glm::vec3;
    // 時代的眼淚💧
    // point operator - (const point &p1,const point &p2){
    //     return point(p1[0]-p2[0],p1[1]-p2[1],p1[2]-p2[2]);
    // }
    // point operator *(const point &p1,const double x){
    //     return point(p1[0]*x,p1[1]*x,p1[2]*x);
    // }
    // point operator ^(const point &p1,const point &p2){
    //     return point(p1[1]*p2[2]-p1[2]*p2[1],p1[2]*p2[0]-p1[0]*p2[2],p1[0]*p2[1]-p1[1]*p2[0]);
    // }
    // double operator & (const point &p1,const point &p2){
    //     return (p1[0]*p2[0]+p1[1]*p2[1]+p1[2]*p2[2]);
    // }
    /******************* 1. Ray-triangle intersection *****************/
    bool IntersectTriangle(Intersection & output, Ray const & ray, glm::vec3 const & p1, glm::vec3 const & p2, glm::vec3 const & p3) {
        point  E1  = p2 - p1;
        point  E2  = p3 - p1;
        point  T   = ray.Origin - p1;
        point  P   = glm::cross(ray.Direction, E2);
        point  Q   = glm::cross(T, E1);
        double inv = glm::dot(P, E1);
        if (inv == 0) return 0;
        double u = glm::dot(P, T) / inv;
        double v = glm::dot(Q, ray.Direction) / inv;
        double t = glm::dot(Q, E2) / inv;
        if (u >= 0 && v >= 0 && (u + v) <= 1 && t >= 0) {
            output.t = t;
            output.u = u;
            output.v = v;
            return 1;
        }
        return 0;
    }

#define max(x, y) ((x) > (y) ? (x) : (y))
#define min(x, y) ((x) < (y) ? (x) : (y))

    using std::cout;
    using std::string;
    using Vec = glm::vec3;

    void outp(glm::vec3 a, string b) {
        cout << b;
        printf(" %.3f %.3f %.3f\n", a[0], a[1], a[2]);
        return;
    }

    void outp(glm::vec4 a, string b) {
        cout << b;
        printf(" %.3f %.3f %.3f %.3f\n", a[0], a[1], a[2],a[3]);
        return;
    }

    glm::vec3 phong(glm::vec3 lightIntensity, glm::vec3 lightDir, glm::vec3 normal, glm::vec3 viewDir, glm::vec3 diffuseColor, glm::vec3 specularColor, float shininess) {
        // lightintensity 是光强
        //  lightDir : 光照方向
        // normal 法线方向
        // viewDir 视觉方向
        // diffuse color 漫反射颜色
        //  specularcolor镜面反射颜色
        //  shininess 明亮度
        glm::vec3 H  = (lightDir + viewDir) * (float) 0.5;
        float     p1 = dot(H, normal), p2 = dot(lightDir, normal);
        glm::vec3 Ls = specularColor * lightIntensity * pow(max(0, p1), shininess);
        glm::vec3 Ld = diffuseColor * lightIntensity * (max(0, p2));
        return Ls + Ld;
    }
    const float eps = 1e-3;
    bool        cmp(const point & a, const point & b) {
        if (fabs(a[0] - b[0]) > eps) return 0;
        if (fabs(a[1] - b[1]) > eps) return 0;
        if (fabs(a[2] - b[2]) > eps) return 0;
        return 1;
    }
    float sign(float a) {
        if (fabs(a) < eps) return 0;
        if (a > 0) return 1;
        else return -1;
    }
    static Vec operator*(float a, const Vec & b) {
        return Vec(b.x * a, b.y * a, b.z * a);
    }
    static Vec operator*(const Vec & b, float a) {
        return Vec(b.x * a, b.y * a, b.z * a);
    }
    static Vec mult(const Vec & a, const Vec & b) {
        return Vec(a.x * b.x, a.y * b.y, a.z * b.z);
    }
    static Vec operator%(const Vec & a,const Vec & b) {
         return Vec(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
    }

    static int maxDepth = 10;
    /*
    1. 把 Light 加入 intersector
    2. 然后 重新设计每个物体的属性
    3. 然后 重新设计光线采样的部分
    */
    static Vec radiance(const RayIntersector & intersector, const Ray & r, int depth, unsigned short * Xi) {
        auto rayHit = intersector.IntersectRay(r);

        if (! rayHit.IntersectState) return Vec();
        if (depth > maxDepth) return Vec();

        Vec    n  = rayHit.IntersectNormal;
        Vec    x  = rayHit.IntersectPosition;
        Vec    nl = glm::dot(n, r.Direction) < 0 ? n : (-1. * n);
        Vec    f  = rayHit.IntersectAlbedo;
        Vec    e  = rayHit.IntersectMetaSpec;
        double p  = (f.x > f.y && f.x > f.z) ? f.x : (f.y > f.z ? f.y : f.z);

        if (++depth > maxDepth/2) {
            if (erand48(Xi) < p) f = f * (1 / p);
            else return e;
        }
        if (e == Vec(0.,0.,0.) || rayHit.IntersectRel == Engine::ReflectionType::PhysicalDiffusion) {
             // Ideal DIFFUSE reflection 理想漫反射
            // 取决于obj的refl1类型
            double r1 = 2 * M_PI * erand48(Xi);
            double r2 = erand48(Xi), r2s = sqrt(r2);
            // r1 随机方向采样
            // r2 是随机距离
            Vec w = nl, u = glm::normalize((fabs(w.x) > .1 ? Vec(0, 1 ,0) : Vec(1 , 0 , 0)) % w), v = w % u;
            // w,u,v 是一个正交坐标系 w确定后u,v都和他垂直
            Vec d = glm::normalize(u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrt(1 - r2));
            // d是 u * cos(r1) * r2s  + v * sin(r1) * r2s + w * sqrt(1-r2)
            // 采样单位半球
            return e + mult(f,radiance(intersector,Ray(x, d), depth, Xi));
        } 

        if (rayHit.IntersectRel == Engine::ReflectionType::PhysicalSpecular) 
        // Ideal SPECULAR reflection 理想镜面反射
        // 没有漫反射颜色的表面
        {
            return mult(f,radiance(intersector,Ray(x, r.Direction - n * 2 * glm::dot(n,r.Direction)), depth, Xi));
        }
        
        if(rayHit.IntersectRel == Engine::ReflectionType::Empirical) e = Vec();
        
        Ray    reflRay(x, r.Direction - n * 2 * glm::dot(n, r.Direction));
        bool   into = glm::dot(n, nl) > 0;
        double nc = 1;
        double nt = 1.5;
        double nnt = into ? (nc / nt) : (nt / nc);
        double ddn = glm::dot(r.Direction, nl), cos2t;

        if ((cos2t = 1 - nnt * nnt * (1 - ddn * ddn)) < 0)
            return e + mult(f, radiance(intersector, reflRay, depth, Xi));

        Vec    tdir = glm::normalize(r.Direction * nnt - n * ((into ? 1 : -1) * (ddn * nnt + sqrt(cos2t))));
        
        double a    = nt - nc,
               b    = nt + nc,
               R0   = a * a / (b * b),
               c    = 1 - (into ? -ddn : glm::dot(tdir, n));

        double Re   = R0 + (1 - R0) * c * c * c * c * c,
               Tr   = 1 - Re,
               P = .25 + .5 * Re, 
               RP = Re / P,
               TP = Tr / (1 - P);

        return e + mult(f, 
        (depth > maxDepth/3) ? (erand48(Xi) < P ?
         radiance(intersector, reflRay, depth, Xi) * RP : radiance(intersector, Ray(x, tdir), depth, Xi) * TP) 
         : radiance(intersector, reflRay, depth, Xi) * Re + radiance(intersector, Ray(x, tdir), depth, Xi) * Tr);
    }
    int __cnt;
    glm::vec3 RayTrace(const RayIntersector & intersector, Ray ray, unsigned short *Xi, int mxd) {
        // auto rayHit = intersector.IntersectRay(Ray(Vec(0,0,0),Vec(0,1,0)));
        // printf("%d\n", rayHit.IntersectState);
        // outp(rayHit.IntersectPosition,"position : ");
        // outp(rayHit.IntersectAlbedo," ");
        // outp(rayHit.IntersectMetaSpec," ");
        ++__cnt;
        maxDepth=mxd;
        Xi[1] += __cnt;
        // unsigned short Xi[3] = {0,0,__cnt};
        auto color = radiance(intersector,ray,0,Xi);
        // exit(0);
        return color;
    }
} // namespace VCX::Labs::Renderingk