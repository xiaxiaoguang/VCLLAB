#include "Labs/3-Rendering/tasks.h"
#include<iostream>
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
        point E1=p2-p1;
        point E2=p3-p1;
        point T =ray.Origin-p1;
        point P = glm::cross(ray.Direction , E2);
        point Q = glm::cross(T , E1);
        double inv = glm::dot(P , E1);
        if(inv == 0)return 0;
        double u = glm::dot(P , T) / inv;
        double v = glm::dot(Q , ray.Direction) / inv;
        double t = glm::dot(Q, E2) / inv;
        if(u>=0 && v>=0 && (u+v)<=1 && t>=0){
            output.t=t;
            output.u=u;
            output.v=v;
            return 1;
        }
        return 0;
    }

    #define max(x,y) ((x)>(y)?(x):(y))
    #define min(x,y) ((x)<(y)?(x):(y))

    using std::cout;
    using std::string;
    void outp(point a,string b){
        cout<<b;
        printf(" %.3f %.3f %.3f\n",a[0],a[1],a[2]);
        return ;
    }    
    glm::vec3 phong(glm::vec3 lightIntensity, glm::vec3 lightDir, glm::vec3 normal, glm::vec3 viewDir, glm::vec3 diffuseColor, glm::vec3 specularColor, float shininess) {
        //lightintensity 是光强
        // lightDir : 光照方向
        //normal 法线方向
        //viewDir 视觉方向
        //diffuse color 漫反射颜色
        // specularcolor镜面反射颜色
        // shininess 明亮度
        glm::vec3 H = (lightDir + viewDir) * (float)0.5;
        float p1 = dot(H,normal),p2=dot(lightDir,normal);
        glm::vec3 Ls = specularColor * lightIntensity * pow(max(0,p1),shininess);
        glm::vec3 Ld = diffuseColor * lightIntensity * (max(0,p2));
        return Ls+Ld;
    }
    const float eps=1e-3;
    bool cmp(const point &a,const point &b){
        if(fabs(a[0]-b[0])>eps)return 0;
        if(fabs(a[1]-b[1])>eps)return 0;
        if(fabs(a[2]-b[2])>eps)return 0;
        return 1;
    }
    float sign(float a){
        if(fabs(a)<eps)return 0;
        if(a>0)return 1;
        else return -1;
    }
    int cnt=0;
    int precnt=0;
    glm::vec3 RayTrace(const RayIntersector & intersector, Ray ray, int maxDepth, bool enableShadow) {
        glm::vec3 color(0.0f);
        glm::vec3 weight(1.0f);
        const float ambient = 0.1;
        for (int depth = 0; depth < maxDepth; depth++) {
            auto rayHit = intersector.IntersectRay(ray);
            if (! rayHit.IntersectState) return color;
            const glm::vec3 pos       = rayHit.IntersectPosition;
            const glm::vec3 n         = rayHit.IntersectNormal;
            const glm::vec3 kd        = rayHit.IntersectAlbedo;
            const glm::vec3 ks        = rayHit.IntersectMetaSpec;
            const float     alpha     = rayHit.IntersectAlbedo.w;
            const float     shininess = rayHit.IntersectMetaSpec.w * 256;
            glm::vec3 result(0.0f);
            for (const Engine::Light & light : intersector.InternalScene->Lights) {
                glm::vec3 l;
                float     attenuation;
                if (light.Type == Engine::LightType::Point) {
                    l           = pos - light.Position;
                    attenuation = 1.0f / glm::dot(l, l);
                    l = normalize(l);
                    auto hit = intersector.IntersectRay(Ray(light.Position, l));
                    float alpha1 = hit.IntersectAlbedo.w;
                    point tmp = hit.IntersectPosition;
                    if (enableShadow) {
                        while(alpha1<=0.2 && hit.IntersectState){
                            hit = intersector.IntersectRay(Ray(tmp+l,l));
                            if(hit.IntersectState && cmp(hit.IntersectPosition,tmp))break;
                            alpha1 = hit.IntersectAlbedo.w;
                            tmp = hit.IntersectPosition;
                        }
                        if(alpha1 <= 0.2 || cmp(tmp,pos))result+=attenuation*phong(light.Intensity,-l,n,ray.Direction,kd,ks,shininess); // 有光线
                    }else result+=attenuation*phong(light.Intensity,-l,n,ray.Direction,kd,ks,shininess);
                } else if (light.Type == Engine::LightType::Directional) {
                    l           = light.Direction;
                    attenuation = 1.0f;
                    // float rc = -sign(dot(pos,l));
                    // l = l * rc;
                    auto hit = intersector.IntersectRay(Ray(pos,l));
                    float alpha1 = hit.IntersectAlbedo.w;
                    point tmp = hit.IntersectPosition;
                    if (enableShadow) {
                        while(alpha1<=0.2 && hit.IntersectState){
                            hit = intersector.IntersectRay(Ray(tmp+l,l));
                            if(hit.IntersectState && cmp(hit.IntersectPosition,tmp))break;
                            alpha1 = hit.IntersectAlbedo.w;
                            tmp = hit.IntersectPosition;
                            // if(hit.IntersectState && alpha1>0.2){
                            //     outp(hit.IntersectPosition," ");
                            //     outp(tmp," ");
                            //     exit(0);
                            // }
                        }
                        if(alpha1<=0.2 || (!hit.IntersectState)){
                            result+=phong(light.Intensity,l,n,ray.Direction,kd,ks,shininess); // 有光线
                        }
                    }else result+=phong(light.Intensity,l,n,ray.Direction,kd,ks,shininess);
                }
                result += kd * ambient;
            }
            if (alpha < 0.9) {
                // refraction
                // accumulate color
                glm::vec3 R = alpha * glm::vec3(1.0f);
                color += weight * R * result;
                weight *= glm::vec3(1.0f) - R;
                // generate new ray
                ray = Ray(pos, ray.Direction);
            } else {
                // reflection
                // accumulate color
                glm::vec3 R = ks * glm::vec3(0.5f);
                color += weight * (glm::vec3(1.0f) - R) * result;
                weight *= R;
                // generate new ray
                glm::vec3 out_dir = ray.Direction - glm::vec3(2.0f) * n * glm::dot(n, ray.Direction);
                ray               = Ray(pos, out_dir);
            }
        }
        return color;
    }
} // namespace VCX::Labs::Rendering