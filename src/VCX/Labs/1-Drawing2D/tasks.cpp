#include <random>
#include <spdlog/spdlog.h>
#include "Labs/1-Drawing2D/tasks.h"
using VCX::Labs::Common::ImageRGB;

namespace VCX::Labs::Drawing2D {
    /******************* 0.Prepare Tool for computing geometry *****************/

    using COL = glm::vec3;
    using Point=glm::ivec2;
    using fPoint = glm::vec2;
    using std::swap;
    using std::min;
    using std::max;

    fPoint operator*(double b,fPoint a){
        return fPoint(a[0] * b,a[1] * b);
    }
    fPoint operator+(fPoint a,fPoint b){
        return fPoint(a[0] + b[0],a[1] + b[1]);
    }        
    Point operator-(Point a,Point b){
        return Point(a[0]-b[0],a[1]-b[1]);
    }
    void swap(Point &a,Point &b){
        Point c;
        c=a;
        a=b;
        b=c;
        return ;
    }    
    bool operator==(COL a,COL b){
        return (a[0]==b[0]) && (a[1]==b[1]) && (a[2]==b[2]);
    }

    int sgn(int x){
        if(!x)return 0;
        if(x<0)return -1;
        else return 1;
    }
    int operator^(const Point &x,const Point &y){// cross product
        return x.x*y.y-x.y*y.x;
    }
    struct Line{
        Point e;
        Point s;
        Line(){};
        Line(Point _s,Point _e):s(_s),e(_e){};
        int relation(Point p){
            int c=sgn((p-s)^(e-s));
            if(c<0)return 1;
            else if(c>0)return 2;
            else return 3;
        }
    };

    COL operator+(COL a,double x){
        return COL(a[0]+x,a[1]+x,a[2]+x);
    }

    COL operator+(COL a,COL b){
        return COL(a[0]+b[0],a[1]+b[1],a[2]+b[2]);
    }       

    COL operator/(COL a,double b){
        return COL(a[0]/b,a[1]/b,a[2]/b);
    }        
    
    COL Cabs(COL a){
        return COL(fabs(a[0]),fabs(a[1]),fabs(a[2]));
    }
    COL operator-(COL a,COL b){
        return COL(a[0]-b[0],a[1]-b[1],a[2]-b[2]);
    }
    double len(const COL &a){
        return a[0]*a[0]+a[1]*a[1]+a[2]*a[2];
    }

    COL operator*(double b,COL a){
        return COL(a[0]*b,a[1]*b,a[2]*b);
    }
    COL operator*(COL b,COL a){
        return COL(a[0]*b[0],a[1]*b[1],a[2]*b[2]);
    }    
    COL Fv(COL a){
        return COL(pow(a[0],0.5),pow(a[1],0.5),pow(a[2],0.5));
    }
    /******************* 1.Image Dithering *****************/

    void Cwrite(ImageRGB & otp,int x,int y,double c){
        for(int i=0;i<3;++i){
            for(int j=0;j<3;++j){
                otp.At(i+x,j+y)=COL(0,0,0);
            }
        }       
        otp.At(1+x,1+y)=COL(1,1,1);
        if(c>=0.125){
            otp.At(x,1+y)=COL(1,1,1);
        }
        if(c>=0.25){
            otp.At(1+x,2+y)=COL(1,1,1);
        }
        if(c>=0.375){
            otp.At(x+2,1+y)=COL(1,1,1);
        }
        if(c>=0.5){
            otp.At(x+2,y)=COL(1,1,1);
        }
        if(c>=0.625){
            otp.At(x,2+y)=COL(1,1,1);
        }
        if(c>=0.75){
            otp.At(x,y)=COL(1,1,1);
        }
        if(c>=0.875){
            otp.At(x+2,y+2)=COL(1,1,1);
        }
        if(c>=0.99){
            otp.At(x+1,y)=COL(1,1,1);
        }
    }    
    void DitheringThreshold(
        ImageRGB &       output,
        ImageRGB const & input) {
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color = input.At(x, y);
                output.At(x, y) = {
                    color.r > 0.5 ? 1 : 0,
                    color.g > 0.5 ? 1 : 0,
                    color.b > 0.5 ? 1 : 0,
                };
            }
    }

    void DitheringRandomUniform(
        ImageRGB &       output,
        ImageRGB const & input) {
        int n=input.GetSizeX(),m=input.GetSizeY();
        srand(time(NULL));
        for(int i=0;i<n;++i){
            for(int j=0;j<m;++j){
                double rd=(double)rand()/RAND_MAX - 0.5;
                COL color = input.At(i,j);
                output.At(i,j)= Cabs(color + rd);
            }
        }
        DitheringThreshold(output,output);
    }

    void DitheringRandomBlueNoise(
        ImageRGB &       output,
        ImageRGB const & input,
        ImageRGB const & noise) {
        int n=input.GetSizeX(),m=input.GetSizeY();
        for(int i=0;i<n;++i){
            for(int j=0;j<m;++j){
                COL color = input.At(i,j) - noise.At(i,j) + 0.5; // 这里是蓝噪音分布的问题！！
                output.At(i,j)= color;
            }
        }
        DitheringThreshold(output,output);
    }
    void DitheringOrdered(
        ImageRGB &       output,
        ImageRGB const & input) {
        int n = input.GetSizeX(),m = input.GetSizeY();
        for(int i=0;i<n;++i){
            for(int j=0;j<m;++j){
                COL color= input.At(i,j);
                Cwrite(output,i*3,j*3,color[0]);
            }
        }
    }
    void DitheringErrorDiffuse(
        ImageRGB &       output,
        ImageRGB const & input) {
        int n = input.GetSizeX(),m=input.GetSizeY();
        for(int i=0;i<n;++i){
            for(int j=0;j<m;++j){
                output.At(i,j)=input.At(i,j);
            }
        }
        for(int i=0;i<n;++i){
            for(int j=0;j<m;++j){
                COL color = output.At(i,j);
                double res = 0;
                if(color[0]>0.5){
                    res = color[0]-1;
                    output.At(i,j)=COL(1,1,1);
                }
                else {
                    res=color[0];
                    output.At(i,j)=COL(0,0,0);
                }
                if(j<m-1)output.At(i,j+1)=(COL)output.At(i,j+1) + res*7/16;
                if(i<n-1)output.At(i+1,j)=(COL)output.At(i+1,j) + res*5/16;
                if(i<n-1 && j<m-1)output.At(i+1,j+1)=(COL)output.At(i+1,j+1) + res/16;
                if(i<n-1 && j>0)output.At(i+1,j-1)=(COL)output.At(i+1,j-1) + res*3/16;
            }
        }
    }

    /******************* 2.Image Filtering *****************/
    const int dx[] = {0,1,-1,0,1,-1,  0,1,-1};
    const int dy[] = {0,0,0 ,1,1,1 ,-1,-1,-1};
    const double Blurv[] ={0.111,0.111,0.111,0.111,0.111,0.111,0.111,0.111,0.111};
    const int LEdgev[3][3] ={{-1,0,1},{-2,0,2},{-1,0,1}};
    const int HEdgev[3][3] ={{1,2,1},{0,0,0},{-1,-2,-1}};

    COL calcBlursimple(ImageRGB const & canvas,int x,int y){
        COL color1 = canvas.At(x + 1,y);
        COL color2 = canvas.At(x - 1,y);
        COL color3 = canvas.At(x,y + 1);
        COL color4 = canvas.At(x,y - 1);
        COL color5 = canvas.At(x,    y);
        return (color1+color2+color3+color4+color5)/5;
    }

    COL calcBlur(ImageRGB const & canvas,int x,int y){
        COL color = COL(0,0,0);
        for(int i=0;i<9;++i){
            color = color + Blurv[i] * (COL)(canvas.At(x+dx[i],y+dy[i]));
        }
        return color;
    }    

    COL L_calcEdge(ImageRGB const &canvas ,int x,int y){
        COL color = COL(0,0,0);
        for(int i=0;i<3;++i){
            for(int j=0;j<3;++j){
                color = color + LEdgev[i][j] * (COL)canvas.At(x+i-1,y+j-1);
            }
        }
        return color;
    }

    COL H_calcEdge(ImageRGB const &canvas ,int x,int y){
        COL color = COL(0,0,0);
        for(int i=0;i<3;++i){
            for(int j=0;j<3;++j){
                color = color + HEdgev[i][j] * (COL)canvas.At(x+i-1,y+j-1);
            }
        }
        return color;
    }    

    void Blur(
        ImageRGB &       output,
        ImageRGB const & input) {
        int n = output.GetSizeX(),m=output.GetSizeY();
        for(int i=0;i<n;++i){output.At(i,0)=input.At(i,0);output.At(i,m-1)=input.At(i,m-1);}
        for(int j=0;j<m;++j){output.At(0,j)=input.At(0,j);output.At(n-1,j)=input.At(n-1,j);}
        for(int i=1;i<n-1;++i){
            for(int j=1;j<m-1;++j){
                output.At(i,j)=calcBlur(input,i,j);
            }
        }
    }
    void Edge(
        ImageRGB &       output,
        ImageRGB const & input) {
        int n = output.GetSizeX(),m=output.GetSizeY();
        for(int i=0;i<n;++i){output.At(i,0)=input.At(i,0);output.At(i,m-1)=input.At(i,m-1);}
        for(int j=0;j<m;++j){output.At(0,j)=input.At(0,j);output.At(n-1,j)=input.At(n-1,j);}
        for(int i=1;i<n-1;++i){
            for(int j=1;j<m-1;++j){
                COL a = L_calcEdge(input,i,j);
                COL b = H_calcEdge(input,i,j);
                COL c = Fv(a * a + b * b);
                output.At(i,j)= c;
            }
        }
    }

    COL calcgrade(ImageRGB const & canvas,int x,int y){
        COL color1 = canvas.At(x + 1,y);
        COL color2 = canvas.At(x - 1,y);
        COL color3 = canvas.At(x,y + 1);
        COL color4 = canvas.At(x,y - 1);
        return (color1-color2) + (color3-color4);
    }

    /******************* 3. Image Inpainting *****************/
    void Inpainting(
        ImageRGB &         output,
        ImageRGB const &   inputBack,
        ImageRGB const &   inputFront,
        const glm::ivec2 & offset) {
        output             = inputBack;
        std::size_t width  = inputFront.GetSizeX();
        std::size_t height = inputFront.GetSizeY();
        glm::vec3 * g      = new glm::vec3[width * height];

        memset(g, 0, sizeof(glm::vec3) * width * height);

        for (std::size_t y = 0; y < height; ++y) {
            COL color =inputBack.At(offset.x,y+offset.y);
            g[y*width] = color - inputFront.At(0,y);
            color = inputBack.At(width-1+offset.x,y+offset.y);
            g[y*width+width-1]=color - inputFront.At(width-1,y);

        }
        for (std::size_t x = 0; x < width; ++x) {
            COL color = inputBack.At(x+offset.x,offset.y);
            g[x]= color - inputFront.At(x,0);
            color = inputBack.At(x+offset.x,height-1+offset.y);
            g[(height-1) * width + x] = color -inputFront.At(x,height-1);
        }

        // Jacobi iteration, solve Ag = b
        for (int iter = 0; iter < 8000; ++iter) {
            for (std::size_t y = 1; y < height - 1; ++y)
                for (std::size_t x = 1; x < width - 1; ++x) {
                    g[y * width + x] = (g[(y - 1) * width + x] + g[(y + 1) * width + x] + g[y * width + x - 1] + g[y * width + x + 1]);
                    g[y * width + x] = g[y * width + x] * glm::vec3(0.25);
                }
        }

        for (std::size_t y = 0; y < inputFront.GetSizeY(); ++y)
            for (std::size_t x = 0; x < inputFront.GetSizeX(); ++x) {
                COL color =g[y * width + x] + inputFront.At(x, y);
                output.At(x + offset.x, y + offset.y) = color;
            }

        delete[] g;
    }

    /******************* 4. Line Drawing *****************/

    void DrawLine(
        ImageRGB &       canvas,
        glm::vec3 const  color,
        glm::ivec2 const p0,
        glm::ivec2 const p1) {
        Point a=p0,b=p1;
        bool flg=1;
        if(abs((b-a)[0])<abs((b-a)[1])){
            swap(a[0],a[1]);
            swap(b[0],b[1]);
            flg=0;
        }
        if(a[0]>b[0])swap(a,b);
        int Ndx = 2*(b[1]-a[1]);
        int Ndy = 2*(a[0]-b[0]);
        int dy = (a[1]>b[1])?(-1):1;
        bool flg2=1;
        if(((Ndx < 0) && (Ndy < 0)) || ((Ndx > 0) && (Ndy > 0))){
            swap(a[1],b[1]);
            Ndx = 2*(b[1]-a[1]);
            flg2 = 0;
        }
        int F  = Ndx + Ndy/2;
        int nx=a[0],ny=a[1];
        for(;nx<=b[0];++nx){
            if(flg){
                if(flg2){
                    canvas.At(nx,ny)=color;
                }
                else {
                    canvas.At(nx,b[1]+ny-a[1])=color;
                }
            }
            else {
                if(flg2)canvas.At(ny,nx)=color;
                else {
                    canvas.At(b[1]+ny-a[1],nx)=color;
                }
            }
            if(F>0)
            {
                ny += dy;
                F += Ndx + Ndy;
            }
            else {
                F += Ndx;
            }
        }
    }

    /******************* 5. Triangle Drawing *****************/
#define max(x,y,z) (max(max(x,y),z))
#define min(x,y,z) (min(min(x,y),z))

    void DrawTriangleFilled(
        ImageRGB &       canvas,
        glm::vec3 const  color,
        glm::ivec2 const p0,
        glm::ivec2 const p1,
        glm::ivec2 const p2) {
        int upx = max(p0[0],p1[0],p2[0]);
        int upy = max(p0[1],p1[1],p2[1]);
        int dwx = min(p0[0],p1[0],p2[0]);
        int dwy = min(p0[1],p1[1],p2[1]);
        Line a1 = Line(p0,p1);
        Line a2 = Line(p1,p2);
        Line a3 = Line(p2,p0);
        for(int i=dwx;i<=upx;++i){
            for(int j=dwy;j<=upy;++j){
                auto tmp=Point(i,j);
                if(a1.relation(tmp) ==a2.relation(tmp) && a2.relation(tmp)==a3.relation(tmp)){
                    canvas.At(i,j)=color;
                }
            }
        }
    }

    /******************* 6. Image Supersampling *****************/
    int sample[500];
    inline glm::vec3 samplefrom(ImageRGB const & input,int lx,int ly,int rx,int ry,int rate){
        glm::vec3 tmp = glm::vec3(0.,0.,0.);
        for(int i=1;i<=rate;++i){
            int tx=rand()%(rx-lx)+lx;
            int ty=rand()%(ry-ly)+ly;
            tmp = tmp+input.At(tx,ty);
        }
        tmp=tmp/rate;
        return tmp;
    }
    void Supersample(
        ImageRGB &       output,
        ImageRGB const & input,
        int              rate) {
        int Ix=input.GetSizeX(),Iy=input.GetSizeY();
        int Ox=output.GetSizeX(),Oy=output.GetSizeY();
        int nx=0,ny=0;
        for(int i=0;i<Ox;++i){
            sample[i]=Ix/Ox;
        }
        for(int i=0;i<Ix%Ox;++i){
            sample[i]++;
        }
        for(int i=0,nx=0;i<Ox;++i){
            for(int j=0,ny=0;j<Oy;++j){
                output.At(i,j)=samplefrom(input,nx,ny,nx+sample[i],ny+sample[j],rate*rate);
                ny += sample[j];
            } 
            nx+=sample[i];
        }
        return ;
    }

    /******************* 7. Bezier Curve *****************/
    double tk[50],_tk[50];
    int C[50][50];

    inline void Cinit(int n,double t){
        C[0][0]=1;
        for(int i=1;i<=n;++i){
            C[i][0]=1;
            for(int j=1;j<=i;++j){
                C[i][j]=C[i-1][j-1]+C[i-1][j];
            }
        }
        tk[0]=_tk[0]=1;
        for(int i=1;i<=n;++i)tk[i]=tk[i-1]*t;
        for(int i=1;i<=n;++i)_tk[i]=_tk[i-1]*(1-t);
        return ;
    }
    glm::vec2 CalculateBezierPoint(
        std::span<glm::vec2> points,
        float const          t) {
        int n = points.size();
        std::vector<glm::vec2> rc;
        std::vector<glm::vec2> pointrc;
        for(auto u : points){pointrc.emplace_back(u);}
        for(int i=0;i<n-1;++i){
            for(int j=0;j<n-i-1;++j){
                rc.emplace_back(((1-t)*pointrc[j] + t*pointrc[j+1]));
            }
            swap(rc,pointrc);
            rc.clear();
        }
        return pointrc[0];
    }
#undef max
#undef min
} // namespace VCX::Labs::Drawing2D