#include "Labs/5-Visualization/tasks.h"

#include <numbers>

using VCX::Labs::Common::ImageRGB;
namespace VCX::Labs::Visualization {
    using db = double;
#define max(x, y) ((x) > (y) ? (x) : (y))
#define min(x, y) ((x) < (y) ? (x) : (y))
    using std::string;
    using std::vector;
    const static string _rc_[] = { "cylinders", "displacement", "weight", "horsepower", "acceleration (0-60 mph)", "mileage", "year" };
    const static string unit[] = { "", " sq in", " lbs", " hp", " sec", " mpg", "" };
    const float         rcx[]  = { 0.040000, 0.193846, 0.347692, 0.501538, 0.655385, 0.809231, 0.963077 };
    const float         Sy     = 0.04;
    const float         Ey     = 0.84;

    const float Nx = 2;

    const float Wy = 0.02;
    const float LH = 0.02;
    struct CoordinateStates {
        vector<float> data[10];
        vector<float> rd;
        int           n, useLine;
        db            mx[10], mi[10];
        db            up[10], dw[10], upy[10], dwy[10];
        vector<int>   id[10];
        CoordinateStates() {};
        CoordinateStates(vector<Car> a) {
            n       = a.size();
            useLine = 0;
            for (int i = 0; i < n; ++i) {
                data[0].emplace_back(a[i].cylinders);
                data[1].emplace_back(a[i].displacement);
                data[2].emplace_back(a[i].weight);
                data[3].emplace_back(a[i].horsepower);
                data[4].emplace_back(a[i].acceleration);
                data[5].emplace_back(a[i].mileage);
                data[6].emplace_back(a[i].year);
            }
            for (int i = 0; i < n; ++i) {
                rd.emplace_back((rand() - rand()) / (float) RAND_MAX * 0.1 + 0.6);
            }
            for (int i = 0; i < 7; ++i) mi[i] = 1e18, mx[i] = -1e18;
            for (int i = 0; i < n; ++i) {
                for (int j = 0; j < 7; ++j) {
                    mi[j] = min(data[j][i], mi[j]);
                    id[j].emplace_back(i);
                }
            }
            mx[0] = 9.0;
            mx[1] = 456;
            mx[2] = 5141;
            mx[3] = 231;
            mx[4] = 26;
            mx[5] = 48;
            mx[6] = 83;
            for (int j = 0; j < 7; ++j) {
                up[j]  = mx[j];
                dw[j]  = mi[j];
                upy[j] = Ey;
                dwy[j] = Sy;
                sort(id[j].begin(), id[j].end(), [this, j](int i1, int i2) {
                    return (this->data[j][i1]) < (this->data[j][i2]);
                });
            }
            // printf("data amount %d\n",n);
            // for(int i=0;i<7;++i)printf("%d mx %f mi %f\n",i,mx[i],mi[i]);
        }
        bool update(InteractProxy const & proxy) {
            if (! proxy.IsHovering()) return 0;
            if (proxy.IsClicking()) { // 点击了
                const float x = proxy.MousePos().x;
                const float y = proxy.MousePos().y;
                if (y < 0.04 || y > 0.84) return 0;
                int flg = 0;
                for (int i = 0; i < 7; ++i) {
                    if (x >= rcx[i] - 0.01 && x <= rcx[i] + 0.01) {
                        if (useLine != i) {
                            useLine = i;
                            flg     = 1;
                        } else flg = 2;
                        break;
                    }
                }
                const float lx = proxy.DraggingStartPoint().x;
                const float yx = proxy.DraggingStartPoint().y;
                if (fabs(x - lx) + fabs(y - yx) < 0.0001 && flg == 2) {
                    up[useLine]  = mx[useLine];
                    dw[useLine]  = mi[useLine];
                    upy[useLine] = Ey;
                    dwy[useLine] = Sy;
                }
                return flg;
            }
            if (proxy.IsDragging()) { // 拖动了
                const float ex = proxy.MousePos().x;
                float       ey = proxy.MousePos().y;
                const float sx = proxy.DraggingStartPoint().x;
                float       sy = proxy.DraggingStartPoint().y;
                if (sy < 0.04 || sy > 0.84) return 0;
                if (ey < 0.04 || ey > 0.84) return 0;
                int flg = 0;
                for (int i = 0; i < 7; ++i) {
                    if ((sx > rcx[i] + 0.01 && sx <= rcx[i] + 0.03) || (sx >= rcx[i] - 0.03 && sx < rcx[i] - 0.01)) {
                        useLine = i;
                        flg     = 1;
                        break;
                    }

                    if ((sx >= rcx[i] - 0.01 && sx <= rcx[i] + 0.01) && (ey <= upy[i] && ey >= dwy[i])) {
                        useLine = i;
                        flg     = 2;
                        break;
                    }
                }
                if (flg == 1) { // random drag
                    if (sy < ey) std::swap(sy, ey);
                    up[useLine]  = (sy - Sy) / (Ey - Sy) * (mx[useLine] - mi[useLine]) + mi[useLine];
                    dw[useLine]  = (ey - Sy) / (Ey - Sy) * (mx[useLine] - mi[useLine]) + mi[useLine];
                    upy[useLine] = sy;
                    dwy[useLine] = ey;
                    return 1;
                }
                if (flg == 2) {
                    double d1 = proxy.MouseDeltaPos().y;
                    if (dwy[useLine] + d1 < Sy || upy[useLine] + d1 > Ey) return 0;
                    double nd = (d1) / (Ey - Sy) * (mx[useLine] - mi[useLine]);
                    up[useLine] += nd;
                    dw[useLine] += nd;
                    upy[useLine] += d1;
                    dwy[useLine] += d1;
                    return 1;
                }
                return 0;
            }
        }
    };
    static int      X;
    static int      Y;
    const glm::vec4 black = glm::vec4(0, 0, 0, 1);
    const glm::vec4 grey2 = glm::vec4(0.5, 0.5, 0.5, 0.25);
    const glm::vec4 white = glm::vec4(1, 1, 1, 1);

    void setClassLine(Common::ImageRGB & input, glm::vec4 color, const CoordinateStates & st) {
        for (int i = 0; i < 7; ++i) {
            float tmp = rcx[i];
            DrawLine(input, color, glm::vec2(tmp, Sy), glm::vec2(tmp, Ey), Nx);
            const float rate = ((st.dwy[i] + st.upy[i]) / (Sy + Ey)) * 0.5;
            glm::vec4   pur  = glm::vec4(0.7 - rate, 0, rate, 1);
            glm::vec4   rc   = i == st.useLine ? pur : grey2;
            for (float hx = tmp - 0.009; hx < tmp + 0.009; hx += 0.001)
                DrawLine(input, rc, glm::vec2(hx, st.upy[i]), glm::vec2(hx, st.dwy[i]), 1);
            DrawRect(input, white, glm::vec2(tmp - 0.01, st.upy[i]), glm::vec2(0.02, st.dwy[i] - st.upy[i]), 1);
        }
        for (int i = 0; i < 7; ++i) {
            float tmp = rcx[i];
            PrintText(input, black, glm::vec2(tmp, Wy), LH, _rc_[i]);
        }
    }
    char s[10];
    void setClassRange(Common::ImageRGB & input, glm::vec4 color, const CoordinateStates & st) {
        for (int i = 0; i < 7; ++i) {
            float tmp = rcx[i];
            sprintf(s, "%.1f", st.up[i]);
            PrintText(input, color, glm::vec2(tmp, st.upy[i] + 0.008), 0.02, s + unit[i]);
            sprintf(s, "%.1f", st.dw[i]);
            PrintText(input, color, glm::vec2(tmp, st.dwy[i] - 0.008), 0.02, s + unit[i]);
        }
        return;
    }
    void drawData(Common::ImageRGB & input, glm::vec4 color, const CoordinateStates & st, const int & a) {
        for (int i = 0; i < 6; ++i) {
            float rate  = (st.data[i][a] - st.mi[i]) / (st.mx[i] - st.mi[i]);
            float tmpy1 = rate * (Ey - Sy) + Sy;
            float tmpy2 = (st.data[i + 1][a] - st.mi[i + 1]) / (st.mx[i + 1] - st.mi[i + 1]) * (Ey - Sy) + Sy;
            float tmpx1 = rcx[i];
            float tmpx2 = rcx[i + 1];
            DrawLine(input, color, glm::vec2(tmpx1, tmpy1), glm::vec2(tmpx2, tmpy2), 0);
        }
    }
    bool PaintParallelCoordinates(Common::ImageRGB & input, InteractProxy const & proxy, std::vector<Car> const & data, bool force) {
        static CoordinateStates state(data);
        bool                    change = state.update(proxy);
        X                              = input.GetSizeX();
        Y                              = input.GetSizeY();
        if (! force && ! change) return 0;
        SetBackGround(input, white);
        setClassRange(input, black, state);
        for (int i = 0; i < state.n; ++i) {
            const int & x   = state.useLine;
            float       flg = state.rd[i];
            float       tmp = (state.data[x][i] - state.mi[x]) / (state.mx[x] - state.mi[x]) * 0.5;
            glm::vec4   col = glm::vec4((0.5 - tmp), 0.1, tmp, flg);
            for (int j = 0; j < 7; ++j)
                if (state.data[j][i] > state.up[j] || state.data[j][i] < state.dw[j]) {
                    col.a = 0.1;
                    col.r = col.b = col.g = 0.5;
                    break;
                }
            if (flg) drawData(input, col, state, i);
        }
        setClassLine(input, black, state);
        return true;
    }

    void LIC(ImageRGB & output, Common::ImageRGB const & noise, VectorField2D const & field, int const & step) {
        auto _ = field.size;
        int  n = _.first, m = _.second;
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < m; ++j) {
                float       x = i, y = j;
                glm::vec3 forwards = { 0, 0, 0 };
                float     forwardt = 0;
                for (int k = 0; k <= step; ++k) {
                    auto   _d = field.At(x, y);
                    auto   dx = _d.x, dy = _d.y;
                    double dt_x = 0, dt_y = 0;
                    double dt = 0;
                    if (dy > 0)
                        dt_y = ((floor(y) + 1) - y) / dy;
                    else if (dy < 0)
                        dt_y = (y - (ceil(y) - 1)) / -dy;
                    if (dx > 0)
                        dt_x = ((floor(x) + 1) - x) / dx;
                    else if (dx < 0)
                        dt_x = (x - (ceil(x) - 1)) / -dx;

                    if (! (dx == 0 && dy == 0)) dt = min(dt_x, dt_y);
                    x            = min(max(x + dx * dt, 0), n - 1);
                    y            = min(max(y + dy * dt, 0), m - 1);
                    float weight = pow(cos(0.46 * k), 2);
                    forwards += (noise.At(x, y)) * weight;
                    forwardt += weight;
                }
                x = i, y = j;
                glm::vec3 backwards = { 0, 0, 0 }, backwardt = { 0, 0, 0 };
                for (int k = 1; k <= step; ++k) {
                    auto _d = field.At(x, y);
                    auto dx = -_d.x, dy = -_d.y;
                    double dt_x = 0, dt_y = 0;
                    double dt = 0;
                    if (dy > 0)
                        dt_y = ((floor(y) + 1) - y) / dy;
                    else if (dy < 0)
                        dt_y = (y - (ceil(y) - 1)) / -dy;
                    if (dx > 0)
                        dt_x = ((floor(x) + 1) - x) / dx;
                    else if (dx < 0)
                        dt_x = (x - (ceil(x) - 1)) / -dx;

                    if (! (dx == 0 && dy == 0)) dt = min(dt_x, dt_y);
                    x            = min(max(x + dx * dt, 0), n - 1);
                    y            = min(max(y + dy * dt, 0), m - 1);
                    float weight = pow(cos(0.46 * k), 2);
                    backwards += noise.At(x, y) * weight;
                    backwardt += weight;
                }
                output.At(i, j) = (forwards + backwards) / (forwardt + backwardt);
            }
        }
    }
}; // namespace VCX::Labs::Visualization