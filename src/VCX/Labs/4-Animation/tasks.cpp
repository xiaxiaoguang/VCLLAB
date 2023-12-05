#include "Labs/4-Animation/tasks.h"
#include "CustomFunc.inl"
#include "IKSystem.h"
#include <cmath>
#include <cstring>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>
#include <spdlog/spdlog.h>

namespace VCX::Labs::Animation {
    /*
    问题1 如果太远而无法到达,对于第一种CCDIK方法来说，因为这并不会影响其算法流程，也就是说他会仍然按照一样的做法执行，只是最后画出的图像不符合
    对于第二种FABRIK,代码中实际没有考虑到达不了的情况，但是如果到达不了，算法会选择所有关节展开成一条直线指向终点，最后画出的图像同样不符合。
    问题2 对于CCDIK我们使用两次迭代之间最后位置点移动的距离作为判断迭代是否收敛的依据，得出平均迭代次数在3.75左右
    FABRIK则按照给定正常方式判断，平均迭代次数只有2.2左右，相比之下要有40%左右的提升
    问题3 我们每次可以从上一次机械臂的状态开始调整，而不是从初始位置开始调整，这样算法需要兼容计算新的偏移量，但是也极大减少了两种较大差异的迭代结果横跳现象的发生
    */
    using namespace glm;
    using std::cout;
    using std::string;
    quat v3_qt(vec3 a) {
        return quat(0, a[0], a[1], a[2]);
    }
    vec3 qt_v3(quat a) {
        assert(a[3] != 0);
        return vec3(a[0], a[1], a[2]);
    }

    void outp(vec3 a, string s = "") {
        cout << s;
        printf("point : %.2f %.2f %.2f\n", a[0], a[1], a[2]);
    }
    void outq(quat a, string s = "") {
        cout << s;
        printf("quat : %.2f %.2f %.2f %.2f\n", a[0], a[1], a[2], a[3]);
    }

    void ForwardKinematics(IKSystem & ik, int StartIndex) {
        if (StartIndex == 0) {
            ik.JointGlobalRotation[0] = ik.JointLocalRotation[0];
            ik.JointGlobalPosition[0] = ik.JointLocalOffset[0];
            StartIndex                = 1;
        }
        assert(ik.JointGlobalPosition.size() != ik.NumJoints());
        for (int i = StartIndex; i < ik.NumJoints(); i++) {
            // your code here: forward kinematics, update JointGlobalPosition and JointGlobalRotation
            auto &       a = ik.JointGlobalRotation[i];
            const auto & b = ik.JointGlobalRotation[i - 1];
            const auto & c = ik.JointLocalRotation[i];
            a              = b * c;
            // 我们认为局部旋转是一个标记，他和偏移量不同。偏移量是相较于上一次局部旋转为0时的位移差
            // 也就是说它是在到上一次0转时的累积旋转程度。所以我们这次更新偏移量和全局位置/全局旋转之后，要清空他。
            // 这样我们修改时只需要打标记就好了
            // 这里旋转不需要考虑偏置，我们某个点对某条线的旋转需要考虑偏置
            // 因为这里是过原点的。
            ik.JointGlobalPosition[i] = ik.JointGlobalPosition[i - 1] + qt_v3(a * v3_qt(ik.JointLocalOffset[i]) * inverse(a));
        }
    }
    const float eps = 1e-5;
    struct line {
        vec3 s, e;
        line() {};
        line(vec3 a, vec3 b):
            s(a), e(b) {
            e = normalize(e);
        }
        friend line cross(const line & a, const line & b) {
            assert(a.s != b.s);
            auto nd = cross(a.e, b.e);
            while (length(nd) < eps) nd = cross(a.e, normalize(vec3(rand(), rand(), rand())));
            nd = normalize(nd);
            assert(isnan(nd[0] + nd[1] + nd[2]) || isinf(nd[0] + nd[1] + nd[2]));
            return line(a.s, nd);
        }
        friend float gettheta(const line & a, const line & b) {
            float qwq = dot(a.e, b.e);
            if (fabs(qwq - 1) < eps) return 0;
            if (fabs(qwq + 1) < eps) return M_PI;
            return acos(dot(a.e, b.e));
        }
    };
    // 这里得到的是没有偏置的旋转
    quat cons_rotate(const line & u, const float & theta) {
        auto tmp = sin(theta / 2) * u.e;
        return quat(cos(theta / 2), tmp[0], tmp[1], tmp[2]);
    }
    // 考虑偏置，一个点绕任意一条直线的旋转
    void exec_rotate(vec3 & a, const line & b, const float & t) {
        quat ub = cons_rotate(b, t);
        a       = qt_v3(ub * v3_qt(a - b.s) * inverse(ub)) + b.s;
    }
    void InverseKinematicsCCD(IKSystem & ik, const glm::vec3 & EndPosition, int maxCCDIKIteration, float eps) {
        ForwardKinematics(ik, 0);//四元数生成：第一个是常数，然后第二三四个是ijk,然后输出的时候a[0]是i,a[3]才是常数
        vec3 delta = ik.EndEffectorPosition();
        for (int CCDIKIteration = 0; CCDIKIteration < maxCCDIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps * CCDIKIteration * 5; CCDIKIteration++) {
            // your code here: ccd ik
            auto MyEnd = ik.EndEffectorPosition();
            for (int i = ik.NumJoints() - 2; i >= 0; --i) { // 考虑每个关节？
                auto l1    = line(ik.JointGlobalPosition[i], EndPosition - ik.JointGlobalPosition[i]);
                auto l2    = line(ik.JointGlobalPosition[i], MyEnd - ik.JointGlobalPosition[i]);
                auto u     = cross(l2, l1);
                auto theta = gettheta(l2, l1);
                auto newrotate           = cons_rotate(u, theta);
                ik.JointLocalRotation[i] = newrotate * ik.JointLocalRotation[i];
                exec_rotate(MyEnd, u, theta);
            }
            ForwardKinematics(ik, 0); // 整个更新一遍！
            if (glm::l2Norm(delta - ik.EndEffectorPosition()) < (eps / 10)) break;
            else delta = ik.EndEffectorPosition();
        }
    }
    void InverseKinematicsFABR(IKSystem & ik, const glm::vec3 & EndPosition, int maxFABRIKIteration, float eps) {
        ForwardKinematics(ik, 0);
        int                    nJoints = ik.NumJoints();
        std::vector<glm::vec3> backward_positions(nJoints, glm::vec3(0, 0, 0)), forward_positions(nJoints, glm::vec3(0, 0, 0));
        for (int IKIteration = 0; IKIteration < maxFABRIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; IKIteration++) {
            // task: fabr ik
            // backward update
            glm::vec3 next_position         = EndPosition;
            backward_positions[nJoints - 1] = EndPosition;
            for (int i = nJoints - 2; i > 0; i--) {
                const auto & a   = ik.JointGlobalPosition[i];
                const auto & b   = backward_positions[i + 1];
                auto         dir = normalize(a - b);
                backward_positions[i] = (length(ik.JointLocalOffset[i + 1])) * dir + b;
            }
            // forward update
            glm::vec3 now_position = ik.JointGlobalPosition[0];
            forward_positions[0]   = ik.JointGlobalPosition[0];
            for (int i = 1; i < nJoints; i++) {
                const auto & a       = backward_positions[i];
                const auto & b       = forward_positions[i - 1];
                auto         dir     = normalize(a - b);
                forward_positions[i] = dir * length(ik.JointLocalOffset[i]) + b;
            }
            ik.JointGlobalPosition = forward_positions; // copy forward positions to joint_positions
        }
        // Compute joint rotation by position here.
        for (int i = 1; i < nJoints; i++)
            ik.JointGlobalRotation[i] = glm::rotation(glm::normalize(ik.JointLocalOffset[i]), glm::normalize(ik.JointGlobalPosition[i] - ik.JointGlobalPosition[i - 1]));
        ik.JointLocalRotation[0] = ik.JointGlobalRotation[0];
        for (int i = 1; i < nJoints; i++) {
            ik.JointLocalRotation[i] = glm::inverse(ik.JointGlobalRotation[i - 1]) * ik.JointGlobalRotation[i];
        }
        ForwardKinematics(ik, 0);
    }
    float myx(float a) {
        if (a < 0.25) return 1;
        if (a < 0.5) return 1 - (a - 0.25) * 8;
        if (a < 0.75) return 0;
        return -1;
    }
    float myy(float a) {
        if (a < 0.25) return a * 5 - 0.625;
        if (a < 0.5) return 0;
        if (a < 0.75) return (a - 0.5) * 4 - 0.5;
        return (a - 0.75) * 5 - 0.625;
    }
    IKSystem::Vec3ArrPtr IKSystem::BuildCustomTargetPosition() {
        // get function from https://www.wolframalpha.com/input/?i=Albert+Einstein+curve
        int nums      = 500;
        using Vec3Arr = std::vector<glm::vec3>;
        std::shared_ptr<Vec3Arr> custom(new Vec3Arr(nums));
        int                      index = 0;
        for (int i = 0; i < nums; i++) {
            float x_val        = myx((float) i / nums);
            float y_val        = myy((float) i / nums);
            (*custom)[index++] = glm::vec3(x_val, 0.0f, y_val);
        }
        custom->resize(index);
        return custom;
    }

    static Eigen::VectorXf glm2eigen(std::vector<glm::vec3> const & glm_v) {
        Eigen::VectorXf v = Eigen::Map<Eigen::VectorXf const, Eigen::Aligned>(reinterpret_cast<float const *>(glm_v.data()), static_cast<int>(glm_v.size() * 3));
        return v;
    }

    static std::vector<glm::vec3> eigen2glm(Eigen::VectorXf const & eigen_v) {
        return std::vector<glm::vec3>(
            reinterpret_cast<glm::vec3 const *>(eigen_v.data()),
            reinterpret_cast<glm::vec3 const *>(eigen_v.data() + eigen_v.size()));
    }

    static Eigen::SparseMatrix<float> CreateEigenSparseMatrix(std::size_t n, std::vector<Eigen::Triplet<float>> const & triplets) {
        Eigen::SparseMatrix<float> matLinearized(n, n);
        matLinearized.setFromTriplets(triplets.begin(), triplets.end());
        return matLinearized;
    }

    // solve Ax = b and return x
    static Eigen::VectorXf ComputeSimplicialLLT(
        Eigen::SparseMatrix<float> const & A,
        Eigen::VectorXf const &            b) {
        auto solver = Eigen::SimplicialLLT<Eigen::SparseMatrix<float>>(A);
        return solver.solve(b);
    }

    auto getforces(MassSpringSystem & system) {
        std::vector<glm::vec3> forces(system.Positions.size(), glm::vec3(0));
        for (auto const spring : system.Springs) {
            auto const      p0  = spring.AdjIdx.first;
            auto const      p1  = spring.AdjIdx.second;
            glm::vec3 const x01 = system.Positions[p1] - system.Positions[p0];
            glm::vec3 const v01 = system.Velocities[p1] - system.Velocities[p0];
            glm::vec3 const e01 = glm::normalize(x01);
            glm::vec3       f   = (system.Stiffness * (glm::length(x01) - spring.RestLength) + system.Damping * glm::dot(v01, e01)) * e01;
            forces[p0] -= f;
            forces[p1] += f;
        }
        return forces;
    }
    auto v3_convert(vec3 a) {
        Eigen::VectorXd ret(3);
        ret << a[0], a[1], a[2];
        return ret;
    }
    void outvec(const Eigen::VectorXf & xk, int n) {
        for (int i = 0; i < n; ++i) {
            printf("%f ", xk[i]);
        }
        puts("");
    }
    void AdvanceMassSpringSystem(MassSpringSystem & system, float const dt) {
        // your code here: rewrite following code
        // int const   steps = 1000;
        // float const ddt   = dt / steps;
        // for (std::size_t s = 0; s < steps; s++) {
        //     // 显示欧拉
        //     std::vector<glm::vec3> forces(system.Positions.size(), glm::vec3(0));
        //     for (auto const spring : system.Springs) {
        //         auto const      p0  = spring.AdjIdx.first;
        //         auto const      p1  = spring.AdjIdx.second;
        //         glm::vec3 const x01 = system.Positions[p1] - system.Positions[p0];
        //         glm::vec3 const v01 = system.Velocities[p1] - system.Velocities[p0];
        //         glm::vec3 const e01 = glm::normalize(x01);
        //         glm::vec3       f   = (system.Stiffness * (glm::length(x01) - spring.RestLength) + system.Damping * glm::dot(v01, e01)) * e01;
        //         forces[p0] += f;
        //         forces[p1] -= f;
        //     }
        //     for (std::size_t i = 0; i < system.Positions.size(); i++) {
        //         if (system.Fixed[i]) continue;
        //         system.Velocities[i] += (glm::vec3(0, -system.Gravity, 0) + forces[i] / system.Mass) * ddt;
        //         system.Positions[i] += system.Velocities[i] * ddt;
        //     }
        // }
        //隐式欧拉
        float const h = dt;
        int n = system.Positions.size();
        Eigen::VectorXf        xk = glm2eigen(system.Positions);
        std::vector<glm::vec3> tmp_m(n, glm::vec3(0, -system.Gravity / system.Mass, 0));
        Eigen::VectorXf yk = (xk / (h * h))+ glm2eigen(system.Velocities) / h + glm2eigen(tmp_m);
        Eigen::VectorXf gx = (xk / (h * h) - yk) * system.Mass + glm2eigen(getforces(system));
        std::vector<Eigen::Triplet<float>> triplets = {};
        float                              tmp_v    = system.Mass / (h * h);
        for (int i = 0; i < 3 * n; ++i) triplets.emplace_back(i, i, tmp_v);
        auto hx = CreateEigenSparseMatrix(3 * n, triplets);
        triplets.clear();
        for (auto const spring : system.Springs) {
            auto const      p0 = spring.AdjIdx.first;
            auto const      p1 = spring.AdjIdx.second;
            auto const      le = length(system.Positions[p0] - system.Positions[p1]);
            auto const      xy = v3_convert(system.Positions[p0] - system.Positions[p1]);
            //这里是因为他没有真正的转置vector，他只是打上了一个转置的符号！所以如果中间存储强制类型转换就会丢失
            Eigen::MatrixXd mid = xy *  xy.transpose();
            auto m3 = mid / (le * le);
            auto a2 = system.Stiffness * (m3 + (1 - spring.RestLength / le) * (Eigen::MatrixXd::Identity(3, 3) - m3));
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    triplets.emplace_back(p0 * 3 + i, p1 * 3 + j, -a2(i, j));
                    triplets.emplace_back(p1 * 3 + i, p0 * 3 + j, -a2(i, j));
                    triplets.emplace_back(p0 * 3 + i, p0 * 3 + j, a2(i, j));
                    triplets.emplace_back(p1 * 3 + i, p1 * 3 + j, a2(i, j));
                }
            }
        }
        hx += CreateEigenSparseMatrix(3 * n, triplets);
        Eigen::VectorXf result = ComputeSimplicialLLT(hx, -gx);
        std::vector<vec3> xrctmp = eigen2glm(xk + result);
        std::vector<vec3> vrctmp = eigen2glm(result/h);

        for(int i=0;i<n;++i){
            if(system.Fixed[i])continue;
            system.Positions[i]=xrctmp[i];
            system.Velocities[i]=vrctmp[i];
        }
    }
} // namespace VCX::Labs::Animation
