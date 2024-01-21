#include "Labs/3-Rendering/CaseRayTracing.h"

namespace VCX::Labs::Rendering {

    CaseRayTracing::CaseRayTracing(std::initializer_list<Assets::ExampleScene> && scenes):
        _scenes(scenes),
        _program(
            Engine::GL::UniqueProgram({
                Engine::GL::SharedShader("assets/shaders/flat.vert"),
                Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _sceneObject(4),
        _texture({ .MinFilter = Engine::GL::FilterMode::Linear, .MagFilter = Engine::GL::FilterMode::Nearest }) {
        _cameraManager.AutoRotate = false;
        _program.GetUniforms().SetByName("u_Color", glm::vec3(1, 1, 1));
    }

    CaseRayTracing::~CaseRayTracing() {
        _stopFlag = true;
        if (_task.joinable()) _task.join();
    }

    void CaseRayTracing::OnSetupPropsUI() {
        if (ImGui::BeginCombo("Scene", GetSceneName(_sceneIdx))) {
            for (std::size_t i = 0; i < _scenes.size(); ++i) {
                bool selected = i == _sceneIdx;
                // printf("%d?\b",i);
                if (ImGui::Selectable(GetSceneName(i), selected)) {
                    if (! selected) {
                        _sceneIdx   = i;
                        _sceneDirty = true;
                        _treeDirty = true;
                        _resetDirty = true;
                    }
                }
            }
            ImGui::EndCombo();
        }
        if (ImGui::Button("Reset Scene")) _resetDirty = true;
        ImGui::SameLine();
        if (_task.joinable()) {
            if (ImGui::Button("Stop Rendering")) {
                _stopFlag = true;
                if (_task.joinable()) _task.join();
            }
        } else if (ImGui::Button("Start Rendering")) _stopFlag = false;
        ImGui::ProgressBar(float(_pixelIndex) / (_buffer.GetSizeX() * _buffer.GetSizeY()));
        Common::ImGuiHelper::SaveImage(_texture, GetBufferSize(), true);
        ImGui::Spacing();

        if (ImGui::CollapsingHeader("Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
            _resetDirty |= ImGui::SliderInt("Sample Rate", &_superSampleRate, 1, 100);
            _resetDirty |= ImGui::SliderInt("Max Depth", &_maximumDepth, 1, 15);
            _resetDirty |= ImGui::Checkbox("Shadow Ray", &_enableShadow);
        }
        ImGui::Spacing();

        if (ImGui::CollapsingHeader("Control")) {
            ImGui::Checkbox("Zoom Tooltip", &_enableZoom);
        }
        ImGui::Spacing();
    }
    static glm::vec3 operator%(const glm::vec3 & a,const glm::vec3 & b) {
         return glm::vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
    }
    inline float clamp(float x) { return x < 0 ? 0 : x > 1 ? 1 : x;}
    Common::CaseRenderResult CaseRayTracing::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (_resetDirty) {
            _stopFlag = true;
            if (_task.joinable()) _task.join();
            _pixelIndex = 0;
            _resizable  = true;
            _resetDirty = false;
        }
        if (_sceneDirty) {
            _sceneObject.ReplaceScene(GetScene(_sceneIdx));
            _cameraManager.Save(_sceneObject.Camera);
            _sceneDirty = false;
        }
        if (_resizable) {
            _frame.Resize(desiredSize);
            _cameraManager.Update(_sceneObject.Camera);
            _program.GetUniforms().SetByName("u_Projection", _sceneObject.Camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
            _program.GetUniforms().SetByName("u_View"      , _sceneObject.Camera.GetViewMatrix());
            
            gl_using(_frame);

            glEnable(GL_DEPTH_TEST);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            for (auto const & model : _sceneObject.OpaqueModels)
                model.Mesh.Draw({ _program.Use() });
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glDisable(GL_DEPTH_TEST);
        }
        if (! _stopFlag && ! _task.joinable()) {
            if (_pixelIndex == 0) {
                _resizable = false;
                _buffer    = _frame.GetColorAttachment().Download<Engine::Formats::RGB8>();
            }
            _task = std::thread([&]() {
                auto const width  = _buffer.GetSizeX();
                auto const height = _buffer.GetSizeY();
                if (_pixelIndex == 0 && _treeDirty) {
                    Engine::Scene const & scene = GetScene(_sceneIdx);
                    _intersector.InitScene(&scene);
                    _treeDirty = false;
                }
                // Render into tex.
                auto const & camera    = _sceneObject.Camera;
                glm::vec3 lookDir   = glm::normalize(camera.Target - camera.Eye);
                glm::vec3    rightDir  = glm::normalize(glm::cross(lookDir, camera.Up)); 
                glm::vec3    upDir     = glm::normalize(glm::cross(rightDir, lookDir));
                float const  aspect    = width * 1.f / height;
                float const  fovFactor = std::tan(glm::radians(camera.Fovy) / 2);                
                // glm::vec3 cx = glm::vec3(width * .5135 / height), cy = glm::normalize(cx % lookDir) * (float)0.5135, r,
                //         *c = (glm::vec3 *) malloc(sizeof(glm::vec3) * width * height);                
                while (_pixelIndex < std::size_t(width) * height) {
                    int i = _pixelIndex % width;
                    int j = _pixelIndex / width;
                    glm::vec3 sum(0.0f);
                    glm::vec3 r(0.0f);
                    union tmp{
                        unsigned short Xi[3];
                        double y;
                    }a;
                    a.y = (long long)j * j * j;
                    // for (int sy = 0; sy < 2; sy++){         // 2x2 subpixel rows
                    //     for (int sx = 0; sx < 2; sx++,r=glm::vec3()) {                 // 2x2 subpixel cols
                    for(int dy = 0;dy<_superSampleRate;++dy){
                        for (int dx = 0; dx < _superSampleRate; ++dx) {
                                float        step = 1.0f / _superSampleRate;
                                double r1 = 2 * erand48(a.Xi);
                                float di = step * (0.5f + dx) + (r1 < 1 ? sqrt(r1) - 1 : 1 - sqrt(2 - r1));
                                double r2 = 2 * erand48(a.Xi);
                                float dj = step * (0.5f + dy) + (r2 < 1 ? sqrt(r2) - 1 : 1 - sqrt(2 - r2));

                                glm::vec3    lookDir   = glm::normalize(camera.Target - camera.Eye);
                                glm::vec3    rightDir  = glm::normalize(glm::cross(lookDir, camera.Up));
                                glm::vec3    upDir     = glm::normalize(glm::cross(rightDir, lookDir));
                                float const  aspect    = width * 1.f / height;
                                float const  fovFactor = std::tan(glm::radians(camera.Fovy) / 2);
                                lookDir += fovFactor * (2.0f * (j + dj) / height - 1.0f) * upDir;
                                lookDir += fovFactor * aspect * (2.0f * (i + di) / width - 1.0f) * rightDir;
                                Ray       initialRay(camera.Eye, glm::normalize(lookDir));
                                glm::vec3 res = RayTrace(_intersector, initialRay, a.Xi , _maximumDepth);
                                sum += glm::pow(res, glm::vec3(1.0 / 2.2));
                        }
                    }
                    _buffer.At(i, j) = sum  / glm::vec3(_superSampleRate * _superSampleRate);
                    if (_stopFlag) return;
                    ++_pixelIndex;
                    // int x = _pixelIndex % width;
                    // int y = _pixelIndex / width;
                    // unsigned short Xi[3]={0,0,0};
                    // glm::vec3 sum=glm::vec3(0,0,0);
                    // Xi[2] = y*y*y;
                    // for (int sy = 0; sy < 2; sy++){         // 2x2 subpixel rows
                    //     for (int sx = 0; sx < 2; sx++, r = glm::vec3()) {                 // 2x2 subpixel cols
                    //         for (int s = 0; s < _superSampleRate; s++) {
                    //             double r1 = 2 * erand48(Xi), dx = r1 < 1 ? sqrt(r1) - 1 : 1 - sqrt(2 - r1);
                    //             // r1
                    //             double r2 = 2 * erand48(Xi), dy = r2 < 1 ? sqrt(r2) - 1 : 1 - sqrt(2 - r2);
                    //             // r2 dx dy  s时samples次数
                    //             // r1r2 形成一个范围内的随机采样
                    //             glm::vec3 d = cx * (float)(((sx + .5 + dx) / 2 + x) / width - .5) +
                    //                          cy * (float)(((sy + .5 + dy) / 2 + y) / height - .5) + lookDir;
                    //             // d是直线的
                    //             r = r + RayTrace(_intersector,Ray(camera.Eye + d * (float)140, normalize(d)), Xi, 0)
                    //                     * (float)(1. / _superSampleRate);
                    //             // r时采样结果，除以平均便是mento carlo
                    //         } // Camera rays are pushed ^^^^^ forward to start in interior
                    //         sum += glm::vec3(glm::clamp(r.x), glm::clamp(r.y), glm::clamp(r.z)) * (float).25;
                    //         // _buffer.At(x, y) = _buffer.At(x, y) + glm::vec3(glm::clamp(r.x), glm::clamp(r.y), glm::clamp(r.z)) * (float).25;
                    //         // 增加权重
                    //     }
                    // }
                    // printf("%d %d sum : %f %f %f?\n",x,y,sum.x,sum.y,sum.z);
                    // _buffer.At(x, y) = sum;
                    // if (_stopFlag) return;
                    // ++_pixelIndex;
                    }

            });
        }

        if (! _resizable) {
            if (!_stopFlag) _texture.Update(_buffer);
            if (_task.joinable() && _pixelIndex == _buffer.GetSizeX() * _buffer.GetSizeY()) {
                _stopFlag = true;
                _task.join();
            }
        }

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _resizable ? _frame.GetColorAttachment() : _texture,
            .ImageSize = _resizable ? desiredSize : GetBufferSize(),
        };
    }

    void CaseRayTracing::OnProcessInput(ImVec2 const & pos) {
        auto         window  = ImGui::GetCurrentWindow();
        bool         hovered = false;
        bool         anyHeld = false;
        ImVec2 const delta   = ImGui::GetIO().MouseDelta;
        ImGui::ButtonBehavior(window->Rect(), window->GetID("##io"), &hovered, &anyHeld);
        if (! hovered) return;
        if (_resizable) {
            _cameraManager.ProcessInput(_sceneObject.Camera, pos);
        } else {
            if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && delta.x != 0.f)
                ImGui::SetScrollX(window, window->Scroll.x - delta.x);
            if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && delta.y != 0.f)
                ImGui::SetScrollY(window, window->Scroll.y - delta.y);
        }
        if (_enableZoom && ! anyHeld && ImGui::IsItemHovered())
            Common::ImGuiHelper::ZoomTooltip(_resizable ? _frame.GetColorAttachment() : _texture, GetBufferSize(), pos, true);
    }

} // namespace VCX::Labs::Rendering
