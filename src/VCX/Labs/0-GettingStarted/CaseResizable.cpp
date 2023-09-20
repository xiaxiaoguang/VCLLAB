#include "Labs/0-GettingStarted/CaseResizable.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/ImGuiHelper.h"

namespace VCX::Labs::GettingStarted {

//在这里选择每个顶点距离画框多少，中间两行控制三角形上顶点，上面为左下
    static constexpr auto c_PositionData = std::to_array<glm::vec2>({
        { -1, -1 },
        {  0  ,  1 },
        {  1, -1 },
    });
// 在这里选择红绿蓝，中间为绿,上面为红，下面为蓝
    static constexpr auto c_ColorData = std::to_array<glm::vec3>({
        { 1, 1, 1 },
        { 0, 0.9, 0 },
        { 0, 0, 0.9 },
    });

    CaseResizable::CaseResizable() :
         _program(
            Engine::GL::UniqueProgram({
                Engine::GL::SharedShader("assets/shaders/triangle.vert"),
                Engine::GL::SharedShader("assets/shaders/triangle.frag") })),
        _mesh(
            Engine::GL::VertexLayout()
                .Add<glm::vec2>("position", Engine::GL::DrawFrequency::Static, 0)
                .Add<glm::vec3>("color", Engine::GL::DrawFrequency::Static, 1)) {
        _mesh.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec2>(c_PositionData));
        _mesh.UpdateVertexBuffer("color", Engine::make_span_bytes<glm::vec3>(c_ColorData));
    }

    void CaseResizable::OnSetupPropsUI() {
        ImGui::Checkbox("Zoom Tooltip", &_enableZoom);
    }

    Common::CaseRenderResult CaseResizable::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        _frame.Resize(desiredSize);
        gl_using(_frame);
        _mesh.Draw({ _program.Use() });
        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void CaseResizable::OnProcessInput(ImVec2 const& pos) {
        auto         window  = ImGui::GetCurrentWindow();
        bool         hovered = false;
        bool         anyHeld = false;
        ImGui::ButtonBehavior(window->Rect(), window->GetID("##io"), &hovered, &anyHeld);
        if (! hovered) return;
        if (_enableZoom && ! anyHeld && ImGui::IsItemHovered())
            Common::ImGuiHelper::ZoomTooltip(_frame.GetColorAttachment(), _frame.GetSize(), pos, true);
    }
}

