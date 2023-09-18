#include <algorithm>
#include <array>

#include "Labs/0-GettingStarted/CaseFixed.h"
#include "Labs/Common/ImGuiHelper.h"

namespace VCX::Labs::GettingStarted {
    static constexpr auto c_Sizes = std::to_array<std::pair<std::uint32_t, std::uint32_t>>({
        { 320U, 320U },
        { 640U, 640U } 
    });

    static constexpr auto c_SizeItems = std::array<char const *, 2> {
        "Small (320 x 320)",
        "Large (640 x 640)"
    };

    static constexpr auto c_BgItems = std::array<char const *, 3> {
        "White",
        "Black",
        "Checkboard"
    };

    CaseFixed::CaseFixed() :
        _textures(Engine::make_array<Engine::GL::UniqueTexture2D, c_Sizes.size()>(
            Engine::GL::SamplerOptions {
                .MinFilter = Engine::GL::FilterMode::Linear,
                .MagFilter = Engine::GL::FilterMode::Nearest
            })),
        _empty({
            Common::CreatePureImageRGB(c_Sizes[0].first, c_Sizes[0].second, { 2.f / 17, 2.f / 17, 2.f / 17 }),
            Common::CreatePureImageRGB(c_Sizes[1].first, c_Sizes[1].second, { 2.f / 17, 2.f / 17, 2.f / 17 })
        }) {
    }

    void CaseFixed::OnSetupPropsUI() {
        ImGui::Checkbox("Zoom Tooltip", &_enableZoom);
        _recompute |= ImGui::Combo("Size", &_sizeId, c_SizeItems.data(), c_SizeItems.size());
        _recompute |= ImGui::Combo("Background", &_bgId, c_BgItems.data(), c_BgItems.size());
    }

    Common::CaseRenderResult CaseFixed::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        auto const width = c_Sizes[_sizeId].first;
        auto const height = c_Sizes[_sizeId].second;
        if (_recompute) {
            _recompute = false;
            _task.Emplace([this, width, height]() {
                Common::ImageRGB image(0, 0);
                switch (_bgId) {
                case 0:
                    image = Common::CreatePureImageRGB(width, height, { 1., 1., 1. });
                    break;
                case 1:
                    image = Common::CreatePureImageRGB(width, height, { 0., 0., 0. });
                    break;
                case 2:
                    image = Common::CreateCheckboardImageRGB(width, height);
                    break;
                }
                for (std::size_t x = 0; x < width; ++x)
                    for (std::size_t y = 0; y < height; ++y)
                        if (x + y < width) image.At(x, y) = { 1., 0., 0. };
                return image;
            });
        }
        _textures[_sizeId].Update(_task.ValueOr(_empty[_sizeId]));
        return Common::CaseRenderResult {
            .Fixed     = true,
            .Image     = _textures[_sizeId],
            .ImageSize = c_Sizes[_sizeId],
        };
    }

    void CaseFixed::OnProcessInput(ImVec2 const & pos) {
        auto         window  = ImGui::GetCurrentWindow();
        bool         hovered = false;
        bool         anyHeld = false;
        ImVec2 const delta   = ImGui::GetIO().MouseDelta;
        ImGui::ButtonBehavior(window->Rect(), window->GetID("##io"), &hovered, &anyHeld);
        if (! hovered) return;
        if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && delta.x != 0.f)
            ImGui::SetScrollX(window, window->Scroll.x - delta.x);
        if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && delta.y != 0.f)
            ImGui::SetScrollY(window, window->Scroll.y - delta.y);
        if (_enableZoom && ! anyHeld && ImGui::IsItemHovered())
            Common::ImGuiHelper::ZoomTooltip(_textures[_sizeId], c_Sizes[_sizeId], pos);
    }
}
