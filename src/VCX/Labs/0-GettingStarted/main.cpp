#include "Assets/bundled.h"
#include "Labs/0-GettingStarted/App.h"

int main() {
    using namespace VCX;
    return Engine::RunApp<Labs::GettingStarted::App>(Engine::AppContextOptions {
        .Title      = "VCX Labs 0: Getting Started",
        .WindowSize = { 2560, 1440 },
        .FontSize   = 20,

        .IconFileNames = Assets::DefaultIcons,
        .FontFileNames = Assets::DefaultFonts,
    });
}
