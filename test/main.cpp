#include "DxLib.h"
#include "../common/kdop.hpp"

namespace {
    constexpr int kWindowWidth  = 512;
    constexpr int kWindowHeight = 512;
} // unnamed namespace


int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {
    ChangeWindowMode(1);
    SetWindowText(_T("ê¸ï™Ç∆ì_ÇÃç≈ê⁄ãﬂì_"));
    SetGraphMode(kWindowWidth, kWindowHeight, 16);
    SetDrawScreen(DX_SCREEN_BACK);
    if(DxLib_Init() == -1) return 1;

    while(ProcessMessage() != -1 && !CheckHitKey(KEY_INPUT_ESCAPE)) {

        ClearDrawScreen();
        ScreenFlip();
    }

    DxLib_End();
    return 0;
}

// EOF
    