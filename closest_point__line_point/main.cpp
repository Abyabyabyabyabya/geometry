#include "DxLib.h"

///
/// 作成者　 : 板場
/// 最終更新 : 2020/10/05
///
/// 線分と点の最接近点を求める
///


struct Point { int x,y; };
struct Line : Point {
    Line(const Point& S, const Point& E) :
        Point{E.x-S.x, E.y-S.y} {}
};


namespace {
constexpr int kWindowWidth  = 516;
constexpr int kWindowHeight = 516;
constexpr int kPointRadius  = 5;
constexpr Point kStartPoint {150, 256};
constexpr Point kEndPoint {kWindowWidth-kStartPoint.x, 256};
}


Point closestPoint(const Point&, const Point&, const Point); // 線分から点への最接近点を求める

int dot(const Point& A, const Point& B) { return (A.x*B.x) + (A.y*B.y); }
int drawPoint(const Point& P, const unsigned Color=0xffffff ) { return DrawPixel(P.x, P.y, Color); }
int drawLine(const Point& Start, const Point& End, const unsigned Color=0xffffff) { return DrawLine(Start.x, Start.y, End.x, End.y, Color); }
Point mousePoint() { 
    Point point;
    GetMousePoint(&point.x, &point.y);
    return point;
}


int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {
    ChangeWindowMode(1);
    SetWindowText(_T("線分と点の最接近点"));
    SetGraphMode(kWindowWidth, kWindowHeight, 16);
    SetDrawScreen(DX_SCREEN_BACK);
    if(DxLib_Init() == -1) return 1;
    
    while(ProcessMessage() != -1 && !CheckHitKey(KEY_INPUT_ESCAPE)) {
        Point mouse_point = mousePoint();
        Point closest_point = closestPoint(kStartPoint, kEndPoint, mouse_point);

        ClearDrawScreen();
          drawLine(kStartPoint, kEndPoint);               // 線分
          drawLine(kStartPoint, mouse_point);             // 線分の始点から、マウスポイントまでの線分
          drawLine(mouse_point, closest_point, 0x00ffff); // マウスポイントから、線分上の最接近点までの線分
          DrawCircle(closest_point.x, closest_point.y, kPointRadius, 0x00ff0f);
        ScreenFlip();
    }

    DxLib_End();
    return 0;
}


///
/// 線分と点の最接近点を求める
/// 「ゲームプログラミングのためのリアルタイム衝突判定」
///  - Page.128
///
/// in SegStart : 線分の始点
/// in SegEnd   : 線分の終点
/// in Pt       : 点
///
/// ret 最接近点(Seg線分上にある最もPtに近い点)
///
Point closestPoint(const Point& SegStart, const Point& SegEnd, const Point Pt) {
    Line ab{SegStart, SegEnd}; // 線分ベクトル

    float t = float(dot(Line{SegStart,Pt}, ab)) / float(dot(ab,ab));
    if(t < 0.0F) t = 0.0F;
    if(t > 1.0F) t = 1.0F;

    return Point {
        SegStart.x + static_cast<int>(ab.x*t),
        SegStart.y + static_cast<int>(ab.y*t)
    };
}
// EOF
