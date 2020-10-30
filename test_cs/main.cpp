#include <iostream>
#include "../geometry/utility/utility.hpp"
#include "../geometry/utility/access.hpp"
#include "../geometry/traits/traits.hpp"
#include "../geometry/traits/point_traits.hpp"

#include "../geometry/calculation/distance.hpp"

using namespace geometry;
using namespace utility;

class float2 {
public :
    float2(const float X, const float Y) :
        x_{X}, y_{Y} {}

    float x() const noexcept { return x_; }
    float y() const noexcept { return y_; }
    
private :
    float x_;
    float y_;
};
template <>
struct geometry::traits::Category<float2> : MetaType<PointCategory> {};

struct FLT2 {
    float x;
    float y;
};

template <>
struct traits::PointTraits<FLT2> {
    static float x(const FLT2& P) noexcept {
        return P.x;
    }
    static float y(const FLT2& P) noexcept {
        return P.y;
    }
};
template <>
struct traits::Category<FLT2> : MetaType<PointCategory> {};


int main() {
    float2 flt {2.0F, 3.0F};
    FLT2 flt2{ 3.1F, 3.0F };

    std::cout << traits::PointDimension_v<float2> << std::endl;
    std::cout << distance(flt, flt2) << std::endl;
    std::cout << (distance(flt, flt2) < distance(flt, flt2)) << std::endl;

    traits::ValueType<float2>::type t0;
    traits::PointType<float2>::type t1{0, 0};
    traits::PointType<FLT2>::type t2;
    distance(flt, flt2);
}
