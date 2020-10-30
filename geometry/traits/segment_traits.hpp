#ifndef INCLUDED_GEOMETRY_GEOMETRIES_SEGMENT_HEADER_
#define INCLUDED_GEOMETRY_GEOMETRIES_SEGMENT_HEADER_

#include <utility>
#include "traits.hpp"
#include "../utility/access.hpp"
#include "../utility/utility.hpp"

namespace geometry {
namespace traits {

template <class S>
struct SegmentTraits {
    static auto s(const S& S_) noexcept(noexcept(S_.s())) {
        return S_.s();
    }
    static auto d(const S& S_) noexcept(noexcept(S_.d())) -> decltype(s(std::declval<S>())) {
        return S_.d();
    }
};

} // namespace traits
} // namespace geometry

// template specialization
template <>
struct geometry::traits::impl::TraitsImpl<geometry::traits::SegmentCategory> :
    utility::MetaTemplateType<SegmentTraits> {};
template <class S>
struct geometry::traits::impl::DimensionImpl<geometry::traits::SegmentTraits<S>> :
    utility::MetaValue<2> {};
template <class S>
struct geometry::traits::impl::ValueTypeImpl<geometry::traits::SegmentTraits<S>> :
    utility::MetaType<decltype(SegmentTraits<S>::s(std::declval<S>()))> {};
template <class S>
struct geometry::traits::impl::PointTypeImpl<geometry::traits::SegmentTraits<S>> :
    ValueType<SegmentTraits<S>> {};

DEF_ACCESS(geometry::traits::SegmentTraits, 0, s);
DEF_ACCESS(geometry::traits::SegmentTraits, 1, d);

#endif // !INCLUDED_GEOMETRY_GEOMETRIES_SEGMENT_HEADER_
// EOF
