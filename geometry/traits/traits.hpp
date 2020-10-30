#ifndef INCLUDED_GEOMETRY_TRAITS_HEADER_
#define INCLUDED_GEOMETRY_TRAITS_HEADER_

#include "../utility/utility.hpp"

namespace geometry {
namespace traits {

// 幾何カテゴリ
struct PointCategory;
struct SegmentCategory;


// 非公開メタ関数
  namespace impl {
    template <class T> struct TraitsImpl;
    template <class T> struct ValueTypeImpl;
    template <class T> struct PointTypeImpl;
    template <class T> struct DimensionImpl;
  } // namespace impl


// 公開メタ関数
template <class T> struct Category;  template <class T> using Category_t = typename Category<T>::type;

template <class T> struct Traits : utility::MetaType<typename impl::TraitsImpl<Category_t<T>>::template type<T>> {};
template <class T> using Traits_t = typename impl::TraitsImpl<Category_t<T>>::template type<T>;

template <class T> struct ValueType : impl::ValueTypeImpl<Traits_t<T>> {};
template <class T> using ValueType_t = typename impl::ValueTypeImpl<Traits_t<T>>::type;

template <class T> struct PointType : impl::PointTypeImpl<Traits_t<T>> {}; 
template <class T> using PonitType_t = typename impl::PointTypeImpl<Traits_t<T>>::type;

template <class T> struct Dimension : impl::DimensionImpl<Traits_t<T>> {};
template <class T> static constexpr size_t Dimension_v = impl::DimensionImpl<Traits_t<T>>::value;

} // namespace traits
} // namespace geometry

#endif // !INCLUDED_GEOMETRY_TRAITS_HEADER_
// EOF
