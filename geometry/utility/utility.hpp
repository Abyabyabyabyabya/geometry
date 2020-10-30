#ifndef INCLUDED_GEOMETRY_UTILITY_UTILITY_HEADER
#define INCLUDED_GEOMETRY_UTILITY_UTILITY_HEADER

namespace geometry {
namespace utility {

template <auto V>
struct MetaValue {
    static constexpr decltype(V) value = V;
};

template <class T>
struct MetaType {
    using type = T;
};

template <template <class> class T>
struct MetaTemplateType {
    template <class ...Ts>
    using type = T<Ts...>;
};

} // namespace utlity
} // namespace geometry

#endif // !INCLUDED_GEOMETRY_UTILITY_UTILITY_HEADER
// EOF
