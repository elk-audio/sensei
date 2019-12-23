/**
 * @brief Utilities functions shared between modules
 * @copyright MIND Music Labs AB, Stockholm
 */
#ifndef SENSEI_UTILS_H
#define SENSEI_UTILS_H

#include <algorithm>
#include <memory>

template<typename Derived, typename Base>
std::unique_ptr<Derived>
static_unique_ptr_cast( std::unique_ptr<Base>&& p )
{
    auto d = static_cast<Derived *>(p.release());
    return std::unique_ptr<Derived>(d);
}

template <typename T>
T clip(const T& x, const T& lower, const T& upper)
{
    return std::max(lower, std::min(x, upper));
}


#endif //SENSEI_UTILS_H
