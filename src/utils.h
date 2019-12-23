/*
 * Copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk
 *
 * SENSEI is free software: you can redistribute it and/or modify it under the terms of
 * the GNU Affero General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * SENSEI is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License along with
 * SENSEI.  If not, see http://www.gnu.org/licenses/
 */

/**
 * @brief Utilities functions shared between modules
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
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