#pragma once
#include <vector>
#include "../Vector2Int.h"
#include "../Common.h"

namespace DDA::_2D
{
    template <typename T> struct Map
    {
        Map()
        {}

        Map(const std::vector<T>& _cells, Vector2 _origin, float _resolution, Vector2Int _dimensions)
            : cells(&_cells), origin(_origin), resolution(_resolution), dimensions(_dimensions)
        {}

        Vector2 origin;
        float resolution;
        Vector2Int dimensions;

        const T& at(size_t i, size_t j) const
        {
            return (*cells)[j * dimensions.x + i];
        }

    private:
        const std::vector<T>* cells;
    };
} // namespace DDA::_2D