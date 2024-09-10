#pragma once
#include <vector>
#include "../Vector3Int.h"
#include "../Common.h"

namespace DDA::_3D
{
    template <typename T> struct Map
    {
        Map()
        {}

        Map(const std::vector<T>& _cells, Vector3 _origin, float _resolution, Vector3Int _dimensions)
            : cells(&_cells), origin(_origin), resolution(_resolution), dimensions(_dimensions)
        {}

        Vector3 origin;
        float resolution;
        Vector3Int dimensions;

        const T& at(size_t i, size_t j, size_t h) const
        {
            return (*cells)[h * dimensions.x * dimensions.y + j * dimensions.x + i];
        }

    private:
        const std::vector<T>* cells;
    };
} // namespace DDA::_3D