#pragma once
#include <vector>
#include "../Common.h"
#include "../../../third_party/glm/glm/glm.hpp"
#include "../../../third_party/glm/glm/common.hpp"


namespace DDA::_3D
{
    template <typename T> struct Map
    {
        Map()
        {}

        Map(const std::vector<T>& _cells, glm::vec3 _origin, float _resolution, glm::ivec3 _dimensions)
            : cells(&_cells), origin(_origin), resolution(_resolution), dimensions(_dimensions)
        {}

        glm::vec3 origin;
        float resolution;
        glm::ivec3 dimensions;

        const T& at(size_t i, size_t j, size_t h) const
        {
            return cells->at(h * dimensions.x * dimensions.y + j * dimensions.x + i);
        }

    private:
        const std::vector<T>* cells;
    };
} // namespace DDA::_3D