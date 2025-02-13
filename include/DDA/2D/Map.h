#pragma once
#include <vector>
#include "../Common.h"
#include "../../../third_party/glm/glm/glm.hpp"
#include "../../../third_party/glm/glm/common.hpp"

namespace DDA::_2D
{
    template <typename T> struct Map
    {
        Map()
        {}

        Map(const std::vector<T>& _cells, glm::vec2 _origin, float _resolution, glm::ivec2 _dimensions)
            : cells(&_cells), origin(_origin), resolution(_resolution), dimensions(_dimensions)
        {}

        glm::vec2 origin;
        float resolution;
        glm::ivec2 dimensions;

        const T& at(size_t i, size_t j) const
        {
            return cells->at(j * dimensions.x + i);
        }

    private:
        const std::vector<T>* cells;
    };
} // namespace DDA::_2D