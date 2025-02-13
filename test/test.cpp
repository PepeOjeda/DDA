#include "../include/DDA/DDA.h"

#include <iostream>

void test()
{
    std::cout << "Expected result:\n"
                 "  2D\n"
                 "      Raycast:\n"
                 "          0 10m\n"
                 "      Raymarch:\n"
                 "          (0,0): 0.707107\n"
                 "          (1,1): 1.41421\n"
                 "  3D\n"
                 "      Raycast:\n"
                 "          1 5.19615m\n"
                 "      Raymarch:\n"
                 "          (1,1): 1.73205\n"
                 "          (2,2): 1.73205\n"
                 "          (3,3): 1.73205";

    std::cout << "\n\nActual result:\n";

    // 2D
    {
        std::cout << "  2D\n";

        std::vector<uint> cells{
            1, 1, 1, 1, 1, //
            1, 1, 0, 1, 1, //
            1, 1, 0, 1, 1, //
            1, 1, 0, 1, 1, //
            1, 1, 1, 1, 1  //
        };
        DDA::_2D::Map<uint> map(cells, {0, 0}, 1, {5, 5});

        std::cout << "      Raycast:\n";
        {
            auto result = DDA::_2D::castRay<uint>({0.5, 0.5}, {0, 1}, 10.0f, map, [](uint b) { return b != 0; });
            std::cout << "          " << result.hitSomething << " " << result.distance << "m\n";
        }

        std::cout << "      Raymarch:\n";
        {
            auto result = DDA::_2D::marchRay<uint>({0.5, 0.5}, {1, 1}, 10, map, [](uint b) { return b != 0; });
            for (const auto& p : result.lengthInCell)
            {
                glm::ivec2 cell = p.first;
                std::cout << "          (" << cell.x << "," << cell.y << ")" << ": " << p.second << "\n";
            }
        }
    }

    // 3D
    {
        std::cout << "  3D\n";
        std::vector<uint> cells(5 * 5 * 5, true);
        // make the external shell occupied
        for (int x = 0; x < 5; x++)
            for (int y = 0; y < 5; y++)
                for (int z = 0; z < 5; z++)
                    if (x == 0 || y == 0 || z == 0 || x == 4 || y == 4 || z == 4)
                        cells[z + 5 * y + 5 * 5 * x] = false;

        DDA::_3D::Map<uint> map(cells, {0, 0, 0}, 1, {5, 5, 5});
        std::cout << "      Raycast:\n";
        {
            auto result = DDA::_3D::castRay<uint>({1, 1, 1}, {1, 1, 1}, 10, map, [](uint b) { return b != 0; });
            std::cout << "          " << result.hitSomething << " " << result.distance << "m\n";
        }

        std::cout << "      Raymarch:\n";
        {
            auto result = DDA::_3D::marchRay<uint>({1, 1, 1}, {1, 1, 1}, 10, map, [](uint b) { return b != 0; });
            for (const auto& p : result.lengthInCell)
            {
                glm::ivec3 cell = p.first;
                std::cout << "          (" << cell.x << "," << cell.y << ")" << ": " << p.second << "\n";
            }
        }
    }
}

int main()
{
    test();
    return 0;
}