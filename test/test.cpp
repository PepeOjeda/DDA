#include "../include/DDA/DDA.h"

#include <iostream>

void test()
{

    //2D
    {
        std::cout<<"2D\n";

        DDA::_2D::Map<bool> map;
        map.cells = {
            {1, 1, 1, 1, 1},
            {1, 1, 0, 1, 1},
            {1, 1, 0, 1, 1},
            {1, 1, 0, 1, 1},
            {1, 1, 1, 1, 1}
        };
        map.origin = {0,0};
        map.resolution = 1;

        std::cout<<"Raycast:\n";
        {
            auto result = DDA::_2D::castRay<bool>({0.5, 0.5}, {0,1}, 10.0f, map, [](bool b){return b;});
            std::cout<<result.hitSomething<<" "<<result.distance<<"m\n";
        }

        std::cout<<"Raymarch:\n";
        {
            auto result = DDA::_2D::marchRay<bool>({2.7, 0.5}, {1,1}, 10, map,[](bool b){return b;});
            for(const auto& p : result.lengthInCell)
            {
                glm::ivec2 cell =p.first;
                std::cout<<"("<<cell.x<<","<<cell.y<<")"<<": "<<p.second<<"\n";
            }
        }
    }

    //3D
    {
        std::cout<<"3D\n";
        DDA::_3D::Map<bool> map;
        map.origin = {0,0,0};
        map.resolution = 1;
        map.cells = std::vector<std::vector<std::vector<bool>>> (5, std::vector<std::vector<bool>>(5, std::vector<bool>(5, true)) );
        std::cout<<"Raycast:\n";
        {
            auto result = DDA::_3D::castRay<bool>({0, 0, 0}, {1, 1, 1}, 10, map, [](bool b){return b;});
            std::cout<<result.hitSomething<<" "<<result.distance<<"m\n";
        }

        std::cout<<"Raymarch:\n";
        {
            auto result = DDA::_3D::marchRay<bool>({0, 0, 0}, {1,1,1}, 10, map,[](bool b){return b;});
            for(const auto& p : result.lengthInCell)
            {
                glm::ivec2 cell =p.first;
                std::cout<<"("<<cell.x<<","<<cell.y<<")"<<": "<<p.second<<"\n";
            }
            
        }
    }
}

int main()
{
    test();
    return 0; 
}