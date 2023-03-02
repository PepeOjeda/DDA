#include "DDA.h"

#include <iostream>


void test()
{
    std::vector<std::vector<bool>> map = 
    {
        {1, 1, 1, 1, 1},
        {1, 1, 0, 1, 1},
        {1, 1, 0, 1, 1},
        {1, 1, 0, 1, 1},
        {1, 1, 1, 1, 1}
    };

    std::cout<<"Raycast:\n";
    {
        auto result = DDA::castRay({0.5, 0.5}, {0,1}, 10, map, {0,0}, 1);
        std::cout<<result.hitSomething<<" "<<result.distance<<"m\n";
    }

    std::cout<<"Raymarch:\n";
    {
        auto result = DDA::marchRay({2.7, 0.5}, {1,1}, 10, map, {0,0}, 1);
        for(const auto& p : result.lengthInCell)
        {
            glm::ivec2 cell =p.first;
            std::cout<<"("<<cell.x<<","<<cell.y<<")"<<": "<<p.second<<"\n";
        }
    }
}

int main()
{
    test();
    return 0; 
}