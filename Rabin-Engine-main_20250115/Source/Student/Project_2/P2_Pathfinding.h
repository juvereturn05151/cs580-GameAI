#pragma once
#include "Misc/PathfindingDetails.hpp"
#include "../Terrain/Terrain.h"
class AStarPather
{
public:
    /* 
        The class should be default constructible, so you might need to define a constructor.
        If needed, you can modify the framework where the class is constructed in the
        initialize functions of ProjectTwo and ProjectThree.
    */

    /* ************************************************** */
    // DO NOT MODIFY THESE SIGNATURES
    AStarPather();
    bool initialize();
    void shutdown();
    PathResult compute_path(PathRequest &request);
    void print_map(Terrain* terrain);
    /* ************************************************** */

    /*
        You should create whatever functions, variables, or classes you need.
        It doesn't all need to be in this header and cpp, structure it whatever way
        makes sense to you.
    */

};