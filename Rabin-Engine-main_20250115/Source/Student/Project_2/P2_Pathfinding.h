#pragma once
#include "Misc/PathfindingDetails.hpp"
#include "../Terrain/Terrain.h"
#include <queue>
#include <unordered_map>

enum ListStatus 
{ 
    None, Open, Closed 
};

struct Node
{
    GridPos pos;
    float g; // Cost from start
    float h; // Heuristic
    float f() const { return g + h; } // Total estimated cost

    bool operator>(const Node& other) const
    {
        return f() > other.f();
    }
};

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
    void set_terrain(Terrain* terrain);
    void print_map(Terrain* terrain);
    float heuristic(const GridPos& a, const GridPos& b);
    std::vector<GridPos> get_neighbors(const GridPos& pos, std::shared_ptr<Terrain> terrain);
    /* ************************************************** */

    /*
        You should create whatever functions, variables, or classes you need.
        It doesn't all need to be in this header and cpp, structure it whatever way
        makes sense to you.
    */

private:
    Terrain* terrain;
    GridPos start;
    GridPos goal;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;
    std::unordered_map<GridPos, float, GridPosHash> gCost;
    std::unordered_map<GridPos, GridPos, GridPosHash> cameFrom;
};