#pragma once
#include "Misc/PathfindingDetails.hpp"
#include "../Terrain/Terrain.h"
#include <queue>
#include <unordered_map>
#include <unordered_set>

enum ListStatus 
{ 
    None, Open, Closed 
};

struct Node
{
    Node* parent;       // Pointer to the parent node
    GridPos gridPos;    // Node's location (assuming GridPos is a struct with x, y coordinates)
    float finalCost;    // f(x) = g(x) + h(x), total estimated cost
    float givenCost;    // g(x), cost from the start node to this node
    enum class ListStatus { None, Open, Closed } onList; // Track if the node is in open/closed list

    Node(GridPos pos, float g = 0.0f, float f = 0.0f, Node* p = nullptr)
        : parent(p), gridPos(pos), givenCost(g), finalCost(f), onList(ListStatus::None) {}

    bool operator>(const Node& other) const
    {
        return finalCost > other.finalCost;
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
    void print_map();
    float heuristic(const GridPos& a, const GridPos& b, Heuristic heuristic);
    std::vector<GridPos> get_neighbors(const GridPos& pos);
    std::vector<Vec3> reconstruct_path(std::unordered_map<GridPos, GridPos, GridPosHash>& cameFrom, GridPos start, GridPos goal);
    void apply_rubberbanding(std::vector<Vec3>& path);
    bool can_eliminate_middle_node(const Vec3& start, const Vec3& middle, const Vec3& end);
    Vec3 catmull_rom_interpolate(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3, float t);
    void apply_catmull_rom_smoothing(std::vector<Vec3>& path);

    /* ************************************************** */

private:
    GridPos start;
    GridPos goal;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openList;
    std::unordered_set<GridPos, GridPosHash> closedList;
    std::unordered_map<GridPos, float, GridPosHash> gCost;
    std::unordered_map<GridPos, GridPos, GridPosHash> cameFrom;
};