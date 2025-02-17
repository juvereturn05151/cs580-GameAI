#pragma once
#include "Misc/PathfindingDetails.hpp"
#include "../Terrain/Terrain.h"

enum ListStatus 
{ 
    None, Open, Closed 
};

struct Node
{
    Node* parent;
    GridPos gridPos;    // Node's location (assuming GridPos is a struct with x, y coordinates)
    float finalCost;    // f(x) = g(x) + h(x), total estimated cost
    float givenCost;    // g(x), cost from the start node to this node
    ListStatus onList;

    Node() : parent(nullptr), gridPos({ 0, 0 }), finalCost(0), givenCost(0), onList(ListStatus::None) {}
};

static const int8_t NEIGHBOR_OFFSETS[16] = {
    1,  0,   // down      (row +1, col +0)
   -1,  0,   // up        (row -1, col +0)
    0,  1,   // right     (row +0, col +1)
    0, -1,   // left      (row +0, col -1)
    1,  1,   // down-right
    1, -1,   // down-left
   -1,  1,   // up-right
   -1, -1    // up-left
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


    void clear_nodes();
    float heuristic(const GridPos& a, const GridPos& b, PathRequest& request);
    std::vector<GridPos> get_neighbors(const GridPos& pos);
    std::vector<Vec3> reconstruct_path(Node* goalNode);
    void apply_rubberbanding(std::vector<Vec3>& path);
    bool can_eliminate_middle_node(const Vec3& start, const Vec3& middle, const Vec3& end);
    Vec3 catmull_rom_interpolate(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3, float t);
    void apply_catmull_rom_smoothing(std::vector<Vec3>& path);
    void add_intermediate_points(std::vector<Vec3>& path, float maxDistance);

    // Open list operations
    void open_list_push(Node* node, PathRequest& request);
    Node* open_list_pop();
    void open_list_update(Node* node);
    void clear_open_list();
    /* ************************************************** */

private:
    static const int MAP_WIDTH = 40;
    static const int MAP_HEIGHT = 40;
    Node nodes[MAP_HEIGHT][MAP_WIDTH];

    GridPos start;
    GridPos goal;

    std::vector<Node*> openList;
    int lastIndex;
};