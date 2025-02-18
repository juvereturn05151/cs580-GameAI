#pragma once
#include "Misc/PathfindingDetails.hpp"
#include "../Terrain/Terrain.h"

enum ListStatus {
    None, Open, Closed
};

struct Node {
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

static const int MAX_NEIGHBORS = 8;

struct Neighbors {
    GridPos positions[MAX_NEIGHBORS];
    int count = 0; // Number of valid neighbors
};

class AStarPather {
public:
    AStarPather();
    bool initialize();
    void shutdown();
    PathResult compute_path(PathRequest& request);

    void clear_nodes();
    float heuristic(const GridPos& a, const GridPos& b, PathRequest& request);
    const Neighbors& get_neighbors(const GridPos& pos);
    void reconstruct_path(Node* goalNode, std::vector<Vec3>& path);
    void apply_rubberbanding(std::vector<Vec3>& path);
    bool can_eliminate_middle_node(const Vec3& start, const Vec3& middle, const Vec3& end);
    Vec3 catmull_rom_interpolate(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3, float t);
    void apply_catmull_rom_smoothing(std::vector<Vec3>& path);
    void add_intermediate_points(std::vector<Vec3>& path, float maxDistance);

    // Open list operations
    void open_list_push(Node* node, PathRequest& request);
    Node* open_list_pop();
    void clear_open_list();
    void precompute_valid_neighbors();
    void compute_valid_neighbors(const GridPos& pos, Neighbors& neighbors);

private:
    static const int MAP_WIDTH = 40;
    static const int MAP_HEIGHT = 40;
    // Maximum number of neighbors for any cell

    Node nodes[MAP_HEIGHT][MAP_WIDTH];  // 51200 bytes
    Neighbors validNeighbors[MAP_HEIGHT][MAP_WIDTH]; // Fixed-size array for neighbors
    std::vector<Node*> openList;  // 24 bytes (8-byte aligned)
    std::vector<Vec3> finalPath;
    int lastIndex;                // 4 bytes (placing it here may reduce padding)

    GridPos start, goal;  // 16 bytes (8-byte aligned)
};