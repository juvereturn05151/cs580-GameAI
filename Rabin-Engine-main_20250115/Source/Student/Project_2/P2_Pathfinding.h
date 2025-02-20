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

class BucketPriorityQueue {
public:
    // numBuckets: number of buckets (should be small)
    // division: the cost range covered per bucket (choose so that all f_costs fall into one of these few buckets)
    BucketPriorityQueue(int numBuckets, float division);
    ~BucketPriorityQueue();

    // Set the base cost (optional, default is zero)
    inline void SetBaseCost(float baseCost) { m_baseCost = baseCost; }

    // Reset clears all buckets and resets bookkeeping.
    inline void Reset() {
        for (auto& bucket : m_buckets) {
            bucket.clear();
        }
        m_numNodesTracked = 0;
        m_lowestNonEmptyBin = m_numBuckets;
        m_baseCost = 0.0f;
    }

    // Returns true if there are no nodes in any bucket.
    inline bool Empty() const { return m_numNodesTracked == 0; }

    // Inserts a node into the appropriate bucket based on node->finalCost.
    void Push(Node* node);

    // Pops and returns a node from the lowest non-empty bucket.
    Node* Pop();

    // Removes a node from its old bucket (based on oldCost) and reinserts it.
    void DecreaseKey(Node* node, float oldCost);

private:
    int m_numBuckets;         // Total number of buckets.
    int m_lowestNonEmptyBin;  // Index of the lowest bucket that is not empty.
    int m_numNodesTracked;    // Total number of nodes in the queue.
    float m_division;         // Cost range covered per bucket.
    float m_baseCost;         // Base cost offset.

    // The buckets – each bucket is a vector of Node pointers.
    std::vector<std::vector<Node*>> m_buckets;

    // Computes the bucket index for a given cost.
    inline int GetBinIndex(float cost) const {
        int index = static_cast<int>((cost - m_baseCost) / m_division);
        if (index < 0) index = 0;
        if (index >= m_numBuckets) index = m_numBuckets - 1;
        return index;
    }
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
    BucketPriorityQueue m_openList;
    std::vector<Vec3> finalPath;

    GridPos start, goal;  // 16 bytes (8-byte aligned)
};