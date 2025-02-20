#pragma once
#include "Misc/PathfindingDetails.hpp"
#include "../Terrain/Terrain.h"

#define INF 99999

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
    //amount of valid neighbors
    int count = 0; 
};

class BucketPriorityQueue {
public:
    //numBuckets: number of buckets (should be small)
    //division: the cost range covered per bucket (choose so that all f_costs fall into one of these few buckets)
    BucketPriorityQueue(int numBuckets, float division);
    ~BucketPriorityQueue();

    //set the base cost (optional, default is zero)
    inline void SetBaseCost(float baseCost) { baseCost = baseCost; }

    //reset clears all buckets and resets bookkeeping.
    inline void Reset() {
        for (auto& bucket : buckets) {
            bucket.clear();
        }
        m_numNodesTracked = 0;
        lowestNonEmptyBin = numBuckets;
        baseCost = 0.0f;
    }

    //returns true if there are no nodes in any bucket.
    inline bool Empty() const { return m_numNodesTracked == 0; }

    //inserts a node into the appropriate bucket based on node->finalCost.
    void Push(Node* node);

    //pops and returns a node from the lowest non-empty bucket.
    Node* Pop();

    //removes a node from its old bucket (based on oldCost) and reinserts it.
    void DecreaseKey(Node* node, float oldCost);

private:
    //total number of buckets.
    int numBuckets;     
    //index of the lowest bucket that is not empty.
    int lowestNonEmptyBin;  
    //total number of nodes in the queue.
    int m_numNodesTracked;    
    //cost range covered per bucket.
    float division;   
    //base cost offset.
    float baseCost;         

    //buckets – each bucket is an unsorted vector of Node pointers.
    std::vector<std::vector<Node*>> buckets;

    inline int GetBinIndex(float cost) const {
        int index = static_cast<int>((cost - baseCost) / division);
        if (index < 0) index = 0;
        if (index >= numBuckets) index = numBuckets - 1;
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
    void add_intermediate_points(std::vector<Vec3>& path);

    //open list operations
    void open_list_push(Node* node, PathRequest& request);
    Node* open_list_pop();
    void clear_open_list();
    void precompute_valid_neighbors();
    void compute_valid_neighbors(const GridPos& pos, Neighbors& neighbors);
    
    //floyd_warshall
    void init_floyd_warshall();
    void reconstruct_path_floyd_warshall(int startIdx, int goalIdx, std::vector<GridPos>& path);

private:
    //maximum number of neighbors for any cell
    static const int MAP_WIDTH = 40;
    static const int MAP_HEIGHT = 40;

    Node nodes[MAP_HEIGHT][MAP_WIDTH];  
    Neighbors validNeighbors[MAP_HEIGHT][MAP_WIDTH]; 
    BucketPriorityQueue openList;
    std::vector<Vec3> finalPath;

    GridPos start, goal;

    // Floyd-Warshall Distance and Next matrices
    float dist[MAP_HEIGHT][MAP_WIDTH][MAP_HEIGHT][MAP_WIDTH];
    Node* next[MAP_HEIGHT][MAP_WIDTH][MAP_HEIGHT][MAP_WIDTH];

};