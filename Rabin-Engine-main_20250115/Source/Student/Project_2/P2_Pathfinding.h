#pragma once
#include "Misc/PathfindingDetails.hpp"
#include "../Terrain/Terrain.h"

static const float DIAGONAL_COST = 1.414f;

enum ListStatus {
    None, Open, Closed
};

struct Node {
    Node* parent;          // Pointer (8 bytes)
    GridPos gridPos;       // Assuming GridPos is 2 integers (8 bytes)
    float finalCost;       // 4 bytes
    float givenCost;       // 4 bytes
    size_t bucketPosition;
    uint16_t bucketIndex;  // 2 bytes (if buckets are fewer than 65536)
    ListStatus onList;        // 1 byte (if ListStatus has fewer than 256 values)
    uint8_t neighbors;     // 1 byte
    // Padding: 4 bytes (to align to 8 bytes)

    Node() : parent(nullptr), gridPos({ 0, 0 }), finalCost(0), givenCost(0), bucketPosition(0), bucketIndex(-1), onList(ListStatus::None), neighbors(0) {}
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

class BucketPriorityQueue {
public:
    //numBuckets: number of buckets (should be small)
    //division: the cost range covered per bucket (choose so that all f_costs fall into one of these few buckets)
    BucketPriorityQueue(float division,int numBuckets);
    ~BucketPriorityQueue() = default;

    //reset clears all buckets and resets bookkeeping.
    inline void Reset() {
        for (auto& bucket : buckets) {
            bucket.clear();
        }
        numNodesTracked = 0;
        lowestNonEmptyBin = numBuckets;
    }

    //returns true if there are no nodes in any bucket.
    inline bool Empty() const { return numNodesTracked == 0; }

    //inserts a node into the appropriate bucket based on node->finalCost.
    void Push(Node* node);

    //pops and returns a node from the lowest non-empty bucket.
    Node* Pop();

    //removes a node from its old bucket (based on oldCost) and reinserts it.
    void DecreaseKey(Node* node, float oldCost);

private:
    //cost range covered per bucket.
    float division;
    //total number of buckets.
    int numBuckets;     
    //index of the lowest bucket that is not empty.
    int lowestNonEmptyBin;  
    //total number of nodes in the queue.
    int numNodesTracked;    

    //buckets – each bucket is an unsorted vector of Node pointers.
    std::vector<std::vector<Node*>> buckets;

    inline int GetBinIndex(float cost) const {
        int index = static_cast<int>(cost / division);
        if (index < 0) index = 0;
        if (index >= numBuckets) index = numBuckets - 1;
        return index;
    }
};

class AStarPather {
public:
    AStarPather();
    bool initialize();
    inline void shutdown() 
    {
        clear_open_list();
    }
    PathResult compute_path(PathRequest& request);

    void clear_nodes();
    float heuristic(const GridPos& a, const GridPos& b, PathRequest& request);
    void reconstruct_path(Node* goalNode, std::vector<Vec3>& path);
    void apply_rubberbanding(std::vector<Vec3>& path);
    bool can_eliminate_middle_node(const Vec3& start, const Vec3& middle, const Vec3& end);
    Vec3 catmull_rom_interpolate(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3, float t);
    void apply_catmull_rom_smoothing(std::vector<Vec3>& path);
    void add_intermediate_points(std::vector<Vec3>& path);

    //open list operations
    void open_list_push(Node* node, PathRequest& request);
    inline Node* open_list_pop() 
    {
        return openList.Pop();
    }
    inline void clear_open_list()
    {
        openList.Reset();
    }
    void precompute_data();
    uint8_t compute_valid_neighbors(const GridPos& pos);

private:
    //maximum number of neighbors for any cell
    static const int MAP_WIDTH = 40;
    static const int MAP_HEIGHT = 40;


    Node nodes[MAP_HEIGHT * MAP_WIDTH];  
    bool isWall[MAP_HEIGHT * MAP_WIDTH];
    BucketPriorityQueue openList;
    std::vector<Vec3> finalPath;

    GridPos start, goal;
    bool isFirstRequest;
};