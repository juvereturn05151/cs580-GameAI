#pragma once
#include "Misc/PathfindingDetails.hpp"
#include "../Terrain/Terrain.h"

#define INF 99999

enum ListStatus {
    None, Open, Closed
};

struct Node {
    Node* parent;
    GridPos gridPos;   
    float finalCost;    
    float givenCost;    
    ListStatus onList;

    Node() : parent(nullptr), gridPos({ 0, 0 }), finalCost(0), givenCost(0), onList(ListStatus::None) {}
};

static const int8_t NEIGHBOR_OFFSETS[16] = {
    1,  0,   
   -1,  0,  
    0,  1,   
    0, -1,   
    1,  1,  
    1, -1,  
   -1,  1, 
   -1, -1  
};

static const int MAX_NEIGHBORS = 8;

struct Neighbors {
    GridPos positions[MAX_NEIGHBORS];
    //amount of valid neighbors
    int count = 0; 
};

class BucketOpenList {
public:
    BucketOpenList(int numBuckets, float division);
    ~BucketOpenList();

    inline void reset() {
        for (auto& bucket : buckets) {
            bucket.clear();
        }
        numNodesTracked = 0;
        lowestNonEmptyBin = numBuckets;
    }

    inline bool empty() const { return numNodesTracked == 0; }

    void push(Node* node);

    Node* pop();

    void update_fcost(Node* node, float oldCost);

private:
    int numBuckets;     

    int lowestNonEmptyBin;  

    int numNodesTracked;    

    float division;      

    std::vector<std::vector<Node*>> buckets;

    inline int get_bucket_index(float cost) const {
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
    std::vector<GridPos> reconstruct_floyd_warshall_path(const GridPos& start, const GridPos& goal);

private:
    //maximum number of neighbors for any cell
    static const int MAP_WIDTH = 40;
    static const int MAP_HEIGHT = 40;

    Node nodes[MAP_HEIGHT][MAP_WIDTH];  
    Neighbors validNeighbors[MAP_HEIGHT][MAP_WIDTH]; 
    BucketOpenList openList;
    std::vector<Vec3> finalPath;

    GridPos start, goal;

    // Floyd-Warshall Distance and Next matrices
    float fwDistances[MAP_HEIGHT][MAP_WIDTH][MAP_HEIGHT][MAP_WIDTH];
    GridPos fwNext[MAP_HEIGHT][MAP_WIDTH][MAP_HEIGHT][MAP_WIDTH];

};