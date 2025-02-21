#include <pch.h>
#include "Projects/ProjectTwo.h"
#include "P2_Pathfinding.h"

#pragma region Extra Credit 
bool ProjectTwo::implemented_floyd_warshall()
{
    return false;
}

bool ProjectTwo::implemented_goal_bounding()
{
    return false;
}
#pragma endregion

BucketPriorityQueue::BucketPriorityQueue(int numBuckets, float division)
    : numBuckets(numBuckets),
    division(division),
    lowestNonEmptyBin(numBuckets),
    numNodesTracked(0)
{
    //resize the bucket vector so that we have numBuckets buckets.
    buckets.resize(numBuckets);
}

void BucketPriorityQueue::Push(Node* node) {
    int index = GetBinIndex(node->finalCost);
    buckets[index].push_back(node);
    node->bucketIndex = index; // Update the node's bucket index
    numNodesTracked++;
    if (index < lowestNonEmptyBin) {
        lowestNonEmptyBin = index;
    }
}

Node* BucketPriorityQueue::Pop() {
    if (Empty()) {
        return nullptr;
    }

    // Make sure lowestNonEmptyBin points to a non-empty bucket.
    while (lowestNonEmptyBin < numBuckets && buckets[lowestNonEmptyBin].empty()) {
        lowestNonEmptyBin++;
    }
    if (lowestNonEmptyBin >= numBuckets) {
        return nullptr;
    }

    // Remove a node from the back of the bucket.
    Node* node = buckets[lowestNonEmptyBin].back();
    buckets[lowestNonEmptyBin].pop_back();
    node->bucketIndex = -1; // Reset the node's bucket index
    numNodesTracked--;
    return node;
}


void BucketPriorityQueue::DecreaseKey(Node* node, float oldCost) {
    // Get the old bucket index from the node.
    int oldIndex = node->bucketIndex;

    // Remove the node from its old bucket.
    auto& bucket = buckets[oldIndex];
    auto it = std::find(bucket.begin(), bucket.end(), node);
    if (it != bucket.end()) {
        bucket.erase(it);
        numNodesTracked--;
    }

    // Reinsert the node with its updated cost.
    Push(node);
}

AStarPather::AStarPather() : openList(600, 0.25f), start({0,0}), goal({ 0,0 })
{
    for (int row = 0; row < MAP_HEIGHT; ++row)
    {
        for (int col = 0; col < MAP_WIDTH; ++col)
        {
            nodes[row * MAP_WIDTH + col].gridPos = { row, col };
        }
    }
}

bool AStarPather::initialize()
{
    clear_open_list();

    Callback cb = std::bind(&AStarPather::precompute_valid_neighbors, this);
    Messenger::listen_for_message(Messages::MAP_CHANGE, cb);
    
    return true;
}

PathResult AStarPather::compute_path(PathRequest& request)
{
    if (request.newRequest)
    {
        //request.path.clear();
        clear_nodes();
        clear_open_list();  // This now resets m_openList (bucket queue)

        start = terrain->get_grid_position(request.start);
        goal = terrain->get_grid_position(request.goal);

        if (request.settings.debugColoring) {
            terrain->set_color(start, Colors::Orange);
            terrain->set_color(goal, Colors::Orange);
        }

        Node* startNode = &nodes[start.row * MAP_WIDTH + start.col];
        startNode->givenCost = 0;
        startNode->finalCost = heuristic(start, goal, request);
        startNode->onList = ListStatus::Open;
        open_list_push(startNode, request);  
    }

    while (!openList.Empty())
    {
        Node* parentNode = open_list_pop();  
        
        if (goal.row == parentNode->gridPos.row && goal.col == parentNode->gridPos.col)
        {
            reconstruct_path(parentNode, finalPath);
            if (request.settings.rubberBanding)
            {
                apply_rubberbanding(finalPath);
            }
            if (request.settings.smoothing)
            {
                add_intermediate_points(finalPath);
                apply_catmull_rom_smoothing(finalPath);
            }

            while (!finalPath.empty())
            {
                request.path.push_back(finalPath.back());
                finalPath.pop_back();
            }
            return PathResult::COMPLETE;
        }

        parentNode->onList = ListStatus::Closed;
        if (request.settings.debugColoring) {
            terrain->set_color(parentNode->gridPos, Colors::Yellow);
        }

        uint8_t neighborBits = nodes[parentNode->gridPos.row * MAP_WIDTH + parentNode->gridPos.col].neighbors;

        // Iterate over valid neighbors using bitwise operations
        for (int i = 0; i < 8; ++i) {
            if (!(neighborBits & (1 << i))) continue;

            int8_t dRow = NEIGHBOR_OFFSETS[i * 2];
            int8_t dCol = NEIGHBOR_OFFSETS[i * 2 + 1];
            GridPos neighbor = { parentNode->gridPos.row + dRow, parentNode->gridPos.col + dCol };

            if (isWall[neighbor.row * MAP_WIDTH + neighbor.col]) continue;

            Node* childNode = &nodes[neighbor.row * MAP_WIDTH + neighbor.col];

            float cost = (dRow != 0 && dCol != 0) ? 1.414f : 1.0f; // Diagonal cost
            float new_g = parentNode->givenCost + cost;
            float new_f = new_g + heuristic(neighbor, goal, request);
            ListStatus listStatus = childNode->onList;

            if (listStatus == ListStatus::None) {
                childNode->parent = parentNode;
                childNode->givenCost = new_g;
                childNode->finalCost = new_f;
                childNode->onList = ListStatus::Open;
                open_list_push(childNode, request);
                continue;
            }

            if (new_g >= childNode->givenCost) continue;

            float old_f = childNode->finalCost;
            childNode->parent = parentNode;
            childNode->givenCost = new_g;
            childNode->finalCost = new_f;

            if (listStatus == ListStatus::Open) {
                openList.DecreaseKey(childNode, old_f);
                continue;
            }

            if (listStatus == ListStatus::Closed) {
                childNode->onList = ListStatus::Open;
                open_list_push(childNode, request);
            }
        }

        if (request.settings.singleStep)
            return PathResult::PROCESSING;
    }

    return PathResult::IMPOSSIBLE;
}


void AStarPather::clear_nodes()
{
    for (int row = 0; row < MAP_HEIGHT; ++row)
    {
        for (int col = 0; col < MAP_WIDTH; ++col)
        {
            nodes[row * MAP_WIDTH + col].parent = nullptr;
            nodes[row * MAP_WIDTH + col].finalCost = 0;
            nodes[row * MAP_WIDTH + col].givenCost = 0;
            nodes[row * MAP_WIDTH + col].onList = ListStatus::None;
        }
    }
}

float AStarPather::heuristic(const GridPos& a, const GridPos& b, PathRequest& request)
{
    int dx = std::abs(a.col - b.col);
    int dy = std::abs(a.row - b.row);

    float h = 0.0;

    switch (request.settings.heuristic)
    {
    case Heuristic::OCTILE:
        h = (std::min(dx, dy) * 1.414f) + std::max(dx, dy) - std::min(dx, dy);
        break;
    case Heuristic::CHEBYSHEV:
        h = std::max(dx, dy);
        break;
    case Heuristic::INCONSISTENT:
        h = ((a.row + a.col) % 2 > 0) ? std::sqrt(dx * dx + dy * dy) : 0.0f;
        break;
    case Heuristic::MANHATTAN:
        h = dx + dy;
        break;
    case Heuristic::EUCLIDEAN:
        h = std::sqrt(dx * dx + dy * dy);
        break;
    default:
        h = 0.0f;
        break;
    }

    return h * request.settings.weight;
}

void AStarPather::reconstruct_path(Node* goalNode, std::vector<Vec3>& path) {
    Node* current = goalNode;

    // Traverse from the goal node to the start node
    while (current) {
        GridPos currentGridPos = current->gridPos;
        path.push_back(worldPositions[currentGridPos.row * MAP_WIDTH + currentGridPos.col]);
        current = current->parent;
    }
}

void AStarPather::apply_rubberbanding(std::vector<Vec3>& path)
{
    for (size_t i = path.size() - 1; i >= 2; --i)
    {
        Vec3 endPos = path[i];
        Vec3 middlePos = path[i - 1];
        Vec3 startPos = path[i - 2];

        if (can_eliminate_middle_node(startPos, middlePos, endPos))
        {
            path.erase(path.begin() + i - 1); // Remove the middle node
        }
    }
}

bool AStarPather::can_eliminate_middle_node(const Vec3& start, const Vec3& middle, const Vec3& end)
{
    GridPos startGrid = terrain->get_grid_position(start);
    GridPos endGrid = terrain->get_grid_position(end);

    //determine the bounding box for the square area
    int minRow = std::min(startGrid.row, endGrid.row);
    int maxRow = std::max(startGrid.row, endGrid.row);
    int minCol = std::min(startGrid.col, endGrid.col);
    int maxCol = std::max(startGrid.col, endGrid.col);

    //iterate over the square area
    for (int row = minRow; row <= maxRow; ++row)
    {
        for (int col = minCol; col <= maxCol; ++col)
        {
            if (isWall[row * MAP_WIDTH + col])
            {
                //if any wall is found, the middle node cannot be eliminated
                return false;
            }
        }
    }

    //no walls found, the middle node can be eliminated
    return true; 
}

Vec3 AStarPather::catmull_rom_interpolate(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3, float t)
{
    Vec3 outputPoint = p0 * (-0.5f * t * t * t + t * t - 0.5f * t) +
                        p1 * (1.5f*t*t*t + -2.5 * t *t + 1.0f) +
                        p2 * (-1.5f* t * t* t + 2.0*t*t + 0.5*t) +
                        p3 * (0.5*t*t*t - 0.5*t*t);
    return outputPoint;
}

void AStarPather::apply_catmull_rom_smoothing(std::vector<Vec3>& path)
{
    //no need to process if the path is too short
    if (path.size() < 3) 
        return;

    std::vector<Vec3> smoothedPath;
    int n = path.size();

    //add the first point
    smoothedPath.push_back(path[0]);

    //iterate through the path and apply Catmull-Rom interpolation
    for (int i = 0; i < n - 1; ++i)
    {
        Vec3 p0 = (i == 0) ? path[0] : path[i - 1];
        Vec3 p1 = path[i];
        Vec3 p2 = path[i + 1];
        Vec3 p3 = (i == n - 2) ? path[n - 1] : path[i + 2];

        //add intermediate points
        smoothedPath.push_back(catmull_rom_interpolate(p0, p1, p2, p3, 0.25));
        smoothedPath.push_back(catmull_rom_interpolate(p0, p1, p2, p3, 0.5));
        smoothedPath.push_back(catmull_rom_interpolate(p0, p1, p2, p3, 0.75));
    }

    //add the last point
    smoothedPath.push_back(path[n - 1]);

    //replace the original path with the smoothed path
    path = smoothedPath;
}

void AStarPather::add_intermediate_points(std::vector<Vec3>& path)
{
    //no need to process if the path is too short
    if (path.size() < 2) 
        return;

    std::vector<Vec3> newPath;
    //add the first point
    newPath.push_back(path[0]);

    for (size_t i = 1; i < path.size(); ++i)
    {
        Vec3 prevPoint = newPath.back();
        Vec3 currentPoint = path[i];

        float distance = (currentPoint - prevPoint).Length();

        if (distance > 1.5f)
        {
            //calculate the direction vector
            Vec3 direction = (currentPoint - prevPoint) / distance;

            //add intermediate points spaced by maxDistance
            int numPoints = static_cast<int>(distance / 1.5f);
            for (int j = 1; j <= numPoints; ++j)
            {
                Vec3 intermediatePoint = prevPoint + direction * (1.5f * j);
                newPath.push_back(intermediatePoint);
            }
        }
        //add the current point
        newPath.push_back(currentPoint); 
    }
    //replace the original path with the new path
    path = newPath; 
}

void AStarPather::open_list_push(Node* node, PathRequest& request)
{
    if (request.settings.debugColoring) {
        terrain->set_color(node->gridPos, Colors::Blue);
    }
    openList.Push(node);
}

void AStarPather::precompute_valid_neighbors() {
    for (int row = 0; row < MAP_HEIGHT; ++row) {
        for (int col = 0; col < MAP_WIDTH; ++col) {
            GridPos pos = { row, col };
            nodes[row * MAP_WIDTH + col].neighbors = compute_valid_neighbors(pos);
        }
    }

    precompute_terrain_data();
}

uint8_t AStarPather::compute_valid_neighbors(const GridPos& pos) {
    uint8_t neighbors = 0; // Initialize all bits to 0 (invalid)

    for (int i = 0; i < 8; ++i) {
        int8_t dRow = NEIGHBOR_OFFSETS[i * 2];
        int8_t dCol = NEIGHBOR_OFFSETS[i * 2 + 1];

        GridPos newPos = { pos.row + dRow, pos.col + dCol };

        if (!terrain->is_valid_grid_position(newPos)) continue;
        if (terrain->is_wall(newPos)) continue;

        // Diagonal movement: check adjacent cells
        if (dRow != 0 && dCol != 0) {
            GridPos adjacent1 = { pos.row, pos.col + dCol };
            GridPos adjacent2 = { pos.row + dRow, pos.col };

            if (terrain->is_wall(adjacent1)) continue;
            if (terrain->is_wall(adjacent2)) continue;
        }

        // Set the corresponding bit to 1 (valid neighbor)
        neighbors |= (1 << i);
    }

    return neighbors;
}

void AStarPather::precompute_terrain_data()
{
    for (int row = 0; row < MAP_HEIGHT; ++row) {
        for (int col = 0; col < MAP_WIDTH; ++col) {
            worldPositions[row * MAP_WIDTH + col] = terrain->get_world_position({ row, col });
            isWall[row * MAP_WIDTH + col] = terrain->is_wall({ row, col });
        }
    }
}
