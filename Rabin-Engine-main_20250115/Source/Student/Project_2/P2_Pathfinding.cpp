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
    return true;
}
#pragma endregion

BucketPriorityQueue::BucketPriorityQueue(int numBuckets, float division)
    : numBuckets(numBuckets),
    division(division),
    baseCost(0.0f),
    lowestNonEmptyBin(numBuckets),
    numNodesTracked(0)
{
    //resize the bucket vector so that we have numBuckets buckets.
    buckets.resize(numBuckets);
}

BucketPriorityQueue::~BucketPriorityQueue() {
    //nothing to do because std::vector cleans up automatically.
}

void BucketPriorityQueue::Push(Node* node) {
    int index = GetBinIndex(node->finalCost);
    buckets[index].push_back(node);
    numNodesTracked++;
    if (index < lowestNonEmptyBin) {
        lowestNonEmptyBin = index;
    }
}

Node* BucketPriorityQueue::Pop() {
    if (Empty()) {
        return nullptr;
    }

    //make sure m_lowestNonEmptyBin points to a non-empty bucket.
    while (lowestNonEmptyBin < numBuckets && buckets[lowestNonEmptyBin].empty()) {
        lowestNonEmptyBin++;
    }
    if (lowestNonEmptyBin >= numBuckets) {
        return nullptr;
    }

    //remove a node from the back of the bucket.
    Node* node = buckets[lowestNonEmptyBin].back();
    buckets[lowestNonEmptyBin].pop_back();
    numNodesTracked--;
    return node;
}


void BucketPriorityQueue::DecreaseKey(Node* node, float oldCost) {
    //find the bucket corresponding to the old cost.
    int oldIndex = GetBinIndex(oldCost);
    auto& bucket = buckets[oldIndex];
    for (auto it = bucket.begin(); it != bucket.end(); ++it) {
        if (*it == node) {
            bucket.erase(it);
            numNodesTracked--;
            break;
        }
    }
    //reinsert the node with its updated cost.
    Push(node);
}

AStarPather::AStarPather() : openList(600, 0.25f)
{
    for (int row = 0; row < MAP_HEIGHT; ++row)
    {
        for (int col = 0; col < MAP_WIDTH; ++col)
        {
            nodes[row][col].gridPos = { row, col };
        }
    }
}

bool AStarPather::initialize()
{
    clear_open_list();

    Callback cb = std::bind(&AStarPather::precompute_valid_neighbors, this);
    Messenger::listen_for_message(Messages::MAP_CHANGE, cb);

    //cb = std::bind(&AStarPather::init_floyd_warshall, this);
    //Messenger::listen_for_message(Messages::MAP_CHANGE, cb);

    cb = std::bind(&AStarPather::compute_goal_bounding_boxes, this);
    Messenger::listen_for_message(Messages::MAP_CHANGE, cb);

    return true;
}

void AStarPather::shutdown()
{
    /*
        Free any dynamically allocated memory or any other general house-
        keeping you need to do during shutdown.
    */

    clear_open_list();
}

PathResult AStarPather::compute_path(PathRequest& request)
{
    if (!terrain)
    {
        return PathResult::IMPOSSIBLE;
    }

    if (request.newRequest)
    {
        request.path.clear();
        clear_nodes();
        clear_open_list();

        start = terrain->get_grid_position(request.start);
        goal = terrain->get_grid_position(request.goal);

        if (request.settings.method == Method::GOAL_BOUNDING) 
        {
            // Check if the goal is within the bounding box of the start node
            if (goal.row < goalBounds[start.row][start.col].minRow ||
                goal.row > goalBounds[start.row][start.col].maxRow ||
                goal.col < goalBounds[start.row][start.col].minCol ||
                goal.col > goalBounds[start.row][start.col].maxCol) 
            {
                return PathResult::IMPOSSIBLE; // Goal is unreachable
            }

            // Use A* with Goal Bounding pruning
            Node* startNode = &nodes[start.row][start.col];
            startNode->givenCost = 0;
            startNode->finalCost = heuristic(start, goal, request);
            startNode->onList = ListStatus::Open;
            open_list_push(startNode, request);
        }
        else if (request.settings.method == Method::FLOYD_WARSHALL) 
        {
            std::vector<GridPos> path = reconstruct_floyd_warshall_path(start, goal);

            if (path.empty()) {
                return PathResult::IMPOSSIBLE;
            }

            for (const auto& pos : path) {
                request.path.push_back(terrain->get_world_position(pos));
            }

            return PathResult::COMPLETE;
        }
        else if (request.settings.method == Method::ASTAR) {
            Node* startNode = &nodes[start.row][start.col];
            startNode->givenCost = 0;
            startNode->finalCost = heuristic(start, goal, request);
            startNode->onList = ListStatus::Open;
            open_list_push(startNode, request);
        }

        if (request.settings.debugColoring) {
            terrain->set_color(start, Colors::Orange);
            terrain->set_color(goal, Colors::Orange);
        }


    }

    while (!openList.Empty())
    {
        Node* parentNode = open_list_pop();  

        if (parentNode->gridPos == goal)
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

            for (const auto& pos : finalPath)
            {
                request.path.push_back(pos);
            }
            return PathResult::COMPLETE;
        }

        parentNode->onList = ListStatus::Closed;
        if (request.settings.debugColoring) {
            terrain->set_color(parentNode->gridPos, Colors::Yellow);
        }

        Neighbors neighbors = get_neighbors(parentNode->gridPos);
        for (int i = 0; i < neighbors.count; ++i)
        {
            const GridPos& neighbor = neighbors.positions[i];
            if (terrain->is_wall(neighbor))
            {
                continue; 
            }

            // Goal Bounding pruning
            if (request.settings.method == Method::GOAL_BOUNDING) {
                if (neighbor.row < goalBounds[parentNode->gridPos.row][parentNode->gridPos.col].minRow ||
                    neighbor.row > goalBounds[parentNode->gridPos.row][parentNode->gridPos.col].maxRow ||
                    neighbor.col < goalBounds[parentNode->gridPos.row][parentNode->gridPos.col].minCol ||
                    neighbor.col > goalBounds[parentNode->gridPos.row][parentNode->gridPos.col].maxCol) {
                    continue; // Skip nodes outside the bounding box
                }
            }

            Node* childNode = &nodes[neighbor.row][neighbor.col];

            float cost = (neighbor.row != parentNode->gridPos.row && neighbor.col != parentNode->gridPos.col) ? 1.414f : 1.0f;
            float new_g = parentNode->givenCost + cost;
            float new_f = new_g + heuristic(neighbor, goal, request);

            if (childNode->onList == ListStatus::None)
            {
                //node not yet encountered; add it to the open list.
                childNode->parent = parentNode;
                childNode->givenCost = new_g;
                childNode->finalCost = new_f;
                childNode->onList = ListStatus::Open;
                open_list_push(childNode, request);
            }
            else if (childNode->onList == ListStatus::Open || childNode->onList == ListStatus::Closed)
            {
                if (new_g < childNode->givenCost)
                {
                    //store previous final cost.
                    float old_f = childNode->finalCost; 
                    childNode->parent = parentNode;
                    childNode->givenCost = new_g;
                    childNode->finalCost = new_f;

                    if (childNode->onList == ListStatus::Open)
                    {
                        //the node is already in the open list; update its bucket location.
                        openList.DecreaseKey(childNode, old_f);
                    }
                    else if (childNode->onList == ListStatus::Closed)
                    {
                        //if the node was closed, reopen it.
                        childNode->onList = ListStatus::Open;
                        open_list_push(childNode, request);
                    }
                }
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
            nodes[row][col].parent = nullptr;
            nodes[row][col].finalCost = 0;
            nodes[row][col].givenCost = 0;
            nodes[row][col].onList = ListStatus::None;
        }
    }
}

float AStarPather::heuristic(const GridPos& a, const GridPos& b, PathRequest& request)
{
    int dx = std::abs(a.col - b.col);
    int dy = std::abs(a.row - b.row);

    float h = 0.0;

    if (request.settings.heuristic == Heuristic::OCTILE)
    {
        h = (std::min(dx, dy) * 1.414) + std::max(dx, dy) -  std::min(dx, dy);
    }
    else if (request.settings.heuristic == Heuristic::CHEBYSHEV)
    {
        h =  std::max(dx, dy);
    }
    else if (request.settings.heuristic == Heuristic::INCONSISTENT)
    {
        if ((a.row + a.col) % 2 > 0)
        {
            h = std::sqrt(dx * dx + dy * dy); 
        }
        else
        {
            h = 0.0f; 
        }
    }
    else if (request.settings.heuristic == Heuristic::MANHATTAN)
    {
        h = dx + dy;
    }
    else if (request.settings.heuristic == Heuristic::EUCLIDEAN)
    {
        h = std::sqrt(dx * dx + dy * dy);
    }
    else
    {
        //NUM_ENTRIES
        h =  0.0f;
    }

    return h * request.settings.weight;
}

const Neighbors& AStarPather::get_neighbors(const GridPos& pos) {
    return validNeighbors[pos.row][pos.col];
}

void AStarPather::reconstruct_path(Node* goalNode, std::vector<Vec3>& path) {
    path.clear(); // Clear the path vector to ensure it's empty
    Node* current = goalNode;

    // Traverse from the goal node to the start node
    while (current) {
        path.push_back(terrain->get_world_position(current->gridPos));
        current = current->parent;
    }

    // Reverse the path to get the correct order (start -> goal)
    std::reverse(path.begin(), path.end());
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
            if (terrain->is_wall(row, col))
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

Node* AStarPather::open_list_pop()
{
    return openList.Pop();
}

void AStarPather::clear_open_list()
{
    openList.Reset();
}

void AStarPather::precompute_valid_neighbors() {
    for (int row = 0; row < MAP_HEIGHT; ++row) {
        for (int col = 0; col < MAP_WIDTH; ++col) {
            GridPos pos = { row, col };
            compute_valid_neighbors(pos, validNeighbors[row][col]);
        }
    }
}

void AStarPather::compute_valid_neighbors(const GridPos& pos, Neighbors& neighbors) {
    neighbors.count = 0; 

    for (int i = 0; i < 8; ++i) {
        int8_t dRow = NEIGHBOR_OFFSETS[i * 2];
        int8_t dCol = NEIGHBOR_OFFSETS[i * 2 + 1];

        GridPos newPos = { pos.row + dRow, pos.col + dCol };

        if (!terrain->is_valid_grid_position(newPos)) continue;
        if (terrain->is_wall(newPos)) continue;

        bool isDiagonal = (dRow != 0) && (dCol != 0);
        if (isDiagonal) {
            GridPos adjacent1 = { pos.row, pos.col + dCol };
            GridPos adjacent2 = { pos.row + dRow, pos.col };

            if (terrain->is_wall(adjacent1)) continue;
            if (terrain->is_wall(adjacent2)) continue;
        }

        neighbors.positions[neighbors.count++] = newPos;
    }
}

void AStarPather::init_floyd_warshall() {
    // Initialize distances and next nodes
    for (int i = 0; i < MAP_HEIGHT; ++i) {
        for (int j = 0; j < MAP_WIDTH; ++j) {
            for (int k = 0; k < MAP_HEIGHT; ++k) {
                for (int l = 0; l < MAP_WIDTH; ++l) {
                    if (i == k && j == l) {
                        fwDistances[i][j][k][l] = 0; // Distance to self is 0
                    }
                    else {
                        fwDistances[i][j][k][l] = std::numeric_limits<float>::infinity(); // Initialize to infinity
                    }
                    fwNext[i][j][k][l] = { -1, -1 }; // No next node initially
                }
            }
        }
    }

    // Set distances for direct neighbors
    for (int i = 0; i < MAP_HEIGHT; ++i) {
        for (int j = 0; j < MAP_WIDTH; ++j) {
            if (terrain->is_wall(i, j)) continue; // Skip walls

            Neighbors neighbors = get_neighbors({ i, j });
            for (int n = 0; n < neighbors.count; ++n) {
                GridPos neighbor = neighbors.positions[n];
                float cost = (neighbor.row != i && neighbor.col != j) ? 1.414f : 1.0f; // Diagonal cost is sqrt(2)
                fwDistances[i][j][neighbor.row][neighbor.col] = cost;
                fwNext[i][j][neighbor.row][neighbor.col] = neighbor;
            }
        }
    }

    // Floyd-Warshall algorithm
    for (int kRow = 0; kRow < MAP_HEIGHT; ++kRow) {
        for (int kCol = 0; kCol < MAP_WIDTH; ++kCol) {
            if (terrain->is_wall(kRow, kCol)) continue; // Skip walls

            for (int iRow = 0; iRow < MAP_HEIGHT; ++iRow) {
                for (int iCol = 0; iCol < MAP_WIDTH; ++iCol) {
                    if (terrain->is_wall(iRow, iCol)) continue; // Skip walls

                    for (int jRow = 0; jRow < MAP_HEIGHT; ++jRow) {
                        for (int jCol = 0; jCol < MAP_WIDTH; ++jCol) {
                            if (terrain->is_wall(jRow, jCol)) continue; // Skip walls

                            float throughK = fwDistances[iRow][iCol][kRow][kCol] + fwDistances[kRow][kCol][jRow][jCol];
                            if (throughK < fwDistances[iRow][iCol][jRow][jCol]) {
                                fwDistances[iRow][iCol][jRow][jCol] = throughK;
                                fwNext[iRow][iCol][jRow][jCol] = fwNext[iRow][iCol][kRow][kCol];
                            }
                        }
                    }
                }
            }
        }
    }
}

std::vector<GridPos> AStarPather::reconstruct_floyd_warshall_path(const GridPos& start, const GridPos& goal) {
    std::vector<GridPos> path;

    if (fwNext[start.row][start.col][goal.row][goal.col].row == -1) {
        return path; // No path exists
    }

    GridPos current = start;
    while (current != goal) {
        path.push_back(current);
        current = fwNext[current.row][current.col][goal.row][goal.col];
    }
    path.push_back(goal);

    return path;
}

void AStarPather::compute_goal_bounding_boxes() {
    //init bounding boxes
    for (int i = 0; i < MAP_HEIGHT; ++i) 
    {
        for (int j = 0; j < MAP_WIDTH; ++j) 
        {
            //init to invalid values
            goalBounds[i][j] = { MAP_HEIGHT, 0, MAP_WIDTH, 0 }; 
        }
    }

    //use BFS to compute bounding boxes
    for (int i = 0; i < MAP_HEIGHT; ++i) 
    {
        for (int j = 0; j < MAP_WIDTH; ++j) 
        {
            if (terrain->is_wall(i, j)) continue; 

            std::queue<GridPos> queue;
            queue.push({ i, j });

            while (!queue.empty()) {
                GridPos current = queue.front();
                queue.pop();

                //update bounding box for the current node
                goalBounds[i][j].minRow = std::min(goalBounds[i][j].minRow, current.row);
                goalBounds[i][j].maxRow = std::max(goalBounds[i][j].maxRow, current.row);
                goalBounds[i][j].minCol = std::min(goalBounds[i][j].minCol, current.col);
                goalBounds[i][j].maxCol = std::max(goalBounds[i][j].maxCol, current.col);

                //explore neighbors
                Neighbors neighbors = get_neighbors(current);
                for (int n = 0; n < neighbors.count; ++n) {
                    GridPos neighbor = neighbors.positions[n];
                    //already within the bounding box
                    if (goalBounds[i][j].minRow <= neighbor.row && neighbor.row <= goalBounds[i][j].maxRow &&
                        goalBounds[i][j].minCol <= neighbor.col && neighbor.col <= goalBounds[i][j].maxCol) 
                    {
                        continue; 
                    }

                    queue.push(neighbor);
                }
            }
        }
    }
}
