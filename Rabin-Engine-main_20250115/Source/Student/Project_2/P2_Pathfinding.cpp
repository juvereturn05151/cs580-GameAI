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

AStarPather::AStarPather()
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
    openList.clear(); // Clear the open list
    openList.reserve(1600); // Preallocate 1600 slots
    lastIndex = -1; // No elements in the open list initially

    Callback cb = std::bind(&AStarPather::precompute_valid_neighbors, this);
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

PathResult AStarPather::compute_path(PathRequest &request)
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

        if (request.settings.debugColoring) {
            terrain->set_color(start, Colors::Orange);
            terrain->set_color(goal, Colors::Orange);
        }

        Node* startNode = &nodes[start.row][start.col];
        startNode->givenCost = 0;
        startNode->finalCost = heuristic(start, goal, request);
        startNode->onList = ListStatus::Open;
        open_list_push(startNode, request);
    }

    while (lastIndex >= 0)
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
                add_intermediate_points(finalPath, 1.5f);
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
                continue; // Skip walls
            }

            Node* childNode = &nodes[neighbor.row][neighbor.col];

            float cost = (neighbor.row != parentNode->gridPos.row && neighbor.col != parentNode->gridPos.col) ? 1.414f : 1.0f;
            float new_g = parentNode->givenCost + cost; // Assuming uniform cost
            float new_f = new_g + heuristic(neighbor, goal, request);

            if (childNode->onList == ListStatus::None)
            {
                // If child node isn’t on Open or Closed list, put it on Open List
                childNode->parent = parentNode;
                childNode->givenCost = new_g;
                childNode->finalCost = new_f;
                childNode->onList = ListStatus::Open;
                open_list_push(childNode, request);
            }
            else if (childNode->onList == ListStatus::Open || childNode->onList == ListStatus::Closed)
            {
                // If child node is on Open or Closed List, AND this new one is cheaper,
                // then update the node
                if (new_g < childNode->givenCost)
                {
                    childNode->parent = parentNode;
                    childNode->givenCost = new_g;
                    childNode->finalCost = new_f;

                    if (childNode->onList == ListStatus::Closed)
                    {
                        // Reopen the node if it was closed
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
            h = std::sqrt(dx * dx + dy * dy); // Euclidean distance for odd-sum coordinates
        }
        else
        {
            h = 0.0f; // Artificially setting heuristic to zero for even-sum coordinates
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
    //if (path.size() < 3) // No need to process if the path is too short
    //    return;

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

    // Determine the bounding box for the square area
    int minRow = std::min(startGrid.row, endGrid.row);
    int maxRow = std::max(startGrid.row, endGrid.row);
    int minCol = std::min(startGrid.col, endGrid.col);
    int maxCol = std::max(startGrid.col, endGrid.col);

    // Iterate over the square area
    for (int row = minRow; row <= maxRow; ++row)
    {
        for (int col = minCol; col <= maxCol; ++col)
        {
            if (terrain->is_wall(row, col))
            {
                return false; // If any wall is found, the middle node cannot be eliminated
            }
        }
    }

    return true; // No walls found, the middle node can be eliminated
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
    if (path.size() < 3) // No need to process if the path is too short
        return;

    std::vector<Vec3> smoothedPath;
    int n = path.size();

    // Add the first point
    smoothedPath.push_back(path[0]);

    // Iterate through the path and apply Catmull-Rom interpolation
    for (int i = 0; i < n - 1; ++i)
    {
        Vec3 p0 = (i == 0) ? path[0] : path[i - 1];
        Vec3 p1 = path[i];
        Vec3 p2 = path[i + 1];
        Vec3 p3 = (i == n - 2) ? path[n - 1] : path[i + 2];

        // Add intermediate points
        smoothedPath.push_back(catmull_rom_interpolate(p0, p1, p2, p3, 0.25));
        smoothedPath.push_back(catmull_rom_interpolate(p0, p1, p2, p3, 0.5));
        smoothedPath.push_back(catmull_rom_interpolate(p0, p1, p2, p3, 0.75));
    }

    // Add the last point
    smoothedPath.push_back(path[n - 1]);

    // Replace the original path with the smoothed path
    path = smoothedPath;
}

void AStarPather::add_intermediate_points(std::vector<Vec3>& path, float maxDistance)
{
    if (path.size() < 2) // No need to process if the path is too short
        return;

    std::vector<Vec3> newPath;
    newPath.push_back(path[0]); // Add the first point

    for (size_t i = 1; i < path.size(); ++i)
    {
        Vec3 prevPoint = newPath.back();
        Vec3 currentPoint = path[i];

        float distance = (currentPoint - prevPoint).Length();

        if (distance > maxDistance)
        {
            // Calculate the direction vector
            Vec3 direction = (currentPoint - prevPoint) / distance;

            // Add intermediate points spaced by maxDistance
            int numPoints = static_cast<int>(distance / maxDistance);
            for (int j = 1; j <= numPoints; ++j)
            {
                Vec3 intermediatePoint = prevPoint + direction * (maxDistance * j);
                newPath.push_back(intermediatePoint);
            }
        }

        newPath.push_back(currentPoint); // Add the current point
    }

    path = newPath; // Replace the original path with the new path
}

void AStarPather::open_list_push(Node* node, PathRequest& request)
{
    if (request.settings.debugColoring) {
        terrain->set_color(node->gridPos, Colors::Blue);
    }

    // Increment lastIndex and add the node to the open list
    lastIndex++;
    if (lastIndex < openList.size()) {
        openList[lastIndex] = node; // Overwrite existing slot
    }
    else {
        openList.push_back(node); // Add new slot
    }
}

Node* AStarPather::open_list_pop()
{
    if (lastIndex < 0) {
        return nullptr; // No nodes in the open list
    }

    // Find the node with the smallest finalCost
    int cheapestIndex = 0;
    float cheapestCost = openList[0]->finalCost;
    for (int i = 1; i <= lastIndex; ++i) {
        if (openList[i]->finalCost < cheapestCost) {
            cheapestCost = openList[i]->finalCost;
            cheapestIndex = i;
        }
    }

    // Replace the cheapest node with the last node
    Node* cheapestNode = openList[cheapestIndex];
    openList[cheapestIndex] = openList[lastIndex];
    lastIndex--; // Decrement lastIndex

    return cheapestNode;
}

void AStarPather::clear_open_list()
{
    openList.clear();
    lastIndex = -1;
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
