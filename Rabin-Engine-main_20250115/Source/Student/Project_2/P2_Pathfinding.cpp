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

}

bool AStarPather::initialize()
{

    return true;
}

void AStarPather::shutdown()
{
    /*
        Free any dynamically allocated memory or any other general house-
        keeping you need to do during shutdown.
    */

    openList = {};
    closedList.clear();
    gCost.clear();
    cameFrom.clear();
    neighborCache.clear();
}

PathResult AStarPather::compute_path(PathRequest &request)
{
    if (!terrain)
    {
        return PathResult::IMPOSSIBLE;
    }

    if (request.newRequest) 
    {
        if (!hasPrecompute) 
        {
            precompute_neighbors();
            hasPrecompute = true;
        }

        request.path.clear();
        openList = {}; 
        closedList.clear();
        gCost.clear();
        cameFrom.clear();

        start = terrain->get_grid_position(request.start);
        goal = terrain->get_grid_position(request.goal);
        terrain->set_color(start, Colors::Orange);
        terrain->set_color(goal, Colors::Orange);
        request.path.push_back(request.start);

        gCost[start] = 0;
        openList.push({ start, 0, heuristic(start, goal, request.settings.heuristic) });
    }

    while (!openList.empty())
    {
        Node current = openList.top();
        openList.pop();

        if (current.gridPos == goal)
        {
            std::vector<Vec3> finalPath = reconstruct_path(cameFrom, start, goal);
            if (request.settings.rubberBanding)
            {
                apply_rubberbanding(finalPath);
            }
            if (request.settings.smoothing)
            {
                add_intermediate_points(finalPath, 10.0f);
                apply_catmull_rom_smoothing(finalPath);
            }

            for (const auto& pos : finalPath)
            {
                request.path.push_back(pos);
            }
            return PathResult::COMPLETE;
        }

        closedList.insert(current.gridPos);
        terrain->set_color(current.gridPos, Colors::Yellow);

        for (const GridPos& neighbor : neighborCache[current.gridPos])
        {
            if (terrain->is_wall(neighbor) || closedList.count(neighbor)) 
            {
                continue;
            }

            float new_g = gCost[current.gridPos] + 1; 

            if (!gCost.count(neighbor) || new_g < gCost[neighbor])
            {
                gCost[neighbor] = new_g;
                cameFrom[neighbor] = current.gridPos;
                openList.push({ neighbor, new_g, heuristic(neighbor, goal, request.settings.heuristic) });
                terrain->set_color(neighbor, Colors::Blue);
            }
        }

        if (request.settings.singleStep)
            return PathResult::PROCESSING; 
    }

    return PathResult::IMPOSSIBLE;
}

void AStarPather::print_map()
{
    if (!terrain)
    {
        std::cerr << "Error: Terrain is null." << std::endl;
        return;
    }

    int height = terrain->get_map_height();
    int width = terrain->get_map_width();

    for (int row = 0; row < height; ++row)
    {
        for (int col = 0; col < width; ++col)
        {
            if (terrain->is_wall(row, col))
            {
                std::cout << "#"; // Wall
            }
            else
            {
                std::cout << "."; // Open space
            }
        }
        std::cout << std::endl;
    }
}

float AStarPather::heuristic(const GridPos& a, const GridPos& b, Heuristic heuristic)
{
    int dx = std::abs(a.col - b.col);
    int dy = std::abs(a.row - b.row);

    if (heuristic == Heuristic::OCTILE)
    {
        return (std::min(dx, dy) * 1.414) + std::max(dx, dy) -  std::min(dx, dy);
    }
    else if (heuristic == Heuristic::CHEBYSHEV)
    {
        return std::max(dx, dy);
    }
    else if (heuristic == Heuristic::INCONSISTENT)
    {
        return (dx + dy) + (0.25f * std::abs(dx - dy));
    }
    else if (heuristic == Heuristic::MANHATTAN)
    {
        return dx + dy;
    }
    else if (heuristic == Heuristic::EUCLIDEAN)
    {
        return std::sqrt(dx * dx + dy * dy);
    }
    else
    {
        //NUM_ENTRIES
        return 0.0f;
    }
}

std::vector<GridPos> AStarPather::get_neighbors(const GridPos& pos)
{
    std::vector<GridPos> neighbors;
    std::vector<GridPos> directions = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},  // Cardinal directions
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1} // Diagonal directions
    };

    for (const auto& dir : directions)
    {
        GridPos newPos = { pos.row + dir.row, pos.col + dir.col };

        // Check if the position is within bounds and is not a wall
        if (terrain->is_valid_grid_position(newPos) && !terrain->is_wall(newPos))
        {
            // For diagonal movement, ensure we are not cutting corners
            if (std::abs(dir.row) + std::abs(dir.col) == 2) // It's a diagonal move
            {
                GridPos adjacent1 = { pos.row, newPos.col }; // Horizontal neighbor
                GridPos adjacent2 = { newPos.row, pos.col }; // Vertical neighbor

                if (terrain->is_wall(adjacent1) || terrain->is_wall(adjacent2))
                {
                    continue; // Skip this diagonal move if it cuts through a wall
                }
            }

            neighbors.push_back(newPos);
        }
    }
    return neighbors;
}


std::vector<Vec3> AStarPather::reconstruct_path(std::unordered_map<GridPos, GridPos, GridPosHash>& cameFrom, GridPos start, GridPos goal)
{
    std::vector<Vec3> finalPath;
    for (GridPos step = goal; step != start; step = cameFrom[step]) 
    {
        finalPath.push_back(terrain->get_world_position(step));
    }

    finalPath.push_back(terrain->get_world_position(start));
    std::reverse(finalPath.begin(), finalPath.end());

    return finalPath;
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
    float t2 = t * t;
    float t3 = t2 * t;

    Vec3 a = (p1 * 2.0f) + (p2 - p0) * t + (p0 * 2.0f - p1 * 5.0f + p2 * 4.0f - p3) * t2 + (p1 * 3.0f - p0 - p2 * 3.0f + p3) * t3;
    return a * 0.5f;
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
        for (float t = 0.0f; t < 1.0f; t += 0.1f)
        {
            smoothedPath.push_back(catmull_rom_interpolate(p0, p1, p2, p3, t));
        }
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

void AStarPather::precompute_neighbors()
{
    for (int row = 0; row < terrain->get_map_height(); ++row)
    {
        for (int col = 0; col < terrain->get_map_width(); ++col)
        {
            GridPos pos = { row, col };
            neighborCache[pos] = get_neighbors(pos);
        }
    }
}
