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
    // handle any one-time setup requirements you have

    /*
        If you want to do any map-preprocessing, you'll need to listen
        for the map change message.  It'll look something like this:

        Callback cb = std::bind(&AStarPather::your_function_name, this);
        Messenger::listen_for_message(Messages::MAP_CHANGE, cb);

        There are other alternatives to using std::bind, so feel free to mix it up.
        Callback is just a typedef for std::function<void(void)>, so any std::invoke'able
        object that std::function can wrap will suffice.
    */


    return true; // return false if any errors actually occur, to stop engine initialization
}

void AStarPather::shutdown()
{
    /*
        Free any dynamically allocated memory or any other general house-
        keeping you need to do during shutdown.
    */
}

/*
    This is where you handle pathing requests, each request has several fields:

    start/goal - start and goal world positions
    path - where you will build the path upon completion, path should be
        start to goal, not goal to start
    heuristic - which heuristic calculation to use
    weight - the heuristic weight to be applied
    newRequest - whether this is the first request for this path, should generally
        be true, unless single step is on

    smoothing - whether to apply smoothing to the path
    rubberBanding - whether to apply rubber banding
    singleStep - whether to perform only a single A* step
    debugColoring - whether to color the grid based on the A* state:
        closed list nodes - yellow
        open list nodes - blue

    method - which algorithm to use: A*, Floyd-Warshall, JPS+, or goal bounding,
        will be A* generally, unless you implement extra credit features

    The return values are:
        PROCESSING - a path hasn't been found yet, should only be returned in
            single step mode until a path is found
        COMPLETE - a path to the goal was found and has been built in request.path
        IMPOSSIBLE - a path from start to goal does not exist, do not add start position to path
*/

PathResult AStarPather::compute_path(PathRequest &request)
{
    if (!terrain)
    {
        return PathResult::IMPOSSIBLE;
    }

    if (request.newRequest) 
    {
        std::cout << "new request----------" << std::endl;
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
            for (const auto& pos : finalPath)
            {
                request.path.push_back(pos);
            }
            return PathResult::COMPLETE;
        }

        closedList.insert(current.gridPos);
        terrain->set_color(current.gridPos, Colors::Yellow);

        for (const GridPos& neighbor : get_neighbors(current.gridPos))
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
        return (std::min(dx, dy) * std::sqrt(2.0f)) + std::max(dx, dy) -  std::min(dx, dy);
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

    std::cout << "minRow: " << minRow << std::endl;
    std::cout << "maxRow: " << maxRow << std::endl;
    std::cout << "minCol: " << minCol << std::endl;
    std::cout << "maxCol: " << maxCol << std::endl;
    std::cout << "------------------------------ " << std::endl;
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