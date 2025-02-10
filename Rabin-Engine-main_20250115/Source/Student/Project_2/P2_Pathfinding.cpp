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

        use terrain->set_color(row, col, Colors::YourColor);
        also it can be helpful to temporarily use other colors for specific states
        when you are testing your algorithms

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
        request.path.clear();
        openList = {}; 
        gCost.clear();
        cameFrom.clear();
        closedSet.clear();

        start = terrain->get_grid_position(request.start);
        goal = terrain->get_grid_position(request.goal);
        terrain->set_color(start, Colors::Orange);
        terrain->set_color(goal, Colors::Orange);
        request.path.push_back(request.start);

        gCost[start] = 0;
        openList.push({ start, 0, heuristic(start, goal) });
    }

    while (!openList.empty())
    {
        Node current = openList.top();
        openList.pop();

        if (current.gridPos == goal)
        {
            std::vector<Vec3> finalPath = reconstruct_path(cameFrom, start, goal);
            for (const auto& pos : finalPath)
            {
                request.path.push_back(pos);
            }
            return PathResult::COMPLETE;
        }

        closedSet.insert(current.gridPos);

        for (const GridPos& neighbor : get_neighbors(current.gridPos))
        {
            if (terrain->is_wall(neighbor)  || closedSet.count(neighbor))
                continue; // Ignore walls and closed nodes

            float new_g = gCost[current.gridPos] + 1; // Assume uniform cost

            if (!gCost.count(neighbor) || new_g < gCost[neighbor])
            {
                gCost[neighbor] = new_g;
                cameFrom[neighbor] = current.gridPos;
                openList.push({ neighbor, new_g, heuristic(neighbor, goal) });
            }
        }

        if (request.settings.singleStep)
            return PathResult::PROCESSING; // Allow pausing for step-by-step execution
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

float AStarPather::heuristic(const GridPos& a, const GridPos& b)
{
    return std::abs(a.col - b.col) + std::abs(a.row - b.row);
}

std::vector<GridPos> AStarPather::get_neighbors(const GridPos& pos)
{
    std::vector<GridPos> neighbors;
    std::vector<GridPos> directions = { {1, 0}, {-1, 0}, {0, 1}, {0, -1} }; // 4-way movement

    for (const auto& dir : directions)
    {
        GridPos newPos = { pos.row + dir.row, pos.col + dir.col };
        if (terrain->is_valid_grid_position(newPos) && !terrain->is_wall(newPos))
            neighbors.push_back(newPos);
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