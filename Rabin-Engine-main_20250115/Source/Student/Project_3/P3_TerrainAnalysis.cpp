#include <pch.h>
#include "Terrain/TerrainAnalysis.h"
#include "Terrain/MapMath.h"
#include "Agent/AStarAgent.h"
#include "Terrain/MapLayer.h"
#include "Projects/ProjectThree.h"

#include <iostream>

bool ProjectThree::implemented_fog_of_war() const // extra credit
{
    return false;
}

/*
    Check the euclidean distance from the given cell to every other wall cell,
    with cells outside the map bounds treated as walls, and return the smallest
    distance.  Make use of the is_valid_grid_position and is_wall member
    functions in the global terrain to determine if a cell is within map bounds
    and a wall, respectively.
*/
float distance_to_closest_wall(int row, int col)
{
    float min_distance = std::numeric_limits<float>::max();

    int map_height = terrain->get_map_height();
    int map_width = terrain->get_map_width();

    for (int i = 0; i < map_height; ++i) 
    {
        for (int j = 0; j < map_width; ++j) 
        {
            if (terrain->is_wall(i, j)) 
            {
                float distance = std::sqrt((i - row) * (i - row) + (j - col) * (j - col));
                if (distance < min_distance) 
                {
                    min_distance = distance;
                }
            }
        }
    }

    //check the four borders of the map
    for (int i = 0; i < map_height; ++i) 
    {
        //left border where j = -1
        float distance = std::sqrt((i - row) * (i - row) + (-1 - col) * (-1 - col));
        if (distance < min_distance) 
        {
            min_distance = distance;
        }

        //right border where j = map_width
        distance = std::sqrt((i - row) * (i - row) + (map_width - col) * (map_width - col));
        if (distance < min_distance) 
        {
            min_distance = distance;
        }
    }

    for (int j = 0; j < map_width; ++j) 
    {
        //top border where i = -1
        float distance = std::sqrt((-1 - row) * (-1 - row) + (j - col) * (j - col));
        if (distance < min_distance) 
        {
            min_distance = distance;
        }

        //bottom border where i = map_height
        distance = std::sqrt((map_height - row) * (map_height - row) + (j - col) * (j - col));
        if (distance < min_distance) 
        {
            min_distance = distance;
        }
    }

    return min_distance;
}

/*
    Two cells (row0, col0) and (row1, col1) are visible to each other if a line
    between their centerpoints doesn't intersect the four boundary lines of every
    wall cell.  
    You should puff out the four boundary lines by a very tiny amount
    so that a diagonal line passing by the corner will intersect it.  Make use of the
    line_intersect helper function for the intersection test and the is_wall member
    function in the global terrain to determine if a cell is a wall or not.
*/
bool is_clear_path(int row0, int col0, int row1, int col1)
{
    Vec2 start(col0 + 0.5f, row0 + 0.5f);
    Vec2 end(col1 + 0.5f, row1 + 0.5f);

    for (int i = 0; i < terrain->get_map_height(); ++i)
    {
        for (int j = 0; j < terrain->get_map_width(); ++j)
        {
            if (terrain->is_wall(i, j))
            {
                float left = j;
                float right = j + 1;
                float bottom = i;
                float top = i + 1;

                Vec2 topLeft(left, top);
                Vec2 topRight(right, top);
                Vec2 bottomLeft(left, bottom);
                Vec2 bottomRight(right, bottom);

                bool isIntersectLeftBoundary = line_intersect(start, end, bottomLeft, topLeft);
                bool isIntersectRightBoundary = line_intersect(start, end, bottomRight, topRight);
                bool isIntersectBottomBoundary = line_intersect(start, end, bottomLeft, bottomRight);
                bool isIntersectTopBoundary = line_intersect(start, end, topLeft, topRight);

                // Check intersection with the four boundary lines
                if (isIntersectLeftBoundary || isIntersectRightBoundary || isIntersectBottomBoundary || isIntersectTopBoundary)
                {
                    //path is blocked by a wall
                    return false;
                }
            }
        }
    }

    //no walls
    return true;
}

/*
    Mark every cell in the given layer with the value 1 / (d * d),
    where d is the distance to the closest wall or edge.  Make use of the
    distance_to_closest_wall helper function.  Walls should not be marked.
*/
void analyze_openness(MapLayer<float> &layer)
{
    int map_height = terrain->get_map_height();
    int map_width = terrain->get_map_width();

    //iterate over all cells in the grid
    for (int i = 0; i < map_height; ++i) {
        for (int j = 0; j < map_width; ++j) {
            //skip wall cells
            //walls have openness 0
            if (terrain->is_wall(i, j)) 
            {
                layer.set_value(i, j, 0.0f); 
                continue;
            }

            //calculate the distance to the closest wall or edge
            float d = distance_to_closest_wall(i, j);

            //avoid division by zero
            if (d < 0.0001f) {
                layer.set_value(i, j, 0.0f);
            }
            //calculate openness as 1 / (d * d)
            else 
            {
                float openness = 1.0f / (d * d);
                layer.set_value(i, j, openness);
            }
        }
    }
}

/*
    Mark every cell in the given layer with the number of cells that
    are visible to it, divided by 160 (a magic number that looks good).  Make sure
    to cap the value at 1.0 as well.

    Two cells are visible to each other if a line between their centerpoints doesn't
    intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
    helper function.
*/
void analyze_visibility(MapLayer<float> &layer)
{
    int map_height = terrain->get_map_height();
    int map_width = terrain->get_map_width();

    for (int i = 0; i < map_height; ++i) {
        for (int j = 0; j < map_width; ++j) {
            //skip wall cells
            //walls have visibility 0
            if (terrain->is_wall(i, j)) 
            {
                layer.set_value(i, j, 0.0f);
                continue;
            }

            int visible_count = 0;

            for (int x = 0; x < map_height; ++x) 
            {
                for (int y = 0; y < map_width; ++y) 
                {
                    if ((x == i && y == j) || terrain->is_wall(x, y)) 
                    {
                        continue;
                    }

                    if (is_clear_path(i, j, x, y)) 
                    {
                        visible_count++;
                    }
                }
            }

            //calculate visibility as visible_count / 160, capped at 1.0
            float visibility = static_cast<float>(visible_count) / 160.0f;
            visibility = std::min(visibility, 1.0f);

            //set the visibility value in the layer
            layer.set_value(i, j, visibility);
        }
    }
}

/*
    For every cell in the given layer mark it with 1.0
    if it is visible to the given cell, 0.5 if it isn't visible but is next to a visible cell,
    or 0.0 otherwise.

    Two cells are visible to each other if a line between their centerpoints doesn't
    intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
    helper function.
*/
void analyze_visible_to_cell(MapLayer<float> &layer, int row, int col)
{
    int map_height = terrain->get_map_height();
    int map_width = terrain->get_map_width();

    std::vector<std::vector<float>> visibility_grid(map_height, std::vector<float>(map_width, 0.0f));

    for (int i = 0; i < map_height; ++i) 
    {
        for (int j = 0; j < map_width; ++j) 
        {
            if (terrain->is_wall(i, j)) 
            {
                continue;
            }

            if (is_clear_path(row, col, i, j)) 
            {
                visibility_grid[i][j] = 1.0f;
            }
        }
    }

    for (int i = 0; i < map_height; ++i) 
    {
        for (int j = 0; j < map_width; ++j) 
        {
            if (terrain->is_wall(i, j) || visibility_grid[i][j] == 1.0f) 
            {
                continue;
            }

            //check all neighbors for visibility
            bool is_adjacent_to_visible = false;
            for (int dx = -1; dx <= 1; ++dx) 
            {
                for (int dy = -1; dy <= 1; ++dy) 
                {
                    if (dx == 0 && dy == 0) 
                    {
                        continue;
                    } 

                    int x = i + dx;
                    int y = j + dy;

                    //check if the neighboring cell is within bounds and visible
                    if (x >= 0 && x < map_height && y >= 0 && y < map_width && visibility_grid[x][y] == 1.0f) 
                    {
                        is_adjacent_to_visible = true;
                        break;
                    }
                }
                if (is_adjacent_to_visible) 
                {
                    break;
                } 
            }

            //mark the cell as 0.5 if it is adjacent to a visible cell
            if (is_adjacent_to_visible) 
            {
                visibility_grid[i][j] = 0.5f;
            }
        }
    }

    for (int i = 0; i < map_height; ++i) 
    {
        for (int j = 0; j < map_width; ++j) 
        {
            layer.set_value(i, j, visibility_grid[i][j]);
        }
    }
}

/*
    For every cell in the given layer that is visible to the given agent,
    mark it as 1.0, otherwise don't change the cell's current value.

    You must consider the direction the agent is facing.  All of the agent data is
    in three dimensions, but to simplify you should operate in two dimensions, the XZ plane.

    Take the dot product between the view vector and the vector from the agent to the cell,
    both normalized, and compare the cosines directly instead of taking the arccosine to
    avoid introducing floating-point inaccuracy (larger cosine means smaller angle).

    Give the agent a field of view slighter larger than 180 degrees.

    Two cells are visible to each other if a line between their centerpoints doesn't
    intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
    helper function.
*/
void analyze_agent_vision(MapLayer<float>& layer, const Agent* agent)
{
    int map_height = terrain->get_map_height();
    int map_width = terrain->get_map_width();

    Vec3 agent_pos = agent->get_position();
    Vec3 agent_dir = agent->get_forward_vector();

    //normalize the agent's direction vector in the XZ plane
    float agent_dir_length = std::sqrt(agent_dir.x * agent_dir.x + agent_dir.z * agent_dir.z);
    Vec2 agent_dir_xz = { agent_dir.x / agent_dir_length, agent_dir.z / agent_dir_length };

    //define the field of view (FOV) cosine threshold (slightly larger than 180 degrees)
    const float fov_degrees = 190.0f;
    const float fov_cosine_threshold = std::cos((fov_degrees / 2.0f) * (M_PI / 180.0f));

    GridPos agent_grid_pos = terrain->get_grid_position(agent_pos);
    int agent_row = agent_grid_pos.row;
    int agent_col = agent_grid_pos.col;

    for (int i = 0; i < map_height; ++i) 
    {
        for (int j = 0; j < map_width; ++j) 
        {
            if (terrain->is_wall(i, j)) 
            {
                continue;
            }

            Vec3 cell_world_pos = terrain->get_world_position(i, j);
            Vec2 cell_pos_xz = { cell_world_pos.x, cell_world_pos.z };

            //vector from the agent to cell
            Vec2 agent_to_cell = { cell_pos_xz.x - agent_pos.x, cell_pos_xz.y - agent_pos.z };
            float agent_to_cell_length = std::sqrt(agent_to_cell.x * agent_to_cell.x + agent_to_cell.y * agent_to_cell.y);
            
            //skip if the cell is too close to the agent
            if (agent_to_cell_length < 0.0001f) 
            {
                continue; 
            }

            agent_to_cell.x /= agent_to_cell_length;
            agent_to_cell.y /= agent_to_cell_length;

            float dot_product = agent_dir_xz.x * agent_to_cell.x + agent_dir_xz.y * agent_to_cell.y;

            // Check if the cell is within the agent's FOV
            if (dot_product >= fov_cosine_threshold) 
            {
                if (is_clear_path(agent_row, agent_col, i, j)) 
                {
                    layer.set_value(i, j, 1.0f);
                }
            }
        }
    }
}

/*
    For every cell in the given layer:

        1) Get the value of each neighbor and apply decay factor
        2) Keep the highest value from step 1
        3) Linearly interpolate from the cell's current value to the value from step 2
           with the growing factor as a coefficient.  Make use of the lerp helper function.
        4) Store the value from step 3 in a temporary layer.
           A float[40][40] will suffice, no need to dynamically allocate or make a new MapLayer.

    After every cell has been processed into the temporary layer, write the temporary layer into
    the given layer;
*/
void propagate_solo_occupancy(MapLayer<float>& layer, float decay, float growth)
{
    int map_height = terrain->get_map_height();
    int map_width = terrain->get_map_width();

    float temp_layer[40][40] = { 0.0f };

    for (int i = 0; i < map_height; ++i) 
    {
        for (int j = 0; j < map_width; ++j) 
        {
            if (terrain->is_wall(i, j)) 
            {
                temp_layer[i][j] = 0.0f;
                continue;
            }

            //step 1: get the value of each neighbor and apply exponential decay based on distance
            float max_decayed_value = 0.0f;

            // Check all 8 neighbors (including diagonals)
            for (int dx = -1; dx <= 1; ++dx) 
            {
                for (int dy = -1; dy <= 1; ++dy) 
                {
                    if (dx == 0 && dy == 0) 
                    {
                        continue; 
                    } 

                    int x = i + dx;
                    int y = j + dy;

                    //ensure the neighbor is within bounds
                    if (x >= 0 && x < map_height && y >= 0 && y < map_width) 
                    {
                        //calculate the distance between the current cell and the neighbor
                        float distance = std::sqrt(dx * dx + dy * dy);

                        // Apply exponential decay to the neighbor's value
                        float neighbor_value = layer.get_value(x, y);
                        float decayed_value = neighbor_value * exp(-distance * decay);

                        // Keep the maximum decayed value
                        if (decayed_value > max_decayed_value) 
                        {
                            max_decayed_value = decayed_value;
                        }
                    }
                }
            }

            //step 2: keep the highest value from step 1
            float current_value = layer.get_value(i, j);

            //step 3: linearly interpolate from current_value to max_decayed_value
            float new_value = lerp(current_value, max_decayed_value, growth);

            //step 4: store it in temporary layer
            temp_layer[i][j] = new_value;
        }
    }

    // Write the temporary layer back into the given layer
    for (int i = 0; i < map_height; ++i) 
    {
        for (int j = 0; j < map_width; ++j) 
        {
            layer.set_value(i, j, temp_layer[i][j]);
        }
    }
}
/*
    Determine the maximum value in the given layer, and then divide the value
    for every cell in the layer by that amount.  This will keep the values in the
    range of [0, 1].  Negative values should be left unmodified.
*/
void normalize_solo_occupancy(MapLayer<float>& layer)
{
    int map_height = terrain->get_map_height();
    int map_width = terrain->get_map_width();

    // Step 1: Find the maximum value in the layer
    float max_value = 0.0f;
    for (int i = 0; i < map_height; ++i) {
        for (int j = 0; j < map_width; ++j) {
            float cell_value = layer.get_value(i, j);
            if (cell_value > max_value) {
                max_value = cell_value;
            }
        }
    }

    // Step 2: Normalize the values in the layer (skip negative values)
    if (max_value > 0.0f) { // Avoid division by zero
        for (int i = 0; i < map_height; ++i) {
            for (int j = 0; j < map_width; ++j) {
                float cell_value = layer.get_value(i, j);
                if (cell_value >= 0.0f) { // Skip negative values
                    layer.set_value(i, j, cell_value / max_value);
                }
            }
        }
    }
}

/*
    First, clear out the old values in the map layer by setting any negative value to 0.
    Then, for every cell in the layer that is within the field of view cone, from the
    enemy agent, mark it with the occupancy value.  Take the dot product between the view
    vector and the vector from the agent to the cell, both normalized, and compare the
    cosines directly instead of taking the arccosine to avoid introducing floating-point
    inaccuracy (larger cosine means smaller angle).

    If the tile is close enough to the enemy (less than closeDistance),
    you only check if it's visible to enemy.  Make use of the is_clear_path
    helper function.  Otherwise, you must consider the direction the enemy is facing too.
    This creates a radius around the enemy that the player can be detected within, as well
    as a fov cone.
*/
void enemy_field_of_view(MapLayer<float>& layer, float fovAngle, float closeDistance, float occupancyValue, AStarAgent* enemy)
{
    //step 1: clear out old values in the layer by setting any negative value to 0
    for (int i = 0; i < terrain->get_map_height(); ++i) 
    {
        for (int j = 0; j < terrain->get_map_width(); ++j) 
        {
            if (layer.get_value(i, j) < 0.0f) 
            {
                layer.set_value(i, j, 0.0f);
            }
        }
    }

    Vec3 enemy_pos = enemy->get_position();
    Vec3 enemy_dir = enemy->get_forward_vector();
    float enemy_dir_length = std::sqrt(enemy_dir.x * enemy_dir.x + enemy_dir.z * enemy_dir.z);
    Vec2 enemy_dir_xz = { enemy_dir.x / enemy_dir_length, enemy_dir.z / enemy_dir_length };

    GridPos enemy_grid_pos = terrain->get_grid_position(enemy_pos);
    int enemy_row = enemy_grid_pos.row;
    int enemy_col = enemy_grid_pos.col;

    float fov_cosine = std::cos(fovAngle * M_PI / 180.0f);

    for (int i = 0; i < terrain->get_map_height(); ++i) 
    {
        for (int j = 0; j < terrain->get_map_width(); ++j) 
        {
            if (terrain->is_wall(i, j)) 
            {
                continue;
            }

            // Get the center position of the current cell in the XZ plane
            Vec3 cell_world_pos = terrain->get_world_position(i, j);
            Vec2 cell_pos_xz = { cell_world_pos.x, cell_world_pos.z };

            // Calculate the vector from the enemy to the cell in the XZ plane
            Vec2 enemy_to_cell = { cell_pos_xz.x - enemy_pos.x, cell_pos_xz.y - enemy_pos.z };

            // Calculate the distance from the enemy to the cell
            float distance = std::sqrt(enemy_to_cell.x * enemy_to_cell.x + enemy_to_cell.y * enemy_to_cell.y);

            // Normalize the enemy-to-cell vector
            if (distance < 0.0001f) {
                continue; // Skip if the cell is too close to the enemy
            }
            enemy_to_cell.x /= distance;
            enemy_to_cell.y /= distance;

            // Calculate the dot product between the enemy's direction and the enemy-to-cell vector
            float dot_product = enemy_dir_xz.x * enemy_to_cell.x + enemy_dir_xz.y * enemy_to_cell.y;

            // Check if the cell is within the FOV cone or close enough
            if (distance <= closeDistance) {
                // If the cell is within closeDistance, only check visibility
                if (is_clear_path(enemy_row, enemy_col, i, j)) {
                    layer.set_value(i, j, occupancyValue);
                }
            }
            else {
                // If the cell is outside closeDistance, check both visibility and FOV
                if (dot_product >= fov_cosine && is_clear_path(enemy_row, enemy_col, i, j)) 
                {
                    layer.set_value(i, j, occupancyValue);
                }
            }
        }
    }
}

/*
    Check if the player's current tile has a negative value, ie in the fov cone
    or within a detection radius.
*/
bool enemy_find_player(MapLayer<float>& layer, AStarAgent* enemy, Agent* player)
{
    const auto playerGridPos = terrain->get_grid_position(player->get_position());

    if (terrain->is_valid_grid_position(playerGridPos)) 
    {
        if (layer.get_value(playerGridPos.row, playerGridPos.col) < 0.0f) 
        {
            return true;
        }
    }

    //player isn't in the fov coneor within a detection radius.
    return false;
}


/*
    Attempt to find a cell with the highest nonzero value (normalization may
    not produce exactly 1.0 due to floating point error), and then set it as
    the new target, using enemy->path_to.

    If there are multiple cells with the same highest value, then pick the
    cell closest to the enemy.

    Return whether a target cell was found.
*/
bool enemy_seek_player(MapLayer<float>& layer, AStarAgent* enemy)
{
    int map_height = terrain->get_map_height();
    int map_width = terrain->get_map_width();

    //initialize variables to track the highest value and closest cell
    float max_value = 0.0f;
    float min_distance = std::numeric_limits<float>::max();
    int target_row = -1;
    int target_col = -1;

    GridPos enemy_grid_pos = terrain->get_grid_position(enemy->get_position());
    int enemy_row = enemy_grid_pos.row;
    int enemy_col = enemy_grid_pos.col;

    for (int i = 0; i < map_height; ++i) 
    {
        for (int j = 0; j < map_width; ++j) 
        {
            float cell_value = layer.get_value(i, j);

            //skip cells with zero or negative values
            if (cell_value <= 0.0f) 
            {
                continue;
            }

            //check if this cell has a higher value than the current max
            if (cell_value > max_value) 
            {
                max_value = cell_value;
                target_row = i;
                target_col = j;
                min_distance = std::sqrt((i - enemy_row) * (i - enemy_row) + (j - enemy_col) * (j - enemy_col));
            }

            //if the cell has the same value as the current max, check if it's closer
            else if (cell_value == max_value) 
            {
                float distance = std::sqrt((i - enemy_row) * (i - enemy_row) + (j - enemy_col) * (j - enemy_col));
                if (distance < min_distance) 
                {
                    target_row = i;
                    target_col = j;
                    min_distance = distance;
                }
            }
        }
    }

    //if a target cell was found, add to the path
    if (target_row != -1 && target_col != -1) 
    {
        enemy->path_to(Vec3(target_row, target_col, 0));
        return true;
    }

    //nothing is found
    return false;
}