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
    // WRITE YOUR CODE HERE
    float min_distance = std::numeric_limits<float>::max(); // Initialize with a large value

    // Get the map dimensions
    int map_height = terrain->get_map_height();
    int map_width = terrain->get_map_width();

    // Iterate over all possible cells in the grid
    for (int i = 0; i < map_height; ++i) 
    {
        for (int j = 0; j < map_width; ++j) 
        {
            // Check if the cell is a wall
            if (terrain->is_wall(i, j)) 
            {
                // Calculate Euclidean distance
                float distance = std::sqrt((i - row) * (i - row) + (j - col) * (j - col));
                if (distance < min_distance) 
                {
                    min_distance = distance;
                }
            }
        }
    }

    // Handle cells outside the map bounds (treated as walls)
    // Check the four borders of the map
    for (int i = 0; i < map_height; ++i) 
    {
        // Left border (j = -1)
        float distance = std::sqrt((i - row) * (i - row) + (-1 - col) * (-1 - col));
        if (distance < min_distance) 
        {
            min_distance = distance;
        }
        // Right border (j = map_width)
        distance = std::sqrt((i - row) * (i - row) + (map_width - col) * (map_width - col));
        if (distance < min_distance) 
        {
            min_distance = distance;
        }
    }

    for (int j = 0; j < map_width; ++j) 
    {
        // Top border (i = -1)
        float distance = std::sqrt((-1 - row) * (-1 - row) + (j - col) * (j - col));
        if (distance < min_distance) 
        {
            min_distance = distance;
        }
        // Bottom border (i = map_height)
        distance = std::sqrt((map_height - row) * (map_height - row) + (j - col) * (j - col));
        if (distance < min_distance) 
        {
            min_distance = distance;
        }
    }

    return min_distance;
}

// Helper function to check if two line segments intersect
bool line_intersect(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4)
{
    // Calculate the orientation of the triplet (p1, p2, p3)
    auto orientation = [](float x1, float y1, float x2, float y2, float x3, float y3) -> float 
    {
        return (y2 - y1) * (x3 - x2) - (x2 - x1) * (y3 - y2);
    };

    // Calculate the orientations
    float o1 = orientation(x1, y1, x2, y2, x3, y3);
    float o2 = orientation(x1, y1, x2, y2, x4, y4);
    float o3 = orientation(x3, y3, x4, y4, x1, y1);
    float o4 = orientation(x3, y3, x4, y4, x2, y2);

    // General case: Check if the line segments intersect
    if ((o1 * o2 < 0) && (o3 * o4 < 0)) {
        return true;
    }

    // Special case: Check if any endpoint lies on the other line segment
    auto on_segment = [](float x1, float y1, float x2, float y2, float x, float y) -> bool {
        return x >= std::min(x1, x2) && x <= std::max(x1, x2) &&
            y >= std::min(y1, y2) && y <= std::max(y1, y2);
        };

    if (o1 == 0 && on_segment(x1, y1, x2, y2, x3, y3)) return true;
    if (o2 == 0 && on_segment(x1, y1, x2, y2, x4, y4)) return true;
    if (o3 == 0 && on_segment(x3, y3, x4, y4, x1, y1)) return true;
    if (o4 == 0 && on_segment(x3, y3, x4, y4, x2, y2)) return true;

    return false;
}


/*
    Two cells (row0, col0) and (row1, col1) are visible to each other if a line
    between their centerpoints doesn't intersect the four boundary lines of every
    wall cell.  You should puff out the four boundary lines by a very tiny amount
    so that a diagonal line passing by the corner will intersect it.  Make use of the
    line_intersect helper function for the intersection test and the is_wall member
    function in the global terrain to determine if a cell is a wall or not.
*/
bool is_clear_path(int row0, int col0, int row1, int col1)
{
    // Get the center points of the two cells
    float x0 = col0 + 0.5f; // Center of cell (row0, col0)
    float y0 = row0 + 0.5f;
    float x1 = col1 + 0.5f; // Center of cell (row1, col1)
    float y1 = row1 + 0.5f;

    // Iterate over all cells in the grid
    for (int i = 0; i < terrain->get_map_height(); ++i) 
    {
        for (int j = 0; j < terrain->get_map_width(); ++j) 
        {
            // Check if the cell is a wall
            if (terrain->is_wall(i, j)) 
            {
                // Get the four boundary lines of the wall cell, puffed out by a tiny amount
                float epsilon = 0.0001f; // Tiny offset to ensure diagonal lines intersect corners
                float left = j - epsilon;
                float right = j + 1 + epsilon;
                float bottom = i - epsilon;
                float top = i + 1 + epsilon;

                // Check intersection with the four boundary lines
                if (line_intersect(x0, y0, x1, y1, left, bottom, left, top) || // Left boundary
                    line_intersect(x0, y0, x1, y1, right, bottom, right, top) || // Right boundary
                    line_intersect(x0, y0, x1, y1, left, bottom, right, bottom) || // Bottom boundary
                    line_intersect(x0, y0, x1, y1, left, top, right, top)) 
                { // Top boundary
                    return false; // Path is blocked by a wall
                }
            }
        }
    }

    return true; // No walls block the path
}

/*
    Mark every cell in the given layer with the value 1 / (d * d),
    where d is the distance to the closest wall or edge.  Make use of the
    distance_to_closest_wall helper function.  Walls should not be marked.
*/
void analyze_openness(MapLayer<float> &layer)
{
   // Get the map dimensions
    int map_height = terrain->get_map_height();
    int map_width = terrain->get_map_width();

    // Iterate over all cells in the grid
    for (int i = 0; i < map_height; ++i) {
        for (int j = 0; j < map_width; ++j) {
            // Skip wall cells
            if (terrain->is_wall(i, j)) {
                layer.set_value(i, j, 0.0f); // Walls have openness 0
                continue;
            }

            // Calculate the distance to the closest wall or edge
            float d = distance_to_closest_wall(i, j);

            // Avoid division by zero (in case d is 0, which shouldn't happen for non-wall cells)
            if (d < 0.0001f) {
                layer.set_value(i, j, 0.0f);
            }
            else {
                // Calculate openness as 1 / (d * d)
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
    // Get the map dimensions
    int map_height = terrain->get_map_height();
    int map_width = terrain->get_map_width();

    // Iterate over all cells in the grid
    for (int i = 0; i < map_height; ++i) {
        for (int j = 0; j < map_width; ++j) {
            // Skip wall cells
            if (terrain->is_wall(i, j)) {
                layer.set_value(i, j, 0.0f); // Walls have visibility 0
                continue;
            }

            // Count the number of visible cells
            int visible_count = 0;

            // Iterate over all other cells in the grid
            for (int x = 0; x < map_height; ++x) {
                for (int y = 0; y < map_width; ++y) {
                    // Skip the current cell and wall cells
                    if ((x == i && y == j) || terrain->is_wall(x, y)) {
                        continue;
                    }

                    // Check if there is a clear path between the two cells
                    if (is_clear_path(i, j, x, y)) {
                        visible_count++;
                    }
                }
            }

            // Calculate visibility as visible_count / 160, capped at 1.0
            float visibility = static_cast<float>(visible_count) / 160.0f;
            visibility = std::min(visibility, 1.0f); // Cap at 1.0

            // Set the visibility value in the layer
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
    // Get the map dimensions
    int map_height = terrain->get_map_height();
    int map_width = terrain->get_map_width();

    // Initialize a temporary grid to store visibility and adjacency information
    std::vector<std::vector<float>> visibility_grid(map_height, std::vector<float>(map_width, 0.0f));

    // Iterate over all cells in the grid
    for (int i = 0; i < map_height; ++i) {
        for (int j = 0; j < map_width; ++j) {
            // Skip wall cells
            if (terrain->is_wall(i, j)) {
                continue;
            }

            // Check if the cell is visible to (row, col)
            if (is_clear_path(row, col, i, j)) {
                visibility_grid[i][j] = 1.0f; // Mark as visible
            }
        }
    }

    // Iterate over all cells again to mark cells adjacent to visible cells
    for (int i = 0; i < map_height; ++i) {
        for (int j = 0; j < map_width; ++j) {
            // Skip wall cells and cells already marked as visible
            if (terrain->is_wall(i, j) || visibility_grid[i][j] == 1.0f) {
                continue;
            }

            // Check the 8 neighboring cells for visibility
            bool is_adjacent_to_visible = false;
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    if (dx == 0 && dy == 0) continue; // Skip the current cell

                    int x = i + dx;
                    int y = j + dy;

                    // Check if the neighboring cell is within bounds and visible
                    if (x >= 0 && x < map_height && y >= 0 && y < map_width && visibility_grid[x][y] == 1.0f) {
                        is_adjacent_to_visible = true;
                        break;
                    }
                }
                if (is_adjacent_to_visible) break;
            }

            // Mark the cell as 0.5 if it is adjacent to a visible cell
            if (is_adjacent_to_visible) {
                visibility_grid[i][j] = 0.5f;
            }
        }
    }

    // Copy the results to the output layer
    for (int i = 0; i < map_height; ++i) {
        for (int j = 0; j < map_width; ++j) {
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
void analyze_agent_vision(MapLayer<float> &layer, const Agent *agent)
{
    // Get the map dimensions
    int map_height = terrain->get_map_height();
    int map_width = terrain->get_map_width();

    // Get the agent's position and direction in the XZ plane
    Vec3 agent_pos = agent->get_position();
    Vec3 agent_dir = agent->get_forward_vector();

    float agent_dir_length = std::sqrt(agent_dir.x * agent_dir.x + agent_dir.z * agent_dir.z);
    Vec2 agent_dir_xz = { agent_dir.x / agent_dir_length, agent_dir.z / agent_dir_length };

    // Define the field of view (FOV) cosine threshold (slightly larger than 180 degrees)
    const float fov_cosine_threshold = -0.1f;

    // Iterate over all cells in the grid
    for (int i = 0; i < map_height; ++i) {
        for (int j = 0; j < map_width; ++j) {
            // Skip wall cells
            if (terrain->is_wall(i, j)) {
                continue;
            }

            // Get the center position of the current cell in the XZ plane
            Vec3 cell_world_pos = terrain->get_world_position(i, j);
            Vec2 cell_pos_xz = { cell_world_pos.x, cell_world_pos.z };

            // Calculate the vector from the agent to the cell in the XZ plane
            Vec2 agent_to_cell = { cell_pos_xz.x - agent_pos.x, cell_pos_xz.y - agent_pos.z };

            // Normalize the agent-to-cell vector
            float agent_to_cell_length = std::sqrt(agent_to_cell.x * agent_to_cell.x + agent_to_cell.y * agent_to_cell.y);
            if (agent_to_cell_length < 0.0001f) {
                continue; // Skip if the cell is too close to the agent
            }
            agent_to_cell.x /= agent_to_cell_length;
            agent_to_cell.y /= agent_to_cell_length;

            // Calculate the dot product between the agent's direction and the agent-to-cell vector
            float dot_product = agent_dir_xz.x * agent_to_cell.x + agent_dir_xz.y * agent_to_cell.y;

            // Check if the cell is within the agent's FOV
            if (dot_product >= fov_cosine_threshold) {
                // Check if there is a clear path from the agent to the cell
                if (is_clear_path(static_cast<int>(agent_pos.z), static_cast<int>(agent_pos.x), i, j)) {
                    // Mark the cell as 1.0 if it is visible
                    layer.set_value(i, j, 1.0f);
                }
            }
        }
    }
    // WRITE YOUR CODE HERE
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
void propagate_solo_occupancy(MapLayer<float> &layer, float decay, float growth)
{
    // Get the map dimensions
    int map_height = terrain->get_map_height();
    int map_width = terrain->get_map_width();

    // Create a temporary layer to store the updated values
    float temp_layer[40][40] = { 0.0f };

    // Iterate over all cells in the grid
    for (int i = 0; i < map_height; ++i) {
        for (int j = 0; j < map_width; ++j) {
            // Skip wall cells
            if (terrain->is_wall(i, j)) {
                temp_layer[i][j] = 0.0f; // Walls remain 0
                continue;
            }

            // Step 1: Get the value of each neighbor and apply decay factor
            float max_decayed_value = 0.0f;

            // Check all 8 neighbors
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    if (dx == 0 && dy == 0) continue; // Skip the current cell

                    int x = i + dx;
                    int y = j + dy;

                    // Ensure the neighbor is within bounds
                    if (x >= 0 && x < map_height && y >= 0 && y < map_width) {
                        float neighbor_value = layer.get_value(x, y);
                        float decayed_value = neighbor_value * decay;
                        if (decayed_value > max_decayed_value) {
                            max_decayed_value = decayed_value;
                        }
                    }
                }
            }

            // Step 2: Keep the highest value from step 1
            // Step 3: Linearly interpolate from the cell's current value to the value from step 2
            float current_value = layer.get_value(i, j);
            float new_value = lerp(current_value, max_decayed_value, growth);

            // Step 4: Store the value in the temporary layer
            temp_layer[i][j] = new_value;
        }
    }

    // Write the temporary layer back into the given layer
    for (int i = 0; i < map_height; ++i) {
        for (int j = 0; j < map_width; ++j) {
            layer.set_value(i, j, temp_layer[i][j]);
        }
    }
}

/*
    Determine the maximum value in the given layer, and then divide the value
    for every cell in the layer by that amount.  This will keep the values in the
    range of [0, 1].  Negative values should be left unmodified.
*/
void normalize_solo_occupancy(MapLayer<float> &layer)
{
    const float decay = 0.9f; // Example decay factor
    const float growth = 0.5f; // Example growth factor

    // Get the map dimensions
    int map_height = terrain->get_map_height();
    int map_width = terrain->get_map_width();

    // Create a temporary layer to store the updated values
    float temp_layer[40][40] = { 0.0f };

    // Iterate over all cells in the grid
    for (int i = 0; i < map_height; ++i) {
        for (int j = 0; j < map_width; ++j) {
            // Skip wall cells
            if (terrain->is_wall(i, j)) {
                temp_layer[i][j] = 0.0f; // Walls remain 0
                continue;
            }

            // Step 1: Get the value of each neighbor and apply decay factor
            float max_decayed_value = 0.0f;

            // Check all 8 neighbors
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    if (dx == 0 && dy == 0) continue; // Skip the current cell

                    int x = i + dx;
                    int y = j + dy;

                    // Ensure the neighbor is within bounds
                    if (x >= 0 && x < map_height && y >= 0 && y < map_width) {
                        float neighbor_value = layer.get_value(x, y);
                        float decayed_value = neighbor_value * decay;
                        if (decayed_value > max_decayed_value) {
                            max_decayed_value = decayed_value;
                        }
                    }
                }
            }

            // Step 2: Keep the highest value from step 1
            // Step 3: Linearly interpolate from the cell's current value to the value from step 2
            float current_value = layer.get_value(i, j);
            float new_value = lerp(current_value, max_decayed_value, growth);

            // Step 4: Store the value in the temporary layer
            temp_layer[i][j] = new_value;
        }
    }

    // Write the temporary layer back into the given layer
    for (int i = 0; i < map_height; ++i) {
        for (int j = 0; j < map_width; ++j) {
            layer.set_value(i, j, temp_layer[i][j]);
        }
    }
}

void enemy_field_of_view(MapLayer<float> &layer, float fovAngle, float closeDistance, float occupancyValue, AStarAgent *enemy)
{
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

    // WRITE YOUR CODE HERE
}

bool enemy_find_player(MapLayer<float> &layer, AStarAgent *enemy, Agent *player)
{
    /*
        Check if the player's current tile has a negative value, ie in the fov cone
        or within a detection radius.
    */

    const auto &playerWorldPos = player->get_position();

    const auto playerGridPos = terrain->get_grid_position(playerWorldPos);

    // verify a valid position was returned
    if (terrain->is_valid_grid_position(playerGridPos) == true)
    {
        if (layer.get_value(playerGridPos) < 0.0f)
        {
            return true;
        }
    }

    // player isn't in the detection radius or fov cone, OR somehow off the map
    return false;
}

bool enemy_seek_player(MapLayer<float> &layer, AStarAgent *enemy)
{
    /*
        Attempt to find a cell with the highest nonzero value (normalization may
        not produce exactly 1.0 due to floating point error), and then set it as
        the new target, using enemy->path_to.

        If there are multiple cells with the same highest value, then pick the
        cell closest to the enemy.

        Return whether a target cell was found.
    */

    // WRITE YOUR CODE HERE

    return false; // REPLACE THIS
}
