#include "Arduino.h"
#include "graph.h"
#include <cfloat>

float node_ad_list[N_nodes][MAX_connections];

int node_labels[N_nodes] = {4, 5, 6, 7, 8, 9, 13, 14, 15, 16, 20, 22, 40, 41, 42, 43, 23, 24, 25, 26, 36, 37, 38, 39, 30, 31, 35};


// define node connections
int node_conn[N_nodes][MAX_connections] = {
    // -1 means no connection
    {24,  2, -1, -1}, // Node 4
    { 5, 25, -1, -1}, // Node 5
    { 0,  6,  7, 24}, // Node 6
    { 6, 14, -1, -1}, // Node 7
    { 5, 13, -1, -1}, // Node 8
    { 9,  4, 25,  1}, // Node 9
    { 2,  3, 10, -1}, // Node 13
    { 2, 12, -1, -1}, // Node 14
    { 9, 15, -1, -1}, // Node 15
    { 5, 26,  8, -1}, // Node 16
    {16, 11,  6, -1}, // Node 20
    {10, 16, -1, -1}, // Node 22
    { 7, -1, -1, -1}, // Node 40
    { 4, -1, -1, -1}, // Node 41
    { 3, -1, -1, -1}, // Node 42
    { 8, -1, -1, -1}, // Node 43
    {10, 11, 17, 20}, // Node 23
    {16, 18, 21, -1}, // Node 24
    {17, 19, 26, 22}, // Node 25
    {18, 26, 23, -1}, // Node 26
    {16, -1, -1, -1}, // Node 36
    {17, -1, -1, -1}, // Node 37
    {18, -1, -1, -1}, // Node 38
    {19, -1, -1, -1}, // Node 39
    { 0, 25,  2, -1}, // Node 30
    {24,  1,  5, -1}, // Node 31
    { 9, 18, 19, -1}  // Node 35
};

// define coordinates {x,y} of each node
float node_coords[N_nodes][2] = {
    {  0.0f,  0.355f}, // Node 4
    {0.695f,  0.355f}, // Node 5
    {  0.0f,   0.15f}, // Node 6
    {  0.2f,    0.0f}, // Node 7
    {  0.5f,   0.15f}, // Node 8
    {0.695f,   0.15f}, // Node 9
    {  0.0f,    0.0f}, // Node 13
    {  0.2f,   0.15f}, // Node 14
    {  0.5f,    0.0f}, // Node 15
    {0.695f,    0.0f}, // Node 16
    {  0.0f,  -0.15f}, // Node 20
    {  0.0f, -0.355f}, // Node 22
    {0.227f,   0.15f}, // Node 40
    {0.468f,   0.15f}, // Node 41
    {0.227f,    0.0f}, // Node 42
    {0.468f,    0.0f}, // Node 43
    {0.245f, -0.355f}, // Node 23
    {0.395f, -0.355f}, // Node 24
    {0.545f, -0.355f}, // Node 25
    {0.695f, -0.355f}, // Node 26
    {0.245f, -0.385f}, // Node 36
    {0.395f, -0.385f}, // Node 37
    {0.545f, -0.385f}, // Node 38
    {0.695f, -0.385f}, // Node 39
    {0.227f,  0.355f}, // Node 30
    {0.468f,  0.355f}, // Node 31
    {0.695f,  -0.15f} // Node 35
};

int find_nearest_node(float coords[2]) {
    int final_node;
    float saved_distance = FLT_MAX;
    float current_distance = FLT_MAX;
    for (int i = 0; i < N_nodes; i++) {
        current_distance = opt_cost_to_go(node_coords[i], coords);
        if (current_distance < saved_distance) {
            final_node = i;
            saved_distance = current_distance;
        }
    }
    return final_node;
}

void selection_sort(const int size, int idx[], float array[]) {
    // Create index array
    for (int i = 0; i < size; i++) {idx[i] = i;}

    // Variables needed for algorithm
    float temp1, temp2;
    int temp_idx;

    // Sort using the selection sort algorithm
    for (int i = 0; i < size - 1; i++) {
        // Find index for smallest value
        int min_idx = i;
        for (int j = i + 1; j < size; j++) {
            temp1 = array[idx[j]];
            temp2 = array[idx[min_idx]];
            if (temp1 < temp2) {
                min_idx = j;
            }
        }
        // Swap with current element
        temp_idx = idx[min_idx];
        idx[min_idx] = idx[i];
        idx[i] = temp_idx;
    }
}

int find_node_idx_by_label(int label) {
    for (int i = 0; i < N_nodes; i++) {
        if (node_labels[i] == label) {
            return i;
        }
    }
    return -1;
}

void obtain_ad_list(float list[N_nodes][MAX_connections]) {
    // make every value zero
    for (int i = 0; i < N_nodes; i++) {
        for (int j = 0; j < MAX_connections; j++) {
            list[i][j] = 0.0;
        }
    }

    // for every connection, insert cost in list
    //int idx_current_conn[MAX_connections];
    for (int i = 0; i < N_nodes; i++) {
        // // translate labels to indices in connections
        // for (int idx = 0; idx < MAX_connections; idx++) {
        //     if (node_conn[i][idx] != -1) {
        //         idx_current_conn[idx] = find_node_idx_by_label(node_conn[i][idx]);
        //     } else {
        //         idx_current_conn[idx] = -1;
        //     }
        // }
        // go through connections
        for (int j = 0; j < MAX_connections; j++) {
            if (node_conn[i][j] != -1) { // (idx_current_conn[j] != -1)
                int nbr_idx = node_conn[i][j];
                list[i][j] = opt_cost_to_go(node_coords[i], node_coords[nbr_idx]); // cost is the length + we can add other contributions
            } else {
                list[i][j] = 0.0;
            }
        }
    }
}

float opt_cost_to_go(const float node_coord[2], const float stop_node_coord[2]) {
    float dist_x = node_coord[0] - stop_node_coord[0];
    float dist_y = node_coord[1] - stop_node_coord[1];
    return sqrt(dist_x * dist_x + dist_y * dist_y);
}

int is_array_zero(const int size, const int array[]) {
    int result = 1;
    for (int i = 0; i < size; i++) {
        if (array[i] != 0) {
            result = 0;
        }
    }
    return result;
}

int a_star(const int start_idx, const int stop_idx, int final_path[N_nodes]) {
    // define auxiliary arrays
    int open_nodes[N_nodes];       // keep track of which nodes to explore next
    int closed_nodes[N_nodes];     // keep track of which nodes have already been explored
    float past_cost[N_nodes];      // cost of each node's best path
    float est_total_cost[N_nodes]; // cost of each node's best path + estimated cost to go to goal
    int node_parent[N_nodes];      // parent of node in the current best path
    for (int i = 0; i < N_nodes; i++) {
        open_nodes[i] = 0;           // 0 to ignore node, 1 to explore node
        closed_nodes[i] = 0;         // 0 for open node,  1 for closed node
        past_cost[i] = FLT_MAX;      // infinite cost for not-explored nodes
        est_total_cost[i] = FLT_MAX; // infinite cost for not-explored nodes
        node_parent[i] = -1;         // -1 for no parent
        final_path[i] = -1;          // -1 for empty entry
    }

    // define start node
    int current_idx = start_idx;
    past_cost[current_idx] = 0;
    open_nodes[current_idx] = 1;
    est_total_cost[current_idx] = past_cost[current_idx] + opt_cost_to_go(node_coords[current_idx], node_coords[stop_idx]);

    // algorithm logic
    while (!is_array_zero(N_nodes, open_nodes)) {
        // end the search if we are arrive in the stop node
        if (current_idx == stop_idx) {
            // obtain the amount of nodes in final path
            int size_final = 0;
            int idx = stop_idx;
            while (idx != start_idx) {
                idx = node_parent[idx];
                size_final += 1;
            }
            // insert nodes in final_path in order of start node -> stop node
            idx = stop_idx;
            int node_pos = 0;
            while (idx != start_idx) {
                final_path[size_final - node_pos] = idx;
                idx = node_parent[idx];
                node_pos += 1;
            }
            final_path[size_final - node_pos] = idx;
            return 1;
        }

        // find neighbors of current node and explore
        for (int con_idx = 0; con_idx < MAX_connections; con_idx++) {
            // needs to be connected to current node, and not closed
            if (node_ad_list[current_idx][con_idx] > 0.0) {
                // int nbr_idx = find_node_idx_by_label(node_conn[current_idx][con_idx]);
                int nbr_idx = node_conn[current_idx][con_idx];
                if (closed_nodes[nbr_idx] == 0) {
                    // if the neighbor node has no parent, update its cost and parent to the current node
                    if (node_parent[nbr_idx] == -1) {
                        past_cost[nbr_idx] = past_cost[current_idx] + node_ad_list[current_idx][con_idx];
                        node_parent[nbr_idx] = current_idx;
                        open_nodes[nbr_idx] = 1;
                        est_total_cost[nbr_idx] = past_cost[nbr_idx] + opt_cost_to_go(node_coords[nbr_idx], node_coords[stop_idx]);
                    } else {
                        // if the total cost to go to the neighbor node, passing through the current node is better than the previous best path to the neighbor node
                        // then set the new best path to the one going through the current node
                        float new_cost = past_cost[current_idx] + node_ad_list[current_idx][con_idx];
                        if (new_cost < past_cost[nbr_idx]) {
                            past_cost[nbr_idx] = new_cost;
                            node_parent[nbr_idx] = current_idx;
                            open_nodes[nbr_idx] = 1;
                            est_total_cost[nbr_idx] = past_cost[nbr_idx] + opt_cost_to_go(node_coords[nbr_idx], node_coords[stop_idx]);
                        }
                    }
                }
            } 
        }
        
        // close current node
        closed_nodes[current_idx] = 1;
        open_nodes[current_idx] = 0;
        // sort the nodes by lowest estimated total cost
        int sorted_idx[N_nodes];
        selection_sort(N_nodes, sorted_idx, est_total_cost);
        // select the first node that is still open and not closed
        bool found_next = false;
        for (int i = 0; i < N_nodes; i++) {
            int idx = sorted_idx[i];
            if (closed_nodes[idx] == 0 && open_nodes[idx] == 1) {
                current_idx = idx;
                found_next = true;
                break;
            }
        }
        if (!found_next) {
            break;
        }
    }
    return 0; // failed to find a path
}

