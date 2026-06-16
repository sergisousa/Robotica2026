#include "Arduino.h"
#include "graph.h"
#include <cfloat>

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
                list[i][j] = opt_cost_to_go(node_coords[i], node_coords[j]); // cost is the length + we can add other contributions
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
        // select the first node that is not closed
        for (int i = 0; i < N_nodes; i++) {
            if (closed_nodes[sorted_idx[i]] == 0) {
                current_idx = sorted_idx[i];
                break;
            }
        }
    }
    return 0; // failed to find a path
}

