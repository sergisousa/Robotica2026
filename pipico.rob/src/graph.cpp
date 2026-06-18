#include "Arduino.h"
#include "graph.h"
#include <cfloat>

float node_ad_list[N_nodes][MAX_connections];

int node_labels[N_nodes] = {4, 5, 6, 7, 8, 9, 13, 14, 15, 16, 20, 22, 40, 41, 42, 43, 23, 24, 25, 26, 36, 37, 38, 39, 30, 31, 35};

float Rotation_Weight = 1.0;

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

int node_conn_layered[N_layers * N_nodes][MAX_connections_layer];
float node_ad_list_layered[N_layers * N_nodes][MAX_connections_layer];

// store the direction of each node in each layer
float node_theta_layers[N_nodes * N_layers];

void generate_graph_with_layers() {
    // initialize arrays
    for (int i = 0; i < N_nodes * N_layers; i++) {
        node_theta_layers[i] = FLT_MAX;       // unoccupied layers have an infinite value (as high as the variable type will go)
    }
    for (int i = 0; i < N_nodes * N_layers; i++) {
        for (int j = 0; j < MAX_connections_layer; j++) {
            node_conn_layered[i][j] = -1;     // empty connections have value -1
            node_ad_list_layered[i][j] = 0.0; // empty connections have weight 0.0
        }
    }

    int nbr_idx;
    float theta;
    bool already_has_layer;
    int curr_layer_idx;

    // iterate through nodes
    for (int curr_idx = 0; curr_idx < N_nodes; curr_idx++) { // curr_idx -> index of current node in not-layered graph
        // iterate through node connections
        for (int con_idx = 0; con_idx < MAX_connections_layer; con_idx++) { // conn_idx -> index of connection of current node in not-layered graph
            nbr_idx = node_conn[curr_idx][con_idx];
            
            // if a connection exists
            if (nbr_idx != -1) {
                // calculate the angle of the connection
                theta = atan2(node_coords[nbr_idx][1] - node_coords[curr_idx][1], node_coords[nbr_idx][0] - node_coords[curr_idx][0]); // returns angle in range pi to -pi

                // check if the current node already has a layer with the same orientation
                already_has_layer = false;
                for (int layer_idx = 0; layer_idx < N_layers; layer_idx++) { // layer_idx -> index of the current node's layer in layered graph
                    if (fabs(node_theta_layers[N_layers * curr_idx + layer_idx] - theta) < theta_thresh) {
                        // store current node's layer
                        curr_layer_idx = layer_idx;
                        already_has_layer = true;
                        break;
                    }
                }

                // if not, find an unoccupied layer in current node and create it
                if (!already_has_layer) {
                    for (int layer_idx = 0; layer_idx < N_layers; layer_idx++) { // layer_idx -> index of the current node's layer in layered graph
                        if (node_theta_layers[N_layers * curr_idx + layer_idx] > 2 * M_PI) {
                            // define layer direction
                            node_theta_layers[N_layers * curr_idx + layer_idx] = theta;
                            // store current node's layer
                            curr_layer_idx = layer_idx;
                            break;
                        }
                    }
                }
                
                // check if the neighbor node already has a layer with the same orientation
                already_has_layer = false;
                for (int layer_idx = 0; layer_idx < N_layers; layer_idx++) { // layer_idx -> index of the neighbor node's layer in layered graph
                    if (fabs(node_theta_layers[N_layers * nbr_idx + layer_idx] - theta) < theta_thresh) {
                        // check for an empty connection slot, and apply connection from current new node to neighbor's new node
                        for (int conn_layer_idx = 0; conn_layer_idx < MAX_connections_layer; conn_layer_idx++) { // conn_layer_idx -> index of connection of current node in layered graph
                            if (node_conn_layered[N_layers * curr_idx + curr_layer_idx][conn_layer_idx] == -1) {
                                node_conn_layered[N_layers * curr_idx + curr_layer_idx][conn_layer_idx] = N_layers * nbr_idx + layer_idx;

                                // apply translation cost to connection
                                node_ad_list_layered[N_layers * curr_idx + curr_layer_idx][conn_layer_idx] = opt_cost_to_go(node_coords[curr_idx], node_coords[nbr_idx]);
                                break;
                            }
                        }
                        already_has_layer = true;
                        break;
                    }
                }

                // if not, find an unoccupied layer in neighbor node and create it
                if (!already_has_layer) {
                    for (int layer_idx = 0; layer_idx < N_layers; layer_idx++) { // layer_idx -> index of the neighbor node's layer in layered graph
                        if (node_theta_layers[N_layers * nbr_idx + layer_idx] > 2 * M_PI) {
                            // define layer direction
                            node_theta_layers[N_layers * nbr_idx + layer_idx] = theta;

                            // check for an empty connection slot, and apply connection from current new node to neighbor's new node
                            for (int conn_layer_idx = 0; conn_layer_idx < MAX_connections_layer; conn_layer_idx++) { // conn_layer_idx -> index of connection of current node in layered graph
                                if (node_conn_layered[N_layers * curr_idx + curr_layer_idx][conn_layer_idx] == -1) {
                                    node_conn_layered[N_layers * curr_idx + curr_layer_idx][conn_layer_idx] = N_layers * nbr_idx + layer_idx;

                                    // apply translation cost to connection
                                    node_ad_list_layered[N_layers * curr_idx + curr_layer_idx][conn_layer_idx] = opt_cost_to_go(node_coords[curr_idx], node_coords[nbr_idx]);
                                    break;
                                }
                            }
                            break;
                        }
                    }
                }
            }
        }
    }

    float curr_layer_theta[N_layers];
    float ang_difference;
    int sort_idx[N_layers];
    int idx_last_layer;
    int num_layers;
    int idx_in_sorted_array;
    int nbr_left_idx;
    int nbr_right_idx;

    // go through all connections in the same layer and introduce rotation cost
    for (int curr_idx = 0; curr_idx < N_nodes; curr_idx++) {
        // transfer theta value of layers to an isolated array to be sorted, and also finding the index of the last occupied layer (which will be useful later)
        idx_last_layer = -2;
        for (int layer_idx = 0; layer_idx < N_layers; layer_idx++) {
            curr_layer_theta[layer_idx] = node_theta_layers[N_layers * curr_idx + layer_idx];

            // if we go after the last layer (infinite value in theta) and we haven't yet found the last layer
            if ((node_theta_layers[N_layers * curr_idx + layer_idx] > 2 * M_PI) && idx_last_layer == -2) {
                idx_last_layer = layer_idx - 1;
            }
        }
        // if all layers are occupied -> idx_last_layer = -2
        if (idx_last_layer == -2) {
            idx_last_layer = N_layers - 1;
        }

        // if idx_last_layer = -1 it means this node has no layers (and no connections), so we ignore it
        if (idx_last_layer != -1) {
            num_layers = idx_last_layer + 1; // number of occupied layers in this node
            // sort layers by order of orientation
            selection_sort(N_layers, sort_idx, curr_layer_theta);

            // go through each occupied layer
            for (int layer_idx = 0; layer_idx < num_layers; layer_idx++) {
                // find where it is in the sorted array
                for (int i = 0; i < N_layers; i++) {
                    if (sort_idx[i] == layer_idx) {
                        idx_in_sorted_array = i;
                        break;
                    }
                }

                // find index of the two nodes with closest orientation (to the right and the left of the current nodes in the sorted array)
                if (idx_in_sorted_array - 1 < 0) {
                    nbr_left_idx = sort_idx[idx_last_layer];
                } else {
                    nbr_left_idx = sort_idx[idx_in_sorted_array - 1];
                }
                if (idx_in_sorted_array == idx_last_layer) {
                    nbr_right_idx = sort_idx[0];
                } else {
                    nbr_right_idx = sort_idx[idx_in_sorted_array + 1];
                }
                
                // check for an empty connection slot, create a connection and apply a rotation cost to the neighbor nodes
                for (int conn_idx = 0; conn_idx < MAX_connections_layer; conn_idx++) {
                    if (node_conn_layered[N_layers * curr_idx + layer_idx][conn_idx] == -1) {
                        // if there is more than 1 layer -> connect to the neighbor node
                        if (num_layers > 1) {
                            node_conn_layered[N_layers * curr_idx + layer_idx][conn_idx] = N_layers * curr_idx + nbr_left_idx;
                            ang_difference = node_theta_layers[N_layers * curr_idx + layer_idx] - node_theta_layers[N_layers * curr_idx + nbr_left_idx];
                            if (ang_difference >  M_PI) {
                                ang_difference -= 2 * M_PI;
                            }
                            if (ang_difference < -M_PI) {
                                ang_difference += 2 * M_PI;
                            }
                            node_ad_list_layered[N_layers * curr_idx + layer_idx][conn_idx] = Rotation_Weight * fabs(ang_difference);
                        }
                        // if there is more than 2 layers -> connect to the other neighbor node
                        if (num_layers > 2) {
                            node_conn_layered[N_layers * curr_idx + layer_idx][conn_idx + 1] = N_layers * curr_idx + nbr_right_idx;
                            ang_difference = node_theta_layers[N_layers * curr_idx + layer_idx] - node_theta_layers[N_layers * curr_idx + nbr_right_idx];
                            if (ang_difference >  M_PI) {
                                ang_difference -= 2 * M_PI;
                            }
                            if (ang_difference < -M_PI) {
                                ang_difference += 2 * M_PI;
                            }
                            node_ad_list_layered[N_layers * curr_idx + layer_idx][conn_idx + 1] = Rotation_Weight * fabs(ang_difference);
                        }

                        break;
                    }
                }
            }
        }
    }
}

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
    return N_layers*final_node;
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
    for (int i = 0; i < N_nodes; i++) {
        // go through connections
        for (int j = 0; j < MAX_connections; j++) {
            if (node_conn[i][j] != -1) {
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

int a_star(const int start_idx, const int stop_idx, int final_path[N_nodes * N_layers]) {
    // define auxiliary arrays
    int open_nodes[N_nodes * N_layers];       // keep track of which nodes to explore next
    int closed_nodes[N_nodes * N_layers];     // keep track of which nodes have already been explored
    float past_cost[N_nodes * N_layers];      // cost of each node's best path
    float est_total_cost[N_nodes * N_layers]; // cost of each node's best path + estimated cost to go to goal
    int node_parent[N_nodes * N_layers];      // parent of node in the current best path
    for (int i = 0; i < N_nodes * N_layers; i++) {
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
    est_total_cost[current_idx] = past_cost[current_idx] + opt_cost_to_go(node_coords[current_idx / N_layers], node_coords[stop_idx / N_layers]);

    // algorithm logic
    while (!is_array_zero(N_nodes * N_layers, open_nodes)) {
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
        for (int con_idx = 0; con_idx < MAX_connections_layer; con_idx++) {
            // needs to be connected to current node, and not closed
            if (node_ad_list_layered[current_idx][con_idx] > 0.0) {
                int nbr_idx = node_conn_layered[current_idx][con_idx];
                if (closed_nodes[nbr_idx] == 0) {
                    // if the neighbor node has no parent, update its cost and parent to the current node
                    if (node_parent[nbr_idx] == -1) {
                        past_cost[nbr_idx] = past_cost[current_idx] + node_ad_list_layered[current_idx][con_idx];
                        node_parent[nbr_idx] = current_idx;
                        open_nodes[nbr_idx] = 1;
                        est_total_cost[nbr_idx] = past_cost[nbr_idx] + opt_cost_to_go(node_coords[nbr_idx / N_layers], node_coords[stop_idx / N_layers]);
                    } else {
                        // if the total cost to go to the neighbor node, passing through the current node is better than the previous best path to the neighbor node
                        // then set the new best path to the one going through the current node
                        float new_cost = past_cost[current_idx] + node_ad_list_layered[current_idx][con_idx];
                        if (new_cost < past_cost[nbr_idx]) {
                            past_cost[nbr_idx] = new_cost;
                            node_parent[nbr_idx] = current_idx;
                            open_nodes[nbr_idx] = 1;
                            est_total_cost[nbr_idx] = past_cost[nbr_idx] + opt_cost_to_go(node_coords[nbr_idx / N_layers], node_coords[stop_idx / N_layers]);
                        }
                    }
                }
            } 
        }
        
        // close current node
        closed_nodes[current_idx] = 1;
        open_nodes[current_idx] = 0;
        // sort the nodes by lowest estimated total cost
        int sorted_idx[N_nodes * N_layers];
        selection_sort(N_nodes * N_layers, sorted_idx, est_total_cost);
        // select the first node that is still open and not closed
        bool found_next = false;
        for (int i = 0; i < N_nodes * N_layers; i++) {
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

