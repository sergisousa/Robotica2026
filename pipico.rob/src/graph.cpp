#include "Arduino.h"
#include "graph.h"
#include <cfloat>

float Rotation_Weight = 1.0;
float node_thresh = 0.03;

int blocked_nodes[N_blocked];

// define node connections
int node_conn[N_nodes + 1][MAX_connections] = {
    // -1 means no connection
    {20,  2, -1, -1}, // Node 4
    { 5, 21, -1, -1}, // Node 5
    { 0,  6,  7, 20}, // Node 6
    { 6, -1, -1, -1}, // Node 7
    { 5, -1, -1, -1}, // Node 8
    { 9,  4, 21,  1}, // Node 9
    { 2,  3, 10, -1}, // Node 13
    { 2, -1, -1, -1}, // Node 14
    { 9, -1, -1, -1}, // Node 15
    { 5, 22,  8, -1}, // Node 16
    {12, 11,  6, -1}, // Node 20
    {10, 12, -1, -1}, // Node 22
    {10, 11, 13, 16}, // Node 23
    {12, 14, 17, -1}, // Node 24
    {13, 15, 22, 18}, // Node 25
    {14, 22, 19, -1}, // Node 26
    {12, -1, -1, -1}, // Node 36
    {13, -1, -1, -1}, // Node 37
    {14, -1, -1, -1}, // Node 38
    {15, -1, -1, -1}, // Node 39
    { 0, 21,  2, -1}, // Node 30
    {20,  1,  5, -1}, // Node 31
    { 9, 14, 15, -1},  // Node 35
    {-1, -1, -1, -1}  // Initial node
};

// define coordinates {x,y} of each node
float node_coords[N_nodes + 1][2] = {
    {  0.0f,  0.355f}, // Node 4
    {0.695f,  0.355f}, // Node 5
    {  0.0f,   0.15f}, // Node 6
    {0.127f,    0.0f}, // Node 7
    {0.567f,   0.15f}, // Node 8
    {0.695f,   0.15f}, // Node 9
    {  0.0f,    0.0f}, // Node 13
    {0.127f,   0.15f}, // Node 14
    {0.567f,    0.0f}, // Node 15
    {0.695f,    0.0f}, // Node 16
    {  0.0f,  -0.15f}, // Node 20
    {  0.0f, -0.315f}, // Node 22
    // {0.227f,   0.15f}, // Node 40
    // {0.468f,   0.15f}, // Node 41
    // {0.227f,    0.0f}, // Node 42
    // {0.468f,    0.0f}, // Node 43
    {0.245f, -0.250f}, // Node 23
    {0.395f, -0.250f}, // Node 24
    {0.545f, -0.250f}, // Node 25
    {0.695f, -0.250f}, // Node 26
    {0.245f, -0.355f}, // Node 36
    {0.395f, -0.355f}, // Node 37
    {0.545f, -0.355f}, // Node 38
    {0.695f, -0.355f}, // Node 39
    {0.227f,  0.355f}, // Node 30
    {0.468f,  0.355f}, // Node 31
    {0.695f,  -0.15f}, // Node 35
    {0.0f, 0.0f} // Initial node
};

int node_conn_layered[N_layers * (N_nodes + 1) + 1][MAX_connections_layer];
float node_ad_list_layered[N_layers * (N_nodes + 1) + 1][MAX_connections_layer];

// store the direction of each node in each layer
float node_theta_layers[(N_nodes + 1) * N_layers + 1];

// function to fill blocked_nodes
void fill_blocked_nodes(){
    int idx = 0;
    for (int i = 0; i < N_nodes; i++){
        int sum = 0;
        for (int j = 0; j < MAX_connections; j++){
            if (node_conn[i][j] != - 1){
                sum += 1;;
            } else {
                j = MAX_connections;
            }
        }
        if (sum == 1){
            blocked_nodes[idx] = i;
            idx += 1;
        }
    }
}

// function to check if an element exists in an array
int array_has_element(int arr[], int size, int element) {
    for (int i = 0; i < size; i++) {
        if (arr[i] == element) {
            return 1; // Element found
        }
    }
    return 0; // Element not found
}

// function to fill array with a certain value
void fill_array(int array[], int value, int size) {
    for (int i = 0; i < size; i++) {
        array[i] = value;
    }
}

// function to fill initial node and its connections to the 3 closest nodes
void initial_node(float coords[2]){
    // empty initial node connections
    fill_array(node_conn[N_nodes], -1, MAX_connections);

    // fill coordinates of initial node
    node_coords[N_nodes][0] = coords[0];
    node_coords[N_nodes][1] = coords[1];

    // fill connections of initial node to the 3 closest nodes
    // caculate distances from initial node to all other nodes
    float node_distances[N_nodes];
    for (int i = 0; i < N_nodes; i++){
        node_distances[i] = opt_cost_to_go(node_coords[i], coords);
    }
    // determine the 3 closest nodes that are not blocked
    int sort_idx[N_nodes];
    int conn_idx = 0;
    selection_sort(N_nodes, sort_idx, node_distances);

    if (array_has_element(blocked_nodes, N_blocked, sort_idx[0]) && opt_cost_to_go(node_coords[sort_idx[0]], coords) < node_thresh){
        node_conn[N_nodes][conn_idx] = sort_idx[0];
    } else {
        for (int i = 0; i < N_nodes; i++){
            if (array_has_element(blocked_nodes, N_blocked, sort_idx[i]) == 0){
                node_conn[N_nodes][conn_idx] = sort_idx[i];
                conn_idx += 1;
            }
            if (conn_idx == MAX_connections_layer){
                break;
            }
        }
    }    
}


// function to generate layered graph from not-layered graph
void generate_graph_with_layers(float robot_theta) {
    // initialize arrays
    for (int i = 0; i < ((N_nodes + 1) * N_layers + 1); i++) {
        node_theta_layers[i] = FLT_MAX;       // unoccupied layers have an infinite value (as high as the variable type will go)
    }
    for (int i = 0; i < ((N_nodes + 1) * N_layers + 1); i++) {
        for (int j = 0; j < MAX_connections_layer; j++) {
            node_conn_layered[i][j] = -1;     // empty connections have value -1
            node_ad_list_layered[i][j] = 0.0; // empty connections have weight 0.0
        }
    }

    int nbr_idx;
    float theta;
    bool already_has_layer;
    bool nbr_is_blocked;
    int curr_layer_idx;

    // iterate through nodes
    for (int curr_idx = 0; curr_idx < N_nodes + 1; curr_idx++) { // curr_idx -> index of current node in not-layered graph
        // iterate through node connections
        for (int con_idx = 0; con_idx < MAX_connections; con_idx++) { // conn_idx -> index of connection of current node in not-layered graph
            nbr_idx = node_conn[curr_idx][con_idx];
            
            // if a connection exists, and the node is not blocked
            if (nbr_idx != -1 && !array_has_element(blocked_nodes, 8, curr_idx)) {
                // calculate the angle of the connection
                theta = atan2(node_coords[nbr_idx][1] - node_coords[curr_idx][1], node_coords[nbr_idx][0] - node_coords[curr_idx][0]); // returns angle in range pi to -pi

                // check if the current node already has a layer with the same orientation
                already_has_layer = false;
                for (int layer_idx = 0; layer_idx < N_layers; layer_idx++) { // layer_idx -> index of the current node's layer in layered graph
                    if (fabs(dif_angle1(node_theta_layers[N_layers * curr_idx + layer_idx], theta)) < theta_thresh) {
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

                // check if neighbor is a blocked node
                nbr_is_blocked = array_has_element(blocked_nodes, 8, nbr_idx);
                
                // check if the neighbor node already has a layer with the same orientation
                already_has_layer = false;
                for (int layer_idx = 0; layer_idx < N_layers; layer_idx++) { // layer_idx -> index of the neighbor node's layer in layered graph
                    if (fabs(dif_angle1(node_theta_layers[N_layers * nbr_idx + layer_idx], theta)) < theta_thresh) {
                        // check for an empty connection slot, and apply connection from current new node to neighbor's new node
                        for (int conn_layer_idx = 0; conn_layer_idx < MAX_connections_layer; conn_layer_idx++) { // conn_layer_idx -> index of connection of current node in layered graph
                            if (node_conn_layered[N_layers * curr_idx + curr_layer_idx][conn_layer_idx] == -1) {
                                node_conn_layered[N_layers * curr_idx + curr_layer_idx][conn_layer_idx] = N_layers * nbr_idx + layer_idx;

                                // apply translation cost to connection
                                node_ad_list_layered[N_layers * curr_idx + curr_layer_idx][conn_layer_idx] = opt_cost_to_go(node_coords[curr_idx], node_coords[nbr_idx]);
                                break;
                            }
                        }
                        // if neighbor is a blocked node, create a connection from the neighbor back to the current node
                        if (nbr_is_blocked) {
                            for (int conn_layer_idx = 0; conn_layer_idx < MAX_connections_layer; conn_layer_idx++) { // conn_layer_idx -> index of connection of current node in layered graph
                                if (node_conn_layered[N_layers * nbr_idx + layer_idx][conn_layer_idx] == -1) {
                                    node_conn_layered[N_layers * nbr_idx + layer_idx][conn_layer_idx] = N_layers * curr_idx + curr_layer_idx;

                                    // apply translation cost to connection
                                    node_ad_list_layered[N_layers * nbr_idx + layer_idx][conn_layer_idx] = opt_cost_to_go(node_coords[curr_idx], node_coords[nbr_idx]);
                                    break;
                                }
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

                            // if neighbor is a blocked node, create a connection from the neighbor back to the current node
                            if (nbr_is_blocked) {
                                for (int conn_layer_idx = 0; conn_layer_idx < MAX_connections_layer; conn_layer_idx++) { // conn_layer_idx -> index of connection of current node in layered graph
                                    if (node_conn_layered[N_layers * nbr_idx + layer_idx][conn_layer_idx] == -1) {
                                        node_conn_layered[N_layers * nbr_idx + layer_idx][conn_layer_idx] = N_layers * curr_idx + curr_layer_idx;

                                        // apply translation cost to connection
                                        node_ad_list_layered[N_layers * nbr_idx + layer_idx][conn_layer_idx] = opt_cost_to_go(node_coords[curr_idx], node_coords[nbr_idx]);
                                        break;
                                    }
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
    for (int curr_idx = 0; curr_idx < N_nodes + 1; curr_idx++) {
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
                            ang_difference = dif_angle1(node_theta_layers[N_layers * curr_idx + layer_idx], node_theta_layers[N_layers * curr_idx + nbr_left_idx]);
                            node_ad_list_layered[N_layers * curr_idx + layer_idx][conn_idx] = Rotation_Weight * fabs(ang_difference);
                        }
                        // if there is more than 2 layers -> connect to the other neighbor node
                        if (num_layers > 2) {
                            node_conn_layered[N_layers * curr_idx + layer_idx][conn_idx + 1] = N_layers * curr_idx + nbr_right_idx;
                            ang_difference = dif_angle1(node_theta_layers[N_layers * curr_idx + layer_idx], node_theta_layers[N_layers * curr_idx + nbr_right_idx]);
                            node_ad_list_layered[N_layers * curr_idx + layer_idx][conn_idx + 1] = Rotation_Weight * fabs(ang_difference);
                        }

                        break;
                    }
                }
            }
        }
    }

    // calculate rotation cost for last layer in initial node (layer with the robot's orientation)
    int end_node_idx = N_nodes;
    int end_layer_idx = N_layers;
    int end_sort_idx[N_layers + 1];
    float end_curr_layer_theta[N_layers + 1];
    node_theta_layers[N_layers * end_node_idx + end_layer_idx] = robot_theta;
    // transfer theta value of layers to an isolated array to be sorted, and also finding the index of the last occupied layer (which will be useful later)
    idx_last_layer = -2;
    for (int layer_idx = 0; layer_idx < N_layers + 1; layer_idx++) {
        end_curr_layer_theta[layer_idx] = node_theta_layers[N_layers * end_node_idx + layer_idx];

        // if we go after the last layer (infinite value in theta) and we haven't yet found the last layer
        if ((node_theta_layers[N_layers * end_node_idx + layer_idx] > 2 * M_PI) && idx_last_layer == -2) {
            idx_last_layer = layer_idx - 1;
        }
    }
    // if all layers are occupied -> idx_last_layer = -2
    if (idx_last_layer == -2) {
        idx_last_layer = N_layers - 1;
    }
    idx_last_layer += 1;

    // if idx_last_layer = -1 it means this node has no layers (and no connections), so we ignore it
    if (idx_last_layer != -1) {
        num_layers = idx_last_layer + 1; // number of occupied layers in this node
        // sort layers by order of orientation
        selection_sort(N_layers + 1, end_sort_idx, end_curr_layer_theta);

        // find where it is in the sorted array
        for (int i = 0; i < N_layers + 1; i++) {
            if (end_sort_idx[i] == end_layer_idx) {
                idx_in_sorted_array = i;
                break;
            }
        }

        // find index of the two nodes with closest orientation (to the right and the left of the current nodes in the sorted array)
        if (idx_in_sorted_array - 1 < 0) {
            nbr_left_idx = end_sort_idx[idx_last_layer];
        } else {
            nbr_left_idx = end_sort_idx[idx_in_sorted_array - 1];
        }
        if (idx_in_sorted_array == idx_last_layer) {
            nbr_right_idx = end_sort_idx[0];
        } else {
            nbr_right_idx = end_sort_idx[idx_in_sorted_array + 1];
        }
        
        // check for an empty connection slot, create a connection and apply a rotation cost to the neighbor nodes
        for (int conn_idx = 0; conn_idx < MAX_connections_layer; conn_idx++) {
            if (node_conn_layered[N_layers * end_node_idx + end_layer_idx][conn_idx] == -1) {
                // if there is more than 1 layer -> connect to the neighbor node
                if (num_layers > 1) {
                    node_conn_layered[N_layers * end_node_idx + end_layer_idx][conn_idx] = N_layers * end_node_idx + nbr_left_idx;
                    ang_difference = dif_angle1(node_theta_layers[N_layers * end_node_idx + end_layer_idx], node_theta_layers[N_layers * end_node_idx + nbr_left_idx]);
                    node_ad_list_layered[N_layers * end_node_idx + end_layer_idx][conn_idx] = Rotation_Weight * fabs(ang_difference);
                }
                // if there is more than 2 layers -> connect to the other neighbor node
                if (num_layers > 2) {
                    node_conn_layered[N_layers * end_node_idx + end_layer_idx][conn_idx + 1] = N_layers * end_node_idx + nbr_right_idx;
                    ang_difference = dif_angle1(node_theta_layers[N_layers * end_node_idx + end_layer_idx], node_theta_layers[N_layers * end_node_idx + nbr_right_idx]);
                    node_ad_list_layered[N_layers * end_node_idx + end_layer_idx][conn_idx + 1] = Rotation_Weight * fabs(ang_difference);
                }

                break;
            }
        }
    }
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

float opt_cost_to_go(const float node_coord[2], const float stop_node_coord[2]) {
    float dist_x = node_coord[0] - stop_node_coord[0];
    float dist_y = node_coord[1] - stop_node_coord[1];
    return sqrt(dist_x * dist_x + dist_y * dist_y);
}

float normalize_angle1(float angle)
{
  if (fabs(angle) < M_PI)
    return angle;

  if (angle >= 0) {
    angle = fmod(angle + PI, TWO_PI);
    return angle - PI;
  } else {
    angle = fmod(-angle + PI, TWO_PI);
    return -(angle - PI);
  }
}

float dif_angle1(float a0, float a1)
{
  return normalize_angle1(normalize_angle1(a0) - normalize_angle1(a1));
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

int a_star(const int start_idx, const int stop_idx, int final_path[(N_nodes + 1) * N_layers + 1]) {
    // define auxiliary arrays
    int open_nodes[(N_nodes + 1) * N_layers + 1];       // keep track of which nodes to explore next
    int closed_nodes[(N_nodes + 1) * N_layers + 1];     // keep track of which nodes have already been explored
    float past_cost[(N_nodes + 1) * N_layers + 1];      // cost of each node's best path
    float est_total_cost[(N_nodes + 1) * N_layers + 1]; // cost of each node's best path + estimated cost to go to goal
    int node_parent[(N_nodes + 1) * N_layers + 1];      // parent of node in the current best path
    for (int i = 0; i < (N_nodes + 1) * N_layers + 1; i++) {
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
    int dist_idx = current_idx / N_layers;
    if (dist_idx > N_nodes) {
        dist_idx -= 1;
    }
    est_total_cost[current_idx] = past_cost[current_idx] + opt_cost_to_go(node_coords[dist_idx], node_coords[stop_idx / N_layers]);

    // algorithm logic
    while (!is_array_zero((N_nodes + 1) * N_layers + 1, open_nodes)) {
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
                        dist_idx = nbr_idx / N_layers;
                        if (dist_idx > N_nodes) {
                            dist_idx -= 1;
                        }
                        est_total_cost[nbr_idx] = past_cost[nbr_idx] + opt_cost_to_go(node_coords[dist_idx], node_coords[stop_idx / N_layers]);
                    } else {
                        // if the total cost to go to the neighbor node, passing through the current node is better than the previous best path to the neighbor node
                        // then set the new best path to the one going through the current node
                        float new_cost = past_cost[current_idx] + node_ad_list_layered[current_idx][con_idx];
                        if (new_cost < past_cost[nbr_idx]) {
                            past_cost[nbr_idx] = new_cost;
                            node_parent[nbr_idx] = current_idx;
                            open_nodes[nbr_idx] = 1;
                            dist_idx = nbr_idx / N_layers;
                            if (dist_idx > N_nodes) {
                                dist_idx -= 1;
                            }
                            est_total_cost[nbr_idx] = past_cost[nbr_idx] + opt_cost_to_go(node_coords[dist_idx], node_coords[stop_idx / N_layers]);
                        }
                    }
                }
            } 
        }
        
        // close current node
        closed_nodes[current_idx] = 1;
        open_nodes[current_idx] = 0;
        // sort the nodes by lowest estimated total cost
        int sorted_idx[(N_nodes + 1) * N_layers + 1];
        selection_sort((N_nodes + 1) * N_layers + 1, sorted_idx, est_total_cost);
        // select the first node that is still open and not closed
        bool found_next = false;
        for (int i = 0; i < (N_nodes + 1) * N_layers + 1; i++) {
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

