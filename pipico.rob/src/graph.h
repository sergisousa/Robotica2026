#ifndef graph_h
#define graph_h

#define N_nodes 27
#define MAX_connections 4

// define node connections
const int node_conn[N_nodes][MAX_connections] = {
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
const float node_coords[N_nodes][2] = {
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

extern float node_ad_list[N_nodes][MAX_connections];
const int node_labels[N_nodes] = {4, 5, 6, 7, 8, 9, 13, 14, 15, 16, 20, 22, 40, 41, 42, 43, 23, 24, 25, 26, 36, 37, 38, 39, 30, 31, 35};

int a_star(const int start_idx, const int stop_idx, int final_path[N_nodes]);
int find_node_idx_by_label(int label);
void obtain_ad_list(float list[N_nodes][MAX_connections]);
int find_nearest_node(float coords[2]);
float opt_cost_to_go(const float node_coord[2], const float stop_node_coord[2]);

#endif