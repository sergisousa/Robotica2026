#ifndef graph_h
#define graph_h

#define N_nodes 27
#define MAX_connections 4

// define node connections
const int node_conn[N_nodes][MAX_connections] = {
    // -1 means no connection
    {30,  6, -1, -1}, // Node 4
    { 9, 31, -1, -1}, // Node 5
    { 4, 13, 14, 30}, // Node 6
    {13, 42, -1, -1}, // Node 7
    { 9, 41, -1, -1}, // Node 8
    {16,  8, 31,  5}, // Node 9
    { 6,  7, 20, -1}, // Node 13
    { 6, 40, -1, -1}, // Node 14
    {16, 43, -1, -1}, // Node 15
    { 9, 35, 15, -1}, // Node 16
    {23, 22, 13, 35}, // Node 20
    {20, 23, -1, -1}, // Node 22
    {20, 22, 24, 36}, // Node 23
    {23, 25, 37, -1}, // Node 24
    {24, 26, 35, 38}, // Node 25
    {25, 35, 39, -1}, // Node 26
    { 4, 31,  6, -1}, // Node 30
    {30,  5,  9, -1}, // Node 31
    {16, 20, 25, 26}, // Node 35
    {23, -1, -1, -1}, // Node 36
    {24, -1, -1, -1}, // Node 37
    {25, -1, -1, -1}, // Node 38
    {26, -1, -1, -1}, // Node 39
    {14, -1, -1, -1}, // Node 40
    { 8, -1, -1, -1}, // Node 41
    { 7, -1, -1, -1}, // Node 42
    {15, -1, -1, -1}  // Node 43
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
    {0.695f,  -0.15f}, // Node 35
};

const int node_labels[N_nodes] = {4, 5, 6, 7, 8, 9, 13, 14, 15, 16, 20, 22, 40, 41, 42, 43, 23, 24, 25, 26, 36, 37, 38, 39, 30, 31, 35};

int a_star(const int start_idx, const int stop_idx, int final_path[N_nodes]);
int find_node_idx_by_label(int label);
void obtain_adj_matrix(float matrix[N_nodes][N_nodes]);
int find_nearest_node(float coords[2]);

#endif