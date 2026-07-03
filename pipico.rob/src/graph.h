#ifndef graph_h
#define graph_h

#define N_nodes 23
#define N_layers 6
#define N_blocked 8
#define MAX_connections 4
#define MAX_connections_layer 3

#define theta_thresh 1e-3

extern float Rotation_Weight;
extern int blocked_nodes[N_blocked];
extern int node_conn[N_nodes + 1][MAX_connections];
extern float node_coords[N_nodes + 1][2];

extern int node_conn_layered[N_layers * (N_nodes + 1) + 1][MAX_connections_layer];
extern float node_ad_list_layered[N_layers * (N_nodes + 1) + 1][MAX_connections_layer];
extern float node_theta_layers[(N_nodes + 1) * N_layers + 1];


float sqr(float x);
float sign(float x);
float norm(float x, float y);
void normalize(float& x, float& y);

float dist(float x0, float y0, float x1, float y1);
float normalize_angle(float ang);
float dif_angle(float a0, float a1);
float opt_cost_to_go(const float node_coord[2], const float stop_node_coord[2]);

void fill_blocked_nodes();
void initial_node(float coords[2]);
int array_has_element(int arr[], int size, int element);
void fill_array(int array[], int value, int size);
void generate_graph_with_layers(float robot_theta);
int a_star(const int start_idx, const int stop_idx, int final_path[(N_nodes + 1) * N_layers + 1]);
void selection_sort(const int size, int idx[], float array[]);

#endif