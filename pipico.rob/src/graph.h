#ifndef graph_h
#define graph_h

#define N_nodes 27
#define N_layers 6
#define MAX_connections 4
#define MAX_connections_layer 3

#define K_theta 1.0
#define theta_thresh 1e-3

extern int node_conn[N_nodes][MAX_connections];
extern float node_coords[N_nodes][2];

extern float node_ad_list[N_nodes][MAX_connections];
extern int node_labels[N_nodes];

extern int node_conn_layered[N_layers * N_nodes][MAX_connections_layer];
extern float node_ad_list_layered[N_layers * N_nodes][MAX_connections_layer];
extern float node_theta_layers[N_nodes * N_layers];

void generate_graph_with_layers();
int a_star(const int start_idx, const int stop_idx, int final_path[N_nodes]);
int find_node_idx_by_label(int label);
void obtain_ad_list(float list[N_nodes][MAX_connections]);
int find_nearest_node(float coords[2]);
float opt_cost_to_go(const float node_coord[2], const float stop_node_coord[2]);

#endif