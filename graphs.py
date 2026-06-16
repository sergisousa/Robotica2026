import math
import itertools
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from typing import Dict, List, Tuple

PHYSICAL_COORDS: Dict[int, Tuple[float, float]] = {
    4:  (0.0,   0.355), 5:  (0.695,  0.355), 6:  (0.0,   0.15),
    7:  (0.2,   0.0),   8:  (0.5,    0.15),  9:  (0.695, 0.15),
    13: (0.0,   0.0),   14: (0.2,    0.15),  15: (0.5,   0.0),
    16: (0.695, 0.0),   20: (0.0,   -0.15),  22: (0.0,  -0.355),
    40: (0.227, 0.15),  41: (0.468,  0.15),  42: (0.227, 0.0),
    43: (0.468, 0.0),   23: (0.245, -0.355), 24: (0.395,-0.355),
    25: (0.545,-0.355), 26: (0.695, -0.355), 36: (0.245,-0.385),
    37: (0.395,-0.385), 38: (0.545, -0.385), 39: (0.695,-0.385),
    30: (0.227, 0.355), 31: (0.468,  0.355), 35: (0.695,-0.15),
}

PHYSICAL_NEIGHBORS: Dict[int, List[int]] = {
    4:  [30, 6],          5:  [9, 31],      6:  [4, 13, 14, 30],
    7:  [13, 42],         8:  [9, 41],      9:  [16, 8, 31, 5],
    13: [6, 7, 20],       14: [6, 40],      15: [16, 43],
    16: [9, 35, 15],      20: [23, 22, 13], 22: [20, 23],
    23: [20, 22, 24, 36], 24: [23, 25, 37], 25: [24, 26, 35, 38],
    26: [25, 35, 39],     30: [4, 31, 6],   31: [30, 5, 9],
    35: [16, 25, 26],     36: [23],         37: [24],
    38: [25],             39: [26],         40: [14],
    41: [8],              42: [7],          43: [15],
}

K_ROT = 0.5

def normalize_angle(a):
    while a > math.pi:  a -= 2*math.pi
    while a <= -math.pi: a += 2*math.pi
    return a

def angles_collinear(a1, a2, tol=1e-6):
    diff = normalize_angle(a1 - a2)
    return abs(diff) < tol or abs(abs(diff) - math.pi) < tol

def build_state_graph(k_rot=K_ROT):
    state_nodes = []
    phys_to_states = {k: [] for k in PHYSICAL_COORDS}

    for phys_id, neighbors in PHYSICAL_NEIGHBORS.items():
        x1, y1 = PHYSICAL_COORDS[phys_id]
        is_terminal = len(neighbors) == 1

        if is_terminal:
            nb = neighbors[0]
            x2, y2 = PHYSICAL_COORDS[nb]
            # seta aponta para fora do nó vizinho (sentido de chegada)
            theta = math.atan2(y1 - y2, x1 - x2)
            idx = len(state_nodes)
            state_nodes.append((phys_id, theta))
            phys_to_states[phys_id].append(idx)
        else:
            angles = []
            for nb in neighbors:
                x2, y2 = PHYSICAL_COORDS[nb]
                theta = math.atan2(y2 - y1, x2 - x1)
                found = any(angles_collinear(e, theta) for e in angles)
                if not found:
                    angles.append(theta)
                    for t in [theta, normalize_angle(theta + math.pi)]:
                        idx = len(state_nodes)
                        state_nodes.append((phys_id, t))
                        phys_to_states[phys_id].append(idx)

    state_edges = []
    seen_phys_edges = set()
    for phys_u, neighbors in PHYSICAL_NEIGHBORS.items():
        xu, yu = PHYSICAL_COORDS[phys_u]
        for phys_v in neighbors:
            edge_key = tuple(sorted((phys_u, phys_v)))
            if edge_key in seen_phys_edges:
                continue
            seen_phys_edges.add(edge_key)
            xv, yv = PHYSICAL_COORDS[phys_v]
            dist = math.hypot(xv - xu, yv - yu)
            theta_uv = math.atan2(yv - yu, xv - xu)
            theta_vu = normalize_angle(theta_uv + math.pi)
            state_u = _find_state(phys_to_states, state_nodes, phys_u, theta_uv)
            state_v = _find_state(phys_to_states, state_nodes, phys_v, theta_vu)
            if state_u is not None and state_v is not None:
                state_edges.append((state_u, state_v, dist, 'trans'))
                state_edges.append((state_v, state_u, dist, 'trans'))

    for phys_id, states in phys_to_states.items():
        for i, j in itertools.combinations(states, 2):
            _, ti = state_nodes[i]
            _, tj = state_nodes[j]
            rot = abs(normalize_angle(ti - tj))
            cost = k_rot * rot
            state_edges.append((i, j, cost, 'rot'))
            state_edges.append((j, i, cost, 'rot'))

    return state_nodes, state_edges, phys_to_states

def _find_state(phys_to_states, state_nodes, phys_id, theta, tol=0.1):
    best, best_diff = None, math.inf
    for idx in phys_to_states[phys_id]:
        diff = abs(normalize_angle(state_nodes[idx][1] - theta))
        if diff < best_diff:
            best_diff, best = diff, idx
    return best if best_diff < tol else None

ARROW_LEN = 0.018

def draw_state_graph(state_nodes, state_edges, phys_to_states, ax=None):
    if ax is None:
        fig, ax = plt.subplots(figsize=(10, 9))

    col_trans = '#2196F3'
    col_rot   = '#E91E63'
    col_node  = '#1B5E20'

    # grafo físico de fundo — linhas simples sem setas
    seen = set()
    for phys_u, neighbors in PHYSICAL_NEIGHBORS.items():
        xu, yu = PHYSICAL_COORDS[phys_u]
        for phys_v in neighbors:
            key = tuple(sorted((phys_u, phys_v)))
            if key in seen: continue
            seen.add(key)
            xv, yv = PHYSICAL_COORDS[phys_v]
            ax.plot([xu, xv], [yu, yv], color='#BDBDBD', lw=1.2, zorder=1)

    # setas de estado
    for idx, (phys_id, theta) in enumerate(state_nodes):
        x, y = PHYSICAL_COORDS[phys_id]
        dx = ARROW_LEN * math.cos(theta)
        dy = ARROW_LEN * math.sin(theta)
        ax.annotate('', xy=(x + dx, y + dy), xytext=(x, y),
                    arrowprops=dict(arrowstyle='->', color=col_rot, lw=1.5),
                    zorder=4)
        # número imediatamente na ponta da seta
        label_scale = 1.15
        ax.text(x + dx * label_scale, y + dy * label_scale, str(idx),
                fontsize=8, ha='center', va='center',
                color='#4A148C', zorder=6, fontweight='bold')

    # nós físicos
    for phys_id, (x, y) in PHYSICAL_COORDS.items():
        ax.plot(x, y, 'o', color=col_node, ms=5, zorder=5)

    # ax.legend(handles=[
    #     mpatches.Patch(color=col_rot,   label='direção do estado'),
    #     mpatches.Patch(color='#BDBDBD', label='grafo físico'),
    # ], fontsize=8, loc='upper right')

    ax.set_aspect('equal')
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    ax.set_title(f'Grafo de estados (x, y, θ)  —  {len(state_nodes)} nós de estado')
    ax.grid(True, linestyle='--', alpha=0.3)
    return ax

if __name__ == '__main__':
    state_nodes, state_edges, phys_to_states = build_state_graph()

    print(f"Nós de estado: {len(state_nodes)}")
    print(f"Arestas de estado: {len(state_edges)}")
    print(f"\nNós por posição física:")
    for phys_id, states in phys_to_states.items():
        thetas = [math.degrees(state_nodes[s][1]) for s in states]
        print(f"  Nó físico {phys_id:2d} → estados {states}  θ={[f'{t:.0f}°' for t in thetas]}")

    ax = draw_state_graph(state_nodes, state_edges, phys_to_states)
    plt.tight_layout()
    plt.savefig('C:/Users/salom/OneDrive/MEF/1.º ano/2.º semestre/Robótica/Projeto 2 - carrinho/Código/state_graph.png', dpi=150)
    plt.show()