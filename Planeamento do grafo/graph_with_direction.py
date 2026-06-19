import math
import matplotlib.pyplot as plt
from typing import Dict, List, Tuple

NODE_LABELS = [4, 5, 6, 7, 8, 9, 13, 14, 15, 16, 20, 22,
               23, 24, 25, 26, 36, 37, 38, 39, 30, 31, 35]
N_NODES  = len(NODE_LABELS)
N_LAYERS = 6
K_ROT    = 1.0
THETA_THRESH = 1e-3
INF = float('inf')

PHYSICAL_COORDS: Dict[int, Tuple[float, float]] = {
    4:(0.0,0.355), 5:(0.695,0.355), 6:(0.0,0.15), 7:(0.107,0.0),
    8:(0.593,0.15), 9:(0.695,0.15), 13:(0.0,0.0), 14:(0.107,0.15),
    15:(0.593,0.0), 16:(0.695,0.0), 20:(0.0,-0.15), 22:(0.0,-0.315),
    23:(0.245,-0.315), 24:(0.395,-0.315), 25:(0.545,-0.315), 26:(0.695,-0.315),
    36:(0.245,-0.365), 37:(0.395,-0.365), 38:(0.545,-0.365), 39:(0.695,-0.365),
    30:(0.227,0.355), 31:(0.468,0.355), 35:(0.695,-0.15),
}

PHYSICAL_NEIGHBORS: Dict[int, List[int]] = {
    4: [30, 6],
    5: [9, 31],
    6: [4, 13, 14, 30],
    7: [13],
    8: [9],
    9: [16, 8, 31, 5],
    13:[6, 7, 20],
    14:[6],
    15:[16],
    16:[9, 35, 15],
    20:[23, 22, 13],
    22:[20, 23],
    23:[20, 22, 24, 36],
    24:[23, 25, 37],
    25:[24, 26, 35, 38],
    26:[25, 35, 39],
    36:[23],
    37:[24],
    38:[25],
    39:[26],
    30:[4, 31, 6],
    31:[30, 5, 9],
    35:[16, 25, 26],
}

def normalize_angle(a):
    while a >  math.pi: a -= 2*math.pi
    while a <= -math.pi: a += 2*math.pi
    return a

def dif_angle(a0, a1):
    return normalize_angle(normalize_angle(a0) - normalize_angle(a1))

def phys_to_node_idx(phys_id):
    return NODE_LABELS.index(phys_id)

def state_index(node_idx, layer_idx):
    return N_LAYERS * node_idx + layer_idx

# nós com apenas 1 ligação física (equivalente a blocked_nodes / N_blocked no graph.cpp)
BLOCKED_PHYS = set(p for p in NODE_LABELS if len(PHYSICAL_NEIGHBORS[p]) == 1)

def build_state_graph():
    node_theta = [[INF]*N_LAYERS for _ in range(N_NODES)]
    total = N_NODES * N_LAYERS
    node_conn = [[-1]*3 for _ in range(total)]
    node_ad   = [[0.0]*3 for _ in range(total)]

    def find_or_create_slot(ni, theta):
        for k in range(N_LAYERS):
            if node_theta[ni][k] < INF and abs(dif_angle(node_theta[ni][k], theta)) < THETA_THRESH:
                return k
        for k in range(N_LAYERS):
            if node_theta[ni][k] == INF:
                node_theta[ni][k] = theta
                return k
        return -1

    def add_conn(si, sj, cost):
        for c in range(3):
            if node_conn[si][c] == -1:
                node_conn[si][c] = sj
                node_ad[si][c] = cost
                return

    # translação
    for node_idx, phys_id in enumerate(NODE_LABELS):
        # tal como em graph.cpp: um no bloqueado (grau 1) nunca gera camada a partir
        # de si proprio - a sua unica camada e criada pelo vizinho nao bloqueado
        if phys_id in BLOCKED_PHYS:
            continue
        for nbr_phys in PHYSICAL_NEIGHBORS[phys_id]:
            nbr_idx = phys_to_node_idx(nbr_phys)
            xu, yu = PHYSICAL_COORDS[phys_id]
            xv, yv = PHYSICAL_COORDS[nbr_phys]
            theta = math.atan2(yv - yu, xv - xu)
            dist  = math.hypot(xv - xu, yv - yu)
            k_curr = find_or_create_slot(node_idx, theta)
            k_nbr  = find_or_create_slot(nbr_idx, theta)
            if k_curr >= 0 and k_nbr >= 0:
                add_conn(state_index(node_idx, k_curr),
                         state_index(nbr_idx,  k_nbr), dist)
                # se o vizinho e um no bloqueado, ele nunca sera processado
                # como curr (foi saltado acima), entao precisa de uma ligacao
                # de regresso explicita - tal como o bloco "if (nbr_is_blocked)" no graph.cpp
                if nbr_phys in BLOCKED_PHYS:
                    add_conn(state_index(nbr_idx,  k_nbr),
                             state_index(node_idx, k_curr), dist)

    # rotação
    for node_idx in range(N_NODES):
        slots = [(k, node_theta[node_idx][k])
                 for k in range(N_LAYERS) if node_theta[node_idx][k] < INF]
        if len(slots) <= 1:
            continue
        slots_sorted = sorted(slots, key=lambda x: x[1])
        n = len(slots_sorted)
        for i, (k, t) in enumerate(slots_sorted):
            k_left, t_left   = slots_sorted[(i-1) % n]
            k_right, t_right = slots_sorted[(i+1) % n]
            si = state_index(node_idx, k)
            diff_left  = abs(normalize_angle(t - t_left))
            diff_right = abs(normalize_angle(t - t_right))
            add_conn(si, state_index(node_idx, k_left),  K_ROT * diff_left)
            if n > 2:
                add_conn(si, state_index(node_idx, k_right), K_ROT * diff_right)

    return node_theta, node_conn, node_ad

ARROW_LEN = 0.025

def draw_open_storage(ax, x0, x1, y0, y1, opening='top'):

    # laterais
    ax.plot([x0,x0],[y0,y1],'k',lw=2.5)
    ax.plot([x1,x1],[y0,y1],'k',lw=2.5)

    # fundo
    if opening == 'top':
        ax.plot([x0,x1],[y0,y0],'k',lw=2.5)

    else:
        ax.plot([x0,x1],[y1,y1],'k',lw=2.5)

    # divisórias com exatamente a mesma altura
    for i in range(1,4):
        x = x0 + i*(x1-x0)/4
        ax.plot([x,x],[y0,y1],'k',lw=2.5)


def draw_machine_pair(ax, x0, x1, y0, y1):

    # parede superior
    ax.plot([x0, x1], [y1, y1], 'k', lw=2.5)

    # parede inferior
    ax.plot([x0, x1], [y0, y0], 'k', lw=2.5)

    # divisória central
    xm = (x0 + x1)/2
    ax.plot([xm, xm], [y0, y1], 'k', lw=2.5)

def draw_state_graph(node_theta, ax=None):
    if ax is None:
        fig, ax = plt.subplots(figsize=(10, 9))

    seen = set()
    for phys_u, neighbors in PHYSICAL_NEIGHBORS.items():
        xu, yu = PHYSICAL_COORDS[phys_u]
        for phys_v in neighbors:
            key = tuple(sorted((phys_u, phys_v)))
            if key in seen: continue
            seen.add(key)
            xv, yv = PHYSICAL_COORDS[phys_v]
            ax.plot([xu, xv], [yu, yv], color='#BDBDBD', lw=1.2, zorder=1)

    for node_idx, phys_id in enumerate(NODE_LABELS):
        x, y = PHYSICAL_COORDS[phys_id]
        for k in range(N_LAYERS):
            theta = node_theta[node_idx][k]
            if theta == INF:
                continue
            si = state_index(node_idx, k)
            dx = ARROW_LEN * math.cos(theta)
            dy = ARROW_LEN * math.sin(theta)
            ax.annotate('', xy=(x+dx, y+dy), xytext=(x, y),
                        arrowprops=dict(arrowstyle='->', color='hotpink', lw=1.5), zorder=4)
            ax.text(x+dx*1.5, y+dy*1.5, str(si), fontsize=11,
                    ha='center', va='center', color='darkblue', alpha = 0.8, zorder=6, fontweight='bold')

    for phys_id, (x, y) in PHYSICAL_COORDS.items():
        ax.plot(x, y, 'o', color='mediumpurple', ms=5, zorder=5)

    occupied = sum(1 for ni in range(N_NODES)
                   for k in range(N_LAYERS) if node_theta[ni][k] < INF)
    ax.set_aspect('equal')
    ax.margins(x=0.10, y=0.10)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    ax.set_title(f'Grafo de estados (indexacao C++)  -  {occupied} nos ocupados')
    ax.grid(True, linestyle='--', alpha=0.3)
    return ax

if __name__ == '__main__':
    node_theta, node_conn, node_ad = build_state_graph()

    print("Nos ocupados por posicao fisica:")
    for node_idx, phys_id in enumerate(NODE_LABELS):
        slots = [(k, node_theta[node_idx][k])
                 for k in range(N_LAYERS) if node_theta[node_idx][k] < INF]
        if slots:
            print(f"  phys {phys_id:2d} (node_idx={node_idx}): " +
                  ", ".join(f"state {state_index(node_idx,k)}={math.degrees(t):.0f}" for k,t in slots))

    ax = draw_state_graph(node_theta)
    draw_open_storage(ax, 0.17, 0.77, -0.565, -0.465, opening='top')
    draw_machine_pair(ax, 0.2, 0.45, 0.075, 0.225)
    draw_machine_pair(ax, 0.2, 0.45, -0.075, 0.075)
    plt.tight_layout()
    plt.savefig('C:/Users/salom/OneDrive/MEF/1.º ano/2.º semestre/Robótica/Projeto 2 - carrinho/Código/Planeamento do grafo/grafografo_rotações.png', dpi=150)
    plt.show()