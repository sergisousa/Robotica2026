"""
Gera os dois grafos (topologia simples e grafo de estados com direcoes)
com o MESMO figsize, limites de eixo, fontes e tamanhos de marcador,
para que fiquem visualmente coerentes lado a lado no relatorio.
"""
import math
import matplotlib.pyplot as plt
from pathlib import Path

import graph_initial as gi
import graph_with_direction as gd

# ---------- Parametros de estilo partilhados ----------
FIGSIZE      = (8.5, 8.5)   # mesmo tamanho de figura em polegadas
DPI          = 300
FONT_TITLE   = 17
FONT_LABEL   = 16
FONT_TICK    = 16
FONT_NODEIDX = 10          # numeros dos nos / estados
NODE_MS      = 10           # tamanho dos marcadores dos nos fisicos
MARGIN       = 0.14
TEXT_OFFSET  = 1.25          # multiplicador da posicao do texto em relacao a seta

# Uma cor por "layer" (slot de orientacao k). A seta e o numero do mesmo
# slot ficam sempre com a mesma cor, para ser facil associar visualmente
# qual numero pertence a qual seta em nos com varias direcoes.
LAYER_COLORS = ['mediumpurple', 'xkcd:dodger blue', 'yellowgreen', 'gold', 'orange']

# Nos bloqueados (grau 1 na malha fisica) - calculados dinamicamente a
# partir do proprio grafo (gd.BLOCKED_PHYS), para nao correr o risco de
# ficarem desatualizados se a topologia mudar.
BLOCKED_PHYS = gd.BLOCKED_PHYS
BLOCKED_COLOR = 'xkcd:dull red'  # vermelho escuro, para contrastar com o azul dos nos normais
NORMAL_COLOR_TOPO  = 'xkcd:dodger blue'
NORMAL_COLOR_STATE = 'black'

# limites comuns, calculados a partir das coordenadas fisicas + armazem/maquinas
xs = [x for x, y in gd.PHYSICAL_COORDS.values()]
ys = [y for x, y in gd.PHYSICAL_COORDS.values()]

# extremos do armazem aberto (draw_open_storage) e das maquinas (draw_machine_pair)
STORAGE_X = (0.17, 0.77)
STORAGE_Y = (-0.565, -0.465)
xs += list(STORAGE_X)
ys += list(STORAGE_Y)

dx = (max(xs) - min(xs)) * MARGIN
dy = (max(ys) - min(ys)) * MARGIN
XLIM = (min(xs) - dx, max(xs) + dx)
YLIM = (min(ys) - dy, max(ys) + dy)

# comprimento das setas escalado ao span dos eixos, para manter o mesmo
# tamanho visual que no script original (onde ARROW_LEN=0.025 e o span
# dos eixos era menor)
_ORIG_ARROW_LEN = 0.02
_ORIG_SPAN = 0.84   # span aproximado de x no script original (sem armazem no calculo)
_NEW_SPAN = XLIM[1] - XLIM[0]
ARROW_LEN = _ORIG_ARROW_LEN * (_NEW_SPAN / _ORIG_SPAN) * 1.35


def style_axes(ax, title):
    ax.set_xlim(*XLIM)
    ax.set_ylim(*YLIM)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)', fontsize=FONT_LABEL)
    ax.set_ylabel('y (m)', fontsize=FONT_LABEL)
    ax.set_title(title, fontsize=FONT_TITLE)
    ax.tick_params(labelsize=FONT_TICK)
    ax.grid(True, linestyle='--', alpha=0.3)


def plot_topology():
    fig, ax = plt.subplots(figsize=FIGSIZE)

    graph_data = gi.build_graph_data()
    coords = graph_data['coords']
    edges = graph_data['connected_points']

    for i, phys_id in enumerate(coords.keys()):
        x, y = coords[phys_id]
        # node_id aqui e o indice sequencial (0..N-1) atribuido em
        # build_graph_data, nao o id fisico original; para saber se e
        # bloqueado, comparamos pela posicao com PHYSICAL_COORDS
        
        color = BLOCKED_COLOR if phys_id in BLOCKED_PHYS else NORMAL_COLOR_TOPO
        ax.scatter(x, y, s=NODE_MS**2 * 5, zorder=3, color=color)
        ax.text(x, y, f"{i}", fontsize=FONT_NODEIDX*1.5, ha='center', va='center',
                zorder=4, color='white', fontweight='bold')

    for u, v in edges:
        if u not in coords or v not in coords:
            continue
        x1, y1 = coords[u]
        x2, y2 = coords[v]
        ax.plot([x1, x2], [y1, y2], color='k', linewidth=1.2, alpha=0.8, zorder=2)

    gi.draw_open_storage(ax, 0.17, 0.77, -0.565, -0.465, opening='top')
    gi.draw_machine_pair(ax, 0.2, 0.45, 0.075, 0.225)
    gi.draw_machine_pair(ax, 0.2, 0.45, -0.075, 0.075)

    style_axes(ax, 'Grafo de inicial (índices dos pontos físicos)')

    handles = [
        plt.Line2D([0], [0], marker='o', linestyle='', color=NORMAL_COLOR_TOPO,
                   markersize=10, label='Nó normal'),
        plt.Line2D([0], [0], marker='o', linestyle='', color=BLOCKED_COLOR,
                   markersize=10, label='Nó bloqueado'),
    ]
    ax.legend(handles=handles, loc='lower left', fontsize=FONT_TICK, framealpha=0.9)

    fig.tight_layout()
    out_path = Path(__file__).resolve().parent / 'grafo_topologia_matched.png'
    fig.savefig(out_path, dpi=DPI, bbox_inches = 'tight')
    plt.close(fig)
    print(f"Guardado em: {out_path}")


def plot_state_graph():
    fig, ax = plt.subplots(figsize=FIGSIZE)

    node_theta, node_conn, node_ad = gd.build_state_graph()

    seen = set()
    for phys_u, neighbors in gd.PHYSICAL_NEIGHBORS.items():
        xu, yu = gd.PHYSICAL_COORDS[phys_u]
        for phys_v in neighbors:
            key = tuple(sorted((phys_u, phys_v)))
            if key in seen:
                continue
            seen.add(key)
            xv, yv = gd.PHYSICAL_COORDS[phys_v]
            ax.plot([xu, xv], [yu, yv], color='#BDBDBD', lw=1.2, zorder=1)

    for node_idx, phys_id in enumerate(gd.NODE_LABELS):
        x, y = gd.PHYSICAL_COORDS[phys_id]
        for k in range(gd.N_LAYERS):
            theta = node_theta[node_idx][k]
            if theta == gd.INF:
                continue
            si = gd.state_index(node_idx, k)
            color = BLOCKED_COLOR if phys_id in BLOCKED_PHYS else LAYER_COLORS[k % len(LAYER_COLORS)]
            dxv = ARROW_LEN * math.cos(theta)
            dyv = ARROW_LEN * math.sin(theta)
            ax.annotate('', xy=(x + dxv, y + dyv), xytext=(x, y),
                        arrowprops=dict(arrowstyle='->', color=color, lw=1.8), zorder=4)
            # posicao base do texto ao longo da seta
            tx = x + dxv * TEXT_OFFSET
            ty = y + dyv * TEXT_OFFSET
            # numeros com mais algarismos ("120", "135", etc.) sao mais
            # largos, por isso precisam de um pequeno empurrao extra na
            # horizontal para nao tocarem na seta - mas so na horizontal;
            # nao faz sentido esticar tambem na vertical (setas verticais
            # ficavam com o numero longe demais)
            n_digits = len(str(si))
            extra_h = 0.006 * (n_digits - 1) * (_NEW_SPAN / _ORIG_SPAN)
            if abs(math.cos(theta)) > 0.15:
                tx += math.copysign(extra_h, math.cos(theta)) if abs(math.cos(theta)) > 0.05 else extra_h
            ax.text(tx, ty, str(si), fontsize=FONT_NODEIDX,
                    ha='center', va='center', color=color, alpha=0.95,
                    zorder=6, fontweight='bold')

    for phys_id, (x, y) in gd.PHYSICAL_COORDS.items():
        color = BLOCKED_COLOR if phys_id in BLOCKED_PHYS else NORMAL_COLOR_STATE
        ax.plot(x, y, 'o', color=color, ms=NODE_MS*0.5, zorder=5)

    gd.draw_open_storage(ax, 0.17, 0.77, -0.565, -0.465, opening='top')
    gd.draw_machine_pair(ax, 0.2, 0.45, 0.075, 0.225)
    gd.draw_machine_pair(ax, 0.2, 0.45, -0.075, 0.075)

    style_axes(ax, 'Grafo de estados (com direções)')

    handles = [
        plt.Line2D([0], [0], marker='o', linestyle='', color=NORMAL_COLOR_STATE,
                   markersize=8, label='Nó normal'),
        plt.Line2D([0], [0], marker='o', linestyle='', color=BLOCKED_COLOR,
                   markersize=8, label='Nó bloqueado'),
    ]
    ax.legend(handles=handles, loc='lower left', fontsize=FONT_TICK, framealpha=0.9)

    fig.tight_layout()
    out_path = Path(__file__).resolve().parent / 'grafo_estados_matched.png'
    fig.savefig(out_path, dpi=DPI, bbox_inches = 'tight')
    plt.close(fig)
    print(f"Guardado em: {out_path}")


if __name__ == '__main__':
    plot_topology()
    plot_state_graph()
    print('Feito: grafo_topologia_matched.png e grafo_estados_matched.png')