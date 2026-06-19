"""Desenhar grafos a partir de uma linked list ou de uma lista de arestas.

Estrutura mínima:
- Node: nó simples de lista ligada com atributo `id` e `next`.
- coords: dicionário {id: (x, y)} com posições de cada nó.

Funções principais:
- draw_graph_from_linked_list(head, coords, ax=None)
- draw_graph(edges, coords, ax=None)
Retorna o matplotlib Axes com o grafo desenhado.
"""
import math
import numpy as np
from typing import Any, Dict, Iterable, Iterator, Optional, Sequence, Tuple
import matplotlib.pyplot as plt


class Node:
    """Nó simples para formar uma linked list."""

    def __init__(self, id: Any, next: 'Node' = None):
        self.id = id
        self.next = next

    def __repr__(self):
        return f"Node({self.id})"


FLOOR_POINTS: Sequence[Tuple[int, Tuple[float, float]]] = (
    (4, (0.0, 0.355)),
    (5, (0.695, 0.355)),
    (6, (0.0, 0.150)),
    (7, (0.107, 0.0)),
    (8, (0.593, 0.150)),
    (9, (0.695, 0.150)),
    (13, (0.0, 0.0)),
    (14, (0.107, 0.150)),
    (15, (0.593, 0.0)),
    (16, (0.695, 0.0)),
    (20, (0.0, -0.150)),
    (22, (0.0, -0.315)),
    (23, (0.245, -0.315)),
    (24, (0.395, -0.315)),
    (25, (0.545, -0.315)),
    (26, (0.695, -0.315)),
    (36, (0.245, -0.365)), # Pontos artificias 36-39
    (37, (0.395, -0.365)),
    (38, (0.545, -0.365)),
    (39, (0.695, -0.365)),
    (30, (0.227, 0.355)),
    (31, (0.468, 0.355)),
    (35, (0.695, -0.150)),
)


def floor_points_to_coords(points: Sequence[Tuple[Any, Tuple[float, float]]] = FLOOR_POINTS) -> Dict[Any, Tuple[float, float]]:
    """Converte a tabela de pontos do piso em coords, omitindo entradas inválidas."""
    coords: Dict[Any, Tuple[float, float]] = {}
    for node_id, (x, y) in points:
        if math.isnan(x) or math.isnan(y):
            continue
        coords[node_id] = (x, y)
    return coords


def connect_nearby_points(coords: Dict[Any, Tuple[float, float]]) -> list[Tuple[Any, Any]]:
    """Gera arestas a partir de uma lista manual explícita de vizinhança."""
    manual_neighbors: Dict[Any, list[Any]] = {
        4: [30, 6],
        5: [9, 31],
        6: [4, 13,14,30],
        7: [13],
        8: [9],
        9: [16, 8, 31, 5],
        13: [6, 7, 20],
        14: [6],
        15: [16],
        16: [9, 35, 15],
        20: [23,22,13],
        22: [20, 23],
        23: [20,22, 24, 36],
        24: [23, 25, 37],
        25: [24, 26, 35, 38],
        26: [25, 35, 39],
        30: [4, 31,6],
        31: [30, 5, 9,],
        35: [16, 25, 26],
        36: [23],
        37: [24],
        38: [25],
        39: [26],
    }

    seen_edges: set[Tuple[Any, Any]] = set()
    edges: list[Tuple[Any, Any]] = []

    for u, neighbors in manual_neighbors.items():
        if u not in coords:
            continue
        for v in neighbors:
            if v not in coords or u == v:
                continue
            edge_key = tuple(sorted((u, v)))
            if edge_key in seen_edges:
                continue
            edges.append((u, v))
            seen_edges.add(edge_key)

    return edges

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


def build_graph_data(points: Sequence[Tuple[Any, Tuple[float, float]]] = FLOOR_POINTS) -> Dict[str, Any]:
    """Devolve os dados do grafo: nós, coordenadas e ligações.

    Returns:
        Um dicionário com:
        - node_ids: lista ordenada dos nós válidos
        - coords: dicionário {id: (x, y)}
        - connected_points: lista de pares (u, v)
        - index_by_id: mapeamento id -> índice
    """
    coords = floor_points_to_coords(points)
    # node_ids = list(coords.keys())
    node_ids = np.linspace(0, len(coords), len(coords), dtype=int).tolist()
    index_by_id = {node_id: index for index, node_id in enumerate(node_ids)}
    connected_points = connect_nearby_points(coords)

    return {
        'node_ids': node_ids,
        'coords': coords,
        'connected_points': connected_points,
        'index_by_id': index_by_id,
    }


def export_graph_data(
    graph_data: Dict[str, Any] = None,
    format: str = 'python',
) -> str:
    """Exporta dados do grafo (pontos e ligações) em diferentes formatos.

    Args:
        graph_data: dicionário retornado por build_graph_data(). Se None, constrói automaticamente.
        format: 'python', 'json', 'cpp', ou 'csv'

    Returns:
        String com os dados formatados.
    """
    if graph_data is None:
        graph_data = build_graph_data()

    if format == 'python':
        return _export_python(graph_data)
    elif format == 'json':
        import json
        data = {
            'node_ids': graph_data['node_ids'],
            'coords': {str(k): v for k, v in graph_data['coords'].items()},
            'connected_points': graph_data['connected_points'],
        }
        return json.dumps(data, indent=2)
    elif format == 'cpp':
        return _export_cpp(graph_data)
    elif format == 'csv':
        return _export_csv(graph_data)
    else:
        raise ValueError(f"Formato desconhecido: {format}")


def _export_python(graph_data: Dict[str, Any]) -> str:
    """Exporta para formato Python."""
    lines = []
    lines.append("# Dados do Grafo")
    lines.append("")

    lines.append("# IDs dos nós")
    lines.append(f"node_ids = {graph_data['node_ids']}")
    lines.append("")

    lines.append("# Coordenadas")
    lines.append("coords = {")
    for node_id, (x, y) in graph_data['coords'].items():
        lines.append(f"    {node_id}: ({x}, {y}),")
    lines.append("}")
    lines.append("")

    lines.append("# Pares conectados")
    lines.append(f"connected_points = {graph_data['connected_points']}")

    return "\n".join(lines)


def _export_cpp(graph_data: Dict[str, Any]) -> str:
    """Exporta para formato C++."""
    lines = []
    lines.append("// Dados do Grafo")
    lines.append("")

    lines.append("// IDs dos nós")
    lines.append(f"const int NODE_IDS[] = {{{', '.join(map(str, graph_data['node_ids']))}}};")
    lines.append(f"const int NUM_NODES = {len(graph_data['node_ids'])};")
    lines.append("")

    lines.append("// Coordenadas (x, y)")
    lines.append("struct Point { float x, y; };")
    lines.append("const Point COORDS[] = {")
    for node_id, (x, y) in graph_data['coords'].items():
        lines.append(f"    // Node {node_id}")
        lines.append(f"    {{{x}f, {y}f}},")
    lines.append("};")
    lines.append("")

    lines.append("// Pares conectados")
    lines.append("struct Edge { int u, v; };")
    lines.append("const Edge EDGES[] = {")
    for u, v in graph_data['connected_points']:
        lines.append(f"    {{{u}, {v}}},")
    lines.append("};")
    lines.append(f"const int NUM_EDGES = {len(graph_data['connected_points'])};")

    return "\n".join(lines)


def _export_csv(graph_data: Dict[str, Any]) -> str:
    """Exporta para formato CSV."""
    lines = []
    lines.append("node_id,x,y")
    for node_id, (x, y) in graph_data['coords'].items():
        lines.append(f"{node_id},{x},{y}")
    lines.append("")
    lines.append("u,v")
    for u, v in graph_data['connected_points']:
        lines.append(f"{u},{v}")
    return "\n".join(lines)


def linked_list_to_edges(head: Optional[Node]) -> Iterable[Tuple[Any, Any]]:
    """Converte uma lista ligada em arestas (u, v)."""
    cur = head
    seen = set()
    while cur is not None and cur not in seen:
        seen.add(cur)
        if cur.next is not None:
            yield (cur.id, cur.next.id)
        cur = cur.next


def draw_graph(
    edges: Sequence[Tuple[Any, Any]],
    coords: Dict[Any, Tuple[float, float]],
    ax: Optional[plt.Axes] = None,
) -> plt.Axes:
    """Desenha um grafo a partir de uma lista de arestas (sem pesos).

    Args:
        edges: sequência de arestas no formato (origem, destino).
        coords: dicionário que mapeia id -> (x, y).
        ax: Axes existente opcional.

    Returns:
        O objeto matplotlib.axes.Axes com o desenho.
    """
    if ax is None:
        fig, ax = plt.subplots()

    # Desenhar nós
    for i, node_id in enumerate(coords.keys()):
        (x, y) = coords[node_id]
        if math.isnan(x) or math.isnan(y):
            continue
        ax.scatter(x, y, s=200, zorder=3, color='xkcd:dodger blue')
        ax.text(x, y, f"{i}", fontsize=10, ha='center', va='center', zorder=4, color='white', fontweight='bold')

    # Desenhar arestas
    for u, v in edges:
        if u not in coords or v not in coords:
            continue
        x1, y1 = coords[u]
        x2, y2 = coords[v]
        ax.plot([x1, x2], [y1, y2], color='k', linewidth=1.2, alpha=0.8, zorder=2)

    ax.set_aspect('equal')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.grid(True, linestyle='--', alpha=0.3)


    draw_open_storage(ax, 0.17, 0.77, -0.565, -0.465, opening='top')
    draw_machine_pair(ax, 0.2, 0.45, 0.075, 0.225)
    draw_machine_pair(ax, 0.2, 0.45, -0.075, 0.075)
    return ax


def draw_graph_from_linked_list(head: Optional[Node], coords: Dict[Any, Tuple[float, float]], ax: Optional[plt.Axes] = None) -> plt.Axes:
    """Desenha o grafo a partir de uma linked list e coordenadas.

    Args:
        head: nó inicial da lista ligada.
        coords: dicionário que mapeia id -> (x, y).
        ax: Axes existente opcional.

    Returns:
        O objeto matplotlib.axes.Axes com o desenho.
    """
    return draw_graph(list(linked_list_to_edges(head)), coords, ax=ax)


if __name__ == '__main__':
    # Construir dados do grafo
    graph_data = build_graph_data()

    # Opção 1: Visualizar o grafo
    print("=== Visualizando Grafo ===")
    coords_example = floor_points_to_coords()
    edges_example = connect_nearby_points(coords_example)
    ax = draw_graph(edges_example, coords_example)
    plt.title('Grafo')
    plt.savefig('C:/Users/salom/OneDrive/MEF/1.º ano/2.º semestre/Robótica/Projeto 2 - carrinho/Código/Planeamento do grafo/grafo.png', dpi=300, bbox_inches='tight')
    plt.show()

    # Opção 2: Exportar em formato Python
    print("\n=== Formato Python ===")
    print(export_graph_data(graph_data, format='python'))

    # Opção 3: Exportar em formato C++
    print("\n=== Formato C++ ===")
    print(export_graph_data(graph_data, format='cpp'))

    # Opção 4: Exportar em formato CSV
    print("\n=== Formato CSV ===")
    print(export_graph_data(graph_data, format='csv'))