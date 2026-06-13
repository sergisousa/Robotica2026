"""Desenhar grafos a partir de uma linked list ou de uma lista de arestas.

Estrutura mínima:
- Node: nó simples de lista ligada com atributo `id` e `next`.
- coords: dicionário {id: (x, y)} com posições de cada nó.

Funções principais:
- draw_graph_from_linked_list(head, coords, ax=None)
- draw_weighted_graph(edges, coords, ax=None)
Retorna o matplotlib Axes com o grafo desenhado.
"""
import math
from typing import Any, Dict, Iterable, Iterator, Optional, Sequence, Tuple, Union
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
    (7, (0.2, 0.0)),
    (8, (0.5, 0.150)),
    (9, (0.695, 0.150)),
    (13, (0.0, 0.0)),
    (14, (0.2, 0.150)),
    (15, (0.5, 0.0)),
    (16, (0.695, 0.0)),
    (20, (0.0, -0.150)),
    (22, (0.0, -0.355)),
    (40, (0.227, 0.150)),
    (41, (0.468, 0.150)),
    (42, (0.227, 0.0)),
    (43, (0.468, 0.0)),
    (23, (0.245, -0.355)),
    (24, (0.395, -0.355)),
    (25, (0.545, -0.355)),
    (26, (0.695, -0.355)),
    (36, (0.245, -0.385)), # Pontos artificias 36-39
    (37, (0.395, -0.385)),
    (38, (0.545, -0.385)),
    (39, (0.695, -0.385)),
    (27, (math.nan, math.nan)),
    (28, (math.nan, math.nan)),
    (29, (math.nan, math.nan)),
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


def connect_nearby_points(
    coords: Dict[Any, Tuple[float, float]],
    max_distance: float = 0.26,
) -> list[Tuple[Any, Any, float]]:
    """Gera arestas a partir de uma lista manual explícita de vizinhança.

    O parâmetro max_distance fica só por compatibilidade com a assinatura.
    """
    manual_neighbors: Dict[Any, list[Any]] = {
        4: [30, 6],
        5: [9, 31],
        6: [4, 13,14,30],
        7: [13, 42],
        8: [ 9, 41],
        9: [16, 8, 31, 5],
        13: [6, 7, 20],
        14: [6, 40],
        15: [16, 43],
        16: [9, 35, 15],
        20: [23,22,13,35],
        22: [20, 23],
        23: [20,22, 24, 36],
        24: [23, 25, 37],
        25: [24, 26, 35, 38],
        26: [25, 35, 39],
        30: [4, 31,6],
        31: [30, 5, 9,],
        35: [16, 20, 25, 26],
        36: [23],
        37: [24],
        38: [25],
        39: [26],
        40: [14],
        41: [8],
        42: [7],
        43: [15],
    }

    seen_edges: set[Tuple[Any, Any]] = set()
    edges: list[Tuple[Any, Any, float]] = []

    for u, neighbors in manual_neighbors.items():
        if u not in coords:
            continue
        x1, y1 = coords[u]
        for v in neighbors:
            if v not in coords or u == v:
                continue
            edge_key = tuple(sorted((u, v)))
            if edge_key in seen_edges:
                continue
            x2, y2 = coords[v]
            distance = math.hypot(x2 - x1, y2 - y1)
            edges.append((u, v, round(distance, 3)))
            seen_edges.add(edge_key)

    return edges


def build_graph_data(
    points: Sequence[Tuple[Any, Tuple[float, float]]] = FLOOR_POINTS,
    max_distance: float = 0.26,
    default_weight: float = 1.0,
) -> Dict[str, Any]:
    """Devolve os dados do grafo em listas e uma matriz de adjacência ponderada.

    Returns:
        Um dicionário com:
        - node_ids: lista ordenada dos nós válidos
        - coords: dicionário {id: (x, y)}
        - connected_points: lista de pares (u, v)
        - weighted_edges: lista de triplos (u, v, peso)
        - adjacency_matrix: matriz ponderada de conectividade
        - weight_matrix: cópia da matriz ponderada
        - index_by_id: mapeamento id -> índice na matriz
    """
    coords = floor_points_to_coords(points)
    node_ids = list(coords.keys())
    index_by_id = {node_id: index for index, node_id in enumerate(node_ids)}
    weighted_edges = connect_nearby_points(coords, max_distance=max_distance)
    connected_points = [(u, v) for u, v, _ in weighted_edges]

    size = len(node_ids)
    adjacency_matrix = [[0.0 for _ in range(size)] for _ in range(size)]

    for u, v, weight in weighted_edges:
        i = index_by_id[u]
        j = index_by_id[v]
        final_weight = weight if weight is not None else default_weight
        adjacency_matrix[i][j] = final_weight
        adjacency_matrix[j][i] = final_weight

    return {
        'node_ids': node_ids,
        'coords': coords,
        'connected_points': connected_points,
        'weighted_edges': weighted_edges,
        'adjacency_matrix': adjacency_matrix,
        'weight_matrix': adjacency_matrix,
        'index_by_id': index_by_id,
    }


def export_graph_data(
    graph_data: Dict[str, Any] = None,
    format: str = 'python',
) -> str:
    """Exporta dados do grafo em diferentes formatos.
    
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
            'weighted_edges': graph_data['weighted_edges'],
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
    lines.append("")
    
    lines.append("# Arestas com peso")
    lines.append("weighted_edges = [")
    for u, v, w in graph_data['weighted_edges']:
        lines.append(f"    ({u}, {v}, {w}),")
    lines.append("]")
    lines.append("")
    
    lines.append("# Matriz de adjacência")
    lines.append("adjacency_matrix = [")
    for row in graph_data['adjacency_matrix']:
        lines.append(f"    {row},")
    lines.append("]")
    lines.append("")
    
    lines.append("# Matriz de pesos")
    lines.append("weight_matrix = [")
    for row in graph_data['weight_matrix']:
        lines.append(f"    {row},")
    lines.append("]")
    
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
    lines.append("")
    
    lines.append("// Arestas com peso")
    lines.append("struct WeightedEdge { int u, v; float weight; };")
    lines.append("const WeightedEdge WEIGHTED_EDGES[] = {")
    for u, v, w in graph_data['weighted_edges']:
        lines.append(f"    {{{u}, {v}, {w}f}},")
    lines.append("};")
    lines.append("")
    
    lines.append("// Matriz de adjacência")
    size = len(graph_data['node_ids'])
    lines.append(f"const int ADJACENCY[{size}][{size}] = {{")
    for row in graph_data['adjacency_matrix']:
        lines.append(f"    {{{', '.join(map(str, row))}}},")
    lines.append("};")
    lines.append("")
    
    lines.append("// Matriz de pesos")
    lines.append(f"const float WEIGHTS[{size}][{size}] = {{")
    for row in graph_data['weight_matrix']:
        lines.append(f"    {{{', '.join(f'{w}f' for w in row)}}},")
    lines.append("};")
    
    return "\n".join(lines)


def _export_csv(graph_data: Dict[str, Any]) -> str:
    """Exporta para formato CSV."""
    lines = []
    lines.append("node_id,x,y")
    for node_id, (x, y) in graph_data['coords'].items():
        lines.append(f"{node_id},{x},{y}")
    lines.append("")
    lines.append("u,v,weight")
    for u, v, w in graph_data['weighted_edges']:
        lines.append(f"{u},{v},{w}")
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


EdgeSpec = Union[Tuple[Any, Any], Tuple[Any, Any, float]]


def _normalize_edges(edges: Sequence[EdgeSpec]) -> Iterator[Tuple[Any, Any, Optional[float]]]:
    """Converte arestas em um formato uniforme (origem, destino, custo)."""
    for edge in edges:
        if len(edge) == 2:
            u, v = edge
            yield (u, v, None)
        elif len(edge) == 3:
            u, v, cost = edge
            yield (u, v, cost)
        else:
            raise ValueError("Cada aresta deve ter 2 ou 3 elementos: (origem, destino[, custo])")


def draw_weighted_graph(
    edges: Sequence[EdgeSpec],
    coords: Dict[Any, Tuple[float, float]],
    ax: Optional[plt.Axes] = None,
    default_cost: Optional[float] = None,
) -> plt.Axes:
    """Desenha um grafo ponderado a partir de uma lista de arestas.

    Args:
        edges: sequência de arestas no formato (origem, destino) ou (origem, destino, custo).
        coords: dicionário que mapeia id -> (x, y).
        ax: Axes existente opcional.

    Returns:
        O objeto matplotlib.axes.Axes com o desenho.
    """
    if ax is None:
        fig, ax = plt.subplots()

    # Desenhar nós
    for node_id, (x, y) in coords.items():
        if math.isnan(x) or math.isnan(y):
            continue
        ax.scatter(x, y, s=80, zorder=3, color='C0')
        ax.text(x, y, f"{node_id}", fontsize=9, ha='center', va='center', zorder=4, color='white')

    # Desenhar arestas sem direção com custo opcional
    for u, v, cost in _normalize_edges(edges):
        if u not in coords or v not in coords:
            continue
        x1, y1 = coords[u]
        x2, y2 = coords[v]
        ax.plot([x1, x2], [y1, y2], color='k', linewidth=1.2, alpha=0.8, zorder=2)
        if cost is None:
            cost = default_cost

        if cost is not None:
            ax.text(
                (x1 + x2) / 2,
                (y1 + y2) / 2,
                f"{cost}",
                fontsize=8,
                ha='center',
                va='center',
                zorder=5,
                color='darkred',
                bbox=dict(boxstyle='round,pad=0.15', facecolor='white', alpha=0.8, edgecolor='none'),
            )

    ax.set_aspect('equal')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.grid(True, linestyle='--', alpha=0.3)
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
    return draw_weighted_graph(list(linked_list_to_edges(head)), coords, ax=ax)


if __name__ == '__main__':
    # Construir dados do grafo
    graph_data = build_graph_data()
    
    # Opção 1: Visualizar o grafo
    print("=== Visualizando Grafo ===")
    coords_example = floor_points_to_coords()
    edges_example = connect_nearby_points(coords_example)
    ax = draw_weighted_graph(edges_example, coords_example, default_cost=1.0)
    plt.title('Grafo ponderado')
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
