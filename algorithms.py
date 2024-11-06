import heapq

def depth_first_search(graph, start):
    visited = set()
    stack = [start]

    while stack:
        vertex = stack.pop()
        if vertex not in visited:
            visited.add(vertex)
            stack.extend(set(graph[vertex]) - visited)
    
    return visited

def floyd_warshall(graph):
    # Initialize distance matrix
    dist = {u: {v: float('inf') for v in graph} for u in graph}
    for u in graph:
        dist[u][u] = 0
        for v in graph[u]:
            dist[u][v] = 1  # Assuming the edge weight is 1

        # Floyd-Warshall algorithm
    for k in graph:
        for i in graph:
            for j in graph:
                if dist[i][j] > dist[i][k] + dist[k][j]:
                    dist[i][j] = dist[i][k] + dist[k][j]
    return dist

def dijkstra(graph, start):
    # Initialize distances and priority queue
    distances = {node: float('infinity') for node in graph}
    distances[start] = 0
    priority_queue = [(0, start)]  # (distance, node)

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        # Nodes can only get added once, so skip if we already found a better path
        if current_distance > distances[current_node]:
            continue

        # Explore neighbors
        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight

            # Only consider this new path if it's better
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(priority_queue, (distance, neighbor))

    return distances

# Example usage
if __name__ == "__main__":
    # Define the graph as an adjacency list
    graph = {
        'A': {'B': 1, 'C': 4},
        'B': {'A': 1, 'C': 2, 'D': 5},
        'C': {'A': 4, 'B': 2, 'D': 1},
        'D': {'B': 5, 'C': 1}
    }
    graph2 = {
        'A': {'B': -4, 'C': 6},
        'B': {'A': 6, 'C': 2, 'D': 2},
        'C': {'A': 4, 'B': 12, 'D': -1},
        'D': {'B': 5, 'C': 21}
    }
    graph3 = {
        'A': {'B': -4, 'C': 6},
        'B': {'A': 6, 'C': 2},
        'C': {'A': 6, 'B': 2}
    }
    start_node = 'A'
    distances = dijkstra(graph, start_node)
    print("Shortest distances from node", start_node, ":", distances)


# Example usage:
# graph = {
#     'A': ['B', 'C'],
#     'B': ['A', 'D', 'E'],
#     'C': ['A', 'F'],
#     'D': ['B'],
#     'E': ['B', 'F'],
#     'F': ['C', 'E']
# }
# print(depth_first_search(graph, 'A'))
