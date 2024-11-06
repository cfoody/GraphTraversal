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
        'A': {'B': 1, 'C': 4, 'D': 7, 'E': 9, 'I': 1, 'K': 6},
        'B': {'A': 1, 'C': 2, 'D': 5, 'E': 7, 'K': 7, 'I': 2, 'H': 3},
        'C': {'A': 4, 'B': 2, 'D': 1, 'G': 8, 'E': 5, 'F': 12},
        'D': {'B': 5, 'C': 1, 'A': 12, 'E': 4, 'K': 9, 'G': 4, 'H': 21, 'I': 16},
        'E': {'A': 4, 'D': 3, 'B': 9, 'C': 3, 'H': 5, 'E': 2, 'F': 29, 'I': 4, 'K': 14, 'G': 8},
        'F': {'A': 3, 'K': 6, 'E':2, 'B': 2, 'C': 12, 'I': 19, 'G': 9},
        'G': {'B': 47, 'E': 19, 'K': 19, 'A':9, 'F':10, 'D': 5, 'C': 12, 'E': 4, 'J': 14},
        'H': {'J': 5, 'B': 3, 'K': 12, 'A': 12, 'D': 4, 'C': 12},
        'I': {'E': 14, 'B': 15, 'F': 1, 'C': 8, 'K': 12, 'J': 4, 'D':10,},
        'J': {'K': 3, 'H': 4, 'C':9, 'G': 12, 'B': 6, 'F': 5}, 
        'K': {'H': 14, 'D':10, 'G': 4, 'B': 3, 'A':9, 'E': 4}
    }
    graph2 = {
        'A': {'I': 1, 'K': 6, 'B': 1, 'D': 7, 'C': 4, 'E': 9},
        'B': {'A': 1, 'K': 7, 'I': 2, 'C': 2, 'D': 5, 'E': 7, 'H': 3},
        'C': {'D': 1, 'A': 4, 'B': 2, 'F': 12, 'G': 8, 'E': 5},
        'D': {'B': 5, 'C': 1, 'A': 12, 'I': 16, 'G': 4, 'E': 4, 'H': 21, 'K': 9},
        'E': {'D': 3, 'A': 4, 'F': 29, 'G': 8, 'I': 4, 'K': 14, 'C': 3, 'B': 9, 'H': 5, 'E': 2},
        'F': {'B': 2, 'A': 3, 'K': 6, 'I': 19, 'G': 9, 'C': 12, 'E': 2},
        'G': {'F': 10, 'B': 47, 'E': 4, 'D': 5, 'K': 19, 'A': 9, 'C': 12, 'J': 14},
        'H': {'J': 5, 'B': 3, 'C': 12, 'A': 12, 'K': 12, 'D': 4},
        'I': {'E': 14, 'B': 15, 'D': 10, 'C': 8, 'F': 1, 'K': 12, 'J': 4},
        'J': {'C': 9, 'K': 3, 'B': 6, 'G': 12, 'F': 5, 'H': 4},
        'K': {'B': 3, 'A': 9, 'D': 10, 'E': 4, 'G': 4, 'H': 14}
    }
    graph3 = {
        'A': {'D': 7, 'B': 1, 'K': 6, 'I': 1, 'C': 4, 'E': 9},
        'B': {'C': 2, 'A': 1, 'I': 2, 'E': 7, 'D': 5, 'H': 3, 'K': 7},
        'C': {'F': 12, 'B': 2, 'D': 1, 'A': 4, 'G': 8, 'E': 5},
        'D': {'B': 5, 'C': 1, 'A': 12, 'I': 16, 'K': 9, 'G': 4, 'H': 21, 'E': 4},
        'E': {'I': 4, 'A': 4, 'B': 9, 'F': 29, 'D': 3, 'H': 5, 'K': 14, 'G': 8, 'C': 3, 'E': 2},
        'F': {'G': 9, 'C': 12, 'B': 2, 'I': 19, 'K': 6, 'A': 3, 'E': 2},
        'G': {'D': 5, 'A': 9, 'K': 19, 'B': 47, 'C': 12, 'F': 10, 'E': 4, 'J': 14},
        'H': {'K': 12, 'D': 4, 'J': 5, 'B': 3, 'A': 12, 'C': 12},
        'I': {'F': 1, 'B': 15, 'C': 8, 'K': 12, 'J': 4, 'D': 10, 'E': 14},
        'J': {'B': 6, 'C': 9, 'F': 5, 'K': 3, 'H': 4, 'G': 12},
        'K': {'B': 3, 'E': 4, 'A': 9, 'D': 10, 'G': 4, 'H': 14}
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
