import heapq

def depth_first_search(graph, start, visited=None):
    if visited is None:
        visited = set()  # Keep track of visited nodes

    # Mark the current node as visited
    visited.add(start)
    print(start + ' ', end = '')  # This prints the node (you can modify it to collect nodes or other operations)

    # Recur for all the vertices adjacent to this node
    for neighbor in graph[start]:
        if neighbor not in visited:
            depth_first_search(graph, neighbor, visited)

def floyd(graph):
    # Initialize distance matrix
    dist = {u: {v: float('inf') for v in graph} for u in graph}
    for u in graph:
        for v, weight in graph[u].items():
            dist[u][v] = weight

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
        'A': {'B': 4, 'C': 3, 'H': 10},
        'B': {'A': 4, 'C': 1, 'D': 6, 'E': 2, 'H': 4, 'I': 7},
        'C': {'A': 3, 'B': 1, 'D': 5, 'F': 7},
        'D': {'B': 6, 'C': 5, 'E': 3, 'F': 8},
        'E': {'B': 2, 'D': 3, 'G': 9, 'J': 2},
        'F': {'C': 7, 'D': 8, 'G': 2},
        'G': {'E': 9, 'F': 2, 'J': 11},
        'H': {'A': 10, 'B': 4, 'I': 2},
        'I': {'H': 2, 'B': 7},
        'J': {'G': 11, 'E': 2}
    }
    graph1 = {
        'A': {'B': 5, 'C': 2, 'D': 8},
        'B': {'A': 5, 'C': 3, 'E': 4},
        'C': {'A': 2, 'B': 3, 'F': 7},
        'D': {'A': 8, 'E': 6},
        'E': {'B': 4, 'D': 6, 'F': 1},
        'F': {'C': 7, 'E': 1}   
    }

    graph2 = {
        'A': {'B': 6, 'C': 2, 'D': 5},
        'B': {'A': 6, 'C': 1, 'E': 4},
        'C': {'A': 2, 'B': 1, 'D': 3},
        'D': {'A': 5, 'C': 3, 'E': 7},
        'E': {'B': 4, 'D': 7},
        'F': {'G': 6},
        'G': {'F': 6}
    }
    
    start_node = 'A'
    distances = dijkstra(graph2, start_node)
    dijkstraStr = ""
    for key, value in distances.items():
        dijkstraStr += "\t" + f"  {key}: {value}"
    print("Shortest distances from node", start_node, ":", dijkstraStr)
    
    floydRet = floyd(graph)
    print('floyd')
    for vertex, endpts in floydRet.items():
        print(f"{vertex}:", end='')
        for key, value in endpts.items():
            print("\t" + f"  {key}: {value}",end='')
        print()

    print()
    print("DFS Traversal")
    depth_first_search(graph, 'A')
