from collections import defaultdict
import heapq


class Graph:
    """
    A class to represent a directed graph using an adjacency list.
    """

    def __init__(self):
        self.adj = defaultdict(list)
        # --- FIX: Add a set to keep track of all unique nodes ---
        self.nodes = set()

    def add_edge(self, u, v, weight):
        """
        Adds a directed edge from node u to node v with a given weight.
        """
        self.adj[u].append((v, weight))
        # --- FIX: Add both nodes to our set of known nodes ---
        self.nodes.add(u)
        self.nodes.add(v)

    def dijkstra(self, start_node):
        """
        Calculates the shortest path from a start_node to all other nodes
        using Dijkstra's algorithm.
        """
        # --- FIX: Initialize distances for ALL nodes in the graph, not just source nodes ---
        distances = {node: float('inf') for node in self.nodes}

        # Check if the start node is actually in the graph
        if start_node not in self.nodes:
            # Or you could raise an error
            return {}

        distances[start_node] = 0
        priority_queue = [(0, start_node)]

        while priority_queue:
            current_distance, current_node = heapq.heappop(priority_queue)

            if current_distance > distances[current_node]:
                continue

            # Use g.adj.get(current_node, []) to avoid KeyErrors for destination-only nodes
            for neighbor, weight in self.adj.get(current_node, []):
                distance = current_distance + weight

                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    heapq.heappush(priority_queue, (distance, neighbor))

        return distances

    def __repr__(self):
        """
        A helper function to make printing the graph object readable.
        """
        output = ""
        for u, neighbors in self.adj.items():
            output += f"{u} -> {neighbors}\n"
        return output


# --- Let's test our new class ---
if __name__ == "__main__":
    g = Graph()
    g.add_edge('A', 'B', 10)
    g.add_edge('A', 'C', 3)
    g.add_edge('B', 'C', 1)
    g.add_edge('B', 'D', 2)
    g.add_edge('C', 'B', 4)
    g.add_edge('C', 'D', 8)
    g.add_edge('C', 'E', 2)
    g.add_edge('D', 'E', 7)
    g.add_edge('E', 'D', 9)

    # 1. Manually define our graph partitions.
    partitions = {
        'P1': ['A', 'C'],
        'P2': ['B', 'D', 'E']
    }
    node_to_partition = {node: p_id for p_id, nodes in partitions.items() for node in nodes}

    # 2. Identify the boundary nodes.
    boundary_nodes = set()
    for u, neighbors in g.adj.items():
        for v, weight in neighbors:
            if node_to_partition.get(u) != node_to_partition.get(v):
                boundary_nodes.add(u)
                boundary_nodes.add(v)

    print("--- New Algorithm Steps ---")
    print(f"Node to Partition Mapping: {node_to_partition}")
    print(f"Identified Boundary Nodes (Reduced Frontier): {sorted(list(boundary_nodes))}")

    # 2. Build the supergraph
    supergraph = Graph()

    # Step 2a: Add intra-partition shortest paths (with the 'inf' fix)
    for p_id, nodes_in_partition in partitions.items():
        subgraph = Graph()
        partition_boundary_nodes = []
        for u in nodes_in_partition:
            if u in boundary_nodes:
                partition_boundary_nodes.append(u)
            for v, weight in g.adj.get(u, []):
                if v in nodes_in_partition:
                    subgraph.add_edge(u, v, weight)

        for start_node_sub in partition_boundary_nodes:
            shortest_paths = subgraph.dijkstra(start_node_sub)
            for end_node_sub, distance in shortest_paths.items():
                if end_node_sub in partition_boundary_nodes and start_node_sub != end_node_sub and distance != float(
                        'inf'):
                    supergraph.add_edge(start_node_sub, end_node_sub, distance)

    # --- FIX: Step 2b: Add all cross-partition edges ("the bridges") ---
    for u, neighbors in g.adj.items():
        for v, weight in neighbors:
            if node_to_partition.get(u) != node_to_partition.get(v):
                supergraph.add_edge(u, v, weight)

    print("\nFinal, Correct Supergraph:")
    print(supergraph)

    # 3. Solve the shortest paths on the completed supergraph
    start_node_main = 'A'
    supergraph_paths = supergraph.dijkstra(start_node_main)

    print(f"\nFinal Shortest Paths from '{start_node_main}':")
    for node, distance in sorted(supergraph_paths.items()):
        # We only care about the boundary nodes that were in our supergraph
        if node in boundary_nodes:
            print(f"  Distance to {node}: {distance}")