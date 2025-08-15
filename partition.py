from collections import defaultdict
import heapq
import random
import time


class Graph:
    """
    A class to represent a directed graph using an adjacency list.
    """

    def __init__(self):
        self.adj = defaultdict(list)
        self.nodes = set()

    def add_edge(self, u, v, weight):
        """
        Adds a directed edge from node u to node v with a given weight.
        """
        self.adj[u].append((v, weight))
        self.nodes.add(u)
        self.nodes.add(v)

    def dijkstra(self, start_node):
        """
        The standard Dijkstra's algorithm.
        """
        distances = {node: float('inf') for node in self.nodes}
        if start_node not in self.nodes:
            return {}
        distances[start_node] = 0
        priority_queue = [(0, start_node)]

        while priority_queue:
            current_distance, current_node = heapq.heappop(priority_queue)

            if current_distance > distances[current_node]:
                continue

            for neighbor, weight in self.adj.get(current_node, []):
                distance = current_distance + weight
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    heapq.heappush(priority_queue, (distance, neighbor))
        return distances

    def partitioned_dijkstra(self, start_node, num_partitions=10):
        """
        The new partitioned algorithm for benchmarking.
        """
        # 1. Automatically partition nodes using a simple hash
        node_to_partition = {node: hash(node) % num_partitions for node in self.nodes}

        # 2. Identify all boundary nodes
        boundary_nodes = set()
        for u, neighbors in self.adj.items():
            for v, weight in neighbors:
                if node_to_partition.get(u) != node_to_partition.get(v):
                    boundary_nodes.add(u)
                    boundary_nodes.add(v)

        # Ensure the start node is included for the search
        boundary_nodes.add(start_node)

        # 3. Build the supergraph
        supergraph = Graph()
        partitions = defaultdict(list)
        for node, p_id in node_to_partition.items():
            partitions[p_id].append(node)

        # 3a. Add intra-partition shortest paths between boundary nodes
        for p_id, nodes_in_partition in partitions.items():
            subgraph = Graph()
            partition_boundary_nodes = []
            for u in nodes_in_partition:
                if u in boundary_nodes:
                    partition_boundary_nodes.append(u)
                for v, weight in self.adj.get(u, []):
                    if v in nodes_in_partition:
                        subgraph.add_edge(u, v, weight)

            for start_node_sub in partition_boundary_nodes:
                shortest_paths = subgraph.dijkstra(start_node_sub)
                for end_node_sub, distance in shortest_paths.items():
                    if end_node_sub in partition_boundary_nodes and start_node_sub != end_node_sub and distance != float(
                            'inf'):
                        supergraph.add_edge(start_node_sub, end_node_sub, distance)

        # 3b. Add all cross-partition "bridge" edges
        for u, neighbors in self.adj.items():
            for v, weight in neighbors:
                if node_to_partition.get(u) != node_to_partition.get(v):
                    supergraph.add_edge(u, v, weight)

        # 4. Solve the final pathfinding on the much smaller supergraph
        return supergraph.dijkstra(start_node)

    def __repr__(self):
        """
        A helper function to make printing the graph object readable.
        """
        output = ""
        for u, neighbors in self.adj.items():
            output += f"{u} -> {neighbors}\n"
        return output


# --- Standalone function to generate a large random graph ---
def generate_large_graph(num_nodes, num_edges):
    g = Graph()
    print(f"Generating a large graph with {num_nodes} nodes and {num_edges} edges...")
    for i in range(num_edges):
        u = random.randint(0, num_nodes - 1)
        v = random.randint(0, num_nodes - 1)
        weight = random.uniform(1, 100)
        if u != v:
            g.add_edge(u, v, weight)
    print("Graph generation complete.")
    return g


# --- Main execution block for benchmarking ---
if __name__ == "__main__":
    # --- Benchmark Settings ---
    NUM_NODES = 5000
    NUM_EDGES = 25000  # A sparse graph where edges are not much more than nodes
    NUM_PARTITIONS = 20  # Tunable parameter for the new algorithm

    large_g = generate_large_graph(NUM_NODES, NUM_EDGES)
    start_node = random.randint(0, NUM_NODES - 1)
    print(f"Starting benchmark with start node: {start_node}\n")

    # --- Time original Dijkstra ---
    print("Running standard Dijkstra...")
    start_time = time.time()
    dijkstra_results = large_g.dijkstra(start_node)
    end_time = time.time()
    dijkstra_duration = end_time - start_time
    print(f"  -> Standard Dijkstra took: {dijkstra_duration:.4f} seconds")

    # --- Time our Partitioned Algorithm ---
    print("\nRunning Partitioned Dijkstra...")
    start_time = time.time()
    partitioned_results = large_g.partitioned_dijkstra(start_node, num_partitions=NUM_PARTITIONS)
    end_time = time.time()
    partitioned_duration = end_time - start_time
    print(f"  -> Partitioned Dijkstra took: {partitioned_duration:.4f} seconds\n")

    # --- Conclusion ---
    if partitioned_duration < dijkstra_duration:
        speedup = (dijkstra_duration - partitioned_duration) / dijkstra_duration * 100
        print(f"✅ Success! The partitioned algorithm was {speedup:.2f}% faster.")
    else:
        print("❌ The partitioned algorithm was not faster on this run.")