#include "Graph.h"
#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <random>
#include <chrono>
#include <iomanip>

// Implementation of the add_edge method for the Graph class
void Graph::add_edge(int u, int v, double weight) {
    this->adj[u].push_back({v, weight});
    this->nodes.insert(u);
    this->nodes.insert(v);
}

// The standard Dijkstra's algorithm in C++
std::unordered_map<int, double> Graph::dijkstra(int start_node) {
    std::unordered_map<int, double> distances;
    for (int node : this->nodes) {
        distances[node] = std::numeric_limits<double>::max();
    }

    if (this->nodes.find(start_node) == this->nodes.end()) {
        return distances; // Start node not in graph
    }

    distances[start_node] = 0.0;

    // Priority queue stores {distance, node}. C++'s PQ is a max-heap,
    // so we store negative distances to simulate a min-heap.
    std::priority_queue<std::pair<double, int>> pq;
    pq.push({0.0, start_node});

    while (!pq.empty()) {
        double current_dist = -pq.top().first;
        int current_node = pq.top().second;
        pq.pop();

        if (current_dist > distances[current_node]) {
            continue;
        }

        if (this->adj.count(current_node)) {
            for (const auto& edge : this->adj.at(current_node)) {
                int neighbor = edge.first;
                double weight = edge.second;
                double distance = current_dist + weight;

                if (distance < distances[neighbor]) {
                    distances[neighbor] = distance;
                    pq.push({-distance, neighbor});
                }
            }
        }
    }
    return distances;
}

// The new partitioned algorithm for benchmarking in C++
std::unordered_map<int, double> Graph::partitioned_dijkstra(int start_node, int num_partitions) {
    // 1. Automatically partition nodes
    std::unordered_map<int, int> node_to_partition;
    for (int node : this->nodes) {
        node_to_partition[node] = std::hash<int>{}(node) % num_partitions;
    }

    // 2. Identify boundary nodes
    std::unordered_set<int> boundary_nodes;
    for (const auto& pair : this->adj) {
        int u = pair.first;
        for (const auto& edge : pair.second) {
            int v = edge.first;
            if (node_to_partition.count(u) && node_to_partition.count(v) && node_to_partition.at(u) != node_to_partition.at(v)) {
                boundary_nodes.insert(u);
                boundary_nodes.insert(v);
            }
        }
    }
    boundary_nodes.insert(start_node);

    // 3. Build the supergraph
    Graph supergraph;
    std::unordered_map<int, std::vector<int>> partitions;
    for (const auto& pair : node_to_partition) {
        partitions[pair.second].push_back(pair.first);
    }

    // 3a. Add intra-partition paths
    for (const auto& pair : partitions) {
        Graph subgraph;
        std::vector<int> partition_boundary_nodes;
        for (int u : pair.second) {
            if (boundary_nodes.count(u)) {
                partition_boundary_nodes.push_back(u);
            }
            if (this->adj.count(u)) {
                for (const auto& edge : this->adj.at(u)) {
                    int v = edge.first;
                    // Check if neighbor is in the same partition
                    if (node_to_partition.at(v) == pair.first) {
                        subgraph.add_edge(u, v, edge.second);
                    }
                }
            }
        }

        for (int start_node_sub : partition_boundary_nodes) {
            auto shortest_paths = subgraph.dijkstra(start_node_sub);
            for (const auto& path_pair : shortest_paths) {
                int end_node_sub = path_pair.first;
                double distance = path_pair.second;
                if (boundary_nodes.count(end_node_sub) && start_node_sub != end_node_sub && distance != std::numeric_limits<double>::max()) {
                    supergraph.add_edge(start_node_sub, end_node_sub, distance);
                }
            }
        }
    }

    // 3b. Add cross-partition edges
    for (const auto& pair : this->adj) {
        int u = pair.first;
        for (const auto& edge : pair.second) {
            int v = edge.first;
            if (node_to_partition.at(u) != node_to_partition.at(v)) {
                supergraph.add_edge(u, v, edge.second);
            }
        }
    }

    // 4. Solve on the supergraph
    return supergraph.dijkstra(start_node);
}

// Standalone function to generate a large random graph
Graph generate_large_graph(int num_nodes, int num_edges) {
    Graph g;
    std::cout << "Generating a large graph with " << num_nodes << " nodes and " << num_edges << " edges..." << std::endl;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> node_dist(0, num_nodes - 1);
    std::uniform_real_distribution<> weight_dist(1.0, 100.0);

    for (int i = 0; i < num_edges; ++i) {
        int u = node_dist(gen);
        int v = node_dist(gen);
        if (u != v) {
            g.add_edge(u, v, weight_dist(gen));
        }
    }
    std::cout << "Graph generation complete." << std::endl;
    return g;
}

// Main execution block for benchmarking
int main() {
    // Benchmark Settings
    const int NUM_NODES = 5000;
    const int NUM_EDGES = 25000;
    const int NUM_PARTITIONS = 20;

    Graph large_g = generate_large_graph(NUM_NODES, NUM_EDGES);
    int start_node = 0; // Use a consistent start node
    std::cout << "Starting benchmark with start node: " << start_node << std::endl << std::endl;

    // Time original Dijkstra
    std::cout << "Running standard Dijkstra..." << std::endl;
    auto start_time = std::chrono::high_resolution_clock::now();
    auto dijkstra_results = large_g.dijkstra(start_node);
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> dijkstra_duration = end_time - start_time;
    std::cout << std::fixed << std::setprecision(4) << "  -> Standard Dijkstra took: " << dijkstra_duration.count() << " seconds" << std::endl;

    // Time our Partitioned Algorithm
    std::cout << "\nRunning Partitioned Dijkstra..." << std::endl;
    start_time = std::chrono::high_resolution_clock::now();
    auto partitioned_results = large_g.partitioned_dijkstra(start_node, NUM_PARTITIONS);
    end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> partitioned_duration = end_time - start_time;
    std::cout << std::fixed << std::setprecision(4) << "  -> Partitioned Dijkstra took: " << partitioned_duration.count() << " seconds" << std::endl << std::endl;

    // Conclusion
    if (partitioned_duration.count() < dijkstra_duration.count()) {
        double speedup = (dijkstra_duration.count() - partitioned_duration.count()) / dijkstra_duration.count() * 100.0;
        std::cout << std::fixed << std::setprecision(2) << "✅ Success! The partitioned algorithm was " << speedup << "% faster." << std::endl;
    } else {
        double slowdown = (partitioned_duration.count() - dijkstra_duration.count()) / dijkstra_duration.count() * 100.0;
        std::cout << std::fixed << std::setprecision(2) << "❌ The partitioned algorithm was " << slowdown << "% slower on this run." << std::endl;
    }

    return 0;
}