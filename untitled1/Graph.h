#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <utility> // Required for std::pair

// A pair to represent a neighbor and the weight of the edge
typedef std::pair<int, double> Edge;


class Graph {
public:
    // Adjacency list: maps a node (int) to a vector of its neighbors (Edge)
    std::unordered_map<int, std::vector<Edge>> adj;

    // Set to keep track of all unique nodes in the graph
    std::unordered_set<int> nodes;

    // Method to add a directed edge to the graph
    void add_edge(int u, int v, double weight);

    // --- ADD THE MISSING DECLARATIONS HERE ---

    // The standard Dijkstra's algorithm
    std::unordered_map<int, double> dijkstra(int start_node);

    // The new partitioned algorithm
    std::unordered_map<int, double> partitioned_dijkstra(int start_node, int num_partitions);
};

#endif // GRAPH_H