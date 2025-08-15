# Breaking the Sorting Barrier: A SSSP Algorithm Implementation 🚀

![Language](https://img.shields.io/badge/C%2B%2B-17-blue.svg) ![Language](https://img.shields.io/badge/Python-3.x-blue.svg) ![Build](https://img.shields.io/badge/Build-CMake-orange.svg) ![License](https://img.shields.io/badge/License-MIT-yellow.svg)

This repository contains a practical implementation and performance analysis of the advanced Single-Source Shortest Path (SSSP) algorithm presented in the research paper "Breaking the Sorting Barrier for Directed Single-Source Shortest Paths". The project chronicles the journey from a theoretical concept to a proof-of-concept in Python, and finally to a high-performance benchmark in C++.

The goal is to explore the real-world performance of this novel algorithm, which is theoretically faster than the classic Dijkstra's algorithm on sparse graphs.

## Key Features ✨

* **Novel Algorithm Implementation**: A full implementation of the paper's core concepts: graph partitioning and frontier reduction to break the sorting barrier.
* **Dual-Language Approach**: The algorithm is built in both **Python** for rapid prototyping and **C++** for high-performance benchmarking.
* **Performance Analysis**: A side-by-side benchmark against a standard Dijkstra's algorithm to analyze the practical performance on a large, sparse graph.
* **In-depth Debugging**: The project journey includes solving real-world C++ build environment and linker issues.

## The Core Concept: How It Works 💡

The algorithm is designed to overcome the primary bottleneck in Dijkstra's algorithm: managing a massive "frontier" of nodes to visit next.

#### The Challenge with Dijkstra's

Think of the standard Dijkstra's algorithm like a search party expanding in a circle. The "frontier" is the entire edge of that circle. On a huge graph, this frontier becomes enormous and slow to sort and manage, creating a performance barrier.

#### Our Algorithm's Strategic Approach

This algorithm works more like a team of strategic surveyors mapping a country:

1.  **Partitioning**: First, the surveyors divide the entire map into smaller, manageable regions. In graph terms, this is **partitioning the vertex set**.
2.  **Frontier Reduction**: Instead of exploring every local street, they identify a few key "highway interchanges" or "pivots" that connect these regions. Their main effort is focused on finding the fastest routes *only between these major hubs*. This small, intelligently chosen set of hubs is the **reduced frontier**.

By running a high-level search on this simplified "highway map" (a **supergraph**) and then doing smaller, local searches within each region, the algorithm avoids managing one massive to-do list and breaks the sorting bottleneck.

## Final C++ Benchmark Results ⏱️

The final benchmark was run on a randomly generated graph with **5,000 nodes** and **25,000 edges**.

Generating a large graph with 5000 nodes and 25000 edges...
Graph generation complete.
Starting benchmark with start node: 0

Running standard Dijkstra...
-> Standard Dijkstra took: 0.0217 seconds

Running Partitioned Dijkstra...
-> Partitioned Dijkstra took: 0.4280 seconds

❌ The partitioned algorithm was 1874.68% slower on this run.


#### Analysis

The result is a fantastic lesson in theoretical vs. practical performance. The partitioned algorithm was slower because its massive setup overhead (partitioning, building subgraphs, running Dijkstra multiple times) outweighs its clever scaling advantage on a graph of this size. The benefits of its `O(m log²/³n)` complexity would likely only appear in a highly-optimized, production-level implementation on graphs with millions or billions of nodes.

## Technology Stack

* **C++ (20)**: For the high-performance implementation and final benchmark.
* **Python**: For the initial proof-of-concept and prototyping.
* **CMake**: For building and managing the C++ project.

## Getting Started

### Prerequisites

* A C++ compiler (like g++)
* CMake (3.16 or higher)
* An IDE like CLion (optional, but recommended)

### Compilation & Execution

Clone the repository and navigate to the C++ project directory. Then, compile and run the benchmark:

```bash
# Navigate to the C++ project folder
cd shortest_path
cd untitled1

# Create a build directory
mkdir build && cd build

# Configure the project with CMake
cmake ..

# Build the executable
cmake --build .

# Run the benchmark
./SSSP

Acknowledgments

This project is an implementation of the concepts described in the following research paper. All credit for the theoretical work goes to the authors.

    Ran Duan, Jiayi Mao, Xiao Mao, Xinkai Shu, and Longhui Yin. Breaking the Sorting Barrier for Directed Single-Source Shortest Paths. arXiv:2504.17033v2 [cs.DS], 2025.

License

This project is licensed under the MIT License - see the LICENSE file for details.
