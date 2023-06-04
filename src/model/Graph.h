#ifndef GRAPH
#define GRAPH

#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include <set>
#include "NodesEdges.h"

using namespace std;

class Graph
{
public:
    string name;
    ~Graph();

    /**
     * @brief Auxiliary function to find a Node with a given Name.
     * @brief Complexity O(1)
     * @param index the index of the node we want to find
     * @return if the index is valid returns a pointer to that Node, otherwise returns nullptr
     */
    Node *findNode(const int &index) const;

    /**
     * @brief Auxiliary function to find a Edge with a given source and destination.
     * @brief Complexity O(1)
     * @param src the source of the edge we want to find
     * @param dst the destiny of the edge we want to find
     * @return if the Edge is found returns a pointer to that Edge, otherwise returns nullptr
     */
    Edge* findEdge( Node *src,  Node *dst) const;

    /**
     * @brief Adds a Node with a given content or info to a graph.
     * @brief Complexity O(1)
     * @param Node Node to be added.
     */
    void addNode(Node *node);

    /**
     * @brief Adds an Edge to a graph, given the contents of the source and destination stations and the Edge capacity (w).
     * @brief Complexity O(i)
     * @param src Node Source
     * @param dest Node Destination
     * @param w Capacity of the Edges
     * @param service Type of Trains that will use the Edge
     * @return the Edge if successful, and nullptr if the source or destination Node does not exist.
     */
    Edge* addEdge(Node *src, Node *dest, double w);

    /**
     * @param src Node Source
     * @param dest Node Destination
     * @param w Capacity of the Edges
     * @param service Type of Trains that will use the Edge
     * @return False if one of the Stations does not exist, true otherwise.
     * @brief Complexity O(log N) being N the number of stations.
     * @brief Adds two edges at the same time, one in each direction, between the source and destination stations, with the given capacity (w).
     * complexity O(1)
     */
    bool addBidirectionalEdge(Node *src, Node *dst, double w);

    /**
     * @return the vector with all the stations
     * @brief Complexity O(1)
     */
    vector<Node *> getNodes() const;

    /**
     * @brief Sets the attributes visited and processing of the Node to false and the flow of the edges to 0
     * @brief Complexity O(V+E) being V the number of stations and E the number of edges
     */
    void reset();

    /**
     * @return Vector with all the edges
     * @brief Complexity O(1)
     */
    vector<vector <Edge *>> getEdgeVector() const;

    /**
     * @param node starting Node
     * @param path vector with the path
     * @brief dfs algorithm to find the Hamiltonian Path
     * @brief Complexity
     */
    void dfs(Node* node, vector<Node*> &path) const;

    /**
     * @brief Find the minimum spanning tree (MST) of the graph.
     *
     * This function finds the minimum spanning tree of the graph using Kruskal's algorithm.
     *
     * @return A vector of Edge objects representing the edges in the minimum spanning tree.
     *
     * @brief O(E log V), where E is the number of edges in the graph and V is the number of nodes.
     */
    vector<Edge*> findMinimumSpanningTree(Node* source);

    pair<vector<int>, double> christofidesTSP();

    void completeToyEdges();

    double calculateDistance( Node* node1,  Node* node2) ;

protected:
    vector<Node *> nodes;
    vector<vector<Edge *>> edges;
    double **distMatrix = nullptr; // dist matrix for Floyd-Warshall
    int **pathMatrix = nullptr; // path matrix for Floyd-Warshall

    /**
     * @brief deletes the graph
     * @brief Complexity (V+E) being V the number of stations and E the number of edges
     */
    void deleteGraph();



    vector<int> oddDegreeVertices( vector<Edge> &edges) const;
    vector<Edge> minimumPerfectMatching (vector<int> nodes) ;
    vector<int> findEulerianCircuit( vector<Edge> &edges);
    vector<int> tspTours(vector<int> &eulerianCircuit);
    double calculateWeight(vector<int> &tsp);
    Node* findNearestNeighbor( Node* node,  vector<Node*>& unvisitedNodes) ;
};

void deleteMatrix(int **m, int n);
void deleteMatrix(double **m, int n);

#endif /* GRAPH */