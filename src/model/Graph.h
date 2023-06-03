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
     * @brief Auxiliary function to find a Station with a given Name.
     * @param name the name of the station we want to find
     * @return if the station is found returns a pointer to the station, otherwise returns nullptr
     * @brief Complexity O(V) being V the number of stations
     */
    Node *findNode(const int &position) const;

    /**
     * @brief Auxiliary function to find a Edge with a given source and destination.
     * @param src the source of the edge we want to find
     * @param dst the destiny of the edge we want to find
     * @return if the Edge is found returns a pointer to that Edge, otherwise returns nullptr
     * @brief Complexity O(E) being E the number of edges
     */
    Edge * findEdge( Node *src,  Node *dst) const;

    /**
     * @brief Adds a Station with a given content or info to a graph.
     * @param Station Station to be added.
     * @return true if successful, and false if a Station with that content already exists.
     * @brief Complexity O(1)
     */
    bool addNode(Node *node);

    /**
     * @brief Adds an Edge to a graph, given the contents of the source and destination stations and the Edge capacity (w).
     * @param src Station Source
     * @param dest Station Destination
     * @param w Capacity of the Edges
     * @param service Type of Trains that will use the Edge
     * @return true if successful, and false if the source or destination Station does not exist.
     * @brief Complexity O(1)
     */
    Edge*  addEdge(Node *src, Node *dest, double w);

    /**
     * @param src Station Source
     * @param dest Station Destination
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
     * @brief Sets the attributes visited and processing of the station to false and the flow of the edges to 0
     * @brief Complexity O(V+E) being V the number of stations and E the number of edges
     */
    void reset();

    /**
     * @return Vector with all the edges
     * @brief Complexity O(1)
     */
    vector<vector <Edge *>> getEdgeVector() const;

    /**
     * @brief Removes the last station inserted in the StationSet
     * @brief Complexity O(1)
     */
    void removeLastNode();

    /**
     * @param station starting station
     * @param path vector with the path
     * @brief dfs algorithm to find the Hamiltonian Path
     * @brief Complexity
     */
    void dfs(Node* station, vector<Node*> &path) const;

    /**
     * @brief Find the parent node of a given node in the disjoint set.
     *
     * This function finds the parent node of a given node in the disjoint set using path compression.
     *
     * @param parent The vector representing the parent nodes in the disjoint set.
     * @param i The index of the node to find the parent for.
     * @return The index of the parent node.
     *
     * @brief O(log n), where n is the number of nodes in the disjoint set.
     */
    int findParent(vector<int> &parent, int i);

    /**
     * @brief Merge two sets in the disjoint set.
     *
     * This function merges two sets in the disjoint set using union by rank.
     *
     * @param parent The vector representing the parent nodes in the disjoint set.
     * @param x The index of the first node to merge.
     * @param y The index of the second node to merge.
     *
     * @brief O(log n), where n is the number of nodes in the disjoint set.
     */
    void mergeSets(vector<int> &parent, int x, int y);


    /**
     * @brief Find the minimum spanning tree (MST) of the graph.
     *
     * This function finds the minimum spanning tree of the graph using Kruskal's algorithm.
     *
     * @return A vector of Line objects representing the edges in the minimum spanning tree.
     *
     * @brief O(E log V), where E is the number of edges in the graph and V is the number of nodes.
     */
    vector<Edge*> findMinimumSpanningTree(Node* source);

    pair<vector<Node*>, double> christofidesTSP();

    pair<vector<Node*>,double>  tspTriangularApproximation() ;
    void greddyImprovement(bool * run,double * solution, pair<vector<Node *>, double> &tsp );

    void completeRealEdges();

    void completeToyEdges();


    double calculateTourCost(vector<Node*> &tour);

    vector<pair<int, int>> generate2OptMoves(int size);

    void apply2OptMove(vector<Node*> &tour, pair<int, int> move);

    void LinKernighan(bool * run, double * solution,pair<vector<Node *>, double> &initialTour );

    double calculateDistance( Node* node1,  Node* node2) ;


protected:
    vector<Node *> nodes;
    vector<vector<Edge *>> edges;
    double **distMatrix = nullptr; // dist matrix for Floyd-Warshall
    int **pathMatrix = nullptr;
    // path matrix for Floyd-Warshall

    /**
     * @brief deletes the graph
     * @brief Complexity (V+E) being V the number of stations and E the number of edges
     */
    void deleteGraph();



    vector<int> oddDegreeVertices( vector<Edge> &edges) const;
    vector<Edge> minimumPerfectMatching (vector<int> nodes) ;
    vector<int> findEulerianCircuit( vector<Edge> &edges);

    vector<Node*> tspTours(vector<int> &eulerianCircuit);
    double calculateWeight(vector<Node*> &tsp);

    double calculateWeight(vector<int> &tsp);

    Node* findNearestNeighbor( Node* node,  vector<Node*>& unvisitedNodes) ;
};

void deleteMatrix(int **m, int n);
void deleteMatrix(double **m, int n);

#endif /* GRAPH */