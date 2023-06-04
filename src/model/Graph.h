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
typedef  pair<vector<int>,double> Path;
class Graph
{
public:

    ~Graph();

    /**
     * @brief Auxiliary function to find a Node with a given index.
     * @param index the index of the Node we want to find
     * @return if the Node is found returns a pointer to the Node, otherwise returns nullptr
     * @brief Complexity O(1)
     */
    Node *findNode(const int &index) const;

    /**
     * @brief Auxiliary function to find a Edge with a given source and destination.
     * @param src the source of the edge we want to find
     * @param dst the destiny of the edge we want to find
     * @return if the Edge is found returns a pointer to that Edge, otherwise returns nullptr
     * @brief Complexity O(1)
     */
    Edge * findEdge( Node *src,  Node *dst) const;

    /**
     * @brief Adds a Node to the graph
     * @param node node to be added.
     * @return true if successful, and false if a Node with that content already exists.
     * @brief Complexity O(1)
     */
    bool addNode(Node *node);

    /**
     * @brief Adds an Edge to a graph, given the source and destination nodes and the Edge capacity (w).
     * @param src  Source node
     * @param dest  Destination node
     * @param w Capacity of the Edges
     * @return A pointer to the edge that was added, nullptr if either the source or the destination node did not exist
     * @brief Complexity O(N) being N the higher index between the source node and the destination
     */
    Edge*  addEdge(Node *src, Node *dest, double w);

    /**
     * @param src  Source Node
     * @param dest  Destination Node
     * @param w Capacity of the Edges
     * @return False if one of the Nodes does not exist, true otherwise.
     * @brief Adds two edges at the same time, one in each direction, between the source and destination nodes, with the given capacity (w).
     * @brief Complexity O(N) being N the higher index between the source node and the destination
     */
    bool addBidirectionalEdge(Node *src, Node *dst, double w);

    /**
     * @return the vector with all the nodes
     * @brief Complexity O(1)
     */
    vector<Node *> getNodes() const;

    /**
     * @brief Sets the attributes visited and processing of the Node to false and the flow of the edges to 0
     * @brief Complexity O(V+E) being V the number of nodes and E the number of edges
     */
    void reset();


    /**
     * @param node starting node
     * @param path vector with all the the nodes
     * @brief dfs algorithm to find the Hamiltonian Path
     * @brief Complexity O(V+E) being V the number of nodes and E the number of edges
     */
    void dfs(Node* node, vector<Node*> &path) const;

    /**
     * @brief Find the minimum spanning tree (MST) of the graph using the Prim algorithm
     * @param source Starting node
     * @return A vector of Edges which representing the edges in the minimum spanning tree.
     * @brief O(E log N), where E is the number of edges in the graph and N is the number of nodes.
     */
    vector<Edge*> MST(Node* source);

    /**Creates an MST, then checks which nodes from the mst are connected to an odd number of other nodes, after that matches all
     * the nodes that haded an odd number of connections together forming pairs, adds the pairs to the MST making all nodes even
     * degree and then calculates a path that travers all the edges only once,finally it removes the repeated nodes from the path
     * and adds the first node to the last position of the path
     * @brief Applies the Christofide algorithm to solve the the TSP
     * @return a pair being the first element a vector of nodes which consist of the path, and being the second element the
     * weight of the path
     * @brief Complexity O(N² + E log N) being N the number of nodes and E the number of edges
     */
    pair<vector<Node*>, double> christofidesTSP();

    /**Select an initial node and adds to the path, then repeatedly selects the nearest node to the last selected node that has yet
     * to be selected and adds it to the path, in the end adds the initial node to the end of the path
     * @brief Applies the nearest neighbor algorithm to solve the TSP
     * @return a pair being the first element a vector of nodes which consist of the path, and being the second element the
     * weight of the path
     * @brief Complexity O(N²) being N the number of Nodes
     */
    pair<vector<Node*>,double>  nearestNeighborTSP() ;
    /**
     * Order the edges of the graph in descent order of weight and takes the the edges with the least weight and switches
     * the destiny node of the edge with the one that follows the source node in the tour and checks if the total wight improved
     * ,if not it changes back the node and tries the next edge
     * @brief Applies a greedy improvement to a TSP solution
     * @param run Variable that indicates if the algorithm can continue to run
     * @param solution the current solution
     * @param tsp the tsp solution
     * @brief Complexity O(E log E) being E the number of Edges in the graph
     */
    void greedyImprovement(bool * run,double * solution, pair<vector<Node *>, double> &tsp );


    /**
     * @brief completes the toy edges missing from the scrapper
     * @brief Complexity O(N²) being N the number of nodes
     */
    void completeToyEdges();

    /**
     * @brief Calculates the weight of the tour
     * @param tour the tour no be analyse
     * @return the total weight of the tour
     * @brief Complexity O(N) being N the number of nodes of the tour
     */
    double calculateTourCost(vector<Node*> &tour);

    /**Calculates all the combinations of two nodes
     * @brief Generates 2-opt moves
     * @param size number of nodes
     * @return vector of pair with each pair being a combination
     * @brief Complexity O(N²) being N the number of nodes
     */
    vector<pair<int, int>> generate2OptMoves(int size);
    /**
     * @brief Applies move to tour
     * @param tour tour with the path
     * @param move move to be applied
     * @brief Complexity O(N) being N the difference between the position of the first node in the move and the second node
     * in the move
     */
    void apply2OptMove(vector<Node*> &tour, pair<int, int> move);

    /**
     * @brief Lin-Kernighan Algorithm, test various moves in a tour, checking if the total weight decreases and if so
     * maintains the move, otherwise reverts it
     * @param run Variable that indicates if the algorithm can kept running
     * @param solution the current solution
     * @param initialTour the the initial tour to be improve
     * @brief Complexity O(N²2^N) being N the number of nodes in the tour
     */
    void LinKernighan(bool * run, double * solution,pair<vector<Node *>, double> &initialTour );

    /**
     * @brief Calculates the distance between two nodes
     * @param node1 first node
     * @param node2 second node
     * @return the distance between the nodes
     * @brief Complexity O(1)
     */
    double calculateDistance( Node* node1,  Node* node2) ;


protected:
    vector<Node *> nodes;
    vector<vector<Edge *>> edges;
    double **distMatrix = nullptr; // dist matrix for Floyd-Warshall
    int **pathMatrix = nullptr;
    // path matrix for Floyd-Warshall

    /**
     * @brief deletes the graph
     * @brief Complexity (V+E) being V the number of nodes and E the number of edges
     */
    void deleteGraph();

    /**
     * @brief Calculate the nodes that have an odd number of edges in the edges given
     * @param edges vector with the edges
     * @return the indexes of the nodes that have an odd number of edges
     * @brief Complexity O(E) being E the number of edges given
     */
    vector<int> oddDegreeNodes( vector<Edge*> &edges) const;

    /**
     * @brief Given an vector of nodes it matches the nodes with a singular other node forming pairs between the nodes
     * @param nodes vector containing the nodes to be matched
     * @return The edges formed by the matching
     * @brief Complexity O(N²) being N the number of nodes in the vector
     */
    vector<Edge*> perfectMatching (vector<int> nodes) ;
    /**
     * @brief Given a vector of edges, forms a path that traverses every edge once
     * @param edges Edges to form the path
     * @return The path formed
     * @brief Complexity O(E) being E the number of edges in the vector
     */
    vector<int> eulerianCircuit( vector<Edge*> &edges);

    /**
     * @brief Takes a path represented by a vector of int and removes the repeated nodes, in the end it adds
     * the first node to end forming a cycle path
     * @param circuit The path to be processed
     * @return A vector of Nodes containing the new path
     */
    vector<Node*> tspTours(vector<int> &circuit);

    /**
     * @brief Calculates the total weight of a tour
     * @param tsp the tour
     * @return the total weight of the tour
     * @brief Complexity O(N) being N the number of nodes in the tour
     */
    double calculateWeight(vector<Node*> &tsp);

    /**
     * @brief finds the closest node from a given node that has yet to be visited
     * @param node Node to find the closest node
     * @return A pointer to the node closest to node given
     * @brief Complexity O(N) being N the number of nodes
     */
    Node* nearestNeighbor( Node* node) ;
};

void deleteMatrix(int **m, int n);
void deleteMatrix(double **m, int n);

#endif /* GRAPH */