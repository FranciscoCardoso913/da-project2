#ifndef DA_PROJECT1_NODES_EDGES
#define DA_PROJECT1_NODES_EDGES

#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include "string"

using namespace std;

class Edge;

#define INF std::numeric_limits<int>::max()

class Node
{
public:

    /**
     * @brief Construct a new Node object
     * @param index Node index
     * @param lon Node longitude
     * @param lat Node latitude
     * @brief Complexity O(1)
     */
    Node(int index, double lon=0,double lat=0);


    /**
     * @brief Less then operator
     * @param Node the Node which is being compared to the current object.
     * @return true if the current Node has a smaller distance then the Node being compared to, false otherwise.
     * @brief Complexity O(1)
     */
    bool operator<(Node node) const; // // required by MutablePriorityQueue

    /**
     * @return Vector of Adjacent Edges to the Node
     * @brief Complexity O(1)
     */
    std::vector<Edge *> getAdj() const;

    /**
     * @return Vector of MST Edges to the Node
     * @brief Complexity O(1)
     */
    std::vector<Edge *> getMST() const;

    /**
     * @brief Get the Node index
     * @return Node index
     * @brief Complexity O(1)
     */
    int getIndex()const;

    /**
     * @brief checks if the Node has been visited
     * @return visited parameter status
     * @brief Complexity O(1)
     */
    bool isVisited() const;

    /**
     * @brief Check if Node is being processed
     * @return processing parameter status
     * @brief Complexity O(1)
     */
    bool isProcessing() const;

    /**
     * @brief Get the current distance from a source
     * @return distance from source Node
     * @brief Complexity O(1)
     */
    double getDist() const;

    /**
     * @brief Get the last connected Path to this Node
     * @return Edge* pointer to edge that forms the path
     * @brief Complexity O(1)
     */
    Edge *getPath() const;

    /**
     * @brief Set the Visited object true or false
     * @param visited state to be set
     * @brief Complexity O(1)
     */
    void setVisited(bool visited);

    /**
     * @brief Set the Processing object true or false
     * @param processing state to be set
     * @brief Complexity O(1)
     */
    void setProcessing(bool processing);

    /**
     * @brief Set the current distance from a source
     * @param dist distance to be set
     * @brief Complexity O(1)
     */
    void setDist(double dist);

    /**
     * @brief Set the last connected Path to the Node
     * @param path last connected path
     * @brief Complexity O(1)
     */
    void setPath(Edge *path);

    /**
     * @brief Auxiliary function to add an outgoing Edge to a Node
     * @param dest Destination Node
     * @param w  Edge weight
     * @return Edge*
     * @brief Complexity O(1)
     */
    Edge *addEdge(Node *d, double w);

    /**
     * @brief Auxiliary function to add an MST Edge to a Node
     * @param mstEdge MST Edge
     */
    void addMSTEdge(Edge *mstEdge);

    int queueIndex;
    double lon,lat;
    int tspIndex;
protected:

   int index;

    std::vector<Edge *> adj; // outgoing Edges
    std::vector<Edge *> mst; // MST Edges

    // auxiliary fields
    bool visited = false;    // used by DFS, BFS, Prim ...
    bool processing = false; // used by isDAG (in addition to the visited attribute)
    int dist = 0;

    Edge *path = nullptr;

    std::vector<Edge *> incoming; // incoming Edges

};

class Edge
{
public:
    Edge(Node *orig, Node *dest, double w);

    /**
     * @return pointer to Destination's Node
     * @brief Complexity O(1)
     */
    Node *getDest() const;

    /**
     * @return Edge Capacity
     * @brief Complexity O(1)
     */
    double getWeight() const;

    /**
     * @return pointer to Origin's Node
     * @brief Complexity O(1)
     */
    Node *getOrig() const;

    /**
     * @return Pointer to Reverse Edge from Destination to Origin
     * @brief Complexity O(1)
     */
    Edge *getReverse() const;

    /**
     * @brief Changes Edge's reverse edge to the edge passed into _reverse
     * @param _reverse Reverse Edge to be attributed
     * @brief Complexity O(1)
     */
    void setReverse(Edge *_reverse);


    /**
     * @brief Sets current edge's weight to _capacity
     * @param _capacity desired weight to be attributed
     * @brief Complexity O(1)
     */
    void setWeight(int _capacity);

private:

    Node *dest;           // destination Node
    double weight;         // Edge weight
    Node *orig;           // Origin Node
    Edge *reverse = nullptr; // Opposite Edge
};

#endif /* DA_PROJECT1_NODES_EDGES */