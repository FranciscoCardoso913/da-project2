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
     * @brief Constructor of a Station object
     * @param name the name of the station
     * @param district the district of the station
     * @param municipality the municipality of the station
     * @param township the township of the station
     * @param station_edge the edge of the station
     */
    Node(int position, double lon=0,double lat=0,string label="");


    /**
     * @brief Less then operator
     * @param Station the station which is being compared to the current object.
     * @return true if name is alphabetically lesser then the object being compared
     * @return false if name is alphabetically bigger then the object being compared
     * @brief Complexity O(1)
     */
    bool operator<(Node node) const; // // required by MutablePriorityQueue

    /**
     * @return Vector of Adjacent Edges to the Station
     * @brief Complexity O(1)
     */
    std::vector<Edge *> getAdj() const;

    std::vector<Edge *> getMST() const;

    int getIndex()const;

    /**
     * @brief checks if the station has been visited
     * @return visited parameter status
     * @brief Complexity O(1)
     */
    bool isVisited() const;

    /**
     * @brief Check if station is being processed
     * @return processing parameter status
     * @brief Complexity O(1)
     */
    bool isProcessing() const;

    /**
     * @brief Get the current distance from a source
     * @return distance from source Station
     * @brief Complexity O(1)
     */
    double getDist() const;

    /**
     * @brief Get the last connected Path to this Station
     * @return Edge* pointer to edge that forms the path
     * @brief Complexity O(1)
     */
    Edge *getPath() const;

    /**
     * @brief Get the vector of incoming Edges to the Station
     * @return vector of pointers to station's incoming edges
     * @brief Complexity O(1)
     */
    std::vector<Edge *> getIncoming() const;



    /**
     * @brief Disable or enable Station
     * @param _disabled the state to be set
     * @brief Complexity O(1)
     */
    void setDisabled(bool _disabled);

    /**
     * @brief Check if the Station is Disabled
     * @return true if is disabled, false otherwise
     * @brief Complexity O(1)
     */
    bool isDisabled() const;

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
     * @brief Set the last connected Path to the Station
     * @param path last connected path
     * @brief Complexity O(1)
     */
    void setPath(Edge *path);

    /**
     * @brief Change the value of the station's edge
     * @param _station_edge new value of the station's edge
     * @brief Complexity O(1)
     */
    void setStationEdge(string _station_edge);

    /**
     * @brief Auxiliary function to add an outgoing Edge to a Station
     * @param dest Destination Station
     * @param w  Edge capacity
     * @param s  Service
     * @return Edge*
     * @brief Complexity O(1)
     */
    Edge *addEdge(Node *dest, double w);


    bool addMSTEdge(Edge *mstEdge);

    /**
     * @brief Auxiliary function to remove an outgoing Edge to a Station
     * @param destName Station name
     * @return true if successful, and false if such Edge does not exist.
     * @brief Complexity O(n)
     */
    bool removeEdge(int dest);

    /**
     * @brief Removes all outgoing edges of Station
     * @brief Complexity O(n)
     */
    void removeOutgoingEdges();

    int queueIndex;
    double lon,lat;

protected:

   int index;

   string label;
    // identifier
    std::vector<Edge *> adj; // outgoing Edges
    std::vector<Edge *> mst; // MST Edges

    // auxiliary fields
    bool disabled = false;
    bool visited = false;    // used by DFS, BFS, Prim ...
    bool processing = false; // used by isDAG (in addition to the visited attribute)
    int dist = 0;
    Edge *path = nullptr;

    std::vector<Edge *> incoming; // incoming Edges

    /**
     * @brief Deletes the Edge from the Station's Adjacent and Incoming Edges
     * @param Edge
     * @brief Complexity O(L) being L the number of Edges incoming to the Destination Station
     */
    void deleteEdge(Edge *Edge);
};

class Edge
{
public:
    Edge(Node *orig, Node *dest, double w);

    /**
     * @return pointer to Destination's station
     * @brief Complexity O(1)
     */
    Node *getDest() const;

    /**
     * @return Edge Capacity
     * @brief Complexity O(1)
     */
    double getCapacity() const;

    /**
     * @return pointer to Origin's Station
     * @brief Complexity O(1)
     */
    Node *getOrig() const;

    /**
     * @return Pointer to Reverse Edge from Destination to Origin
     * @brief Complexity O(1)
     */
    Edge *getReverse() const;

    /**
     * @return True if Edge is Disabled, false otherwise
     * @brief Complexity O(1)
     */
    bool isDisabled() const;

    /**
     * @return Current Flow passing in the Edge
     * @brief Complexity O(1)
     */
    int getFlow() const;


    /**
     * @brief Changes disabled parameter to true or false depending on the value passed by _disabled
     * @param _disabled desired value for disabled
     * @brief Complexity O(1)
     */
    void setDisabled(bool _disabled);

    /**
     * @brief Changes Edge's reverse edge to the edge passed into _reverse
     * @param _reverse Reverse Edge to be attributed
     * @brief Complexity O(1)
     */
    void setReverse(Edge *_reverse);

    /**
     * @brief Sets current edge's flow to _flow
     * @param _flow desired flow to be attributed
     * @brief Complexity O(1)
     */
    void setFlow(int _flow);


    /**
     * @brief Sets current edge's capacity to _capacity
     * @param _capacity desired capacity to be attributed
     * @brief Complexity O(1)
     */
    void setCapacity(int _capacity);

    /**
     * @brief Prints the information of the Edge
     * @brief Complexity O(1)
     */
    void print(int i);

private:

    Node *dest;           // destination Station
    double capacity;         // Edge capacity, can also be used for capacity
    bool disabled = false;   // is the edge disabled?
    Node *orig;           // Origin station
    Edge *reverse = nullptr; // Opposite Edge
    double flow;             // for flow-related problems
};

#endif /* DA_PROJECT1_NODES_EDGES */