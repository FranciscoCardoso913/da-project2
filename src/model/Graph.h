#ifndef GRAPH
#define GRAPH

#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include <set>

using namespace std;

typedef vector<int> Connections;
typedef pair<Connections, int> Path;
typedef int Station,Line;
class Graph
{
public:
    ~Graph();

    /**
     * @brief Auxiliary function to find a Station with a given Name.
     * @param name the name of the station we want to find
     * @return if the station is found returns a pointer to the station, otherwise returns nullptr
     * @brief Complexity O(V) being V the number of stations
     */
    int *findStation(const string &name) const;

    /**
     * @brief Auxiliary function to find a Line with a given source and destination.
     * @param src the source of the line we want to find
     * @param dst the destiny of the line we want to find
     * @return if the Line is found returns a pointer to that Line, otherwise returns nullptr
     * @brief Complexity O(E) being E the number of lines
     */

    int *findLine(const string &src, const string &dst) const;

    /**
     * @brief Adds a Station with a given content or info to a graph.
     * @param Station Station to be added.
     * @return true if successful, and false if a Station with that content already exists.
     * @brief Complexity O(1)
     */
    bool addStation(int *Station);

    /**
     * @brief Adds an Line to a graph, given the contents of the source and destination stations and the Line capacity (w).
     * @param src Station Source
     * @param dest Station Destination
     * @param w Capacity of the Lines
     * @param service Type of Trains that will use the Line
     * @return true if successful, and false if the source or destination Station does not exist.
     * @brief Complexity O(1)
     */

    bool addLine( int *src, int *dest, int w, int service);

    /**
     * @param src Station Source
     * @param dest Station Destination
     * @param w Capacity of the Lines
     * @param service Type of Trains that will use the Line
     * @return False if one of the Stations does not exist, true otherwise.
     * @brief Complexity O(log N) being N the number of stations.
     * @brief Adds two lines at the same time, one in each direction, between the source and destination stations, with the given capacity (w).
     * complexity O(1)
     */
    bool addBidirectionalLine(Station *src, Station *dst, int w);

    /**
     * @return the vector with all the stations
     * @brief Complexity O(1)
     */
    vector<Station *> getStationVector() const;

    /**
     * @brief Calculates the max flow between two stations
     * @param src station where the trains leave
     * @param dst destination station  where trains arrive
     * @return a pair of integers being the first element the max flow and the second the cost
     * @brief Complexity O(V+E) being V the number of stations and E the number of lines
     */
    pair<int, int> maxFlow(string src, string dst);

    /**
     *@brief Calculates the max flow between two stations and the minimum cost possible with that flow
     *@param src station where the trains leave
     *@param dst destination station where trains arrive
     *@return a pair of ints being the first element the max flow and the second the cost
     *@brief Complexity O( |E| * |V| ) being V the number of stations and E the number of lines
     */
    pair<int, int> cheapestMaxFlow(string src, string dst);

    /**
     * @brief Sets the attributes visited and processing of the station to false and the flow of the lines to 0
     * @brief Complexity O(V+E) being V the number of stations and E the number of lines
     */
    void reset();

    /**
     * @brief Finds a path between two stations having in consideration the flow.
     * @param src station where the trains leave
     * @param dst destination station where the trains arrive
     * @return true if a path exits, false otherwise
     * @brief Complexity O(V+E) being V the number of stations and E the number of lines
     */
    bool findPath(Station *src, Station *dst);

    /**
     * @brief Finds the cheapest path between two stations having in consideration the flow
     * @param src station where the trains leave
     * @param dst destination station where the trains arrive
     * @return true if a path exits, false otherwise
     * @brief Complexity O(|E|*|V| ) being V the number of stations and E the number of lines
     */
    bool findCheapestPath(Station *src, Station *dst);

    /**
     * @brief Finds the bottleNeck of the last path found between two stations
     * @param src station where the trains leave
     * @param dst destination station where the trains arrive
     * @return the bottleNeck
     * @brief Complexity O(V) being V the number of station that exist in the path
     */
    int bottleNeck(Station *src, Station *dst);

    /**
     * @brief Updates the flow of the last path found between two stations
     * @param src station where the trains leave
     * @param dst destination station where the trains arrive
     * @param value bottleNeck
     * @brief Complexity O(V) being V the number of station that exist in the path
     */
    void incrementFlow(Station *src, Station *dst, int value);

    /**
     * @brief Calculates the cost of a flow path between two stations
     * @param src station where the trains leave
     * @param dst destination station where the trains arrive
     * @return the cost of the network between two stations
     * @brief Complexity O(V+E) being V the number of stations and E the number of lines in the path
     */
    int calculateCost(Station *src, Station *dst);

    /**
     * @brief Returns all the paths needed to achieve the maximum flow between two station
     * @param src station where the trains will leave
     * @param dst station where the trains will arrive
     * @return a vector with all the paths needed to achieve the max flow
     * @brief Complexity O(E + V) being E and V the number of lines and station in the paths respectively
     */
    vector<Path> getPaths(string src, string dst);

    /**
     * @brief Disables a line
     * @param line Line to be disabled
     * @brief Complexity O(1)
     */
    void disableLine(Line *line);

    /**
     * @brief Enables a line
     * @param line Line to be enabled
     * @brief Complexity O(1)
     */
    void enableLine(Line *line);

    /**
     * @brief Disables a station
     * @param station Station to be disabled
     * @brief Complexity O(1)
     */
    void disableStation(Station *station);

    /**
     * @brief Enables a station
     * @param station Station to be enabled
     * @brief Complexity O(1)
     */
    void enableStation(Station *station);

    /**
     * @brief Disables all the lines
     * @brief Complexity O(E) being E the number of lines
     */
    void disableAllLines();

    /**
     * @brief Enables all the lines
     * @brief Complexity O(E) being E the number of lines
     */
    void enableAllLines();

    /**
     * @brief Disables all the stations
     * @brief Complexity O(V) being V the number of stations
     */
    void disableAllStations();

    /**
     * @brief Enables all the stations
     * @brief Complexity O(V) being V the number of stations
     */
    void enableAllStations();

    /**
     * @brief Returns all the paths needed to achieve the maximum flow with minimum cost between two station
     * @param src station where the trains will leave
     * @param dst station where the trains will arrive
     * @return a vector with all the paths needed to achieve the max flow with minimum cost
     * @brief Complexity O( |E| * log |V| ) being V and E the number of lines and stations in the paths respectively
     */
    vector<Path> getCheapestPaths(string src, string dst);

    /**
     * @brief Determines which stations are Origins and connects the Distributor to them
     * @brief Complexity O(V) being V the number os stations in the graph
     */
    void calculateOrigins();

    /**
     * @return Vector with all the lines
     * @brief Complexity O(1)
     */
    vector<Line *> getLineVector() const;

    /**
     * @return the origins Vector
     * @brief Complexity O(1)
     */
    vector<Station *> getOrigins() const;

    /**
     * @return the distributor Station
     * @brief Complexity O(1)
     */
    Station getDistributor() const;

    /**
     * @brief Removes the last station inserted in the StationSet
     * @brief Complexity O(1)
     */
    void removeLastStation();

    /**
     * @brief Calculates the number of Strongly Connected Components
     * @return the number of Strongly Connected Components in the graph
     * @brief Complexity O(V+E) being V the number of stations  and E the number of lines
     */
    int sCC();

    /**
     * @param string origin - origin station's name
     * @param string destination - destination station's name
     * @brief Finds the shortest Path between two stations.
     * @brief Complexity O(V+E) being V the number of stations and E the number of lines
     */
    int bfs(Station *station);

protected:
    vector<Station *> stationVector;
    // Station set
    vector<Line *> lineVector;
    vector<Station *> origins;
    double **distMatrix = nullptr; // dist matrix for Floyd-Warshall
    int **pathMatrix = nullptr;    // path matrix for Floyd-Warshall

    /**
     * @brief Returns the current path of flow from one station to another and relaxes the lines of the path
     * @param origin Station where the trains leave
     * @param dst Station where the trains arrive
     * @return the current path of flow from one station to another
     * @brief Complexity O(E) being E the number of lines in the Path
     */
    Path getPath(Station *origin, Station *dst);

    /**
     * @brief Searches for a path of flow with the least stops between two stations
     * @param origin station where the trains will leave
     * @param destination station where the trains will arrive
     * @param paths vector of path where the path is going to be inserted
     * @return true if a path is found, false otherwise.
     * @brief Complexity O(V+E) being V the number of stations and E the number of lines
     */
    bool path_bfs(Station *origin, Station *destination, vector<Path> &paths);

    /**
     * @brief Searches for a path of flow with the minimum cost between two stations
     * @param origin station where the trains will leave
     * @param destination station where the trains will arrive
     * @param paths vector of path where the path is going to be inserted
     * @return true if a path is found, false otherwise.
     * @brief Complexity O(|E| log |V|) being E the number of lines and V the number of stations
     */
    bool path_dijkstra(Station *origin, Station *destination, vector<Path> &paths);

    /**
     * @brief Searches for a path of flow with the least stops between two stations
     * @param origin station where the trains will leave
     * @param destination station where the trains will arrive
     * @param paths vector of path where the path is going to be inserted
     * @param path path that is being explored
     * @brief Complexity O(V+E) being V the number of stations and E the number of lines
     */
    void path_dfs(Station *origin, Station *destination, vector<Path> &paths, Path path);

    /**
     * @brief deletes the graph
     * @brief Complexity (V+E) being V the number of stations and E the number of lines
     */
    void deleteGraph();
};

void deleteMatrix(int **m, int n);
void deleteMatrix(double **m, int n);

#endif /* GRAPH */