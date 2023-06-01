#include <stack>
#include <climits>
#include <valarray>
#include "Graph.h"
#include "MutablePriorityQueue.h"


Node *Graph::findNode(const int &index) const
{
    if (index >= nodes.size())
        return nullptr;

    return nodes[index];
}

Edge *Graph::findEdge( Node *src,  Node *dst) const
{
    if (src == nullptr || dst == nullptr)
        return nullptr;

    return edges[src->getIndex()][dst->getIndex()];
}

bool Graph::addNode(Node *node)
{
    while (node->getIndex() >= nodes.size())
        nodes.push_back(nullptr);
    nodes[node->getIndex()] = node;
    return true;
}

Edge* Graph::addEdge(Node *src, Node *dest, double w)
{

        if (src == nullptr || dest == nullptr)
            return nullptr;
        while (src->getIndex() >= edges.size())
            edges.push_back(vector<Edge*> (nodes.size(), nullptr));
        while (dest->getIndex()>= edges[src->getIndex()].size())
            edges[src->getIndex()].push_back(nullptr);
        Edge* edge=src->addEdge(dest, w);
        edges[src->getIndex()][dest->getIndex()]=edge;
        return edge;

}

vector<vector<Edge *>> Graph::getEdgeVector() const
{
    return edges;
}

bool Graph::addBidirectionalEdge(Node *src, Node *dst, double w)
{
    if (src == nullptr || dst == nullptr)
        return false;

    auto e1 = addEdge(src,dst,w);
    auto e2 = addEdge(dst,src,w);
    e1->setReverse(e2);
    e2->setReverse(e1);
    return true;
}

vector<Node *> Graph::getNodes() const
{
    return nodes;
}

void Graph::reset()
{
    for (Node *node : nodes)
    {
        node->setVisited(false);
        node->setProcessing(false);
        for (Edge *edge : node->getAdj())
        {
            edge->setFlow(0);
        }
    }
}

vector<int> Graph::oddDegreeVertices(vector<Edge> &edges) const {
    vector<int> oddDegreeVertices;
    vector<int> degreeCount(nodes.size(), 0);
    for (const auto& edge : edges) {

        degreeCount[edge.getOrig()->getIndex()]++;
        degreeCount[edge.getDest()->getIndex()]++;
    }
    for (int i = 0; i < nodes.size(); i++)
    {
        if (degreeCount[i] % 2 == 1)
        {
            oddDegreeVertices.push_back(i);
        }
    }
    return oddDegreeVertices;
}


vector<Edge> Graph::minimumPerfectMatching(vector<int> oddNodes)
{
    vector<Edge> perfectMatching;
    reset();
    for (int i = 0; i < oddNodes.size() ; i += 1)
    {
        int min=-1;
        int min_weight=INT_MAX;
        Node *src = findNode(oddNodes[i]);
        for(int j=0; j<oddNodes.size();j++) {
            Node *dest = findNode(oddNodes[j]);
            if(!src->isProcessing() and !dest->isProcessing() and src->getIndex()!=dest->getIndex()) {
                Edge* edge=findEdge(src, findNode(oddNodes[min]));
                if(edge== nullptr) continue;
                int weight = edge->getCapacity();
                if(weight<min_weight) min=j;
            }
        }
        if(min!=-1){
            Edge* edge= findEdge(src, findNode(oddNodes[min]));
            if(edge== nullptr) continue;
            double weight = edge->getCapacity();
            perfectMatching.push_back(Edge(src, findNode(oddNodes[min]), weight));
            src->setProcessing(true);
            findNode(oddNodes[min])->setProcessing(true);
        }
    }

    int unmatchedVertex = -1;
    for (int i = 0; i < oddNodes.size(); i++)
    {
        if (!findNode(oddNodes[i])->isProcessing())
        {
            unmatchedVertex = i;
            break;
        }
    }

    if (unmatchedVertex != -1)
    {
        int minWeight = INT_MAX;
        int closestVertex = -1;


        for (int i = 0; i < oddNodes.size(); i++)
        {
            Edge* edge=findEdge(findNode(oddNodes[unmatchedVertex]), findNode(oddNodes[i]));
            if(edge== nullptr) continue;
            if (oddNodes[i] != unmatchedVertex && edge->getCapacity() < minWeight)
            {

                minWeight = edge->getCapacity();
                closestVertex = oddNodes[i];
            }
        }

        perfectMatching.push_back(Edge(findNode(oddNodes[unmatchedVertex]), findNode(closestVertex), minWeight));
    }
    return perfectMatching;
}


vector<int> Graph::findEulerianCircuit( vector<Edge> &edges)
{
    vector<int> circuit;
    vector<vector<int>> adjList(nodes.size());

    for (const auto &edge : edges)
    {
        adjList[edge.getOrig()->getIndex()].push_back(edge.getDest()->getIndex());
        adjList[edge.getDest()->getIndex()].push_back(edge.getOrig()->getIndex());
    }
    int currVertex = 0;
    circuit.push_back(currVertex);

    while (!adjList[currVertex].empty())
    {
        int nextVertex = adjList[currVertex].back();
        adjList[currVertex].pop_back();

        auto it = find(adjList[nextVertex].begin(), adjList[nextVertex].end(), currVertex);
        adjList[nextVertex].erase(it);

        circuit.push_back(nextVertex);
        currVertex = nextVertex;
    }

    return circuit;
}

vector<int> Graph::tspTours( vector<int> &eulerianCircuit)
{
    vector<int> tspTour;
    reset();
    for (int node : eulerianCircuit)
    {
        if (!findNode(node)->isVisited())
        {
            tspTour.push_back(node);
            findNode(node)->setVisited(true);
        }
    }

    return tspTour;
}

double Graph::calculateWeight( vector<int> &tsp)
{
    double weight = 0.0;
    double max=0;
    for (int i = 0; i < tsp.size() - 1; i++)
    {
        weight += findEdge(findNode(tsp[i]), findNode(tsp[i + 1]))->getCapacity();
    }
    return weight;
}


pair<vector<int>, double> Graph::christofidesTSP()
{
    // Step 1: Find the minimum spanning tree
    vector<Edge> minimumSpanningTree = findMinimumSpanningTree();

    // Step 2: Find the set of vertices with odd degree in the minimum spanning tree
    vector<int> oddNodes = oddDegreeVertices(minimumSpanningTree);

    // Step 3: Find a minimum-weight perfect matching among the odd degree vertices

    vector<Edge> perfectMatching = minimumPerfectMatching(oddNodes);

    // Step 4: Combine the minimum spanning tree and the perfect matching to form a multigraph
    vector<Edge> multigraph;
    multigraph.insert(multigraph.end(), minimumSpanningTree.begin(), minimumSpanningTree.end());
    multigraph.insert(multigraph.end(), perfectMatching.begin(), perfectMatching.end());

    // Step 5: Find an Eulerian circuit in the multigraph
    vector<int> eulerianCircuit = findEulerianCircuit(multigraph);

    // Step 6: Convert the Eulerian circuit into a TSP tour
    vector<int> tspTour = tspTours(eulerianCircuit);

    return make_pair(tspTour, calculateWeight(tspTour));
}
vector<Edge> Graph::findMinimumSpanningTree() {
    // Reset auxiliary info
    for (auto v : nodes) {
        v->setDist(INF);
        v->setPath(nullptr);
        v->setVisited(false);
    }

    // start with an arbitrary vertex
    Node* s = nodes.front();
    s->setDist(0);

    // initialize priority queue
    MutablePriorityQueue<Node> q;
    q.insert(s);

    // process vertices in the priority queue
    while (!q.empty()) {
        auto v = q.extractMin();
        v->setVisited(true);
        for (auto& e : v->getAdj()) {
            Node* w = e->getDest();
            if (!w->isVisited()) {
                auto oldDist = w->getDist();
                if (e->getOrig()->getDist() + e->getCapacity() < oldDist) {
                    w->setDist(e->getOrig()->getDist()+ e->getCapacity());
                    w->setPath(e);
                    if (oldDist == INF) {
                        q.insert(w);
                    } else {
                        q.decreaseKey(w);
                    }
                }
            }
        }
    }

    for (auto node : nodes) {
        cout << node->getIndex() << ":" << node->getDist() << endl;
    }

    vector<Edge> res;
    for (auto node : nodes) {
        if (node->getPath() != nullptr) {
            res.push_back(*node->getPath());
        }
    }
    for(auto edge:res){
        cout<<edge.getOrig()->getIndex()<<"-"<<edge.getDest()->getIndex()<<endl;
    }

    return res;
}

void deleteMatrix(int **m, int n)
{
    if (m != nullptr)
    {
        for (int i = 0; i < n; i++)
            if (m[i] != nullptr)
                delete[] m[i];
        delete[] m;
    }
}

void deleteMatrix(double **m, int n)
{
    if (m != nullptr)
    {
        for (int i = 0; i < n; i++)
            if (m[i] != nullptr)
                delete[] m[i];
        delete[] m;
    }
}

Graph::~Graph()
{
    deleteMatrix(distMatrix, nodes.size());
    deleteMatrix(pathMatrix, nodes.size());
}
constexpr double EARTH_RADIUS = 6371000.0;

double toRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

// Function to calculate the Euclidean distance between two nodes
double Graph:: calculateDistance( Node* node1,  Node* node2) {
    Edge * line=findEdge(node1,node2);
    if(line!= nullptr) return line->getCapacity();
    double lat2=node2->lat;
    double lat1= node1->lat;
    double lon1= node1->lon;
    double  lon2= node2->lon;
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;

    lat1 = lat1 * M_PI / 180.0;
    lat2 = lat2 * M_PI / 180.0;

    double a = pow(sin(dLat / 2), 2) +
               pow(sin(dLon / 2), 2) * cos(lat1) * cos(lat2);
    double rad = 6371000;
    double c = 2 * asin(sqrt(a));
    return rad * c;
}


// Function to find the nearest unvisited neighbor of a node
Node* Graph:: findNearestNeighbor( Node* node,  vector<Node*>& unvisitedNodes) {
    double minDistance = numeric_limits<double>::max();
    Node* nearestNeighbor = nullptr;
    for (Node* neighbor : unvisitedNodes) {
        double distance = calculateDistance(node, neighbor);
        if (distance < minDistance) {
            minDistance = distance;
            nearestNeighbor = neighbor;
        }
    }
    return nearestNeighbor;
}

// Triangular Approximation Heuristic for TSP
pair<vector<Node*>,double> Graph:: tspTriangularApproximation() {
    // Create a vector to store the TSP tour
    vector<Node*> tspTour;

    // Get the nodes from the graph


    // Choose a starting node (can be any node)
    Node* startNode = nodes[0];
    Node* currentNode = startNode;

    // Mark the starting node as visited
    currentNode->setVisited(true);

    // Add the starting node to the TSP tour
    tspTour.push_back(currentNode);

    // Create a vector to store the unvisited nodes
    vector<Node*> unvisitedNodes(nodes.begin() + 1, nodes.end());

    // Repeat until all nodes are visited
    while (!unvisitedNodes.empty()) {

        // Find the nearest unvisited neighbor of the current node
        Node* nearestNeighbor = findNearestNeighbor(currentNode, unvisitedNodes);

        // Mark the nearest neighbor as visited
        nearestNeighbor->setVisited(true);

        // Add the nearest neighbor to the TSP tour
        tspTour.push_back(nearestNeighbor);

        // Set the nearest neighbor as the current node
        currentNode = nearestNeighbor;

        // Remove the nearest neighbor from the unvisited nodes
        unvisitedNodes.erase(find(unvisitedNodes.begin(), unvisitedNodes.end(), currentNode));
    }

    // Add the start node to complete the tour
    tspTour.push_back(startNode);
    double weight=0;
    for(int i=0; i<tspTour.size()-1;i++){
        weight+= calculateDistance(tspTour[i], tspTour[i+1]);
    }

    return {tspTour, weight};
}

void Graph::completeRealEdges() {

    for (int i = 0; i < nodes.size(); i++) {
        for (int j = i + 1; j < nodes.size(); j++) {
            if (findEdge(nodes[i], nodes[j]) == NULL) {
                double dist = calculateDistance(nodes[i], nodes[j]);
                Edge *edge = new Edge(nodes[i], nodes[j], dist);
                nodes[i]->addEdge(nodes[j], dist);
                nodes[j]->addEdge(nodes[i], dist);
                edges[i][j] = edge;
                edges[j][i] = edge;
            }

        }
    }

}

void Graph::completeToyEdges() {

    while(edges.size()<nodes.size()) edges.push_back(vector<Edge*>(nodes.size(), nullptr));
    for (int i = 0; i < nodes.size(); i++ ) {
        while (edges[i].size()<nodes.size()) edges[i].push_back(nullptr);
    }

}