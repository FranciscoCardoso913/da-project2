#include <stack>
#include <climits>
#include <valarray>
#include <cfloat>
#include <unordered_map>
#include "Graph.h"
#include "MutablePriorityQueue.h"

Node *Graph::findNode(const int &index) const
{
    if (index >= nodes.size())
        return nullptr;

    return nodes[index];
}

Edge *Graph::findEdge(Node *src, Node *dst) const
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

Edge *Graph::addEdge(Node *src, Node *dest, double w)
{

    if (src == nullptr || dest == nullptr)
        return nullptr;
    while (src->getIndex() >= edges.size())
        edges.push_back(vector<Edge *>(nodes.size(), nullptr));
    while (dest->getIndex() >= edges[src->getIndex()].size())
        edges[src->getIndex()].push_back(nullptr);
    Edge *edge = src->addEdge(dest, w);
    edges[src->getIndex()][dest->getIndex()] = edge;
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

    auto e1 = addEdge(src, dst, w);
    auto e2 = addEdge(dst, src, w);
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


void Graph::dfs(Node* station, vector<Node*> &path) const {

    station->setVisited(true);
    path.push_back(station);

    for (Edge* edge : station->getMST()) {
        Node* nextStation = edge->getDest();
        if (!nextStation->isVisited()) {
            dfs(nextStation, path);
        }
    }


}


vector<int> Graph::oddDegreeVertices(vector<Edge*> &edges) const {

    vector<int> oddDegreeVertices;
    vector<int> degreeCount(nodes.size(), 0);
    for (const auto &edge : edges)
    {

        degreeCount[edge->getOrig()->getIndex()]++;
        degreeCount[edge->getDest()->getIndex()]++;
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



vector<Edge*> Graph::minimumPerfectMatching(vector<int> oddNodes)

{
    vector<Edge*> perfectMatching;
    reset();
    for (int i = 0; i < oddNodes.size(); i += 1)
    {
        int min = -1;
        int min_weight = INT_MAX;
        Node *src = findNode(oddNodes[i]);
        for (int j = 0; j < oddNodes.size(); j++)
        {
            Node *dest = findNode(oddNodes[j]);
            if(!src->isProcessing() and !dest->isProcessing() and src->getIndex()!=dest->getIndex()) {
                Edge* edge=findEdge(src, dest);
                if(edge== nullptr) continue;

                int weight = edge->getCapacity();
                if (weight < min_weight)
                    min = j;
            }
        }
        if (min != -1)
        {
            Edge *edge = findEdge(src, findNode(oddNodes[min]));
            if (edge == nullptr)
                continue;
            double weight = edge->getCapacity();
            perfectMatching.push_back(edge);
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
            Edge *edge = findEdge(findNode(oddNodes[unmatchedVertex]), findNode(oddNodes[i]));
            if (edge == nullptr)
                continue;
            if (oddNodes[i] != unmatchedVertex && edge->getCapacity() < minWeight)
            {

                minWeight = edge->getCapacity();
                closestVertex = oddNodes[i];
            }
        }

        perfectMatching.push_back(findEdge(findNode(oddNodes[unmatchedVertex]), findNode(closestVertex)));
    }
    return perfectMatching;
}




void findEulerianCircuitHelper(int node, unordered_map<int, vector<Edge*>>& adjacencyList, vector<int>& circuit)


{
    while (!adjacencyList[node].empty())
    {
        Edge* edge = adjacencyList[node].back();
        adjacencyList[node].pop_back();
        findEulerianCircuitHelper(edge->getDest()->getIndex(), adjacencyList, circuit);
    }
    circuit.push_back(node);
}

vector<int> Graph:: findEulerianCircuit(vector<Edge*>& edges)
{
    // Create an adjacency list to represent the graph
    unordered_map<int, vector<Edge*>> adjacencyList;
    for (Edge* edge : edges)
    {
        adjacencyList[edge->getOrig()->getIndex()].push_back(edge);
    }

    // Find the starting node for the Eulerian circuit
    int startNode = edges[0]->getOrig()->getIndex();

    // Find the Eulerian circuit using Hierholzer's algorithm
    vector<int> circuit;
    findEulerianCircuitHelper(startNode, adjacencyList, circuit);

    // Reverse the circuit to obtain the correct order
    reverse(circuit.begin(), circuit.end());

    return circuit;
}



vector<Node*> Graph::tspTours(vector<int> &eulerianCircuit)

{
    vector<Node*> tspTour;
    reset();
    for (int node : eulerianCircuit)
    {
        if (!findNode(node)->isVisited())
        {
            tspTour.push_back(nodes[node]);
            findNode(node)->setVisited(true);
        }
    }

    return tspTour;
}

double Graph::calculateWeight(vector<Node*> &tsp)
{
    double weight = 0.0;
    double max = 0;
    for (int i = 0; i < tsp.size() - 1; i++)
    {
        weight += findEdge(findNode(tsp[i]->getIndex()), findNode(tsp[i + 1]->getIndex()))->getCapacity();
    }
    return weight;
}



pair<vector<Node*>, double> Graph::christofidesTSP()

{
    // Step 1: Find the minimum spanning tree
    vector<Edge*> minimumSpanningTree = findMinimumSpanningTree(nodes[0]);


    // Step 2: Find the set of vertices with odd degree in the minimum spanning tree
    vector<int> oddNodes = oddDegreeVertices(minimumSpanningTree);

    // Step 3: Find a minimum-weight perfect matching among the odd degree vertices

    vector<Edge*> perfectMatching = minimumPerfectMatching(oddNodes);

    // Step 4: Combine the minimum spanning tree and the perfect matching to form a multigraph
    vector<Edge*> multigraph;
    multigraph.insert(multigraph.end(), minimumSpanningTree.begin(), minimumSpanningTree.end());
    multigraph.insert(multigraph.end(), perfectMatching.begin(), perfectMatching.end());
    int size= multigraph.size();
    // Step 5: Find an Eulerian circuit in the multigraph
    vector<int> eulerianCircuit = findEulerianCircuit(multigraph);
    size= eulerianCircuit.size();
    // Step 6: Convert the Eulerian circuit into a TSP tour
    vector<Node*> tspTour = tspTours(eulerianCircuit);
    size= tspTour.size();

    return make_pair(tspTour, calculateWeight(tspTour));


}
vector<Edge*> Graph::findMinimumSpanningTree(Node* source) {

    if (nodes.empty()) return vector<Edge*>();

    MutablePriorityQueue<Node> q;


    for (auto v : nodes) {
        v->setDist(INF);
        v->setPath(nullptr);
        v->setVisited(false);
        q.insert(v);
    }

    source->setDist(0);

    q.decreaseKey(source);

    // process vertices in the priority queue
    while (!q.empty()) {
        auto v = q.extractMin();
        v->setVisited(true);
        for (auto& e : v->getAdj()) {
            Node* w = e->getDest();
            if (!w->isVisited() && e->getCapacity() < w->getDist()) {
                w->setDist(e->getCapacity());
                w->setPath(e);
                q.decreaseKey(w);
            }
        }
    }

    vector<Edge*> res;
    for (auto node : nodes) {
        if (node->getPath() != nullptr) {
            res.push_back(node->getPath());
        }
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
void Graph::deleteGraph() {
    for (auto node : nodes)
    {
        node->removeOutgoingEdges();
        node->getAdj().clear();
    }
    for (auto node : nodes)
    {
        delete node;
    }
    nodes.clear();
    for(auto edges_: edges){
        for(auto edge: edges_){
            delete edge;
        }
        edges_.clear();
    }
    edges.clear();
}
Graph::~Graph()
{
    deleteMatrix(distMatrix, nodes.size());
    deleteMatrix(pathMatrix, nodes.size());
    deleteGraph();
}
constexpr double EARTH_RADIUS = 6371000.0;

double toRadians(double degrees)
{
    return degrees * M_PI / 180.0;
}

// Function to calculate the Euclidean distance between two nodes
double Graph::calculateDistance(Node *node1, Node *node2)
{
    Edge *line = findEdge(node1, node2);
    if (line != nullptr)
        return line->getCapacity();
    double lat2 = node2->getLat();
    double lat1 = node1->getLat();
    double lon1 = node1->getLon();
    double lon2 = node2->getLon();
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


Node *Graph::findNearestNeighbor(Node *node)
{
    double minDistance = numeric_limits<double>::max();
    Node *nearestNeighbor = nullptr;
    for (Node *neighbor : nodes)
    {
        if(neighbor->isVisited()) continue;
        double distance = calculateDistance(node, neighbor);
        if (distance < minDistance)
        {
            minDistance = distance;
            nearestNeighbor = neighbor;
        }
    }
    return nearestNeighbor;
}




pair<vector<Node *>, double> Graph::nearestNeighborTSP()
{
    reset();
    vector<Node *> tspTour;

    Node *startNode = nodes[0];
    Node *currentNode = startNode;

    currentNode->setVisited(true);

    tspTour.push_back(currentNode);
    int unvisitedNodes=nodes.size()-1;

    while (unvisitedNodes>0)
    {

        Node *nearestNeighbor = findNearestNeighbor(currentNode);

        nearestNeighbor->setVisited(true);

        tspTour.push_back(nearestNeighbor);

        currentNode = nearestNeighbor;

        unvisitedNodes--;
    }

    tspTour.push_back(startNode);
    double weight = 0;
    for (int i = 0; i < tspTour.size() - 1; i++)
    {
        weight += calculateDistance(tspTour[i], tspTour[i + 1]);
        tspTour[i]->setTSPIndex(i);
    }
    pair<vector<Node*>,double> tsp={tspTour, weight};

    return tsp;

}
void Graph::greedyImprovement(bool* run,double *solution,pair<vector<Node*>,double> &tsp){
    vector<Edge> edgesVector;
    for(auto edges_ : edges){
        for(auto edge: edges_){
            if(edge!= nullptr)edgesVector.push_back(*edge);
        }
    }
    sort(edgesVector.begin(),edgesVector.end(),[](Edge a, Edge b){
        return a.getCapacity() > b.getCapacity();
    });

    while (!edgesVector.empty() and *run){


        Edge edge=edgesVector.back();
        edgesVector.pop_back();
        if(edge.getDest()->getTSPIndex()==0 || tsp.first[edge.getOrig()->getTSPIndex()+1]->getTSPIndex()==0) continue;
        double weightBefore=0;
        if(findEdge(tsp.first[edge.getOrig()->getTSPIndex() ],tsp.first[edge.getOrig()->getTSPIndex() +1 ])== nullptr){
            continue;
        }
        weightBefore+= findEdge(tsp.first[edge.getOrig()->getTSPIndex() ],tsp.first[edge.getOrig()->getTSPIndex() +1 ])->getCapacity();
        if(findEdge(tsp.first[edge.getOrig()->getTSPIndex() +1 ],tsp.first[edge.getOrig()->getTSPIndex() +2 ])== nullptr){
            continue;
        }
        weightBefore+= findEdge(tsp.first[edge.getOrig()->getTSPIndex() +1 ],tsp.first[edge.getOrig()->getTSPIndex() +2 ])->getCapacity();
        if(findEdge(tsp.first[edge.getDest()->getTSPIndex() -1 ],tsp.first[edge.getDest()->getTSPIndex()  ])== nullptr){
            continue;
        }
        weightBefore+= findEdge(tsp.first[edge.getDest()->getTSPIndex() -1 ],tsp.first[edge.getDest()->getTSPIndex()  ])->getCapacity();
        if(findEdge(tsp.first[edge.getDest()->getTSPIndex() ],tsp.first[edge.getDest()->getTSPIndex() +1 ])== nullptr){
            continue;
        }
        weightBefore+= findEdge(tsp.first[edge.getDest()->getTSPIndex() ],tsp.first[edge.getDest()->getTSPIndex() +1 ])->getCapacity();

        double  weightAfter=0;
        if(findEdge(tsp.first[edge.getOrig()->getTSPIndex() ],tsp.first[edge.getDest()->getTSPIndex()  ])==nullptr){
            continue;
        }
        weightAfter+= findEdge(tsp.first[edge.getOrig()->getTSPIndex() ],tsp.first[edge.getDest()->getTSPIndex()  ])->getCapacity();
        if(findEdge(tsp.first[edge.getDest()->getTSPIndex() ],tsp.first[edge.getOrig()->getTSPIndex() +2 ])==nullptr){

            continue;
        }
        weightAfter+= findEdge(tsp.first[edge.getDest()->getTSPIndex() ],tsp.first[edge.getOrig()->getTSPIndex() +2 ])->getCapacity();
        if(findEdge(tsp.first[edge.getDest()->getTSPIndex() -1 ],tsp.first[edge.getOrig()->getTSPIndex() +1 ])==nullptr){
            continue;
        }
        weightAfter+= findEdge(tsp.first[edge.getDest()->getTSPIndex() -1 ],tsp.first[edge.getOrig()->getTSPIndex() +1 ])->getCapacity();
        if(findEdge(tsp.first[edge.getOrig()->getTSPIndex() +1],tsp.first[edge.getDest()->getTSPIndex() +1 ])==nullptr){
            continue;
        }
        weightAfter+= findEdge(tsp.first[edge.getOrig()->getTSPIndex() +1],tsp.first[edge.getDest()->getTSPIndex() +1 ])->getCapacity();
        double diff= weightAfter-weightBefore;



        if(diff<0) {
                swap(tsp.first[edge.getOrig()->getTSPIndex() + 1], tsp.first[edge.getDest()->getTSPIndex()]);

                int aux = edge.getOrig()->getTSPIndex();
                edge.getOrig()->setTSPIndex( edge.getDest()->getTSPIndex());
                edge.getDest()->setTSPIndex(  aux);
                tsp.second += diff;
                *solution=tsp.second;
        }

    }

}

void Graph::completeRealEdges()
{
    while (edges.size()<nodes.size()) edges.push_back(vector<Edge*>(nodes.size(), nullptr));

    for (int i = 0; i < nodes.size(); i++) {
        while (edges[i].size() < nodes.size()) edges[i].push_back(nullptr);
    }
    for (int i = 0; i < nodes.size(); i++) {
        for (int j = i; j < nodes.size(); j++) {
            if(edges[i][j]== nullptr) addBidirectionalEdge(nodes[i],nodes[j], calculateDistance(nodes[i],nodes[j]));
        }
    }

}


void Graph::completeToyEdges() {


    while (edges.size() < nodes.size())
        edges.push_back(vector<Edge *>(nodes.size(), nullptr));
    for (int i = 0; i < nodes.size(); i++)
    {
        while (edges[i].size() < nodes.size())
            edges[i].push_back(nullptr);
    }
}

double Graph::calculateTourCost(vector<Node *> &tour)
{
    double cost = 0;
    for (int i = 0; i < tour.size() - 1; i++)
    {
        cost += calculateDistance(tour[i], tour[i + 1]);
    }
    return cost;
}

vector<pair<int, int>> Graph::generate2OptMoves(int size)
{
    vector<pair<int, int>> moves;
    for (int i = 1; i < size - 1; i++)
    {
        for (int j = i +1; j < size - 1; j++)
        {
            moves.push_back({i, j});
        }
    }
    return moves;
}

void Graph::apply2OptMove(vector<Node *> &tour, pair<int, int> move)
{
    while (move.first <= move.second)
    {
        swap(tour[move.first], tour[move.second]);
        move.first++;
        move.second--;
    }
}

void Graph::LinKernighan(bool *run, double * solution,pair<vector<Node *>, double> &initialTour )
{

    vector<Node *> tour = initialTour.first;

    vector<Node *> bestTour = tour;
    double bestCost = initialTour.second;
    *solution=bestCost;
    vector<pair<int, int>> moves = generate2OptMoves(tour.size());

    bool improvement = true;
    while (improvement and *run)
    {
        for (const auto &move : moves)
        {
            improvement = false;

            int i = move.first;
            int j = move.second;

            apply2OptMove(tour, move);

            double cost = calculateTourCost(tour);

            if (cost < bestCost)
            {
                bestCost = cost;
                bestTour = tour;
                *solution=bestCost;
                improvement = true;
                break;

            }

            apply2OptMove(tour, move);
        }
    }
    initialTour.second=bestCost;
    initialTour.first=bestTour;

}
