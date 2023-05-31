#include <stack>
#include "Graph.h"

Node *Graph::findNode(const int &index) const
{
    if (index >= nodes.size())
        return nullptr;
    return nodes[index];
}

Edge *Graph::findEdge( Node *src,  Node *dst) const
{
    for(auto edge: src->getAdj()){
        if(edge->getDest()==dst) return edge;
    }
    return nullptr;
}

bool Graph::addNode(Node *node)
{
    nodes.push_back(node);
    return true;
}

bool Graph::addEdge(Node *src, Node *dest, int w)
{
    if (src == nullptr || dest == nullptr)
        return false;
    src->addEdge(dest, w);
    return true;
}

vector<Edge *> Graph::getEdgeVector() const
{
    return edges;
}

bool Graph::addBidirectionalEdges(Node *src, Node *dst, double w)
{
    if (src == nullptr || dst == nullptr)
        return false;
    auto e1 = src->addEdge(dst, w);
    auto e2 = dst->addEdge(src, w);
    edges.push_back(e1);
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
vector<int> Graph::oddDegreeVertices(vector<Edge> edges) const {
    std::vector<int> oddDegreeVertices;
    std::vector<int> degreeCount(this->nodes.size(), 0);
    for (const auto& edge : edges) {
        degreeCount[edge.getOrig()->getIndex()]++;
        degreeCount[edge.getDest()->getIndex()]++;
    }
    for (int i = 0; i < this->nodes.size(); i++) {
        if (degreeCount[i] % 2 == 1) {
            oddDegreeVertices.push_back(i);
        }
    }
    return oddDegreeVertices;
}
vector<Edge > Graph:: minimumPerfectMatching (vector<int> oddNodes) {
    std::vector<Edge> perfectMatching;
    reset();
    for (int i = 0; i < oddNodes.size() - 1; i += 2) {
        Node* src = findNode(oddNodes[i]) ;
        Node* dest = findNode(oddNodes[i+1]) ;
        int weight = findEdge(src,dest)->getCapacity();
        perfectMatching.push_back(Edge(src, dest,weight));
        src->setVisited(true);
        dest->setVisited(true);
    }

    int unmatchedVertex = -1;
    for (int i = 0; i < nodes.size(); i++) {
        if (!findNode(i)) {
            unmatchedVertex = i;
            break;
        }
    }

    if (unmatchedVertex != -1) {
        int minWeight = std::numeric_limits<int>::max();
        int closestVertex = -1;

        for (int i = 0; i < oddNodes.size(); i++) {
            if (oddNodes[i] != unmatchedVertex && findEdge(findNode(unmatchedVertex), findNode(oddNodes[i]))->getCapacity() < minWeight) {
                minWeight = findEdge(findNode(unmatchedVertex), findNode(oddNodes[i]))->getCapacity() ;
                closestVertex = oddNodes[i];
            }
        }

        perfectMatching.push_back(Edge(findNode(unmatchedVertex), findNode(closestVertex), minWeight));
    }

    return perfectMatching;
}
vector<int> Graph:: findEulerianCircuit(vector<Edge> edges ) {
    std::vector<int> circuit;
    std::vector<std::vector<int>> adjList(nodes.size());

    for (const auto& edge : edges) {
        adjList[edge.getOrig()->getIndex()].push_back(edge.getDest()->getIndex());
        adjList[edge.getDest()->getIndex()].push_back(edge.getOrig()->getIndex());
    }
    int currVertex = 0;
    circuit.push_back(currVertex);

    while (!adjList[currVertex].empty()) {
        int nextVertex = adjList[currVertex].back();
        adjList[currVertex].pop_back();

        auto it = std::find(adjList[nextVertex].begin(), adjList[nextVertex].end(), currVertex);
        adjList[nextVertex].erase(it);

        circuit.push_back(nextVertex);
        currVertex = nextVertex;
    }

    return circuit;
}
vector<int> Graph:: tspTours(vector<int> &eulerianCircuit){
    std::vector<int> tspTour;
    reset();
    for (int node : eulerianCircuit) {
        if (!findNode(node)->isVisited()) {
            tspTour.push_back(node);
            findNode(node)->setVisited(true);
        }
    }

    return tspTour;
}
double Graph:: calculateWeight(vector<int> tsp){
    int weight=0;
    for(int i=0; i<tsp.size()-1;i++){
        weight+= findEdge(findNode(tsp[i]), findNode(tsp[i+1]))->getCapacity();
    }
    return weight;
}
pair<vector<int>,int> Graph::christofidesSTP(){
    // Step 1: Find the minimum spanning tree
    std::vector<Edge> minimumSpanningTree = findMinimumSpanningTree();

    // Step 2: Find the set of vertices with odd degree in the minimum spanning tree
    vector<int> oddNodes=oddDegreeVertices(minimumSpanningTree);

    // Step 3: Find a minimum-weight perfect matching among the odd degree vertices
    std::vector<Edge> perfectMatching = minimumPerfectMatching( oddNodes);

    // Step 4: Combine the minimum spanning tree and the perfect matching to form a multigraph
    std::vector<Edge> multigraph;
    multigraph.insert(multigraph.end(), minimumSpanningTree.begin(), minimumSpanningTree.end());
    multigraph.insert(multigraph.end(), perfectMatching.begin(), perfectMatching.end());

    // Step 5: Find an Eulerian circuit in the multigraph
    std::vector<int> eulerianCircuit = findEulerianCircuit(multigraph);

    // Step 6: Convert the Eulerian circuit into a TSP tour
    vector<int> tspTour=tspTours(eulerianCircuit);

    return {tspTour, calculateWeight(tspTour)};
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

int Graph::findParent(vector<int> &parent, int i)
{
    if (parent[i] == i)
        return i;
    return findParent(parent, parent[i]);
}

void Graph::mergeSets(vector<int> &parent, int x, int y)
{
    int xset = findParent(parent, x);
    int yset = findParent(parent, y);
    parent[xset] = yset;
}

vector<Edge> Graph::findMinimumSpanningTree()
{
    vector<Edge> result;
    vector<int> parent(this->getNodes().size());

    for (int i = 0; i < this->getNodes().size(); i++)
        parent[i] = i;

    int edgeCount = 0;
    int index = 0;

    while (edgeCount < this->getNodes().size() - 1)
    {
        Edge *nextEdge = this->getEdgeVector()[index++];
        Node *src = nextEdge->getOrig();
        Node *dst = nextEdge->getDest();

        int x = findParent(parent, src->getIndex());
        int y = findParent(parent, dst->getIndex());

        if (x != y)
        {
            result.emplace_back(src, dst, nextEdge->getCapacity());
            mergeSets(parent, x, y);
            edgeCount++;
        }
    }
    return result;
}