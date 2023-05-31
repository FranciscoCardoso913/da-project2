#include <stack>
#include <climits>
#include "Graph.h"

Node *Graph::findNode(const int &index) const
{
    if (index >= nodes.size() || index<0)
        return nullptr;

    return nodes[index];
}
Line *Graph::findLine( Node *src,  Node *dst) const
{
    for(auto line: src->getAdj()){
        if(line->getDest()==dst) return line;
    }
    return nullptr;
}

bool Graph::addNode(Node *node)
{
    while (node->getIndex()>=nodes.size())nodes.push_back(nullptr);
    nodes[node->getIndex()]=node;
    return true;
}

bool Graph::addLine(Node *src, Node *dest, int w)
{
    if (src == nullptr || dest == nullptr)
        return false;
    src->addLine(dest, w);
    return true;
}

vector<Line *> Graph::getLineVector() const
{
    return lines;
}

bool Graph::addBidirectionalLine(Node *src, Node *dst, double w)
{
    if (src == nullptr || dst == nullptr)
        return false;
    auto e1 = src->addLine(dst, w);
    auto e2 = dst->addLine(src, w);
    lines.push_back(e1);
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
        for (Line *line : node->getAdj())
        {
            line->setFlow(0);
        }
    }
}
vector<int> Graph::oddDegreeVertices(vector<Line> lines) const {
    std::vector<int> oddDegreeVertices;
    std::vector<int> degreeCount(this->nodes.size(), 0);
    for (const auto& edge : lines) {
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
vector<Line > Graph:: minimumPerfectMatching (vector<int> oddNodes) {
    std::vector<Line> perfectMatching;
    reset();
    for (int i = 0; i < oddNodes.size() - 1; i += 2) {
        Node* src = findNode(oddNodes[i]) ;
        Node* dest = findNode(oddNodes[i+1]) ;
        int weight = findLine(src,dest)->getCapacity();
        perfectMatching.push_back(Line(src, dest,weight));
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
            if (oddNodes[i] != unmatchedVertex && findLine(findNode(unmatchedVertex), findNode(oddNodes[i]))->getCapacity() < minWeight) {
                minWeight = findLine(findNode(unmatchedVertex), findNode(oddNodes[i]))->getCapacity() ;
                closestVertex = oddNodes[i];
            }
        }

        perfectMatching.push_back(Line(findNode(unmatchedVertex), findNode(closestVertex), minWeight));
    }

    return perfectMatching;
}
vector<int> Graph:: findEulerianCircuit(vector<Line> &edges ) {
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
        weight+= findLine(findNode(tsp[i]), findNode(tsp[i+1]))->getCapacity();
    }
    return weight;
}
pair<vector<int>,int> Graph::christofidesSTP(){
    // Step 1: Find the minimum spanning tree
    /*std::vector<int> minimumSpanningTree = findMinimumSpanningTree();
    cout<<"1";

    // Step 2: Find the set of vertices with odd degree in the minimum spanning tree
    vector<int> oddNodes=oddDegreeVertices(minimumSpanningTree);
    cout<<"2";
    // Step 3: Find a minimum-weight perfect matching among the odd degree vertices
    std::vector<Line> perfectMatching = minimumPerfectMatching( oddNodes);
    cout<<"3";
    // Step 4: Combine the minimum spanning tree and the perfect matching to form a multigraph
    std::vector<Line> multigraph;
    multigraph.insert(multigraph.end(), minimumSpanningTree.begin(), minimumSpanningTree.end());
    multigraph.insert(multigraph.end(), perfectMatching.begin(), perfectMatching.end());
    cout<<"4";
    // Step 5: Find an Eulerian circuit in the multigraph
    std::vector<int> eulerianCircuit = findEulerianCircuit(multigraph);
    cout<<"5";
    // Step 6: Convert the Eulerian circuit into a TSP tour
    vector<int> tspTour=tspTours(eulerianCircuit);
    cout<<"6";
    return {tspTour, calculateWeight(tspTour)};*/
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

vector<int> Graph::findMinimumSpanningTree()
{
    priority_queue<Node, vector<Node>> pq;
    vector<int> key(nodes.size(), INT_MAX);
    vector<int> parent(nodes.size(), -1);
    vector<bool> inMST(nodes.size(), false);

    int src = 0;  // Start from vertex 0

    pq.push(Node(src, 0));
    key[src] = 0;

    while (!pq.empty()) {
        int u = pq.top().getIndex();
        pq.pop();

        inMST[u] = true;

        for (auto node : this->nodes[u]->getAdj()) {
            int v = node->getDest()->getIndex();
            int weight = node->getCapacity();

            if (!inMST[v] && weight < key[v]) {
                key[v] = weight;
                parent[v] = u;
                pq.push(Node(v, key[v]));
            }
        }
    }

    // Store the MST in a vector
    vector<int> mst;
    for (int i = 1; i < nodes.size(); ++i)
        mst.push_back(parent[i]);

    return mst;
    /*vector<Line> result;
    vector<int> parent(this->getNodes().size());

    for (int i = 0; i < this->getNodes().size(); i++)
        parent[i] = i;

    int edgeCount = 0;
    int index = 0;

    while (edgeCount < this->getNodes().size() - 1)
    {
        Line *nextLine = this->getLineVector()[index++];
        Node *src = nextLine->getOrig();
        Node *dst = nextLine->getDest();

        int x = src->getIndex();
        int y =dst->getIndex();

        if (x != y)
        {
            result.emplace_back(src, dst, nextLine->getCapacity());
            mergeSets(parent, x, y);
            edgeCount++;
        }
    }
    return result;*/
}
vector<int> primMST(vector<vector<Node>>& graph, int numVertices) {

}