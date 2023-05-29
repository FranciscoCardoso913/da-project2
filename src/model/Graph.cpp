#include <stack>
#include "Graph.h"

/*Station *Graph::findStation(const string &name) const
{
    for (Station *station : stationVector)

        if (station->getName() == name)
            return station;
    return nullptr;
}

Line *Graph::findLine(const string &src, const string &dst) const
{
    for (Line *line : lineVector)
    {
        if (line->getOrig()->getName() == src && line->getDest()->getName() == dst)
        {
            return line;
        }
    }
    for (Line *line : lineVector)
    {
        if (line->getOrig()->getName() == dst && line->getDest()->getName() == src)
        {
            return line;
        }
    }
    return nullptr;
}

bool Graph::addStation(Station *Station)
{
    stationVector.push_back(Station);
    return true;
}

bool Graph::addLine(Station *src, Station *dest, int w, services service)
{
    if (src == nullptr || dest == nullptr)
        return false;
    src->addLine(dest, w, service);
    return true;
}

void Graph::calculateOrigins()
{
    origins.clear();
    distributor.removeOutgoingLines();

    for (auto station : stationVector)
    {
        if (station->getAdj().size() == 1)
        {
            origins.push_back(station);
            distributor.addLine(station, 1000, NONE);
        }
    }
}

vector<Station *> Graph::getOrigins() const
{
    return this->origins;
}

Station Graph::getDistributor() const
{
    return this->distributor;
}

vector<Line *> Graph::getLineVector() const
{
    return vector<Line *>(lineVector);
}

bool Graph::addBidirectionalLine(Station *src, Station *dst, int w, services service)
{
    if (src == nullptr || dst == nullptr)
        return false;
    auto e1 = src->addLine(dst, w, service);
    auto e2 = dst->addLine(src, w, service);
    lineVector.push_back(e1);
    e1->setReverse(e2);
    e2->setReverse(e1);
    return true;
}

vector<Station *> Graph::getStationVector() const
{

    return stationVector;
}



int Graph::sCC()
{

    for (auto station : stationVector)
        station->setVisited(false);
    int count = 0;
    for (auto station : stationVector)
    {
        if (!station->isVisited())
        {

            bfs(station);
            count++;
        }
    }
    return count;
}

int Graph::bfs(Station *station)
{
    int reachable = 0;
    station->setVisited(true);
    queue<Station *> q;
    q.push(station);
    while (!q.empty())
    {
        auto u = q.front();
        reachable++;
        q.pop();
        for (auto e : u->getAdj())
        {
            if (!e->getDest()->isVisited() and !e->isDisabled() and !e->getDest()->isDisabled())
            {
                e->getDest()->setVisited(true);
                q.push(e->getDest());
            }
        }
    }
    return reachable - 1;
}

void Graph::reset()
{
    for (auto lines : distributor.getAdj())
        lines->setFlow(0);

    distributor.setVisited(false);
    for (Station *station : stationVector)
    {
        station->setVisited(false);
        station->setProcessing(false);
        for (Line *line : station->getAdj())
        {
            line->setFlow(0);
        }
    }
}

int Graph::calculateCost(Station *src, Station *dst)
{
    int cost = 0;
    if (dst->isProcessing())
        return 0;
    dst->setProcessing(true);
    if (src->getName() == dst->getName())
        return 0;
    for (auto line : dst->getIncoming())
    {
        if (line->getFlow() > 0)
        {
            cost += line->getFlow() * line->getCost();
            cost += calculateCost(src, line->getOrig());
        }
    }
    return cost;
}

bool Graph::findPath(Station *src, Station *dst)
{
    distributor.setVisited(false);
    for (Station *Station : stationVector)
    {
        Station->setVisited(false);
    }

    queue<Station *> s;
    s.push(src);
    src->setVisited(true);

    while (!s.empty())
    {
        auto station = s.front();
        s.pop();
        if (station->getName() == dst->getName())
        {
            return true;
        }
        for (Line *line : station->getAdj())
        {
            if (!line->getDest()->isVisited() and !line->isFull() and !line->isDisabled() and
                !line->getDest()->isDisabled())
            {
                line->getDest()->setVisited(true);
                line->getDest()->setPath(line);
                s.push(line->getDest());
            }
        }
        for (Line *line : station->getIncoming())
        {
            if (!line->getOrig()->isVisited() and line->getFlow() > 0 and !line->isDisabled() and
                !line->getOrig()->isDisabled())
            {
                line->getOrig()->setVisited(true);
                line->getOrig()->setPath(line);
                s.push(line->getOrig());
            }
        }
    }
    return false;
}

bool Graph::findCheapestPath(Station *src, Station *dst)
{
    int price = 0;
    for (Station *Station : stationVector)
    {

        Station->setDist(INT16_MAX);
    }
    src->setDist(0);
    for (int i = 0; i < stationVector.size() - 1; i++)
    {
        bool change = false;
        for (auto station : stationVector)
        {
            for (auto line : station->getAdj())
            {
                price = line->getOrig()->getDist() + line->getCost();
                if (line->getDest()->getDist() > price and !line->isFull() and !line->isDisabled() and !line->getDest()->isDisabled())
                {
                    change = true;
                    line->getDest()->setDist(price);
                    line->getDest()->setPath(line);
                }
            }

            for (auto line : station->getIncoming())
            {
                price = station->getDist() - line->getCost();
                if (line->getOrig()->getDist() > price and line->getFlow() > 0 and !line->isDisabled() and !line->getOrig()->isDisabled())
                {
                    change = true;
                    line->getOrig()->setDist(price);
                    line->getOrig()->setPath(line);
                }
            }
        }
        if (!change)
            break;
    }
    return dst->getDist() < INT16_MAX;
}

int Graph::bottleNeck(Station *src, Station *dst)
{
    auto v1 = dst;
    int min = INT16_MAX;
    while (v1->getName() != src->getName())
    {
        int aux;
        if (v1->getPath()->getDest()->getName() == v1->getName())
            aux = v1->getPath()->getCapacity() - v1->getPath()->getFlow();
        else
            aux = v1->getPath()->getFlow();
        if (aux < min)
            min = aux;
        if (v1->getPath()->getDest()->getName() == v1->getName())
        {
            v1 = v1->getPath()->getOrig();
        }
        else
            v1 = v1->getPath()->getDest();
    }
    return min;
}

void Graph::incrementFlow(Station *src, Station *dst, int value)
{
    auto v1 = dst;
    while (v1->getName() != src->getName())
    {
        if (v1->getPath()->getDest()->getName() == v1->getName())
        {
            v1->getPath()->setFlow(v1->getPath()->getFlow() + value);
            v1 = v1->getPath()->getOrig();
        }
        else
        {
            v1->getPath()->setFlow(v1->getPath()->getFlow() - value);
            v1 = v1->getPath()->getDest();
        }
    }
}

pair<int, int> Graph::maxFlow(string src, string dst)
{

    auto v1 = findStation(src);
    if (src == distributor.getName() and v1 == nullptr)
        v1 = &distributor;
    auto v2 = findStation(dst);
    if (v1 == nullptr)
        throw src + " Not Found!";
    if (v2 == nullptr)
        throw dst + " Not Found!";
    if (v2->isDisabled())
        throw v2->getName() + " is disabled!";
    if (v1->isDisabled())
        throw v1->getName() + " is disabled!";

    reset();
    while (findPath(v1, v2))
    {
        // cout<<"Found path";
        int min = bottleNeck(v1, v2);
        incrementFlow(v1, v2, min);
    }
    int m = 0;
    for (auto line : v2->getIncoming())
    {
        m += line->getFlow();
    }
    int cost = calculateCost(v1, v2);
    return make_pair(m, cost);
}

pair<int, int> Graph::cheapestMaxFlow(string src, string dst)
{
    auto v1 = findStation(src);
    if (src == distributor.getName() and v1 == nullptr)
        v1 = &distributor;
    auto v2 = findStation(dst);
    if (v1 == nullptr)
        throw src + " Not Found!";
    if (v2 == nullptr)
        throw dst + " Not Found!";
    if (v2->isDisabled())
        throw v2->getName() + " is disabled!";
    if (v1->isDisabled())
        throw v1->getName() + " is disabled!";
    reset();
    while (findCheapestPath(v1, v2))
    {
        int min = bottleNeck(v1, v2);
        incrementFlow(v1, v2, min);
    }
    int m = 0;
    for (auto line : v2->getIncoming())
    {
        m += line->getFlow();
    }
    int cost = calculateCost(v1, v2);
    return make_pair(m, cost);
}

vector<Path> Graph::getPaths(string src, string dst)
{
    auto v1 = findStation(src);
    if (v1 == nullptr and src == distributor.getName())
        v1 = &distributor;
    auto v2 = findStation(dst);
    for (auto station : stationVector)
        station->setProcessing(false);
    vector<Path> paths;
    while (path_bfs(v1, v2, paths))
    {
        paths.push_back(getPath(v1, v2));
    }
    return paths;
}

void Graph::path_dfs(Station *origin, Station *destination, vector<Path> &paths, Path path)
{
    // bool ignore=false;

    if (origin->getName() == destination->getName())
    {
        paths.push_back(path);
        return;
    }
    for (auto line : origin->getAdj())
    {
        if (line->getFlow() > 0)
        {
            if (line->getDest()->isProcessing())
                continue;
            line->getDest()->setProcessing(true);
            Path auxPath = path;

            if (origin->getName() != distributor.getName())
            {
                auxPath.first.push_back(line);
                if (line->getFlow() < auxPath.second)
                    auxPath.second = line->getFlow();
            }

            path_dfs(line->getDest(), destination, paths, auxPath);
            line->getDest()->setProcessing(false);
        }
    }
    return;
}



void Graph::disableLine(Line *line)
{
    line->setDisabled(true);
}

void Graph::enableLine(Line *line)
{
    line->setDisabled(false);
}

void Graph::disableStation(Station *station)
{
    station->setDisabled(true);
}

void Graph::enableStation(Station *station)
{

    station->setDisabled(false);
}

void Graph::disableAllLines()
{
    for (auto line : lineVector)
    {
        line->setDisabled(true);
        line->getReverse()->setDisabled(true);
    }
}

void Graph::enableAllLines()
{
    for (auto line : lineVector)
    {
        line->setDisabled(false);
        line->getReverse()->setDisabled(false);
    }
}

void Graph::disableAllStations()
{
    for (auto &station : stationVector)
        station->setDisabled(true);
}

void Graph::enableAllStations()
{
    for (auto &station : stationVector)
        station->setDisabled(false);
}

bool Graph::path_bfs(Station *origin, Station *destination, vector<Path> &paths)
{
    for (Station *station : stationVector)
        station->setVisited(false);
    Path path = make_pair(Connections(), INT32_MAX);
    queue<Station *> q;
    origin->setVisited(true);
    q.push(origin);
    bool found = false;
    while (!q.empty())
    {
        auto u = q.front();
        q.pop();
        if (u->getName() == destination->getName())
        {
            found = true;
            break;
        }
        for (auto line : u->getAdj())
        {
            if (!line->getDest()->isVisited() and line->getFlow() > 0)
            {
                line->getDest()->setVisited(true);
                q.push(line->getDest());

                line->getDest()->setPath(line);
            }
        }
    }

    return found;
}

bool Graph::path_dijkstra(Station *origin, Station *destination, vector<Path> &paths)
{
    int price = 0;
    for (Station *Station : stationVector)
    {
        Station->setDist(INF);
    }
    bool found = false;
    priority_queue<pair<int, Station *>, vector<pair<int, Station *>>, greater<pair<int, Station *>>> pq;
    origin->setDist(0);
    pq.push(make_pair(0, origin));
    while (!pq.empty())
    {
        auto station = pq.top().second;
        pq.pop();

        if (station->getName() == destination->getName())
        {
            found = true;
            break;
        }
        for (auto line : station->getAdj())
        {
            price = station->getDist() + line->getCost();
            if (line->getFlow() > 0 and line->getDest()->getDist() > price)
            {
                line->getDest()->setPath(line);
                line->getDest()->setDist(price);
                pq.push(make_pair(price, line->getDest()));
            }
        }
    }
    return found;
}

vector<Path> Graph::getCheapestPaths(std::string src, std::string dst)
{
    auto v1 = findStation(src);
    auto v2 = findStation(dst);
    for (auto station : stationVector)
        station->setProcessing(false);
    vector<Path> paths;
    while (path_dijkstra(v1, v2, paths))
    {
        paths.push_back(getPath(v1, v2));
    }
    return paths;
}

Path Graph::getPath(Station *origin, Station *destination)
{
    Path path = make_pair(Connections(), INT32_MAX);
    auto u = destination;
    while (u->getName() != origin->getName())
    {
        if (u->getPath()->getOrig()->getName() == distributor.getName())
            break;
        if (u->getPath()->getFlow() < path.second)
            path.second = u->getPath()->getFlow();
        path.first.insert(path.first.begin(), u->getPath());
        u = u->getPath()->getOrig();
    }
    for (auto l : path.first)
    {
        l->setFlow(l->getFlow() - path.second);
    }
    return path;
}

void Graph::removeLastStation()
{

    stationVector.pop_back();
}
*/
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
    deleteMatrix(distMatrix, stationVector.size());
    deleteMatrix(pathMatrix, stationVector.size());

}