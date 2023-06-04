#include "Scrapper.h"

void Scrapper::scrape(Graph &graph, string node_file, string edge_file,int option, bool ignore)
{
    if(option==0) {
        scrapeNodes(graph, node_file);
        scrapeEdges(graph, edge_file);
    }
    else{
        scrapeEdgesWithoutNodes( graph,  edge_file, ignore);
    }
}

void Scrapper::scrapeNodes(Graph &graph, string node_file)
{
    ifstream file(node_file);
    string edge;
    string id,lon,lat;

    getline(file, edge);
    while (getline(file, edge))
    {
        istringstream data(edge);
        getline(data, id, ',');
        getline(data, lon, ',');
        getline(data, lat, ',');
        graph.addNode(new Node(stoi(id),stod(lon), stod(lat)));
    }
}

void Scrapper::scrapeEdges(Graph &graph, string edges_file)
{
    vector<Edge> edges;
    ifstream file(edges_file);
    string line;
    string src,dst,w;
    getline(file, line);

    while (getline(file, line))
    {
        istringstream data(line);
        getline(data, src, ',');
        getline(data, dst, ',');
        getline(data, w, ',');
        auto v1 = graph.findNode(stoi(src));
        auto v2 = graph.findNode(stoi(dst));
        if (v1 == nullptr)
            cout << src << " Not found" << endl;
        if (v2 == nullptr)
            cout << dst << " Not found" << endl;

        graph.addBidirectionalEdge(v1, v2, stod(w));
        edges.push_back(
            Edge(graph.findNode(stoi(src)), graph.findNode(stoi(dst)), stod(w))
        );
        edges.push_back(
            Edge(graph.findNode(stoi(dst)), graph.findNode(stoi(src)), stod(w))
        );
    }
}

void Scrapper:: scrapeEdgesWithoutNodes(Graph &graph,string &edges_file, bool ignore){
    vector<Edge> edges;
    ifstream file(edges_file);
    string line;
    string src,dst,w;
    if (ignore) {
        getline(file, line);
    }

    while (getline(file, line))
    {
        istringstream data(line);
        getline(data, src, ',');
        getline(data, dst, ',');
        getline(data, w, ',');
        auto v1 = graph.findNode(stoi(src));
        auto v2 = graph.findNode(stoi(dst));
        if (v1 == nullptr) {
                v1=new Node(stoi(src));
                graph.addNode(v1);

        }
        if (v2 == nullptr)
            v2=new Node(stoi(dst));
            graph.addNode(v2);

        graph.addBidirectionalEdge(v1, v2, stod(w));
        edges.push_back(
                Edge(graph.findNode(stoi(src)), graph.findNode(stoi(dst)), stod(w))
                    );
        edges.push_back(
                Edge(graph.findNode(stoi(dst)), graph.findNode(stoi(src)), stod(w))
        );
    }
}