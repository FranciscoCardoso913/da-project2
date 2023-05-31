//
// Created by franciscocardoso on 16-03-2023.
//

#include "Scrapper.h"

void Scrapper::scrape(Graph &graph, string node_file, string line_file,int option)
{
    if(option==0) {
        scrapeNodes(graph, node_file);
        scrapeLines(graph, line_file);
    }
    else{
        scrapeLinesWithoutNodes( graph,  line_file);
    }
}

void Scrapper::scrapeNodes(Graph &graph, string node_file)
{
    ifstream file(node_file);
    string line;
    string id,lon,lat;

    getline(file, line);
    while (getline(file, line))
    {
        istringstream data(line);
        getline(data, id, ',');
        getline(data, lon, ',');
        getline(data, lat, ',');
        graph.addNode(new Node(stoi(id),stod(lon), stod(lat)));
    }
}

void Scrapper::scrapeLines(Graph &graph, string lines_file)
{
    vector<Line> lines;
    ifstream file(lines_file);
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

        graph.addBidirectionalLine(v1, v2, stod(w));
        lines.push_back(
            Line(graph.findNode(stoi(src)), graph.findNode(stoi(dst)), stod(w))
            );
        lines.push_back(
                Line(graph.findNode(stoi(dst)), graph.findNode(stoi(src)), stod(w))
        );
    }
}

void Scrapper::getValue(string &value, istringstream &data)
{
    getline(data, value, ',');
    int pos = value.find('"');
    if (pos != string::npos)
        value.erase(pos, 1);
    int pos2 = value.find('"');
    if (pos != string::npos and pos2 == string::npos)
    {
        string aux;
        getline(data, aux, '"');
        value += ',' + aux;
    }
    else if (pos2 != string::npos)
    {
        value.erase(pos2, 1);
    }
    pos = value.find('\r');
    if (pos != string::npos)
    {
        value.erase(pos, 1);
    }
    if (value == "-")
        value = "";
}

void Scrapper:: scrapeLinesWithoutNodes(Graph &graph,string &lines_file ){
    vector<Line> lines;
    ifstream file(lines_file);
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
        if (v1 == nullptr) {
                v1=new Node(stoi(src));
                graph.addNode(v1);

        }
        if (v2 == nullptr)
            v2=new Node(stoi(dst));
            graph.addNode(v2);

        graph.addBidirectionalLine(v1, v2, stod(w));
        lines.push_back(
                Line(graph.findNode(stoi(src)), graph.findNode(stoi(dst)), stod(w))
                    );
        lines.push_back(
                Line(graph.findNode(stoi(dst)), graph.findNode(stoi(src)), stod(w))
        );
    }
}