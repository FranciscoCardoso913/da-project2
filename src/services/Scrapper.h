#ifndef DA_PROJECT1_SCRAPPER_H
#define DA_PROJECT1_SCRAPPER_H

#include "fstream"
#include "sstream"
#include "iostream"
#include "string"
#include "set"
#include "../model/Graph.h"

using namespace std;

class Scrapper
{
public:
    /**
     * @brief Scrapes the information from the Nodes and the edges from the files and stores it in the graph
     * @param graph graph were all the information is going to be stored
     * @param node_file the file containing the nodes
     * @param edge_file the file containing the edges
     * @param option Indicates if the nodes will have to be scrapped from the edge file or not
     * @brief Complexity O(V+E) being V the number of stations and E the number of edges that exists
     */
    void scrape(Graph &graph, string node_file, string edge_file,int option);

    /**
     * @brief Scrapes the information from the Nodes and stores it in the graph
     * @param graph graph were all the information is going to be stored
     * @param node_file the file containing the nodes
     * @brief Complexity O(V) being V the number of nodes that exists
     */
    void scrapeNodes(Graph &graph, string node_file);

    /**
     * @brief Scrapes the information from the edges and stores it in the graph
     * @param graph graph were all the information is going to be stored
     * @param edge_file the file containing the edges
     * @brief Complexity O(E) being E the number of edges that exists
     */
    void scrapeEdges(Graph &graph, string edge_file);

    /**
     * @brief Gets the data from the files checking for error and special cases
     * @param value where the data is going to be stored
     * @param data the edge of the file containing the information
     * @brief Complexity O(1)
     */
    void getValue(string &value, istringstream &data);
    /**
     * @brief Scrapes the edges without having the Nodes, adding the Nodes as they appear in the edges
     * @param graph graph were all the information is going to be stored
     * @param edges_file file containing all the edges
     */
    void scrapeEdgesWithoutNodes(Graph &graph,string &edges_file );
};

#endif // DA_PROJECT1_SCRAPPER_H
