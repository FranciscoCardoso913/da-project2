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
     * @brief Scrapes the information from the nodes and the edges from the files and stores it in the graph
     * @param graph graph were all the information is going to be stored
     * @param node_file the file containing the nodes
     * @param edge_file the file containing the edges
     * @param ignore if the user wants to ignore the first line of the file
     * @brief Complexity O(V+E) being V the number of nodes and E the number of edges that exists
     */
    void scrape(Graph &graph, string node_file, string edge_file,int option, bool ignore);

    /**
     * @brief Scrapes the information from the nodes and stores it in the graph
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
     * @brief Scrapes the edges without having the nodes, adds the nodes as they appear in the edges
     * @param graph graph where the information will be stored
     * @param edge_file file with the edges
     * @param ignore if the user wants to ignore the first line of the file
     * @brief Complexity O(E) being E the number of edges that exists
     */
    void scrapeEdgesWithoutNodes(Graph &graph,string &edge_file, bool ignore);
};

#endif // DA_PROJECT1_SCRAPPER_H