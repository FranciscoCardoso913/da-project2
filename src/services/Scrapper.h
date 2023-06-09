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
     * @brief Scrapes the information from the stations and the edges from the files and stores it in the graph
     * @param graph graph were all the information is going to be stored
     * @param station_file the file containing the stations
     * @param edge_file the file containing the edges
     * @param ignore if the user wants to ignore the first line of the file
     * @brief Complexity O(V+E) being V the number of stations and E the number of edges that exists
     */
    void scrape(Graph &graph, string station_file, string edge_file,int option, bool ignore);

    /**
     * @brief Scrapes the information from the stations and stores it in the graph
     * @param graph graph were all the information is going to be stored
     * @param station_file the file containing the stations
     * @brief Complexity O(V) being V the number of stations that exists
     */
    void scrapeNodes(Graph &graph, string station_file);

    /**
     * @brief Scrapes the information from the edges and stores it in the graph
     * @param graph graph were all the information is going to be stored
     * @param edge_file the file containing the edges
     * @brief Complexity O(E) being E the number of edges that exists
     */
    void scrapeEdges(Graph &graph, string edge_file);

    void scrapeEdgesWithoutNodes(Graph &graph,string &edges_value, bool ignore);
};

#endif // DA_PROJECT1_SCRAPPER_H