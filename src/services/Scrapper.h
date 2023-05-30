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
     * @brief Scrapes the information from the stations and the lines from the files and stores it in the graph
     * @param graph graph were all the information is going to be stored
     * @param station_file the file containing the stations
     * @param line_file the file containing the lines
     * @brief Complexity O(V+E) being V the number of stations and E the number of lines that exists
     */
    void scrape(Graph &graph, string station_file, string line_file,int option);

    /**
     * @brief Scrapes the information from the stations and stores it in the graph
     * @param graph graph were all the information is going to be stored
     * @param station_file the file containing the stations
     * @brief Complexity O(V) being V the number of stations that exists
     */
    void scrapeNodes(Graph &graph, string station_file);

    /**
     * @brief Scrapes the information from the lines and stores it in the graph
     * @param graph graph were all the information is going to be stored
     * @param line_file the file containing the lines
     * @brief Complexity O(E) being E the number of lines that exists
     */
    void scrapeLines(Graph &graph, string line_file);

    /**
     * @brief Gets the data from the files checking for error and special cases
     * @param value where the data is going to be stored
     * @param data the line of the file containing the information
     * @brief Complexity O(1)
     */
    void getValue(string &value, istringstream &data);

    void scrapeLinesWithoutNodes(Graph &graph,string &lines_value );
};

#endif // DA_PROJECT1_SCRAPPER_H
