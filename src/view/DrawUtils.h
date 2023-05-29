#ifndef DA_PROJECT1_DRAWUTILS_H
#define DA_PROJECT1_DRAWUTILS_H

#include "iostream"
#include "string"

using namespace std;

/**
 * @brief Checks an word for the number of special characters it has
 * @param word string related to the word to be checked for special characters
 * @return the number of special characters in the word
 * @brief Complexity O(c) being c the length of the word
 */
int specialChars(string word);

/**
 * @brief Draws an header
 * @param size title for header
 * @param title size of the header
 * @return String to be printed
 * @brief Complexity O(1)
 */
string drawHeader(int size, string title);

/**
 * @brief Draws the Footer
 * @param size footer size
 * @return String to be printed
 * @brief Complexity O(1)
 */
string drawFooter(int size);

/**
 * @brief Draws a number i of fields with a title inside
 * @param fields Title and space for field
 * @param i number of fields
 * @return String to be printed
 * @brief Complexity O(n)
 */
string drawFields(vector<pair<string, int>> fields, int i);

/**
 * @brief Draws line
 * @param size Size of the line
 * @return String to be printed
 * @brief Complexity O(s) being s the size of the line
 */
string drawLine(int size);

/**
 * @brief Draws Sub-Header
 * @param size Size of Subheader
 * @param title Title of SubHeader
 * @return String to be printed
 * @brief Complexity O(|S| + |L|) being S the size of the line and L the size of the title
 */
string drawSubHeader(int size, string title);

#endif // DA_PROJECT1_DRAWUTILS_H
