#ifndef CAL_PROJ_UTILS_H
#define CAL_PROJ_UTILS_H

#include <string>
#include <cmath>
#include <sstream>
#include <iostream>
#include <limits>
#include <iomanip>

/**
 * Calculates the cartesian distance between to points
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 * @return - cartesian distance
 */
double cartesianDistance( double x1, double y1, double x2, double y2 );

/**
 * Calculates the total seconds given a time string
 * @param time - string in the format HH:MM:SS
 * @return time in seconds
 */
long int timeToSeconds(const std::string& time);

/**
 * Calculates the time in string format of a given number os seconds
 * @param seconds
 * @return - time in string format HH:MM:SS
 */
std::string secondsToTime(long int seconds);

/**
 * Function that prompts the user for the input of a single char.
 * Reads only the first char input to the buffer, all else is deleted from the buffer.
 * Allows the EOF character.
 *
 * @param input The variable where the input is stored.
 */
void getChar(char &input);

/// Waits for input to continue;
void waitForKey();

/**
 * Given a distance calculates the time taken to transverse it
 * @param dist
 * @return - time in seconds
 */
double distanceToTime(double dist);



#endif //CAL_PROJ_UTILS_H
