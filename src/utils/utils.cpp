#include "utils.h"

#define SPEED 11

double cartesianDistance( double x1, double y1, double x2, double y2 ){
    return sqrt(pow(x2-x1,2)+pow(y2-y1,2));
}

double distanceToTime(double dist){
    return dist/SPEED;
}

long int timeToSeconds(const std::string& time){
    int hours, minutes;
    long int seconds;
    char trash;

    std::istringstream ssTime(time);

    ssTime >> hours >> trash;
    if(trash != ':') return -1;

    if(ssTime.peek() == EOF) return hours * 60 * 60;

    ssTime >> minutes >> trash;

    if(ssTime.peek() == EOF) return hours * 60 * 60 + minutes * 60 ;

    if(trash != ':') return -1;
    ssTime >> seconds;

    return hours * 60 * 60 + minutes * 60 + seconds;
}

std::string secondsToTime(long int seconds){
    std::ostringstream ssTime;
    int hours, minutes;

    minutes = (seconds - (seconds % 60))/60;

    hours = (minutes - (minutes % 60))/60;

    ssTime  << std::setfill('0') << std::setw(2)
        << hours << ':'  << std::setw(2)
        << minutes % 60 << ':' << std::setw(2)
        << seconds % 60;

    return ssTime.str();
}

void getChar(char &input) {
    input = getchar();
    if(input != '\n' && input != EOF) std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');
}

void waitForKey() {
    char buffer;

    std::cout << "Press ENTER/RETURN to continue..." << std::endl;
    getChar(buffer);
}

