//
//  Timer.hpp
//  globalrecognition
//
//  Created by Lirone Samoun on 27/05/2016.
//
//

#ifndef Timer_hpp
#define Timer_hpp

#include <sys/time.h>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>


#include "math.h"


class Timer
{
public:
    void startTimer();
    void stopTimer();
    long getTime();
    
    void saveTime(std::string path);
    
private:
    timeval m_start;
    timeval m_end;
    
    std::string m_totalTime;
    
};//Timer




#endif /* Timer_hpp */
