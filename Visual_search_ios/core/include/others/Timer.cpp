//
//  Timer.cpp
//  globalrecognition
//
//  Created by Lirone Samoun on 27/05/2016.
//
//

#include "Timer.hpp"

void Timer::startTimer(){
    gettimeofday(&m_start, NULL);
}

void Timer::stopTimer(){
    gettimeofday(&m_end, NULL);
}

long Timer::getTime(){
    long seconds, useconds;
    seconds  = m_end.tv_sec  - m_start.tv_sec;
    useconds = m_end.tv_usec - m_start.tv_usec;
    
    if (seconds >= 1 && seconds < 60){
        m_totalTime = "Time required for execution: " +  std::to_string(seconds) ;
        std::cout << m_totalTime << "\n" <<std::endl;
       
      
    }
    else if (seconds >= 60 && seconds < 3600) //minuts
    {
        double min = floor(seconds/60);
        double sec = seconds - min*60 ;
        m_totalTime = "Time required for execution: " + std::to_string(min) + " min " + std::to_string(sec) + " seconds" ;
        std::cout << m_totalTime << "\n"  << std::endl;
      
    }
    else if(seconds < 1) //milisec
    {
        long time_miliseconds = ((seconds) * 1000 + useconds/1000.0);
        m_totalTime = "Time required for execution: " + std::to_string(time_miliseconds) + " miliseconds ";
        std::cout << m_totalTime << "\n"  << std::endl;
    
    }
    else //hours
    {
        double hours = floor(seconds/3600);
        double min = floor( (seconds - hours*3600)/60 );
        double sec = seconds - min*60 - hours*3600;
        m_totalTime = "Time required for execution: " + std::to_string(hours) + " hours " + std::to_string(min) + " min" +
        std::to_string(sec) +  " sec";
         std::cout << m_totalTime << "\n"  <<  std::endl;
        
    }
    return seconds;
}

void Timer::saveTime(std::string path){
    std::ofstream file;
    file.open (path);
    file << m_totalTime << "\n";
    file.close();
}