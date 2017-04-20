/*
 * File: calibration.cpp
 * Author: Denis Tananaev
 * Date: 20.04.2017
 * 
 */

#include "calibration.h"

calibration::calibration(){
}

calibration::~calibration(){
}

void calibration::getData(std::string dir){
   loadData(dir);//load data
   for(int i=0;i<gimages_.size();++i){
     std::vector<std::pair<float,float> > corners;
     CTensor<float> detected_board;
     bool detect=DetectBoard( gimages_[i], corners,detected_board);
     	if(detect){
          imCorners.push_back(corners);
        }
	detectedIm.push_back(detected_board);
    }
   
}
