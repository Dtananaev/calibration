/*
 * File: calibration.h
 * Author: Denis Tananaev
 * Date: 20.04.2017
 * 
 */
#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#include <iostream>
#include <vector>
#include <string>
#include "CMatrix.h"
#include "CTensor.h"
#include <utility> /* pair*/
#include <algorithm> /* sort */
#include <NMath.h> //SVD
#include <set>
#include <limits>

//my
#include "loader.h" 
#include "tools.h"

class calibration: public tools, public loader{

public:
  calibration();
 ~calibration();
  void getData(std::string dir); 
  std::vector<CTensor<float> > detectedIm;

std::map< int,std::vector<std::pair<float,float> > > imCorners;
private:

};

#endif //CALIBRATION_H_
