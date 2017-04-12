/*
 * File: tools.h
 * Author: Denis Tananaev
 * Date: 12.04.2017
 * 
 */

#ifndef TOOLS_H_
#define TOOLS_H_

#include <iostream>
#include <vector>
#include <string>
#include "CMatrix.h"
class tools{

public:
    tools();
    ~tools();
    CMatrix<float> addNeumannBoundary(CMatrix<float> aImage,int border_size);
    CMatrix<float> cutNeumannBoundary(CMatrix<float>& image,int border_size) 
    CMatrix<float> CannyEdgeDetector(CMatrix<float> image);
    CMatrix<float> HoughTransform(CMatrix<float> image);
    
private:

};

#endif //TOOLS_H_

