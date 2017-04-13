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
    void rounder(float& degrees);
    
    CMatrix<float> GaussKernel(int radius);
    CMatrix<float> CannyTreshold(CMatrix<float> nms, float max_value);
    CMatrix<float> nonMaxSupress(CMatrix<float> grad, CMatrix<float> orient);
    CMatrix<float> applyKernel(CMatrix<float> kernel, CMatrix<float> boundary_Image,int border);
    CMatrix<float> addNeumannBoundary(CMatrix<float> aImage,int border_size);
    CMatrix<float> cutNeumannBoundary(CMatrix<float>& image,int border_size); 
    CMatrix<float> CannyEdgeDetector(CMatrix<float> image);
    CMatrix<float> HoughTransform(CMatrix<float> image);
    
private:

};

#endif //TOOLS_H_

