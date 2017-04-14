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
#include "CTensor.h"
#include <utility> /* pair*/
#include <algorithm> /* sort */
#include <NMath.h> //SVD

class tools{

public:
    tools();
    ~tools();
    typedef  struct{
    std::pair<float,float> begin;
    std::vector<std::pair<int, int> > coord;
    std::vector<float > val;
    }distribution;


    void SobelEdgeDetector(CMatrix<float> image, CMatrix<float>& Gx,CMatrix<float>& Gy,CMatrix<float>& orient,CMatrix<float>& edges);


    void rounder(float& degrees);
    void diffXY(CMatrix<float> Image,  CMatrix<float> &dx, CMatrix<float> &dy);  
    CTensor<float> extractHoughLines(CMatrix<float> edges, CMatrix<float> image);
    CMatrix<float> HarrisEdgeDetector(CMatrix<float> image, std::string regime="corners");   

    CMatrix<float> getLocalMaximum(CMatrix<float> corners, float max_pixel_distance);
    CMatrix<float> HoughTransform(CMatrix<float> edges,std::vector<int>& theta,std::vector<int>& rho);
    std::vector<std::pair<int,int> > HoughPeaks(CMatrix<float> Hough, int num_peaks);    
    CMatrix<float> GaussKernel(int radius);
    CMatrix<float> CannyTreshold(CMatrix<float> nms, float treshold);
    CMatrix<float> nonMaxSupress(CMatrix<float> grad, CMatrix<float> orient);
    CMatrix<float> applyKernel(CMatrix<float> kernel, CMatrix<float> boundary_Image,int border);
    CMatrix<float> addNeumannBoundary(CMatrix<float> aImage,int border_size);
    CMatrix<float> cutNeumannBoundary(CMatrix<float>& image,int border_size); 
    CMatrix<float> CannyEdgeDetector(CMatrix<float> image);

    
private:

};

#endif //TOOLS_H_

