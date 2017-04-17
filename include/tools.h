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
#include <set>
#include <limits>
class tools{

public:
    tools();
    ~tools();

    typedef  struct{
        float theta;
        float rho;        
        std::vector<std::pair<float, float> > intersect;
    }lines;
    //additional tools
    void minMax(CMatrix<float> image, float& min,float& max);
    void diffXY(CMatrix<float> Image,  CMatrix<float> &dx, CMatrix<float> &dy);
    CMatrix<float> GaussKernel(int radius);
    CMatrix<float> applyKernel(CMatrix<float> kernel, CMatrix<float> boundary_Image,int border);
    CMatrix<float> addNeumannBoundary(CMatrix<float> aImage,int border_size);
    CMatrix<float> cutNeumannBoundary(CMatrix<float>& image,int border_size); 
    CMatrix<float> makeBinary(CMatrix<float> image);

    //Morphological operations
    CMatrix<float> Dilation(CMatrix<float> Inimage, int radius);
    CMatrix<float> Erosion(CMatrix<float> Inimage, int radius);
    CMatrix<float> Opening(CMatrix<float> Inimage, int radius);
    CMatrix<float> TopHat(CMatrix<float> Inimage, int radius);

    //based on morphological operation filter for local maximum search (used for Harris corner filtering)
    CMatrix<float> DilationFilter(CMatrix<float> image, int radius);
    std::set<std::pair<float, float> > getALLCornerCoordinates(CMatrix<float> corners, float patch_radius=3);
    //Sobel algorithm
    void SobelEdgeDetector(CMatrix<float> image, CMatrix<float>& Gx,CMatrix<float>& Gy,CMatrix<float>& orient,CMatrix<float>& edges);
    void rounder(float& degrees);

    //Harris algorithm
    CMatrix<float> HarrisEdgeDetector(CMatrix<float> image, std::string regime="corners");

   //Canny edge detector
    CMatrix<float> CannyEdgeDetector(CMatrix<float> image);
    CMatrix<float> CannyTreshold(CMatrix<float> nms, float treshold);
    CMatrix<float> nonMaxSupress(CMatrix<float> grad, CMatrix<float> orient);


    //Hough transforms
    CTensor<float> extractHoughLines(CMatrix<float> edges, CMatrix<float> image, std::vector<std::pair<float,float> >& l1, std::vector<std::pair<float,float> >& l2);
    CMatrix<float> HoughTransform(CMatrix<float> edges,std::vector<int>& theta,std::vector<int>& rho);
    std::vector<std::pair<int,int> > HoughPeaks(CMatrix<float> Hough, int num_peaks);
    std::vector<std::pair<int,int> >  AllHoughPeaks(CMatrix<float> Hough, float treshold); 
    CMatrix<float> secondHoughTransform(std::vector<std::pair<int,int> >  lines,std::vector<int> theta,std::vector<int> rho);



    //find chess board
    void findChessBoardLines(std::vector<std::pair<float,float> > l1, 
                             std::vector<std::pair<float,float> > l2, 
                             std::set< std::pair<float,float> > cornerList, 
                             float dist2corner,
                             CMatrix<float> image);


    void getIntersections(   std::vector<lines>& Lhorizontal,
                             std::vector<lines>& Lvertical,
                             std::vector<std::pair<float,float> >& l1, 
                             std::vector<std::pair<float,float> >& l2, 
                             std::set< std::pair<float,float> > cornerList, 
                             float dist2corner,bool shuffle=true);
    //step 1 filtering of the lines (remove outliers outside of the chessbox)
    void removeOutlierLines(std::vector<lines> Lhorizontal,
                           std::vector<lines> Lvertical,
                           std::vector<std::pair<float,float> >& l1, 
                           std::vector<std::pair<float,float> >& l2,
                           int minIntersect);
    //step 2 filtering of the lines (remove parallel lines inside chessbox)
    void removeChessBoardOutliersLines(std::vector<lines>& Lhorizontal,
                           std::vector<lines>& Lvertical);

    void sortLines(std::vector<std::pair<float,float> >& l1, 
                   std::vector<std::pair<float,float> >& l2);

    void getFinalSetOfLines(std::vector<lines> Lhorizontal,
                           std::vector<lines> Lvertical,
                           std::vector<std::pair<float,float> >& l1, 
                           std::vector<std::pair<float,float> >& l2,CMatrix<float> image);

    //draw Hough lines on top of image
     CTensor<float> drawAllLine(std::vector<std::pair<float,float> > l1, 
                                  std::vector<std::pair<float,float> > l2, 
                                  CMatrix<float> image);
     CTensor<float> drawLine(float rho,float theta, CMatrix<float> image);

private:

};

#endif //TOOLS_H_

