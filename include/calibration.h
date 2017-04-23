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
#include<unordered_set>
//my
#include "loader.h" 
#include "tools.h"

class calibration: public tools, public loader{

public:
  calibration();
 ~calibration();

	void calibrate(std::unordered_set<int> corners2use);
	void getData(std::string folder);
	void CalcMeanSigma(const std::vector<std::pair<float,float> > data, 
				float &meanX, float &sigmaX,
				float &meanY, float &sigmaY);

       CMatrix<float> getHomography(CMatrix<float> V, CMatrix<float> Tmodel, CMatrix<float> Tdata);

       CMatrix<float> CalcTransform(const std::vector<std::pair<float,float> > data );

       std::vector<std::pair<float,float> >  normalize(std::vector<std::pair<float,float> > data, 
                                                                    CMatrix<float> transform);

  	CMatrix<float> create2D2DconstraintsMatrix(std::vector<std::pair<float,float> > model,      
                                                    std::vector<std::pair<float,float> > data );

       std::vector<CTensor<float> > detectedIm;
       std::map< int,std::vector<std::pair<float,float> > > imCorners;

	CMatrix<float> HomographyConstraintsMatrix(std::vector<CMatrix<float> > homography);

	CMatrix<float> calculateInternalParam( CMatrix<float> V, float &lambda);

	CMatrix<float> intrinsics; 
	std::vector<CMatrix<float> > extrinsics;

	CVector<float> calculateLenseDistortion(CMatrix<float> Homography,CMatrix<float> K, std::vector<std::pair<float,float> > model, std::vector<std::pair<float,float> > data);

void calculateExternalParameters(CMatrix<float> &Rot, CMatrix<float> &trans,CMatrix<float> K, CMatrix<float> H );

	void writeIntrinsics();
        
private:
	std::string dir_;
};

#endif //CALIBRATION_H_
