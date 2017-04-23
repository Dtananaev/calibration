
#include <cstdlib>   /// contains: EXIT_SUCCESS
#include <iostream>  /// contains: std::cout etc.
#include "CMatrix.h"
#include "CTensor.h"
#include "CVector.h"
#include <vector>
#include <string>
#include <fstream>
#include <NMath.h> //SVD

#include "calibration.h"

bool read_file(std::string filename, std::vector<std::pair<float,float> > &output){

     std::ifstream file_stream_pose(filename);
    if (file_stream_pose.is_open()) { // check if file exsist
         std::string line;
         while (std::getline(file_stream_pose, line)) {
            std::istringstream file_stream_pose(line);

            //each row of txt file contain 4 points related to the corners of the black square
      	   float x1,y1;
      	   float x2,y2;
      	   float x3,y3;
      	   float x4,y4;

           file_stream_pose>>x1>>y1>>x2>>y2>>x3>>y3>>x4>>y4;
           output.push_back(std::make_pair(x1,y1));
           output.push_back(std::make_pair(x2,y2));
           output.push_back(std::make_pair(x3,y3));
           output.push_back(std::make_pair(x4,y4));
         }
         file_stream_pose.close();
    }else{
       return false;
    }

    return true;
}


int main(){

    
	std::vector<std::vector<std::pair<float,float> > > alldata;

    //read files with 2D point correspondences

    std::vector<std::pair<float,float> > model;
    if (!read_file( "../test/model.txt", model)){
        std::cout<<"Can't read model.txt"<<"\n";
    }

    std::vector<std::pair<float,float> >  data1;
    if (!read_file( "../test/data1.txt", data1)){
        std::cout<<"Can't read data1.txt"<<"\n";
    }
alldata.push_back(data1);
    std::vector<std::pair<float,float> >  data2;
    if (!read_file( "../test/data2.txt", data2)){
        std::cout<<"Can't read data2.txt"<<"\n";
    }
alldata.push_back(data2);
    std::vector<std::pair<float,float> >  data3;
    if (!read_file( "../test/data3.txt", data3)){
        std::cout<<"Can't read data3.txt"<<"\n";
    }
alldata.push_back(data3);
    std::vector<std::pair<float,float> >  data4;
    if (!read_file( "../test/data4.txt", data4)){
        std::cout<<"Can't read data4.txt"<<"\n";
    }
alldata.push_back(data4);
    std::vector<std::pair<float,float> >  data5;
    if (!read_file( "../test/data5.txt", data5)){
        std::cout<<"Can't read data5.txt"<<"\n";
    }
alldata.push_back(data5);



	calibration clb;

         std::vector<CMatrix<float> > homography;
	//normalize model
    	CMatrix<float> Tmodel=clb.CalcTransform(model);
     	std::vector<std::pair<float,float>> Nmodel=clb.normalize(model, Tmodel);

	for(int i=0;i<alldata.size();++i){

	         //transformation matrices for normalization
   		   CMatrix<float> Tdata=clb.CalcTransform(alldata[i]); 
		   std::vector<std::pair<float,float> > Ndata=clb.normalize(alldata[i], Tdata); 
                   CMatrix<float> A=clb.create2D2DconstraintsMatrix(Nmodel,Ndata);
		  //calculate SVD for each matrix A
   		 CMatrix<float> S(9,9); 
    		 CMatrix<float> V(9,9);
    		 NMath::svd(A, S, V, true);
   		 //get homography
   		 CMatrix<float> H=clb.getHomography(V, Tmodel, Tdata);
		 homography.push_back(H);
			


	}
	CMatrix<float> U=clb.HomographyConstraintsMatrix(homography);

    CMatrix<float> S(6,6); 
    CMatrix<float> V(6,6);
    NMath::svd(U, S, V, true);
    float lambda;

    CMatrix<float> K=clb.calculateInternalParam(  V,  lambda);


}
