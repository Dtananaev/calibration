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
void calibration::getData(std::string folder){
	dir_=folder;
	loadData(folder);
}

void calibration::writeIntrinsics(){

	if(intrinsics.xSize()!=0 && intrinsics.ySize()!=0){
    std::string file_name=dir_+"/intrinsics.txt";
    std::ofstream dataFile;
    dataFile.open (file_name, std::ios::out | std::ios::trunc );
    if (dataFile.is_open()){ 
          dataFile<<intrinsics(0,0)<<" "<<intrinsics(0,1)<<" "<<intrinsics(0,2)<<"\n";
          dataFile<<intrinsics(1,0)<<" "<<intrinsics(1,1)<<" "<<intrinsics(1,2)<<"\n";
          dataFile<<intrinsics(2,0)<<" "<<intrinsics(2,1)<<" "<<intrinsics(2,2)<<"\n";
    }   
  dataFile.close();
        }

}
void calibration::calibrate(std::unordered_set<int> corners2use){

	std::vector<CMatrix<float> > homography;
	//normalize model
    	CMatrix<float> Tmodel=CalcTransform(model_);
     	std::vector<std::pair<float,float>> Nmodel=normalize(model_, Tmodel);

	for(auto iter=corners2use.begin();iter!=corners2use.end();++iter){
		auto it=imCorners.find(*iter);
		if(it!=imCorners.end() && it->second.size()==model_.size()){
	         //transformation matrices for normalization
   		   CMatrix<float> Tdata=CalcTransform(it->second); 
		   std::vector<std::pair<float,float> > Ndata=normalize(it->second, Tdata); 
                   CMatrix<float> A=create2D2DconstraintsMatrix(Nmodel,Ndata);
		  //calculate SVD for each matrix A
   		 CMatrix<float> S(9,9); 
    		 CMatrix<float> V(9,9);
    		 NMath::svd(A, S, V, true);
   		 //get homography
   		 CMatrix<float> H=getHomography(V, Tmodel, Tdata);
		 homography.push_back(H);
		}	


	}
	CMatrix<float> U=HomographyConstraintsMatrix(homography);

    CMatrix<float> S(6,6); 
    CMatrix<float> V(6,6);
    NMath::svd(U, S, V, true);
    float lambda;

    intrinsics=calculateInternalParam(  V,  lambda);
    intrinsics(0,1)=0;
    extrinsics.clear();
	for(int i=0;i<homography.size();++i){
	CMatrix<float> R,t;
	
	calculateExternalParameters(R, t,intrinsics,homography[i] );
	CMatrix<float> result (3,4,0);
	result(0,0)=R(0,0); result(0,1)=R(0,1); result(0,2)=R(0,2); result(0,3)=t(0,0);
	result(1,0)=R(1,0); result(1,1)=R(1,1); result(1,2)=R(1,2); result(1,3)=t(1,0);
	result(2,0)=R(2,0); result(2,1)=R(2,1); result(2,2)=R(2,2); result(2,3)=t(2,0);
	extrinsics.push_back(result);

        }
	auto it=imCorners.find(*corners2use.begin());
	CVector<float> k=calculateLenseDistortion(homography[0],intrinsics, model_, it->second);
	writeIntrinsics();
	std::cout<<"k size "<<k.size()<<"\n";

    std::cout<<"k1 "<<k[1]<<"\n";
    std::cout<<"k2 "<<k[2]<<"\n";
}

CMatrix<float> calibration::calculateInternalParam( CMatrix<float> V, float &lambda){

    CMatrix<float> K(3,3,0);
    //B parameters
    float B11=V(5,0);
    float B12=V(5,1);
    float B22=V(5,2);
    float B13=V(5,3);
    float B23=V(5,4);
    float B33=V(5,5);

    float c=sqrt(B11*B11 + B12*B12+B22*B22+B13*B13+B23*B23+B33*B33);
   std::cout<<"c "<<c<<"\n";

    std::cout<<"B11 "<<B11<<"\n";
    std::cout<<"B12 "<<B12<<"\n";    
    std::cout<<"B22 "<<B22<<"\n";
    std::cout<<"B13 "<<B13<<"\n";  
    std::cout<<"B23 "<<B23<<"\n";
    std::cout<<"B33 "<<B33<<"\n";  


    float y0=(B12*B13-B11*B23)/(B11*B22 -B12*B12);
  lambda=B33-(B13*B13+y0*(B12*B13-B11*B23))/B11;
     std::cout<<"lambda "<<lambda/B11<<"\n";     
  float ax=sqrt(lambda/B11);
  float ay=sqrt(lambda*B11/(B11*B22-B12*B12));
  float s=-B12*ax*ax*ay/lambda;
  float x0=s*y0/ay - B13*ax*ax/lambda;
    

    K(0,0)=ax;
    K(0,1)=s;
    K(0,2)=x0;
    K(1,1)=ay;
    K(1,2)=y0;
    K(2,2)=1;
    std::cout<<"ax "<<ax<<"\n";
    std::cout<<"ay "<<ay<<"\n";    
    std::cout<<"x0 "<<x0<<"\n";
    std::cout<<"y0 "<<y0<<"\n";  
    std::cout<<"s "<<s<<"\n";
    std::cout<<"lambda "<<lambda<<"\n";  
    
    return K;

}

CMatrix<float> calibration::getHomography(CMatrix<float> V, CMatrix<float> Tmodel, CMatrix<float> Tdata){

   CMatrix<float> homography(3,3);
        
    CMatrix<float> H(3,3);
    
    H(0,0)= V(8,0);
    H(0,1) =V(8,1);
    H(0,2) =V(8,2);    
    H(1,0) =V(8,3);  
    H(1,1) =V(8,4);  
    H(1,2) =V(8,5);  
    H(2,0) =V(8,6);  
    H(2,1) =V(8,7);  
    H(2,2) =V(8,8);  

 std::cout<<"H00 "<<H(0,0)<<"\n";
    std::cout<<"H01 "<< H(0,1)<<"\n";    
    std::cout<<"H02 "<<H(0,2)<<"\n";
    std::cout<<"H10 "<<H(1,0)<<"\n";  
    std::cout<<"H11 "<<H(1,1)<<"\n";
    std::cout<<"H12 "<<H(1,2)<<"\n";  
    std::cout<<"H20 "<<H(2,0)<<"\n";  
    std::cout<<"H21 "<<H(2,1)<<"\n";
    std::cout<<"H22 "<<H(2,2)<<"\n"; 
//homography = Tdata.inv * H *Tmodel
Tdata.inv();

CMatrix<float> temp(3,3);
    //temp
    temp(0,0)= Tdata(0,0)*H(0,0)+Tdata(0,1)*H(1,0)+Tdata(0,2)*H(2,0);
    temp(0,1)= Tdata(0,0)*H(0,1)+Tdata(0,1)*H(1,1)+Tdata(0,2)*H(2,1);
    temp(0,2)= Tdata(0,0)*H(0,2)+Tdata(0,1)*H(1,2)+Tdata(0,2)*H(2,2);
    temp(1,0)= Tdata(1,0)*H(0,0)+Tdata(1,1)*H(1,0)+Tdata(1,2)*H(2,0);
    temp(1,1)= Tdata(1,0)*H(0,1)+Tdata(1,1)*H(1,1)+Tdata(1,2)*H(2,1);
    temp(1,2)= Tdata(1,0)*H(0,2)+Tdata(1,1)*H(1,2)+Tdata(1,2)*H(2,2);
    temp(2,0)= Tdata(2,0)*H(0,0)+Tdata(2,1)*H(1,0)+Tdata(2,2)*H(2,0);
    temp(2,1)= Tdata(2,0)*H(0,1)+Tdata(2,1)*H(1,1)+Tdata(2,2)*H(2,1);
    temp(2,2)= Tdata(2,0)*H(0,2)+Tdata(2,1)*H(1,2)+Tdata(2,2)*H(2,2);

    //homography
    homography(0,0)= temp(0,0)*Tmodel(0,0)+temp(0,1)*Tmodel(1,0)+temp(0,2)*Tmodel(2,0);
    homography(0,1)= temp(0,0)*Tmodel(0,1)+temp(0,1)*Tmodel(1,1)+temp(0,2)*Tmodel(2,1);
    homography(0,2)= temp(0,0)*Tmodel(0,2)+temp(0,1)*Tmodel(1,2)+temp(0,2)*Tmodel(2,2);
    homography(1,0)= temp(1,0)*Tmodel(0,0)+temp(1,1)*Tmodel(1,0)+temp(1,2)*Tmodel(2,0);
    homography(1,1)= temp(1,0)*Tmodel(0,1)+temp(1,1)*Tmodel(1,1)+temp(1,2)*Tmodel(2,1);
    homography(2,0)= temp(2,0)*Tmodel(0,0)+temp(2,1)*Tmodel(1,0)+temp(2,2)*Tmodel(2,0);
    homography(1,2)= temp(1,0)*Tmodel(0,2)+temp(1,1)*Tmodel(1,2)+temp(1,2)*Tmodel(2,2);
    homography(2,1)= temp(2,0)*Tmodel(0,1)+temp(2,1)*Tmodel(1,1)+temp(2,2)*Tmodel(2,1);
    homography(2,2)= temp(2,0)*Tmodel(0,2)+temp(2,1)*Tmodel(1,2)+temp(2,2)*Tmodel(2,2);


   std::cout<<"homography["<<0<<"]"<<"["<<0<<"]= "<<homography(0,0)<<"\n";
    std::cout<<"homography["<<0<<"]"<<"["<<1<<"]= "<<homography(0,1)<<"\n";
    std::cout<<"homography["<<0<<"]"<<"["<<2<<"]= "<<homography(0,2)<<"\n";
    std::cout<<"homography["<<1<<"]"<<"["<<3<<"]= "<<homography(1,0)<<"\n";
    std::cout<<"homography["<<1<<"]"<<"["<<4<<"]= "<<homography(1,1)<<"\n";
    std::cout<<"homography["<<1<<"]"<<"["<<5<<"]= "<<homography(1,2)<<"\n";
    std::cout<<"homography["<<2<<"]"<<"["<<6<<"]= "<<homography(2,0)<<"\n";
    std::cout<<"homography["<<2<<"]"<<"["<<7<<"]= "<<homography(2,1)<<"\n";
    std::cout<<"homography["<<2<<"]"<<"["<<8<<"]= "<<homography(2,2)<<"\n";

    //homography= Tdata.inv() * H * Tmodel;
    return homography;

}

CMatrix<float> calibration::HomographyConstraintsMatrix(std::vector<CMatrix<float> > homography){
    CMatrix<float> V(6,2*homography.size());//constraints matrix



for(int i=0;i<homography.size();++i){	
	CMatrix<float> H1=homography[i];
        //first row
    	 V(0,2*i)=H1(0,0)*H1(1,0);
   	 V(1,2*i)=H1(0,0)*H1(1,1) + H1(0,1)*H1(1,0);
   	 V(2,2*i)=H1(0,1)*H1(1,1);
   	 V(3,2*i)=H1(0,2)*H1(1,0)+H1(0,0)*H1(1,2);
   	 V(4,2*i)=H1(0,2)*H1(1,1)+H1(0,1)*H1(1,2);
   	 V(5,2*i)=H1(0,2)*H1(1,2);
       //second row 
    V(0,2*i+1)=H1(0,0)*H1(0,0)- H1(1,0)*H1(1,0);
    V(1,2*i+1)=( H1(0,0)*H1(0,1) + H1(0,1)*H1(0,0) ) - ( H1(1,0)*H1(1,1) + H1(1,1)*H1(1,0) );
    V(2,2*i+1)=H1(0,1)*H1(0,1) -H1(1,1)*H1(1,1);
    V(3,2*i+1)= (H1(0,2)*H1(0,0)+H1(0,0)*H1(0,2)) -(H1(1,2)*H1(1,0)+H1(1,0)*H1(1,2)  );
    V(4,2*i+1)=(H1(0,2)*H1(0,1)+H1(0,1)*H1(0,2) ) - (H1(1,2)*H1(1,1)+H1(1,1)*H1(1,2));
    V(5,2*i+1)=H1(0,2)*H1(0,2)-H1(1,2)*H1(1,2);
    }
    return V;
}



std::vector<std::pair<float,float> >  calibration::normalize(std::vector<std::pair<float,float> > data, CMatrix<float> transform){
    //Z transform Z= (x-mean)/sigma
    std::vector<std::pair<float,float> > result(data.size());
    for(unsigned int i=0; i<data.size(); i++){
             result[i].first = transform(0,0)*data[i].first+ transform(0,2);
             result[i].second = transform(1,1)*data[i].second + transform(1,2);

    }

    return result;
}

void calibration::CalcMeanSigma(const std::vector<std::pair<float,float> > data, 
				float &meanX, float &sigmaX,
				float &meanY, float &sigmaY){

    meanX = 0; sigmaX=0; meanY = 0; sigmaY=0;
    //calculate mean
    for(auto it = data.begin(), end = data.end(); it != end; ++it){
        meanX +=it->first;
        meanY +=it->second; 
    }
    meanX= meanX/ data.size();
    meanY= meanY/ data.size();
    //sigma

    for(auto it = data.begin(), end = data.end(); it != end; ++it){
        sigmaX += ( it->first - meanX)  * ( it->first- meanX);
        sigmaY += ( it->second - meanY)  * ( it->second- meanY);
    }
    sigmaX=sqrt(sigmaX/(data.size()-1));
    sigmaY=sqrt(sigmaY/(data.size()-1));   
}

CMatrix<float> calibration::CalcTransform(const std::vector<std::pair<float,float> > data ){

 CMatrix<float> transform(3,3,0);
//Calculate mean and sigma
float meanX, meanY;
float sigmaX, sigmaY;

CalcMeanSigma(data, meanX, sigmaX,meanY,sigmaY);

//Z transform Z= (x-mean)/sigma
                //x=z*sigma +mean
    transform(0,0)=1/sigmaX;
    transform(0,2)=-meanX/sigmaX;
    transform(1,1)=1/sigmaY;
    transform(1,2)=-meanY/sigmaY;
    transform(2,2)=1;

    return transform;
}

CMatrix<float> calibration::create2D2DconstraintsMatrix(  std::vector<std::pair<float,float> > model , std::vector<std::pair<float,float> > data ){

CMatrix<float> A(9, 2*model.size());

    for(unsigned int i=0; i< model.size(); i++){
     
 //first equation         
        A(0,2*i)=0;
        A(1,2*i)=0;
        A(2,2*i)=0; 
        A(3, 2*i)=-model[i].first;
        A(4,2*i)=-model[i].second;       
        A(5,2*i)=-1;  
        A(6,2*i)=model[i].first*data[i].second;   
        A(7,2*i)=model[i].second*data[i].second;   
        A(8,2*i)=data[i].second;

        //second equation
        A(0,2*i+1)=model[i].first;
        A(1,2*i+1)=model[i].second;
        A(2,2*i+1)=1; 
        A(3,2*i+1)=0;
        A(4,2*i+1)=0;       
        A(5,2*i+1)=0;  
        A(6,2*i+1)=-model[i].first*data[i].first;   
        A(7,2*i+1)=-model[i].second*data[i].first;   
        A(8,2*i+1)=-data[i].first;    
    }


return A;
}




CVector<float> calibration::calculateLenseDistortion(CMatrix<float> Homography,CMatrix<float> K, std::vector<std::pair<float,float> > model, std::vector<std::pair<float,float> > data){


std::vector<std::pair<float,float> > proj_points;

    for(unsigned int i=0;i<model.size();i++){
        float x=Homography(0,0)*model[i].first+Homography(0,1)*model[i].second+Homography(0,2);
        float y=Homography(1,0)*model[i].first+Homography(1,1)*model[i].second+Homography(1,2);
        float n=Homography(2,0)*model[i].first+Homography(2,1)*model[i].second+Homography(2,2);
            
      
       float resX=x/n;
       float resY=y/n;
        proj_points.push_back(std::make_pair(resX,resY));                                  
    }

    CMatrix<float> A(2,2*proj_points.size());
    CVector<float> vec(2*proj_points.size());
    
    std::vector<float> result;
    //Create matrix A
    for( unsigned int i=0; i<proj_points.size();i++){
        double a=proj_points[i].first-K(0,2);
        double b=proj_points[i].second-K(1,2);
        double r=sqrt(a*a+b*b);  

        A(0,2*i)=a*r;  
        A(1,2*i)=a*r*r;
        A(0,2*i+1)=b*r;  
        A(1,2*i+1)=b*r*r; 
        vec[2*i] = data[i].first-proj_points[i].first;
        vec[2*i+1] = data[i].second-proj_points[i].second;

    }

    CVector<float> k=NMath::leastSquares(A,vec);


return k;


}



void calibration::calculateExternalParameters(CMatrix<float> &Rot, CMatrix<float> &trans,CMatrix<float> K, CMatrix<float> H ){

    K.inv();

    float n;//normalization factor

    float A= K(0,0)*H(0,0) +  K(0,1)*H(1,0) + K(0,2)*H(2,0);
    float B = K(1,0)*H(0,0) +  K(1,1)*H(1,0) +K(1,2)*H(2,0);
    float C = K(2,0)*H(0,0) +  K(2,1)*H(1,0) +K(2,2)*H(2,0);
    n=1/sqrt( A*A +B*B +C*C);


    CMatrix<float> r1(3,1);//rot1 vector

    r1(0,0)=n*( K(0,0)*H(0,0)+  K(0,1)*H(1,0)  + K(0,2)*H(2,0) );
    r1(1,0)=n*(K(1,0)*H(0,0) +  K(1,1)*H(1,0) +K(1,2)*H(2,0) );
    r1(2,0)=n*(K(2,0)*H(0,0) +  K(2,1)*H(1,0) +K(2,2)*H(2,0) );

    CMatrix<float> r2(3,1);//rot1 vector

    r2(0,0)=n*( K(0,0)*H(0,1)+  K(0,1)*H(1,1)  + K(0,2)*H(2,1) );
    r2(1,0)=n*(K(1,0)*H(0,1) +  K(1,1)*H(1,1) +K(1,2)*H(2,1) );
    r2(2,0)=n*(K(2,0)*H(0,1) +  K(2,1)*H(1,1) +K(2,2)*H(2,1) );

    CMatrix<float> r3(3,1);//rot1 vector

    r3(0,0)= r1(1,0)*r2(2,0)-r1(2,0)*r2(1,0);
    r3(1,0) = r1(2,0)*r2(0,0)-r1(0,0)*r1(3,0);
    r3(2,0) =r1(0,0)*r2(1,0)-r1(1,0)*r2(0,0);

    CMatrix<float> T(3,1);

    T(0,0)=n*( K(0,0)*H(0,2)+  K(0,1)*H(1,2)  + K(0,2)*H(2,2) );
    T(1,0)=n*(K(1,0)*H(0,2) +  K(1,1)*H(1,2) +K(1,2)*H(2,2) );
    T(2,0)=n*(K(2,0)*H(0,2) +  K(2,1)*H(1,2) +K(2,2)*H(2,2) );

//Correct rotation error


 CMatrix<float> R(3,3);

    R(0,0)=r1(0,0);  R(0,1)=r2(0,0); R(0,2)=r3(0,0);
    R(1,0)=r1(1,0);  R(1,1)=r2(1,0); R(1,2)=r3(1,0);
    R(2,0)=r1(2,0);  R(2,1)=r2(2,0); R(2,2)=r3(2,0);

    CMatrix<float> S(3,3); 
    CMatrix<float> V(3,3);
   // NMath::svd(R, S, V, true);
    //S.identity(3);
   // NMath::svdBack(R, S, V);


//results
    trans=T;
    Rot=R;

}



