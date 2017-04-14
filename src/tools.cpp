/*
 * File: tools.cpp
 * Author: Denis Tananaev
 * Date: 12.04.2017
 * 
 */

#include "tools.h"
#include <math.h>       /* sqrt, ceil */
#include <stdlib.h>     /* abs */
tools::tools(){
}
tools::~tools(){
}

//Gaussian Kernel 2D
CMatrix<float> tools::GaussKernel(int radius){

	int filter_size = 2*radius+1;

	CMatrix<float> filter(filter_size,filter_size);

	if ( filter_size % 2 == 0 )
		++filter_size;

	float n=2*radius*radius;
	float sum=0;
	for(int x=-radius;x<=radius;x++)
		for(int y=-radius;y<=radius;y++)
		{ 
			filter(x+radius,y+radius)=(exp(-((x*x+y*y)/n)))/n*3.1415926536;
			sum+=filter(x+radius,y+radius);
		}	

		// normalize the Kernel
		for(int i = 0; i < filter_size; ++i)
			for(int j = 0; j < filter_size; ++j)
			{  filter(i,j) /= sum;}

		return filter;
}

//Gaussian filter
CMatrix<float> tools::applyKernel(CMatrix<float> kernel, CMatrix<float> inImage,int border){
    inImage=addNeumannBoundary(inImage,border);
	CMatrix<float> image(inImage.xSize(),inImage.ySize(),0);
	//Filtering
	for(int x=border;x<inImage.xSize()-border;x++)
		for(int y=border;y<inImage.ySize()-border;y++)
		{
			for(int i=0;i<kernel.xSize();i++)
				for (int j=0;j<kernel.ySize();j++)
				{
					image(x,y)+=inImage(x-border+i,y-border+j)*kernel(i,j);
				}
		}

        image=cutNeumannBoundary(image,border);
		return image;
}




CMatrix<float> tools::cutNeumannBoundary(CMatrix<float>& image, int border_size){ 

	CMatrix<float> realimage(image.xSize()-2*border_size,image.ySize()-2*border_size);
	for(int x=0;x<realimage.xSize();x++)
		for(int y=0; y<realimage.ySize();y++)
		{
			realimage(x,y)=image(x+border_size,y+border_size);
		}

		return realimage;
}

CMatrix<float> tools::addNeumannBoundary(CMatrix<float> aImage,int border_size){

CMatrix<float> result(aImage.xSize()+border_size*2,aImage.ySize()+border_size*2);
	result.fill(0);
	//center matrix
	for(int x=0;x<aImage.xSize();x++)
		for(int y=0;y<aImage.ySize();y++)
		{
			result(x+border_size,y+border_size)=aImage(x,y);
		}
		//Top
		for(int x=0;x<aImage.xSize();x++)
			for(int y=0;y<border_size;y++)
			{
				result(x+border_size,y)=aImage(x,border_size-1-y);
			}
			//Bottom
			for(int x=0;x<aImage.xSize();x++)
				for(int y=0;y<border_size;y++)
				{
					result(x+border_size,y+aImage.ySize()+border_size)=aImage(x,aImage.ySize()-1-y);
				}
				//left side
				for(int x=0;x<border_size;x++)
					for(int y=0;y<aImage.ySize();y++)
					{
						result(x,y+border_size)=aImage(border_size-1-x,y);
					}

					//right side
					for(int x=0;x<border_size;x++)
						for(int y=0;y<aImage.ySize();y++)
						{
							result(x+aImage.xSize()+border_size,y+border_size)=aImage(aImage.xSize()-1-x,y);
						}
						//up left square
						for(int x=0;x<border_size;x++)
							for(int y=0;y<border_size;y++)
							{
								result(x,y)=aImage(0,0);
							}
							//up right square
							for(int x=aImage.xSize()-1;x<(aImage.xSize()+border_size);x++)
								for(int y=0;y<border_size;y++)
								{
									result(x+border_size,y)=aImage(aImage.xSize()-1,0);
								}
								//down left square
								for(int x=0;x<border_size;x++)
									for(int y=aImage.ySize()-1;y<(aImage.ySize()+border_size);y++)
									{
										result(x,y+border_size)=aImage(0,aImage.ySize()-1);
									}
									//down right square
									for(int x=aImage.xSize()-1;x<(aImage.xSize()+border_size);x++)
										for(int y=aImage.ySize()-1;y<(aImage.ySize()+border_size);y++)
										{
											result(x+border_size,y+border_size)=aImage(aImage.xSize()-1,aImage.ySize()-1);
										}		
										return result;

}

CMatrix<float> tools::nonMaxSupress(CMatrix<float> grad, CMatrix<float> orient){
    //Non-maximum suppression for Canny edge detector
    //the function assumes that grad and orient extended with 1 by Neumann boundary 
    grad=addNeumannBoundary(grad,1);
    CMatrix<float> result(grad.xSize(),grad.ySize(),0);
     /*
        p(x-b,y-b) p(x,y-b)  p(x+b,y-b) 
        p(x-b,y)    p(x,y)   p(x+b,y)
        p(x-b,y+b) p(x,y+b)  p(x+b,y+b) 
                    
        */       


    for(int y=1; y<grad.ySize()-1;++y){
     for(int x=1;x<grad.xSize()-1;++x){
            if(orient(x,y)==0){
            //compare p(x,y-b) and p(x,y+b)            
                if( grad(x,y)>= grad(x,y-1) && grad(x,y)>=grad(x,y+1) ){result(x,y)=grad(x,y);}
            } else if(orient(x,y)==45){
            //compare p(x-b,y-b) and p(x+b,y+b)  
               if( grad(x,y)>= grad(x-1,y-1) && grad(x,y)>= grad(x+1,y+1) ){result(x,y)=grad(x,y);}       
           }  else if(orient(x,y)==90){
            //compare p(x-b,y) and p(x+b,y)
               if( grad(x,y)>= grad(x-1,y) && grad(x,y)>= grad(x+1,y) ){result(x,y)=grad(x,y);}      
           } else if(orient(x,y)==135){
            //compare p(x-b,y+b) and p(x+b,y-b)
               if( grad(x,y)>= grad(x+1,y-1) && grad(x,y)>= grad(x-1,y+1) ){result(x,y)=grad(x,y);}  
            }
            


         }
    }
    result=cutNeumannBoundary(result,1);      
    return result;

}



CMatrix<float> tools::CannyTreshold(CMatrix<float> nms,float treshold){
/*
Hysteresis: The final step. Canny does use two thresholds (upper and lower):

If a pixel gradient is higher than the upper threshold, the pixel is accepted as an edge
If a pixel gradient value is below the lower threshold, then it is rejected.
If the pixel gradient is between the two thresholds, then it will be accepted only if it is connected to a pixel that is above the upper threshold.
Canny recommended a upper:lower ratio between 2:1 and 3:1
*/

    nms=addNeumannBoundary(nms,1);

    //treshold upper 2:1 lower 3:1
    float upper_treshold= treshold/2;
    float lower_treshold= treshold/5;
    CMatrix<float> result(nms.xSize(),nms.ySize(),0);
     /*
        p(x-b,y-b) p(x,y-b)  p(x+b,y-b) 
        p(x-b,y)    p(x,y)   p(x+b,y)
        p(x-b,y+b) p(x,y+b)  p(x+b,y+b) 
        */       
    for(int y=1; y<nms.ySize()-1;++y){
     for(int x=1;x<nms.xSize()-1;++x){
            if(nms(x,y)>=upper_treshold){
                result(x,y)=1;
             }else if(nms(x,y)>=lower_treshold && nms(x,y)<upper_treshold){
                bool connected=false;
                for (int i=-1; i<2;i++){
                    if(nms(x+i,y+i)>=upper_treshold){connected=true;}
                    if(nms(x,y+i)>=upper_treshold){connected=true;}                    
                    if(nms(x+i,y)>=upper_treshold){connected=true;}  
                    if(nms(x-i,y+i)>=upper_treshold){connected=true;}
                }
                if(connected){result(x,y)=1;}
            }

      }
    }
    result=cutNeumannBoundary(result,1);            
    return result;

}

void tools::SobelEdgeDetector(CMatrix<float> image, CMatrix<float>& Gx,CMatrix<float>& Gy,CMatrix<float>& orient,CMatrix<float>& edges){

    int x_size=image.xSize();
    int y_size=image.ySize();
    int border=1;
    //apply Gaussian denoising
    CMatrix<float> kernel=GaussKernel(border);
    image=applyKernel(kernel, image, border);

    //apply Sobel Edge detector
    CMatrix<float> kernel_x(3,3);    
    kernel_x(0,0)=-1.0; kernel_x(0,1)=0.0; kernel_x(0,2)=1.0;
    kernel_x(1,0)=-2.0; kernel_x(1,1)=0.0; kernel_x(1,2)=2.0;
    kernel_x(2,0)=-1.0; kernel_x(2,1)=0.0; kernel_x(2,2)=1.0;
    Gx=applyKernel(kernel_x, image, border);
    CMatrix<float> kernel_y(3,3);  
    kernel_y(0,0)=-1.0; kernel_y(0,1)=-2.0; kernel_y(0,2)=-1.0;
    kernel_y(1,0)=0.0; kernel_y(1,1)=0.0; kernel_y(1,2)=0.0;
    kernel_y(2,0)=1.0; kernel_y(2,1)=2.0; kernel_y(2,2)=1.0;
    Gy=applyKernel(kernel_y, image, border);
    //compute gradient

    edges.setSize(image.xSize(),image.ySize());
    orient.setSize(image.xSize(),image.ySize());
    for(int y=0;y<edges.ySize();++y){
        for(int x=0;x<edges.xSize();++x){
            edges(x,y)=sqrt(Gx(x,y)*Gx(x,y)+Gy(x,y)*Gy(x,y));
           // if(Gx(x,y)!=0 && Gy(x,y)!=0 ){
            orient(x,y)=atan2(Gy(x,y),Gx(x,y))*180/M_PI;// from -180 to 180
            //like in openCV we will round to the next angles 0, 45, 90, 135
            // for 0: 0 and 180 and -180
            //for 45: 45 and -135
            //for 90: 90 and -90
            //for 135: 135 and -45
            rounder(orient(x,y));
        }
    }

}


void tools::rounder(float& degrees){
    //dist1<=dist2 ? value_if_true : value_if_false
    float dist1,dist2;

if(degrees>=0){
    
    if(degrees<=45){
        //std::cout<<"0-45"<<"\n";
        dist1=abs(0-degrees);
        dist2=abs(45-degrees);
        dist1<=dist2 ? degrees=0 : degrees=45;     
    }else if(degrees<=90){
        //std::cout<<"45-90"<<"\n";
        dist1=abs(45-degrees);
        dist2=abs(90-degrees);
        dist1<=dist2 ? degrees=45 : degrees=90;       
    }else if(degrees<=135){
        //std::cout<<"90-135"<<"\n";
        dist1=abs(90-degrees);
        dist2=abs(135-degrees);
        dist1<=dist2 ? degrees=90 : degrees=135;         
    }else{
        //std::cout<<"135-180"<<"\n";
        dist1=abs(135-degrees);
        dist2=abs(180-degrees);
        dist1<=dist2 ? degrees=135 : degrees=0;      
    }

}else{
                            
    if(degrees>=-45){//if -45 assign to 135
        //std::cout<<"0-(-45)"<<"\n";
        dist1=abs(0-degrees);
        dist2=abs(45+degrees);
        dist1<=dist2 ? degrees=0 : degrees=135;   
    }else if(degrees>=-90){//if -45 assign to 135
        //std::cout<<"(-45)-(-90)"<<"\n";
        dist1=abs(45+degrees);
        dist2=abs(90+degrees);
        dist1<=dist2  ? degrees=135 : degrees=90;    
    }else if(degrees>=-135){
        //std::cout<<"(-90)-(-135)"<<"\n";
        dist1=abs(90+degrees);
        dist2=abs(135+degrees);
        dist1<=dist2  ? degrees=90 : degrees=45;       
    }else{
        //std::cout<<"(-135)-(-180)"<<"\n";
        dist1=abs(135+degrees);
        dist2=abs(180+degrees);
        dist1<=dist2 ? degrees=45 : degrees=0;       
    }


 }

}
std::vector<std::pair<int,int> > tools::HoughPeaks(CMatrix<float> Hough, int num_peaks){
    std::vector<std::pair<int,int> > result(num_peaks);
    std::vector<int> v(num_peaks,0);
    for(int y=0;y<Hough.ySize();++y){
        for(int x=0;x<Hough.xSize();++x){
            if(Hough(x,y)>0){
       
                std::vector<int> temp;
                for(int i=0;i<v.size();++i){
                    int dist=Hough(x,y)-v[i];
                 
                    temp.push_back(dist);
                }
                  
                std::vector<int>::iterator it=std::max_element(temp.begin(),temp.end());
                if(*it>0){
                    int index=it-temp.begin();
         
                    v[index]=Hough(x,y);
               
                    result[index]=std::make_pair(x,y);
      
                }

           }

        }
    }


    return result;
} 

CMatrix<float> tools::CannyEdgeDetector(CMatrix<float> image){
    CMatrix<float> Gx;
    CMatrix<float> Gy;
    CMatrix<float> orient;
    CMatrix<float> edges;

    SobelEdgeDetector(image, Gx, Gy,orient, edges);
    CMatrix<float> nms=nonMaxSupress(edges,orient);
    float treshold=0;
    for(int i=0;i<nms.xSize();i++){
    for(int j=0;j<nms.ySize();j++){

        if(nms(i,j)>treshold){treshold=nms(i,j);}

    }
    }
    CMatrix<float> result=CannyTreshold(nms,treshold);
    
    return result;
}


CTensor<float> tools::extractHoughLines(CMatrix<float> edges, CMatrix<float> image){

   CTensor<float> lines(edges.xSize(),edges.ySize(),3,0);
     std::vector<int> theta,rho;

    CMatrix<float> H=HoughTransform( edges,theta,rho);

    std::vector<std::pair<int,int> > temp=HoughPeaks(H, 18);
    lines.putMatrix(image,0);
    lines.putMatrix(image,1);
    lines.putMatrix(image,2);

    for(int i=0;i<temp.size();i++){
     float th=theta[temp[i].first]*M_PI/180;
     float rh=rho[temp[i].second];
 

            for(int x=0;x<lines.xSize();++x){
                for(int y=0;y<lines.ySize();++y){
                    if(sin(th)!=0){    
                    float j=(rh-x*cos(th))/sin(th);
                     if(j>=0 && j<lines.ySize()){
                       // if(edges(x,j)==1){
                        lines(x,j,0)=255;
                        lines(x,j,1)=0;
                        lines(x,j,2)=0;
                       // }
                    }
                    }
     
                    if(cos(th)!=0){    
                    float k=(rh-y*sin(th))/cos(th);
                    if(k>=0 && k<lines.xSize()){
                       //  if(edges(k,y)==1){
                        lines(k,y,0)=255;
                        lines(k,y,1)=0;
                        lines(k,y,2)=0;
                           // }
                    }   
                    }                    
         
                }
            }
        }   
    return lines;
    
}



CMatrix<float> tools::HoughTransform(CMatrix<float> edges,std::vector<int>& theta,std::vector<int>& rho){
    int x_size=edges.xSize();
    int y_size=edges.ySize();


    float Diag = sqrt(x_size*x_size +y_size*y_size);

    Diag=Diag; //round up
     //theta from -90 to 89
    theta.clear();
    rho.clear();
    for(int i=-90;i<90;++i){
     theta.push_back(i);
    }
    for(int i=-Diag;i<=Diag;i++){
     rho.push_back(i);
    }

    CMatrix<float> Hough(theta.size(),rho.size(),0);


    for(int y=0; y<y_size;++y){
        for(int x=0; x<x_size;++x){
            if(edges(x,y)==1){
                 for(int i=0;i<theta.size();++i){
                    float t=theta[i]*M_PI/180; //radians
                    float dist=x*cos(t)+y*sin(t);
                    std::vector<int>::iterator low;
                    low=std::lower_bound(rho.begin(), rho.end(), dist);
                    int j=low-rho.begin(); //index of rho     
                    Hough(i,j)+=1;      
                 }
            }

        }
    }   



 
    return Hough;
}


//  diff -0,5 0 0,5
void tools::diffXY(CMatrix<float> Image,  CMatrix<float> &dx, CMatrix<float> &dy){
       int w =Image.xSize();
       int h =Image.ySize();


       Image=addNeumannBoundary(Image,1);
    dx.fill(0);
    dy.fill(0);
    for(int x=0; x<w; x++)
    		for(int y=0; y<h; y++) {
            int a=x+1;
            int b=y+1;
            dx(x,y)=-0.5*Image(a-1,b)+0.5*Image(a+1,b);
            dy(x,y)=-0.5*Image(a,b-1)+0.5*Image(a,b+1);     
 
    }

}

CMatrix<float> tools::HarrisEdgeDetector(CMatrix<float> image, std::string regime){
    bool corners;
    float treshold;
    if(regime=="corners"){
        treshold=10000;
        corners=true;
    }else if(regime=="edges"){
         treshold=-10000;
        corners=false;       
    }else{
       std::cerr<<"Error: Undefined regime for Harris detector. It shoudl be ether: \"corners\" or \"edges\"! Run corner detector regime."<<"\n"; 
        treshold=10000;
        corners=true; 
    }

    int width=image.xSize();
    int height=image.ySize();
    
    CMatrix<float> Gx;
    CMatrix<float> Gy;
    CMatrix<float> orient;
    CMatrix<float> edges;

    SobelEdgeDetector(image, Gx, Gy,orient, edges);

  
    CMatrix<float> Ixx(width,height);
    CMatrix<float> Ixy(width,height);   
    CMatrix<float> Iyy(width,height);
   for(int x=0; x<width; x++){
    for(int y=0;y<height;y++) { 
        Ixx(x,y) = Gx(x,y)*Gx(x,y);
        Ixy(x,y) = Gx(x,y)*Gy(x,y);
        Iyy(x,y) = Gy(x,y)*Gy(x,y);
     }
    }
    int border=1; //7x7 kernel
    //apply Gaussian window
    CMatrix<float> kernel=GaussKernel(border);
    Ixx=applyKernel(kernel, Ixx, border);
    Ixy=applyKernel(kernel, Ixy, border);    
    Iyy=applyKernel(kernel, Iyy, border);

    CMatrix<float> result(width,height);
float max=0;
   for(int x=0; x<width; x++){
    for(int y=0;y<height;y++) { 
                CMatrix<float> A(2,2);
                A(0,0)=Ixx(x,y); A(0,1)=Ixy(x,y);
                A(1,0)=Ixy(x,y); A(1,1)=Iyy(x,y);
                CMatrix<float> S(2,2); 
                CMatrix<float> V(2,2);
                NMath::svd(A, S, V);

              //  std::cout<<"S(0,0)"<<S(0,0)<<"\n";
               // std::cout<<"S(0,1)"<<S(0,1)<<"\n";
              //  std::cout<<"S(1,0)"<<S(1,0)<<"\n";
              //  std::cout<<"S(1,1)"<<S(1,1)<<"\n";
              //  std::cout<<"-----------------------"<<"\n";
               // std::cin.get();
                float trace=S(0,0)+S(1,1);
                float R=S(0,0)*S(1,1)-0.06*trace*trace;
                if(corners){
                   R>treshold ? result(x,y)=R : result(x,y)=0;
                }else if (!corners){ 
                    R<treshold ? result(x,y)=R : result(x,y)=0;
                }

        }
    }


return result;
 
} 
CMatrix<float> tools::getLocalMaximum(CMatrix<float> corners, float max_pixel_distance){

    std::vector<distribution> data;

    //max_pixel_distance distance which we check in order to assign the pixel to the group or make new group
        CMatrix<float> process=corners;
        for(int y=0;y<corners.ySize();++y){
        for(int x=0;x<corners.xSize();++x){
            if(corners(x,y)!=0){
                
                //init data
                if(data.size()==0){
                         distribution temp;
                        temp.begin=std::make_pair(x,y);
                        temp.coord.push_back(std::make_pair(x,y));
                        temp.val.push_back(corners(x,y));                          
                        data.push_back(temp);           
                }   
                
                //find the group

                std::vector<float> distances;
                for(int i=0; i<data.size();++i){
                    float distx=data[i].begin.first/data[i].coord.size()-x;
                    float disty=data[i].begin.second/data[i].coord.size()-y;
                    float dist= sqrt(distx*distx+disty*disty);
                    distances.push_back(dist);
                }  

                    std::vector<float>::iterator it=std::min_element(distances.begin(),distances.end());
                    int index=it-distances.begin();

                    if(*it<max_pixel_distance ){
                      //add points to the existing group
                        data[index].begin.first+=x;
                        data[index].begin.second+=y;                      
                        data[index].coord.push_back(std::make_pair(x,y)); 
                        data[index].val.push_back(corners(x,y));                       
                    }else{
                         distribution temp;
                        temp.begin=std::make_pair(x,y);
                        temp.coord.push_back(std::make_pair(x,y));
                        temp.val.push_back(corners(x,y));                          
                        data.push_back(temp);  
                    }                                
        
               }

          }
        }
    
    CMatrix<float> result(corners.xSize(),corners.ySize(),0);
    std::cout<<"data "<<data.size()<<"\n";
    for(int i=0;i<data.size();i++){
        for(int j=0;j<data[i].coord.size();j++){
            int x = data[i].coord[j].first;
            int y = data[i].coord[j].second;
            float val=data[i].val[j];
            result(x,y)=val;
        }   
    }
    return result;
}




