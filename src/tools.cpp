/*
 * File: tools.cpp
 * Author: Denis Tananaev
 * Date: 12.04.2017
 * 
 */

#include "tools.h"
#include <math.h>       /* sqrt */
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
                if( grad(x,y)> grad(x,y-1) && grad(x,y)> grad(x,y+1) ){result(x,y)=grad(x,y);}
            } else if(orient(x,y)==45){
            //compare p(x-b,y-b) and p(x+b,y+b)  
               if( grad(x,y)> grad(x-1,y-1) && grad(x,y)> grad(x+1,y+1) ){result(x,y)=grad(x,y);}       
           }  else if(orient(x,y)==90){
            //compare p(x-b,y) and p(x+b,y)
               if( grad(x,y)> grad(x-1,y) && grad(x,y)> grad(x+1,y) ){result(x,y)=grad(x,y);}      
           } else if(orient(x,y)==135){
            //compare p(x-b,y+b) and p(x+b,y-b)
               if( grad(x,y)> grad(x-1,y+1) && grad(x,y)> grad(x+1,y-1) ){result(x,y)=grad(x,y);}  
            }
            


         }
    }
    result=cutNeumannBoundary(result,1);      
    return result;

}



CMatrix<float> tools::CannyTreshold(CMatrix<float> nms, float max_value){
/*
Hysteresis: The final step. Canny does use two thresholds (upper and lower):

If a pixel gradient is higher than the upper threshold, the pixel is accepted as an edge
If a pixel gradient value is below the lower threshold, then it is rejected.
If the pixel gradient is between the two thresholds, then it will be accepted only if it is connected to a pixel that is above the upper threshold.
Canny recommended a upper:lower ratio between 2:1 and 3:1
*/
    nms=addNeumannBoundary(nms,1);

    //treshold upper 2:1 lower 3:1
    float upper_treshold= max_value/2;
    float lower_treshold= max_value/3;
    CMatrix<float> result(nms.xSize(),nms.ySize(),0);
     /*
        p(x-b,y-b) p(x,y-b)  p(x+b,y-b) 
        p(x-b,y)    p(x,y)   p(x+b,y)
        p(x-b,y+b) p(x,y+b)  p(x+b,y+b) 
        */       
    for(int y=1; y<nms.ySize()-1;++y){
     for(int x=1;x<nms.xSize()-1;++x){
            if(nms(x,y)>=upper_treshold){
                result(x,y)=max_value;
             }else if(nms(x,y)>=lower_treshold && nms(x,y)<upper_treshold){
                bool connected=false;
                for (int i=-1; i<2;i++){
                    if(nms(x+i,y+i)>=upper_treshold){connected=true;}
                    if(nms(x,y+i)>=upper_treshold){connected=true;}                    
                    if(nms(x+i,y)>=upper_treshold){connected=true;}  
                    if(nms(x-i,y+i)>=upper_treshold){connected=true;}
                }
                if(connected){result(x,y)=max_value;}
            }

      }
    }
    result=cutNeumannBoundary(result,1);            
    return result;

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

CMatrix<float> tools::CannyEdgeDetector(CMatrix<float> image){
   
    int x_size=image.xSize();
    int y_size=image.ySize();
    int border=1;
    //apply Gaussian denoising
    CMatrix<float> kernel=GaussKernel(border);; 
    image=applyKernel(kernel, image, border);

    //apply Sobel Edge detector
    CMatrix<float> kernel_x(3,3);    
    kernel_x(0,0)=-1.0; kernel_x(0,1)=0.0; kernel_x(0,2)=1.0;
    kernel_x(1,0)=-2.0; kernel_x(1,1)=0.0; kernel_x(1,2)=2.0;
    kernel_x(2,0)=-1.0; kernel_x(2,1)=0.0; kernel_x(2,2)=1.0;
    CMatrix<float> Gx=applyKernel(kernel_x, image, border);
    CMatrix<float> kernel_y(3,3);  
    kernel_y(0,0)=-1.0; kernel_y(0,1)=-2.0; kernel_y(0,2)=-1.0;
    kernel_y(1,0)=0.0; kernel_y(1,1)=0.0; kernel_y(1,2)=0.0;
    kernel_y(2,0)=1.0; kernel_y(2,1)=2.0; kernel_y(2,2)=1.0;
    CMatrix<float> Gy=applyKernel(kernel_y, image, border);
    //compute gradient

    CMatrix<float> grad(image.xSize(),image.ySize());
    CMatrix<float> orient(image.xSize(),image.ySize());
    float max_value=0;
    for(int y=0;y<grad.ySize();++y){
        for(int x=0;x<grad.xSize();++x){
            grad(x,y)=sqrt(Gx(x,y)*Gx(x,y)+Gy(x,y)*Gy(x,y));
            if(grad(x,y)>max_value){max_value=grad(x,y);}
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
    CMatrix<float> nms=nonMaxSupress(grad,orient);
    CMatrix<float> edges=CannyTreshold(nms, max_value);

    return edges;
}


