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
CMatrix<float> tools::makeBinary(CMatrix<float> image){
    CMatrix<float> result(image.xSize(),image.ySize());
        for (int y=0;y<image.ySize();++y){
        for (int x=0;x<image.xSize();++x){

               if(image(x,y)>=127){
                result(x,y)=1;
            } else{ 
                 result(x,y)=0;                           
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


CTensor<float> tools::drawLine(float rho,float theta, CMatrix<float> image){
   CTensor<float> lines(image.xSize(),image.ySize(),3,0);
    lines.putMatrix(image,0);
    lines.putMatrix(image,1);
    lines.putMatrix(image,2);
  float th=theta;
     float rh=rho;
 

            for(int x=0;x<image.xSize();++x){
                for(int y=0;y<image.ySize();++y){
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
return lines;
}
CTensor<float> tools::drawAllLine(std::vector<std::pair<float,float> > l1, 
                                  std::vector<std::pair<float,float> > l2, 
                                  CMatrix<float> image){

   CTensor<float> lines(image.xSize(),image.ySize(),3,0);
    lines.putMatrix(image,0);
    lines.putMatrix(image,1);
    lines.putMatrix(image,2);


  for(int i=0;i<l1.size();i++){
     float th=l1[i].first;
     float rh=l1[i].second;
 

            for(int x=0;x<lines.xSize();++x){
                for(int y=0;y<lines.ySize();++y){
                    if(sin(th)!=0){    
                    float j=(rh-x*cos(th))/sin(th);
                     if(j>=0 && j<lines.ySize()){
                        lines(x,j,0)=255;
                        lines(x,j,1)=0;
                        lines(x,j,2)=0;
                    }
                    }
     
                    if(cos(th)!=0){    
                    float k=(rh-y*sin(th))/cos(th);
                    if(k>=0 && k<lines.xSize()){
                        lines(k,y,0)=255;
                        lines(k,y,1)=0;
                        lines(k,y,2)=0;
                    }   
                    }                    
         
                }
            }
        } 


    for(int i=0;i<l2.size();i++){
     float th=l2[i].first;
     float rh=l2[i].second;
 

            for(int x=0;x<lines.xSize();++x){
                for(int y=0;y<lines.ySize();++y){
                    if(sin(th)!=0){    
                    float j=(rh-x*cos(th))/sin(th);
                     if(j>=0 && j<lines.ySize()){
                        lines(x,j,0)=0;
                        lines(x,j,1)=255;
                        lines(x,j,2)=0;
                    }
                    }
     
                    if(cos(th)!=0){    
                    float k=(rh-y*sin(th))/cos(th);
                    if(k>=0 && k<lines.xSize()){
                        lines(k,y,0)=0;
                        lines(k,y,1)=255;
                        lines(k,y,2)=0;
                    }   
                    }                    
         
                }
            }
        }  
return lines;
}

std::vector<std::pair<int,int> >  tools::AllHoughPeaks(CMatrix<float> Hough, float treshold){

std::vector<std::pair<int,int> > result;
    for(int y=0;y<Hough.ySize();++y){
        for(int x=0;x<Hough.xSize();++x){
            if(Hough(x,y)>treshold){
                result.push_back(std::make_pair(x,y));
                }
                  
             
        }
    }


    return result;
} 
void tools::minMax(CMatrix<float> image, float& min,float& max){
     max=0;
     min=std::numeric_limits<float>::max();


    for(int y=0;y<image.ySize();++y){
    for(int x=0;x<image.xSize();++x){
        
        if(image(x,y)>max){max=image(x,y);}
        if(image(x,y)<min){min=image(x,y);}

    }
    }


}

CTensor<float> tools::extractHoughLines(CMatrix<float> edges, CMatrix<float> image, std::vector<std::pair<float,float> >& l1, std::vector<std::pair<float,float> >& l2){

    //first Hough transform 
    std::vector<int> theta,rho;
    CMatrix<float> H=HoughTransform( edges,theta,rho);
    CMatrix<float> tophat=TopHat(H, 1);
    float min, max;
    minMax(tophat,min,max);
    CMatrix<float> dial= DilationFilter(tophat, 1);
    std::vector<std::pair<int,int> > temp=AllHoughPeaks(dial, 0.25*max);
    //second Hough transform 
    CMatrix<float> H2=secondHoughTransform(temp,theta,rho);
    CMatrix<float> dial2= DilationFilter(H2, 1);
    //get first  maxima

    std::vector<std::pair<int,int> > maximum=HoughPeaks(dial2, 1);//find the maximum 
    std::cout<<"Max of the second Hough transform is: rho "<<rho[maximum[0].second]<<" theta "<<theta[maximum[0].first]<<"\n"; 

    float t21=theta[maximum[0].first]*M_PI/180;
    float r21=rho[maximum[0].second];    


    for(int i=0;i<temp.size();i++){
        float th=theta[temp[i].first]*M_PI/180;
        float rh=rho[temp[i].second];

        float dist11=abs(th*cos(t21)+rh*sin(t21)-r21 );
        float dist12=abs(th*cos(t21)+rh*sin(t21)+r21 );
        float dist13=abs(th*cos(-t21)+rh*sin(-t21)-r21 );
        float dist14=abs(th*cos(-t21)+rh*sin(-t21)+r21 );
        float dist1=dist11-dist12+dist13-dist14;
   
        if( dist1==-4 ||  dist1==4 ){

            l1.push_back(std::make_pair(th,rh));
        }else if(dist1==0 || dist1==-2 || dist1==2  ) {
            l2.push_back(std::make_pair(th,rh));
            }

    }

    std::cout<<"l1 horizontal "<<l1.size()<<"\n";
    std::cout<<"l2 vertical "<<l2.size()<<"\n";  
   CTensor<float> lines=drawAllLine( l1, l2, image);
  
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
CMatrix<float> tools::secondHoughTransform(std::vector<std::pair<int,int> >  lines,std::vector<int> theta,std::vector<int> rho){

      CMatrix<float> secondHough(theta.size(),rho.size(),0);
       for(int c=0; c<lines.size();c++){
         float t1=theta[lines[c].first]*M_PI/180;
         float r1=rho[lines[c].second];
            for(int i=0;i<theta.size();++i){
                    float t2=theta[i]*M_PI/180; //radians
                    float dist=t1*cos(t2)+r1*sin(t2);
                    std::vector<int>::iterator low;
                    low=std::lower_bound(rho.begin(), rho.end(), dist);
                    int j=low-rho.begin(); //index of rho     
                    secondHough(i,j)+=1;      
                 }
        
     }
  
    return secondHough;
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
CMatrix<float> tools::DilationFilter(CMatrix<float> image, int radius){

    int before=0;
    int after=0;

    CMatrix<float> result(image.xSize(),image.ySize(),0);
        
    CMatrix<float> dial=Dilation(image, radius);


           for(int y=0;y<image.ySize();++y){
        for(int x=0;x<image.xSize();++x){
            if(image(x,y)!=0){
                before+=1;
                    if(image(x,y)==dial(x,y)){
                    result(x,y)=image(x,y);
                        after+=1;
  
                    }
                }
          }
        } 
    std::cout<<"num pixels before Dilation filter "<<before<<"\n"; 
    std::cout<<"num pixels after Dilation filter "<<after<<"\n"; 
return result;

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

   
                float trace=S(0,0)+S(1,1);
                float R=S(0,0)*S(1,1)-0.05*trace*trace;
                if(corners){
                   R>treshold ? result(x,y)=R : result(x,y)=0;
                }else if (!corners){ 
                    R<treshold ? result(x,y)=R : result(x,y)=0;
                }

        }
    }


    return result;
 
} 

CMatrix<float> tools::Dilation(CMatrix<float> Inimage, int radius){


    CMatrix<float> result(Inimage.xSize(),Inimage.ySize());
    CMatrix<float> image=addNeumannBoundary(Inimage,radius);

    for(int y=radius;y<image.ySize()-radius;++y){
        for(int x=radius;x<image.xSize() -radius;++x){
               
             float max=0;
			for(int i=0;i<radius*2+1;i++)
				for (int j=0;j<radius*2+1;j++)
				{
					if(image(x-radius+i,y-radius+j)>max){max=image(x-radius+i,y-radius+j);}
				}
		

                result(x-radius,y-radius)=max;

        }
    }


  
    return result;
}

CMatrix<float> tools::Erosion(CMatrix<float> Inimage, int radius){


    CMatrix<float> result(Inimage.xSize(),Inimage.ySize());
    CMatrix<float> image=addNeumannBoundary(Inimage,radius);

    for(int y=radius;y<image.ySize()-radius;++y){
        for(int x=radius;x<image.xSize() -radius;++x){
               
             float min=std::numeric_limits<float>::max();
			for(int i=0;i<radius*2+1;i++)
				for (int j=0;j<radius*2+1;j++)
				{
					if(image(x-radius+i,y-radius+j)<min){min=image(x-radius+i,y-radius+j);}
				}
		

                result(x-radius,y-radius)=min;

        }
    }


  
    return result;
}

CMatrix<float> tools::Opening(CMatrix<float> Inimage, int radius){
    CMatrix<float> temp=Erosion(Inimage, radius);
    CMatrix<float> result=Dilation(temp, radius);

    return result;

}

CMatrix<float> tools::TopHat(CMatrix<float> Inimage, int radius){

    CMatrix<float> temp=Opening(Inimage, radius);
  CMatrix<float> result(Inimage.xSize(),Inimage.ySize());
    for(int y=0;y<Inimage.ySize();y++){
        for(int x=0;x<Inimage.xSize();x++){
        result(x,y)=Inimage(x,y)-temp(x,y);
 
    }

    }

    return result;

}


std::set<std::pair<float,float> > tools::getALLCornerCoordinates(CMatrix<float> corners, float patch_radius){

  int before=0;
  int after=0;
    std::vector<std::pair<float,float> > allcorners;
    std::set<std::pair<float,float> > Cor;
    CMatrix<float> result(corners.xSize(),corners.ySize(),0);
    CMatrix<float> dial=Dilation(corners, patch_radius);
    float min,max;
    minMax(corners,min,max);

           for(int y=0;y<corners.ySize();++y){
        for(int x=0;x<corners.xSize();++x){
            if(corners(x,y)>0.04*max){
                before+=1;
                    if(corners(x,y)==dial(x,y)){
                    result(x,y)=1;
                        after+=1;
                       allcorners.push_back(std::make_pair(x,y));
                
                    }
                }
          }
        } 
    std::cout<<"num corners before filtering "<<before<<"\n"; 
    std::cout<<"num corners after filtering "<<after<<"\n"; 

    //check if there are pixels in the local neighbourhood area with the size of patch_radius and merge them
    int treshold = patch_radius;//pixels
    for(int i=0;i<allcorners.size();++i){
                float x1=allcorners[i].first;
                float y1=allcorners[i].second;  
                float X=x1;
                float Y=y1;
                int counter=1;
          for(int j=0; j<allcorners.size();++j){

                float x2=allcorners[j].first;
                float y2=allcorners[j].second;
                float distance=sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
   
                    if(distance<treshold && distance!=0){

                        X+=x2;
                        Y+=y2;
                        counter+=1;
                    }
                
        }
        X=X/counter;
        Y=Y/counter;
        Cor.insert(std::make_pair(X,Y));
    }

    

    
    std::cout<<"number of corners after merging "<<Cor.size()<<"\n";

    return Cor;
}

void tools::findChessBoardLines(std::vector<std::pair<float,float> > l1, 
                                std::vector<std::pair<float,float> > l2, 
                                std::set< std::pair<float,float> > cornerList, 
                                float dist2corner, 
                                CMatrix<float> image){

    //The function searches for the intersection of the lines it accepts intersections in the dist2corner dist
    std::vector<lines> Lhorizontal;
    std::vector<lines> Lvertical;
     int treshold=2;   
    std::cout<<"lines before l1 "<<l1.size()<<" l2 "<<l2.size()<<"\n";
       int size_H=l1.size();
        int size_V=l2.size();
        float coeff=0.5;
        int counter=0;
    do{
        
        
         size_H=l1.size();
         size_V=l2.size();
        getIntersections(Lhorizontal,Lvertical,l1,l2, cornerList,coeff* dist2corner);
        removeOutlierLines(Lhorizontal,Lvertical,l1,l2,treshold);
        treshold+=0.5; 
         coeff+=0.1;        
    }while(size_H!=l1.size() &&size_V!=l2.size());
    cornersVSlines( Lhorizontal, Lvertical, l1,  l2,  cornerList,  dist2corner,image);
    getIntersections(Lhorizontal,Lvertical,l1,l2, cornerList,1.25*dist2corner);

    std::cout<<"lines after l1 "<<l1.size()<<" l2 "<<l2.size()<<"\n";
     CTensor<float> lines=drawAllLine( l1, l2, image);
    lines.writeToPPM("../test/filterLine.ppm");

     removeChessBoardOutliersLines(Lhorizontal,Lvertical);
        
   CTensor<float> lines1(image.xSize(),image.ySize(),3,0);
    lines1.putMatrix(image,0);
    lines1.putMatrix(image,1);
    lines1.putMatrix(image,2);


  for(int i=0;i<Lhorizontal.size();i++){
     float th=Lhorizontal[i].theta;
     float rh=Lhorizontal[i].rho;
 

            for(int x=0;x<lines1.xSize();++x){
                for(int y=0;y<lines1.ySize();++y){
                    if(sin(th)!=0){    
                    float j=(rh-x*cos(th))/sin(th);
                     if(j>=0 && j<lines1.ySize()){
                        lines1(x,j,0)=255;
                        lines1(x,j,1)=0;
                        lines1(x,j,2)=0;
                    }
                    }
     
                    if(cos(th)!=0){    
                    float k=(rh-y*sin(th))/cos(th);
                    if(k>=0 && k<lines1.xSize()){
                        lines1(k,y,0)=255;
                        lines1(k,y,1)=0;
                        lines1(k,y,2)=0;
                    }   
                    }                    
         
                }
            }
        } 


    for(int i=0;i<Lvertical.size();i++){
     float th=Lvertical[i].theta;
     float rh=Lvertical[i].rho;
 

            for(int x=0;x<lines1.xSize();++x){
                for(int y=0;y<lines1.ySize();++y){
                    if(sin(th)!=0){    
                    float j=(rh-x*cos(th))/sin(th);
                     if(j>=0 && j<lines1.ySize()){
                        lines1(x,j,0)=0;
                        lines1(x,j,1)=255;
                        lines1(x,j,2)=0;
                    }
                    }
     
                    if(cos(th)!=0){    
                    float k=(rh-y*sin(th))/cos(th);
                    if(k>=0 && k<lines1.xSize()){
                        lines1(k,y,0)=0;
                        lines1(k,y,1)=255;
                        lines1(k,y,2)=0;
                    }   
                    }                    
         
                }
            }
        }  
 lines1.writeToPPM("../test/chessFilter.ppm");


}

void tools::getIntersections(std::vector<lines>& Lhorizontal,
                             std::vector<lines>& Lvertical,
                             std::vector<std::pair<float,float> > l1, 
                             std::vector<std::pair<float,float> > l2, 
                             std::set< std::pair<float,float> > cornerList, 
                             float dist2corner, bool total_num_intersect){



    //The function searches for the intersection of the lines it accepts intersections in the dist2corner dist
    Lhorizontal.clear();
    Lvertical.clear();
  std::set< std ::pair<float,float> > temp=cornerList;
    //filter out the horizontal lines
    for(int h=0; h<l1.size();++h ){

     float t1=l1[h].first;
     float r1=l1[h].second;
     lines l;
     l.theta=t1;
     l.rho=r1;
    if(total_num_intersect){temp=cornerList;}    

        for(int v=0; v<l2.size();++v){
            //find intersection between two lines
                float t2=l2[v].first;
                float r2=l2[v].second; 
                if(sin(t1-t2)!=0 && sin(t2-t1)!=0){   
            
                    float x=(r2*sin(t1)-r1*sin(t2))/sin(t1-t2);
                    float y=(r2*cos(t1)-r1*cos(t2))/sin(t2-t1);
                    // std::set<std::pair<float,float> >::iterator it=temp.begin();
                      for(std::set<std::pair<float,float> >::iterator it = temp.begin(); it != temp.end(); ){
                    
                       float xtemp=it->first;
                       float ytemp=it->second;
                       float dist=sqrt((x-xtemp)*(x-xtemp)+(y-ytemp)*(y-ytemp) );
                           if(dist<dist2corner){
                                l.intersect.push_back(std::make_pair(x,y));
                                temp.erase(it++);//c++98 problem that is why  it++
                            }else{
                               ++it; 
                            }
                    }                    
                }
        } 
     Lhorizontal.push_back(l);         
    }
     //same code for filtering out the vertical lines
     temp=cornerList; 
    for(int h=l2.size()-1; h>=0;--h ){
     float t1=l2[h].first;
     float r1=l2[h].second;
     lines l;
     l.theta=t1;
     l.rho=r1;
   if(total_num_intersect){temp=cornerList;}  
        for(int v=0; v<l1.size();++v){
            //find intersection between two lines
                float t2=l1[v].first;
                float r2=l1[v].second; 
                if(sin(t1-t2)!=0 && sin(t2-t1)!=0){   
            
                    float x=(r2*sin(t1)-r1*sin(t2))/sin(t1-t2);
                    float y=(r2*cos(t1)-r1*cos(t2))/sin(t2-t1);
                     std::set<std::pair<float,float> >::iterator it=temp.begin();
                        while(it!=temp.end()){
                       float xtemp=it->first;
                       float ytemp=it->second;
                       float dist=sqrt((x-xtemp)*(x-xtemp)+(y-ytemp)*(y-ytemp) );
                           if(dist<dist2corner){
                                l.intersect.push_back(std::make_pair(x,y));
                                temp.erase(it++);//c++98 problem that is why  it++
                            }else{
                               ++it; 
                            }
                    }
                    
                }
        } 
     Lvertical.push_back(l);         
    }

}

void tools::removeOutlierLines(std::vector<lines> Lhorizontal,
                           std::vector<lines> Lvertical,
                           std::vector<std::pair<float,float> >& l1, 
                           std::vector<std::pair<float,float> >& l2,
                           int minIntersect){

    //filter horizontal lines
    for(int i=0;i<Lhorizontal.size();i++){
        if(Lhorizontal[i].intersect.size()<=minIntersect){
            float theta=Lhorizontal[i].theta;
            float rho=Lhorizontal[i].rho;
            std::vector<std::pair<float,float> >::iterator it=std::find(l1.begin(),l1.end(),std::make_pair(theta,rho));
            l1.erase(it);
        } 
    
    }

    //filter vertical lines
    for(int i=0;i<Lvertical.size();i++){
        if(Lvertical[i].intersect.size()<=minIntersect){
            float theta=Lvertical[i].theta;
            float rho=Lvertical[i].rho;
            std::vector<std::pair<float,float> >::iterator it=std::find(l2.begin(),l2.end(),std::make_pair(theta,rho));
            l2.erase(it);
        } 
    
    }

}

void tools::removeChessBoardOutliersLines(std::vector<lines>& Lhorizontal,
                           std::vector<lines>& Lvertical){
    
    std::cout<<"size H before "<<Lhorizontal.size()<<"\n";
    std::cout<<"size W before "<<Lvertical.size()<<"\n";
    int counter=0;
    int size_H= Lhorizontal.size();
     int size_V= Lvertical.size();  
    int n=1;
    do{

        counter+=1;
         size_H= Lhorizontal.size();
         size_V= Lvertical.size();  
    //compute mean for horizontl lines
    float meanH=0;
    for(int i=0;i<Lhorizontal.size();i++){
            meanH+=Lhorizontal[i].intersect.size(); 
            std::cout<<"horizontal["<<i<<"]"<<Lhorizontal[i].intersect.size()<<"\n";           
    }
  


    meanH=meanH/(Lhorizontal.size()+n);
    int index=0;
    std::cout<<"meanH "<<meanH<<"\n";
    //compute mean for vertical lines
    float meanV=0;
    std::cin.get();
    for(int i=0;i<Lvertical.size();i++){
            meanV+=Lvertical[i].intersect.size();            
    }
    meanV=meanV/(Lvertical.size()+n);
    //float mean=(meanH+meanV)/2;
    for( std::vector<lines>::iterator it=Lhorizontal.begin();it!=Lhorizontal.end();){
      
       

        if(it->intersect.size()<meanH){

            Lhorizontal.erase(it);

        }else{
            
            ++it;
        }
  
     
    }



    for( std::vector<lines>::iterator it=Lvertical.begin();it!=Lvertical.end();){

        if(it->intersect.size()<meanV){

            Lvertical.erase(it);
 
        }else{
            ++it;
        }

    }
    
    std::cout<<"size H current "<<Lhorizontal.size()<<"\n";
    std::cout<<"size W current "<<Lvertical.size()<<"\n";
    } while(size_H!=Lhorizontal.size() );
    std::cout<<"size H after "<<Lhorizontal.size()<<"\n";
    std::cout<<"size W after "<<Lvertical.size()<<"\n";
    std::cout<<"number iterations"<<counter<<"\n";
}



void tools::cornersVSlines(std::vector<lines>& Lhorizontal,
                             std::vector<lines>& Lvertical,
                             std::vector<std::pair<float,float> > l1, 
                             std::vector<std::pair<float,float> > l2, 
                             std::set< std::pair<float,float> > cornerList, 
                             float dist2corner, CMatrix<float> image){


    getIntersections( Lhorizontal,Lvertical,l1, l2, cornerList, 0.5*dist2corner,  true);
    //check horizontal lines
    for(std::set< std::pair<float,float> >::iterator itr=cornerList.begin(); itr!=cornerList.end();++itr){
          float cx=itr->first;
          float cy= itr->second; 
         std::vector<lines> tempLines;
        std::vector<int> size;
          for(int j=0;j<Lhorizontal.size();++j){
            for(int k=0;k<Lhorizontal[j].intersect.size();k++){
                  float x=Lhorizontal[j].intersect[k].first;      
                  float y=Lhorizontal[j].intersect[k].second;
                  float dist=sqrt((cx-x)*(cx-x)+(cy-y)*(cy-y));
                 if(dist<dist2corner){
                    tempLines.push_back(Lhorizontal[j]);
                    size.push_back(Lhorizontal[j].intersect.size());
                    break;
                }
            }
          }  
        std::vector<int>::iterator it=std::max_element(size.begin(),size.end());
        int index=it-size.begin();
        //erase lines
        for(std::vector<lines>::iterator fst=tempLines.begin();fst!=tempLines.end();fst++){

        for(std::vector<lines>::iterator scnd=Lhorizontal.begin();scnd!=Lhorizontal.end();++scnd){
            int ind=fst-tempLines.begin();
            if(scnd->theta==fst->theta && scnd->rho==fst->rho && ind!=index ){
             Lhorizontal.erase(scnd);
             break;
            }     
            }
        }
      }
 //check vertical lines
    for(std::set< std::pair<float,float> >::iterator itr=cornerList.begin(); itr!=cornerList.end();++itr){
          float cx=itr->first;
          float cy= itr->second; 
         std::vector<lines> tempLines;
        std::vector<int> size;
          for(int j=0;j<Lvertical.size();++j){
            for(int k=0;k<Lvertical[j].intersect.size();k++){
                  float x=Lvertical[j].intersect[k].first;      
                  float y=Lvertical[j].intersect[k].second;
                  float dist=sqrt((cx-x)*(cx-x)+(cy-y)*(cy-y));
                 if(dist<dist2corner){
                    tempLines.push_back(Lvertical[j]);
                    size.push_back(Lvertical[j].intersect.size());
                    break;
                }
            }
          }  
        std::vector<int>::iterator it=std::max_element(size.begin(),size.end());
        int index=it-size.begin();
        //erase lines
        for(std::vector<lines>::iterator fst=tempLines.begin();fst!=tempLines.end();fst++){

        for(std::vector<lines>::iterator scnd=Lvertical.begin();scnd!=Lvertical.end();++scnd){
            int ind=fst-tempLines.begin();
            if(scnd->theta==fst->theta && scnd->rho==fst->rho && ind!=index ){
             Lvertical.erase(scnd);
             break;
            }     
            }
        }
      }
       

   CTensor<float> lines1(image.xSize(),image.ySize(),3,0);
    lines1.putMatrix(image,0);
    lines1.putMatrix(image,1);
    lines1.putMatrix(image,2);


  for(int i=0;i<Lhorizontal.size();i++){
     float th=Lhorizontal[i].theta;
     float rh=Lhorizontal[i].rho;
 

            for(int x=0;x<lines1.xSize();++x){
                for(int y=0;y<lines1.ySize();++y){
                    if(sin(th)!=0){    
                    float j=(rh-x*cos(th))/sin(th);
                     if(j>=0 && j<lines1.ySize()){
                        lines1(x,j,0)=255;
                        lines1(x,j,1)=0;
                        lines1(x,j,2)=0;
                    }
                    }
     
                    if(cos(th)!=0){    
                    float k=(rh-y*sin(th))/cos(th);
                    if(k>=0 && k<lines1.xSize()){
                        lines1(k,y,0)=255;
                        lines1(k,y,1)=0;
                        lines1(k,y,2)=0;
                    }   
                    }                    
         
                }
            }
        } 


    for(int i=0;i<Lvertical.size();i++){
     float th=Lvertical[i].theta;
     float rh=Lvertical[i].rho;
 

            for(int x=0;x<lines1.xSize();++x){
                for(int y=0;y<lines1.ySize();++y){
                    if(sin(th)!=0){    
                    float j=(rh-x*cos(th))/sin(th);
                     if(j>=0 && j<lines1.ySize()){
                        lines1(x,j,0)=0;
                        lines1(x,j,1)=255;
                        lines1(x,j,2)=0;
                    }
                    }
     
                    if(cos(th)!=0){    
                    float k=(rh-y*sin(th))/cos(th);
                    if(k>=0 && k<lines1.xSize()){
                        lines1(k,y,0)=0;
                        lines1(k,y,1)=255;
                        lines1(k,y,2)=0;
                    }   
                    }                    
         
                }
            }
        }  
 lines1.writeToPPM("../test/c.ppm");


}
