

#include "tools.h"

int main(){

    CMatrix<float> image;
    tools t;
    image.readFromPGM("../test/CalibIm1.pgm");
    CMatrix<float> image1=image;    
    CMatrix<float> result =t.HarrisEdgeDetector(image);
    CMatrix<float> result2 = t.getLocalMaximum(result, 256);



float diff=0;
    for(int x=0;x<result.xSize();x++){
    for(int y=0;y<result.ySize();y++){

diff+=result(x,y)-result2(x,y);
    
        }


    }

std::cout<<"difference "<<diff<<"\n";
    result.normalize(0,255);
result2.normalize(0,255);
    result.writeToPGM("../test/result.pgm");    
    result2.writeToPGM("../test/result2.pgm");   
 

}
