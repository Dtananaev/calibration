

#include "tools.h"

int main(){

    CMatrix<float> image;
    tools t;
    image.readFromPGM("../test/CalibIm3.pgm");
//image.readFromPGM("../test/frame.pgm");
    CMatrix<float> image1=image;   
    CMatrix<float> edges=t.CannyEdgeDetector(image1); 
    CMatrix<float> result =t.HarrisEdgeDetector(image);
    CTensor<float> lines=t.extractHoughLines(edges,image);  
    CMatrix<float> lmaximum=t.getLocalMaximum( result, 3);

   lmaximum.normalize(0,255);
   lmaximum.writeToPGM("../test/lmaximum.pgm");
    result.normalize(0,255);
    result.writeToPGM("../test/Harris.pgm"); 
   
    lines.writeToPPM("../test/lines.ppm");
}
