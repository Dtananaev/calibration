

#include "tools.h"

int main(){

    CMatrix<float> image;
    tools t;
   // image.readFromPGM("../test/CalibIm5.pgm");
    image.readFromPGM("../test/frame7.pgm");
    CMatrix<float> image1=image;   
    CMatrix<float> edges=t.CannyEdgeDetector(image1); 
    CMatrix<float> result =t.HarrisEdgeDetector(image);
    std::vector<std::pair<float,float> > l1;
    std::vector<std::pair<float,float> > l2;
    CTensor<float> lines=t.extractHoughLines(edges,image,l1,l2);  
    std::set<std::pair<float, float> > cornerList=t.getALLCornerCoordinates( result, 3);
    t.findChessBoardLines(l1,l2, cornerList, 5,image);
    
    result.normalize(0,255);
    result.writeToPGM("../test/Harris.pgm");   
    lines.writeToPPM("../test/lines.ppm");
}
