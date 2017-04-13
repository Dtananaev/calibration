

#include "tools.h"

int main(){

    CMatrix<float> image;
    tools t;
    image.readFromPGM("../test/CalibIm1.pgm");
    CMatrix<float> result=t.CannyEdgeDetector(image);
    result.normalize(0,255);
    result.writeToPGM("../test/result.pgm");        
}
