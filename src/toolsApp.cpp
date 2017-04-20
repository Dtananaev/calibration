

#include "tools.h"

typedef struct{
    float x;
    float y;
}point;


bool read_file(std::string filename, std::vector<point> &output){

     std::ifstream file_stream_pose(filename);
    if (file_stream_pose.is_open()) { // check if file exsist
         std::string line;
         while (std::getline(file_stream_pose, line)) {
            std::istringstream file_stream_pose(line);

            //each row of txt file contain 4 points related to the corners of the black square
            point p1;
            point p2;            
            point p3;   
            point p4; 
             
           file_stream_pose>>p1.x>>p1.y>>p2.x>>p2.y>>p3.x>>p3.y>>p4.x>>p4.y;
           output.push_back(p1);
           output.push_back(p2);
           output.push_back(p3);
           output.push_back(p4);
         }
         file_stream_pose.close();
    }else{
       return false;
    }

    return true;
}

int main(){

    CMatrix<float> image;
    tools t;
    image.readFromPGM("../test/CalibIm5.pgm");
    //image.readFromPGM("../test/frame1.pgm");
    CMatrix<float> image1=image;   
    CMatrix<float> edges=t.CannyEdgeDetector(image1); 
std::cout<<"image x size "<<image.xSize()<<"\n";
std::cout<<"image y size "<<image.ySize()<<"\n";
std::cout<<"image1 x size "<<image1.xSize()<<"\n";
std::cout<<"image1 y size "<<image1.ySize()<<"\n";
    CMatrix<float> result =t.HarrisEdgeDetector(image);
    std::vector<std::pair<float,float> > l1;
    std::vector<std::pair<float,float> > l2;
    t.extractHoughLines(edges,image,l1,l2);  
    std::set<std::pair<float, float> > cornerList=t.getALLCornerCoordinates( result, 3);
    t.findChessBoardLines(l1,l2, cornerList, 5,image);
    	
    std::vector<std::pair<float,float> > corners=t.extractCornerCoordinates(result, l1, l2, cornerList,  3);


    CTensor<float> res=t.drawCornerLines(corners,l1, l2, image);
   res.writeToPPM("../test/res.ppm");
}
