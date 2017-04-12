/*
 * File: loader.h
 * Author: Denis Tananaev
 * Date: 12.04.2017
 * 
 */


#ifndef LOADER_H_
#define LOADER_H_

#include <iostream>
#include <utility> //std::pair
#include <vector>
#include <png++/png.hpp>
#include <jpeglib.h>
#include <string>
#include <stdlib.h>
#include <fstream>
#include <sstream>

//Qt
#include <QDir>
//external 
#include "CVector.h"
#include "CMatrix.h"
#include "CTensor.h"

class loader{

public:
    loader();
    virtual ~loader();
    void loadData(std::string directory);
    bool getImagesNames(std::string image_path);
    CTensor<float> GetImageData(std::string file_name);
    CMatrix<float> Convert2Grayscale(CTensor<float> image);
    std::vector<std::string> image_list_;
    std::vector<CTensor<float> > images_;
    std::vector<CMatrix<float> > gimages_;
private:
};





#endif //LOADER_H_
