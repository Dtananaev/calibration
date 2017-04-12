/*
 * File: loader.cpp
 * Author: Denis Tananaev
 * Date: 12.04.2017
 * 
 */


#include "loader.h"


loader::loader(){
}
loader::~loader(){
}

void loader::loadData(std::string directory){
    images_.clear();
    gimages_.clear();
    //load the set of image
    if(getImagesNames(directory)){
        for(int i=0;i<image_list_.size();++i){
            std::string s=directory+"/"+image_list_[i]+".jpg";

            CTensor<float> temp=GetImageData(s);
            CMatrix<float> gtemp=Convert2Grayscale(temp);
                
            gimages_.push_back(gtemp);
            images_.push_back(temp);
        }   
    }
    
}

bool loader::getImagesNames(std::string image_path){
    image_list_.clear();
    QDir dir(QString::fromUtf8(image_path.c_str()));

    if(!dir.exists()) {
        std::cerr << "Folder " << image_path << " doesn't exist." << std::endl;
        std::cerr << "Images won't be loaded" << std::endl;
        return false;
    }

    dir.setFilter(QDir::Files | QDir::Hidden | QDir::NoSymLinks);
    QStringList filters;
    filters << "*.jpg";
    dir.setNameFilters(filters);

    foreach(QFileInfo mitm, dir.entryInfoList()){
        image_list_.push_back(mitm.baseName().toUtf8().constData());
    }
    return true;
}



CTensor<float> loader::GetImageData(std::string file_name){
     
  int kImageChannels=3;
  unsigned char *raw_image = NULL;

  struct jpeg_decompress_struct cinfo;
  struct jpeg_error_mgr jerr;
  JSAMPROW row_pointer[1];
           
  FILE *infile = fopen(file_name.c_str(), "rb");
  unsigned long location = 0;
         
  if (!infile) {
    printf("Error opening jpeg file %s\n!", file_name.c_str());
    
  }
            
  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_decompress(&cinfo);
  jpeg_stdio_src(&cinfo, infile);
  jpeg_read_header(&cinfo, TRUE);
  jpeg_start_decompress(&cinfo);
         
  raw_image = (unsigned char*) malloc(
      cinfo.output_width * cinfo.output_height * cinfo.num_components);
  row_pointer[0] = (unsigned char *) malloc(
      cinfo.output_width * cinfo.num_components);
           
  while (cinfo.output_scanline < cinfo.image_height) {
    jpeg_read_scanlines(&cinfo, row_pointer, 1);
    for (uint i = 0; i < cinfo.image_width * cinfo.num_components; i++)
      raw_image[location++] = row_pointer[0][i];
  }
         
  
  CTensor<float> image(cinfo.image_width,cinfo.image_height,kImageChannels,0);
  for (uint i = 0; i < cinfo.image_height; ++i) {
    for (uint j = 0; j < cinfo.image_width; ++j) {
      for (int k = 0; k < kImageChannels; ++k) {
        image(j,i,k) = raw_image[(i * cinfo.image_width * 3) + (j * 3) + k];
      }
    }
  }
            
  jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);
  free(row_pointer[0]);
  fclose(infile);

    return image;

}


CMatrix<float> loader::Convert2Grayscale(CTensor<float> image){
 // image=0.2989*R + 0.5870*G +0.1140*B 
    CMatrix<float> result(image.xSize(),image.ySize());
    for(int y=0; y<image.ySize();++y){
        for(int x=0; x<image.xSize();++x){
            result(x,y)=0.2989*image(x,y,0) + 0.5870*image(x,y,1) +0.1140*image(x,y,2); 
        }
    }
    
    return result;
}



