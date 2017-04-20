/*
 * File: visualizer.cpp
 * Author: Denis Tananaev 
 * Date: 20.03.2017
 *
 *
 */



#include "pclviewer.h"
#include "../build/ui_pclviewer.h"
#include <pcl/filters/voxel_grid.h>


PCLViewer::PCLViewer (QWidget *parent) :
    QMainWindow (parent),
    ui (new Ui::PCLViewer)
{
  ui->setupUi (this);
  this->setWindowTitle ("SUN3D dataset viewer");
    //setup the menu
    // File menu   
  menuBar()->setNativeMenuBar(false);// this line  is necessary for visualization otherwise menu invisible
  QAction *open = new QAction( "&Open", this);
  QAction *quit = new QAction( "&Quit", this);
  quit->setShortcut(tr("CTRL+Q"));
  QMenu *file;
  file = menuBar()->addMenu("&File");
  file->addAction(open);
  file->addSeparator();
  file->addAction(quit);


  // Setup the cloud pointer
  cloud_.reset(new PointCloudT);
  cloud_viz_.reset(new PointCloudT);
  // Set up the QVTK window
  viewer_.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  viewer_->setBackgroundColor (0.8, 0.8, 1.0);
  ui->qvtkWidget->SetRenderWindow (viewer_->getRenderWindow ());
  viewer_->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  viewer_->addCoordinateSystem (1.0);
  ui->qvtkWidget->update ();

    //CONNECT BUTTONS
   // File menubar connect 
  connect(open, SIGNAL(triggered()), this, SLOT(open()));
  connect(quit, SIGNAL(triggered()), qApp, SLOT(quit()));

  connect(ui->plusButton, SIGNAL(clicked()), this, SLOT(plusButton()));
  connect(ui->minusButton, SIGNAL(clicked()), this, SLOT(minusButton()));

  //connect(ui->indexBox, SIGNAL(valueChanged(int)), this, SLOT(updateImage()));
  connect (ui->horizontalSlider, SIGNAL (sliderReleased()), this, SLOT (MysliderReleased()));
  connect (ui->horizontalSlider, SIGNAL (valueChanged (int)), this, SLOT (sliderValueChanged(int)));
  //connect( ui->XYZcameraButton, SIGNAL( toggled(bool) ), this, SLOT( radioButtonclickedXYZcamera(bool) ) );
 // connect( ui->XYZworldButton, SIGNAL( toggled(bool) ), this, SLOT( radioButtonclickedXYZworld(bool) ) );
  ui->horizontalSlider->setRange(0,0);
  // Color the randomly generated cloud
  viewer_->addPointCloud (cloud_, "cloud");
  viewer_->resetCamera();
  ui->qvtkWidget->update();

}

PCLViewer::~PCLViewer(){

  delete ui;
}


void PCLViewer::plusButton(){

if(clb_.images_.size()==0){return;}
 int index=ui->horizontalSlider->value();
 if(index<clb_.images_.size()-1){
    ui->horizontalSlider->setValue(index+1);
}
}

void PCLViewer::minusButton(){
if(clb_.images_.size()==0){return;}
 int index=ui->horizontalSlider->value();
 if(index>0){
    ui->horizontalSlider->setValue(index-1);
}
}


void PCLViewer::open(){
    QString folder_path = QFileDialog::getExistingDirectory(this, tr("Load data"), "");      
       clb_.loadData(folder_path.toUtf8().constData());//load data
       ui->horizontalSlider->setRange(0,clb_.images_.size()-1);
  //detect chess border
   for(int i=0;i<clb_.gimages_.size();++i){

     QCoreApplication::processEvents();
     std::vector<std::pair<float,float> > corners;
     CTensor<float> detected_board;
      bool detect=clb_.DetectBoard( clb_.gimages_[i],clb_.images_[i], corners,detected_board);
     // if(detect){
          clb_.imCorners.push_back(corners);
       // }
	clb_.detectedIm.push_back(detected_board);
    ui->horizontalSlider->setValue(i);
    MysliderReleased(); 
    }


}

void PCLViewer::MysliderReleased(){
if(clb_.images_.size()==0){return;}
    int index=ui->horizontalSlider->value(); 
     update(index);
}

void PCLViewer::update(int index){
    CTensor<float> image=clb_.detectedIm[index];
    //image 
   QImage img(image.xSize(), image.ySize(), QImage::Format_RGB888);
    for (int x = 0; x <image.xSize(); ++x) {
    for (int y = 0; y <image.ySize(); ++y) {
      img.setPixel(x, y, qRgb(image(x,y,0), image(x,y,1), image(x,y,2)));
    }
  }

   img=img.scaled(ui->imagelb->height(),ui->imagelb->width(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
   ui->imagelb->setPixmap(QPixmap::fromImage(img));
   ui->imagelb->show();

   
   ui->numCorners->setText(QString::number(clb_.imCorners[index].size()));
  
}
void PCLViewer::sliderValueChanged(int index){
if(clb_.images_.size()==0){return;}
      ui->lcdNumber->display(index);
        if(!ui->horizontalSlider->isSliderDown()){ update(index);}     
}
    





