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

  // Connect "Load" and "Save" buttons and their functions
  connect (ui->pushButton_load, SIGNAL(clicked ()), this, SLOT(loadFileButtonPressed ()));
  connect (ui->pushButton_save, SIGNAL(clicked ()), this, SLOT(saveFileButtonPressed ()));
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
  ui->pointNumber->setText(QString::number(cloud_->points.size()));
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
  //detect chess border
   for(int i=0;i<clb_.gimages_.size();++i){

     QCoreApplication::processEvents();
     std::vector<std::pair<float,float> > corners;
     CTensor<float> detected_board;
      bool detect=clb_.DetectBoard( clb_.gimages_[i], corners,detected_board);
      if(detect){
          clb_.imCorners.push_back(corners);
        }
	clb_.detectedIm.push_back(detected_board);
    }
    ui->horizontalSlider->setRange(0,clb_.images_.size()-1);
    MysliderReleased(); 
}

void PCLViewer::MysliderReleased(){
if(clb_.images_.size()==0){return;}
    int index=ui->horizontalSlider->value(); 
     update(index);
}

void PCLViewer::update(int index){
    CTensor<float> image=clb_.images_[index];
    //image 
   QImage img(image.xSize(), image.ySize(), QImage::Format_RGB888);
    for (int x = 0; x <image.xSize(); ++x) {
    for (int y = 0; y <image.ySize(); ++y) {
      img.setPixel(x, y, qRgb(image(x,y,0), image(x,y,1), image(x,y,2)));
    }
  }

   img=img.scaledToWidth(ui->imagelb->width(), Qt::SmoothTransformation);
   ui->imagelb->setPixmap(QPixmap::fromImage(img));
   ui->imagelb->show();

   CTensor<float> gimage=clb_.detectedIm[index];

 QImage gim(gimage.xSize(),gimage.ySize(), QImage::Format_RGB32);
    for (int x = 0; x <gimage.xSize(); ++x) {
    for (int y = 0; y < gimage.ySize(); ++y) {
      gim.setPixel(x, y, qRgb(gimage(x,y,0), gimage(x,y,1), gimage(x,y,2)));
    }
  }
   gim=gim.scaledToWidth(ui->gimagelb->width(), Qt::SmoothTransformation);
   ui->gimagelb->setPixmap(QPixmap::fromImage(gim));
   ui->gimagelb->show();

  
}
void PCLViewer::sliderValueChanged(int index){
if(clb_.images_.size()==0){return;}
      ui->lcdNumber->display(index);
        if(!ui->horizontalSlider->isSliderDown()){ update(index);}     
}
    
void PCLViewer::loadFileButtonPressed(){
  // You might want to change "/home/" if you're not on an *nix platform
  QString filename = QFileDialog::getOpenFileName (this, tr ("Open point cloud"), "/home/", tr ("Point cloud data (*.pcd *.ply)"));

  PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());
  PointCloudT::Ptr cloud_tmp (new PointCloudT);

  if (filename.isEmpty ())
    return;

  int return_status;
  if (filename.endsWith (".pcd", Qt::CaseInsensitive))
    return_status = pcl::io::loadPCDFile (filename.toStdString (), *cloud_tmp);
  else
    return_status = pcl::io::loadPLYFile (filename.toStdString (), *cloud_tmp);

  if (return_status != 0)
  {
    PCL_ERROR("Error reading point cloud %s\n", filename.toStdString ().c_str ());
    return;
  }

  // If point cloud contains NaN values, remove them before updating the visualizer point cloud
  if (cloud_tmp->is_dense)
    pcl::copyPointCloud (*cloud_tmp, *cloud_);
  else
  {
    PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
    std::vector<int> vec;
    pcl::removeNaNFromPointCloud (*cloud_tmp, *cloud_, vec);
  }

  viewer_->updatePointCloud (cloud_, "cloud");
  viewer_->resetCamera ();
  ui->qvtkWidget->update ();
}

void PCLViewer::saveFileButtonPressed(){
  // You might want to change "/home/" if you're not on an *nix platform
  QString filename = QFileDialog::getSaveFileName(this, tr ("Open point cloud"), "/home/", tr ("Point cloud data (*.pcd *.ply)"));

  PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());

  if (filename.isEmpty ())
    return;

  int return_status;
  if (filename.endsWith (".pcd", Qt::CaseInsensitive))
    return_status = pcl::io::savePCDFileBinary (filename.toStdString (), *cloud_);
  else if (filename.endsWith (".ply", Qt::CaseInsensitive))
    return_status = pcl::io::savePLYFileBinary (filename.toStdString (), *cloud_);
  else
  {
    filename.append(".ply");
    return_status = pcl::io::savePLYFileBinary (filename.toStdString (), *cloud_);
  }

  if (return_status != 0)
  {
    PCL_ERROR("Error writing point cloud %s\n", filename.toStdString ().c_str ());
    return;
  }
}






