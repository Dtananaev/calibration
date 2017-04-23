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
  viewer_->addCoordinateSystem (0.01);
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
  connect( ui->checkBox, SIGNAL( toggled(bool) ), this, SLOT( checkBoxClicked(bool) ) );
 // connect( ui->XYZworldButton, SIGNAL( toggled(bool) ), this, SLOT( radioButtonclickedXYZworld(bool) ) );
  ui->horizontalSlider->setRange(0,0);
  // Color the randomly generated cloud
  viewer_->addPointCloud (cloud_, "cloud");
  viewer_->resetCamera();
  ui->qvtkWidget->update();
  ui->calibrateButton->setEnabled(false);
}

PCLViewer::~PCLViewer(){

  delete ui;
}
void PCLViewer::checkBoxClicked(bool checked){
 int index=ui->horizontalSlider->value();
auto it=listOfcornersForUse.find(index);
	if(checked){
		if(it==listOfcornersForUse.end()){

			listOfcornersForUse.insert(index);
 			  ui->status->setText(QString::number(listOfcornersForUse.size())); 
		}
	}
	else{
		if(it!=listOfcornersForUse.end()){
			listOfcornersForUse.erase(it);
 			  ui->status->setText(QString::number(listOfcornersForUse.size())); 	
		}

	}

listOfcornersForUse.size()<5 ? ui->calibrateButton->setEnabled(false):ui->calibrateButton->setEnabled(true);

}

void PCLViewer::plusButton(){

if(clb_.images_.size()==0){return;}
 int index=ui->horizontalSlider->value();


	
 if(index<clb_.images_.size()-1){


    ui->horizontalSlider->setValue(index+1);
}
}


void PCLViewer::showModel(){
    cloud_->points.clear();


    for(int i=0; i<clb_.model_.size();i++){
        QCoreApplication::processEvents();
      
                 PointT p;
                 p.x = clb_.model_[i].first;
                 p.y =clb_.model_[i].second;
                 p.z = 0;
                 p.r = 255;
                 p.g =0;
                 p.b = 0;
                 cloud_->points.push_back(p);
 	}
        viewer_->updatePointCloud(cloud_, "cloud");
         ui->qvtkWidget->update();
   
    

}


void PCLViewer::minusButton(){
if(clb_.images_.size()==0){return;}
 int index=ui->horizontalSlider->value();
 if(index>0){

    ui->horizontalSlider->setValue(index-1);
}
}


void PCLViewer::open(){
	ui->checkBox->setEnabled(false);
   ui->plusButton->setEnabled(false);
   ui->minusButton->setEnabled(false);
 ui->horizontalSlider->setEnabled(false);
    QString folder_path = QFileDialog::getExistingDirectory(this, tr("Load data"), "");      
       clb_.loadData(folder_path.toUtf8().constData());//load data
       ui->horizontalSlider->setRange(0,clb_.images_.size()-1);
	showModel();
  //detect chess border
   for(int i=0;i<clb_.gimages_.size();++i){
     QCoreApplication::processEvents();
     std::vector<std::pair<float,float> > corners;
     CTensor<float> detected_board;
      bool detect=clb_.DetectBoard( clb_.gimages_[i],clb_.images_[i], corners,detected_board);
          clb_.imCorners[i]=corners;
	  clb_.detectedIm.push_back(detected_board);
  
    ui->horizontalSlider->setValue(i);
  	update(i);
   // MysliderReleased(); 
    }
	ui->checkBox->setEnabled(true);
   ui->plusButton->setEnabled(true);
   ui->minusButton->setEnabled(true);
 ui->horizontalSlider->setEnabled(true);
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
   ui->status->setText(QString::number(listOfcornersForUse.size()));  


}
void PCLViewer::sliderValueChanged(int index){
if(clb_.images_.size()==0){return;}
      ui->lcdNumber->display(index);
        if(!ui->horizontalSlider->isSliderDown()){ 
 	bool in_set=listOfcornersForUse.find(index)!=listOfcornersForUse.end();
std::cout<<"in_set "<<in_set<<"\n";
 	in_set==true ?  ui->checkBox->setChecked(true) : ui->checkBox->setChecked(false);
update(index);}     
}
    





