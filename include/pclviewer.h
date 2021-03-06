#ifndef PCLVIEWER_H
#define PCLVIEWER_H
#include <set>
#include <string>
#include <algorithm>
#include <unordered_set>
// Qt
#include <QMainWindow>
#include <QFileDialog>
#include <QMenu>
#include <QMenuBar>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>

// Boost
#include <boost/math/special_functions/round.hpp>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>
//my
#include "calibration.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui
{
  class PCLViewer;
}

class PCLViewer: public QMainWindow
{
  Q_OBJECT

   public slots:
   void checkBoxClicked(bool);
    void plusButton();
    void minusButton(); 
    void calibrateButton();       
    //void updateImage();  
    void MysliderReleased();
    void sliderValueChanged(int index);
    private slots:
    void open();
  public:
   void showModel();
    explicit PCLViewer (QWidget *parent = 0);
    virtual ~PCLViewer ();
    void update(int index);

  protected:
    calibration clb_;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    PointCloudT::Ptr cloud_;
    std::unordered_set<int> listOfcornersForUse;

  private:
    Ui::PCLViewer *ui;
};

#endif // PCLVIEWER_H

