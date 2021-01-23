#ifndef PCLVIEWER_1_H
#define PCLVIEWER_1_H

#include <iostream>
#include <cmath>
#include <ostream>
#include <iterator>
#include <vector>

#include <QMainWindow>

#include <pcl/PointIndices.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <vtk-6.3/vtkRenderWindow.h>

namespace Ui {
class PCLViewer_1;
}

class PCLViewer_1 : public QMainWindow
{
    Q_OBJECT

public:
    explicit PCLViewer_1(QWidget *parent = 0);
    ~PCLViewer_1();

private slots:
    void on_r00_T0i_valueChanged(double arg1);

    void on_r01_T0i_valueChanged(double arg1);

    void on_r02_T0i_valueChanged(double arg1);

    void on_rx_T0i_valueChanged(double arg1);

    void on_r10_T0i_valueChanged(double arg1);

    void on_r11_T0i_valueChanged(double arg1);

    void on_r12_T0i_valueChanged(double arg1);

    void on_ry_T0i_valueChanged(double arg1);

    void on_r20_T0i_valueChanged(double arg1);

    void on_r21_T0i_valueChanged(double arg1);

    void on_r22_T0i_valueChanged(double arg1);

    void on_rz_T0i_valueChanged(double arg1);

    void on_r00_T0d_valueChanged(double arg1);

    void on_r01_T0d_valueChanged(double arg1);

    void on_r02_T0d_valueChanged(double arg1);

    void on_rx_T0d_valueChanged(double arg1);

    void on_r10_T0d_valueChanged(double arg1);

    void on_r11_T0d_valueChanged(double arg1);

    void on_r12_T0d_valueChanged(double arg1);

    void on_ry_T0d_valueChanged(double arg1);

    void on_r20_T0d_valueChanged(double arg1);

    void on_r21_T0d_valueChanged(double arg1);

    void on_r22_T0d_valueChanged(double arg1);

    void on_rz_T0d_valueChanged(double arg1);

    void on_r00_T0s_valueChanged(double arg1);

    void on_r01_T0s_valueChanged(double arg1);

    void on_r02_T0s_valueChanged(double arg1);

    void on_rx_T0s_valueChanged(double arg1);

    void on_r10_T0s_valueChanged(double arg1);

    void on_r11_T0s_valueChanged(double arg1);

    void on_r12_T0s_valueChanged(double arg1);

    void on_ry_T0s_valueChanged(double arg1);

    void on_r20_T0s_valueChanged(double arg1);

    void on_r21_T0s_valueChanged(double arg1);

    void on_r22_T0s_valueChanged(double arg1);

    void on_rz_T0s_valueChanged(double arg1);

    void on_radio_filtro_valueChanged(double arg1);

    void on_vecinos_filtro_valueChanged(double arg1);

protected:
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud0;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudI;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudD;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudS;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr T_cloudI;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr T_cloudD;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr T_cloudS;

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> C_cloud0;
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> C_cloudS;
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> C_cloudD;
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> C_cloudI;

    std::vector<int> index0, indexd, indexs, indexi;

    Eigen::Affine3f T0i = Eigen::Affine3f::Identity();
    Eigen::Affine3f T0d = Eigen::Affine3f::Identity();
    Eigen::Affine3f T0s = Eigen::Affine3f::Identity();

    Eigen::Matrix4f mat_T0i;
    Eigen::Matrix4f mat_T0d;
    Eigen::Matrix4f mat_T0s;

    pcl::visualization::PCLVisualizer::Ptr viewer;

    float theta = M_PI;

    pcl::RadiusOutlierRemoval<pcl::PointXYZRGBA> outrem;

private:
    Ui::PCLViewer_1 *ui;
};

#endif // PCLVIEWER_1_H
