#include "pclviewer_1.h"
#include "ui_pclviewer_1.h"

PCLViewer_1::PCLViewer_1(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::PCLViewer_1)
{
    ui->setupUi(this);

    cloud0.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    cloudD.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    cloudI.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    cloudS.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

    T_cloudD.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    T_cloudI.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    T_cloudS.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::io::loadPCDFile<pcl::PointXYZRGBA>("pos0.pcd", *cloud0);
    pcl::io::loadPCDFile<pcl::PointXYZRGBA>("posd.pcd", *cloudD);
    pcl::io::loadPCDFile<pcl::PointXYZRGBA>("posi.pcd", *cloudI);
    pcl::io::loadPCDFile<pcl::PointXYZRGBA>("poss.pcd", *cloudS);

    pcl::removeNaNFromPointCloud(*cloud0, *cloud0, index0);
    pcl::removeNaNFromPointCloud(*cloudD, *cloudD, indexd);
    pcl::removeNaNFromPointCloud(*cloudS, *cloudS, indexs);
    pcl::removeNaNFromPointCloud(*cloudI, *cloudI, indexi);

    T0i.translation() << 1.51, 0, 1.45;
    T0d.translation() << -1.44, 0, 1.45;
    T0s.translation() << 0, 0, 3.13;

    T0i.rotate(Eigen::AngleAxisf(-theta/2, Eigen::Vector3f::UnitY()));
    T0s.rotate(Eigen::AngleAxisf(-theta, Eigen::Vector3f::UnitY()));
    T0d.rotate(Eigen::AngleAxisf(theta/2, Eigen::Vector3f::UnitY()));

    mat_T0d = T0d.matrix();
    mat_T0i = T0i.matrix();
    mat_T0s = T0s.matrix();

    pcl::transformPointCloud(*cloudD,*T_cloudD,T0d);
    pcl::transformPointCloud(*cloudI,*T_cloudI,T0i);
    pcl::transformPointCloud(*cloudS,*T_cloudS,T0s);

    C_cloud0.setInputCloud(cloud0);
    C_cloudD.setInputCloud(T_cloudD);
    C_cloudI.setInputCloud(T_cloudI);
    C_cloudS.setInputCloud(T_cloudS);

    viewer.reset(new pcl::visualization::PCLVisualizer ("viewer", false));
    ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor (ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow ());
    ui->qvtkWidget->update ();

    viewer->addPointCloud(cloud0,C_cloud0,"pos0");
    viewer->addPointCloud(T_cloudD,C_cloudD,"posd");
    viewer->addPointCloud(T_cloudI,C_cloudI,"posi");
    viewer->addPointCloud(T_cloudS,C_cloudS,"poss");
    ui->qvtkWidget->update();

    ui->r00_T0d->setValue(mat_T0d(0,0));
    ui->r01_T0d->setValue(mat_T0d(0,1));
    ui->r02_T0d->setValue(mat_T0d(0,2));
    ui->rx_T0d->setValue(mat_T0d(0,3));
    ui->r10_T0d->setValue(mat_T0d(1,0));
    ui->r11_T0d->setValue(mat_T0d(1,1));
    ui->r12_T0d->setValue(mat_T0d(1,2));
    ui->ry_T0d->setValue(mat_T0d(1,3));
    ui->r20_T0d->setValue(mat_T0d(2,0));
    ui->r21_T0d->setValue(mat_T0d(2,1));
    ui->r22_T0d->setValue(mat_T0d(2,2));
    ui->rz_T0d->setValue(mat_T0d(2,3));

    ui->r00_T0i->setValue(mat_T0i(0,0));
    ui->r01_T0i->setValue(mat_T0i(0,1));
    ui->r02_T0i->setValue(mat_T0i(0,2));
    ui->rx_T0i->setValue(mat_T0i(0,3));
    ui->r10_T0i->setValue(mat_T0i(1,0));
    ui->r11_T0i->setValue(mat_T0i(1,1));
    ui->r12_T0i->setValue(mat_T0i(1,2));
    ui->ry_T0i->setValue(mat_T0i(1,3));
    ui->r20_T0i->setValue(mat_T0i(2,0));
    ui->r21_T0i->setValue(mat_T0i(2,1));
    ui->r22_T0i->setValue(mat_T0i(2,2));
    ui->rz_T0i->setValue(mat_T0i(2,3));

    ui->r00_T0s->setValue(mat_T0s(0,0));
    ui->r01_T0s->setValue(mat_T0s(0,1));
    ui->r02_T0s->setValue(mat_T0s(0,2));
    ui->rx_T0s->setValue(mat_T0s(0,3));
    ui->r10_T0s->setValue(mat_T0s(1,0));
    ui->r11_T0s->setValue(mat_T0s(1,1));
    ui->r12_T0s->setValue(mat_T0s(1,2));
    ui->ry_T0s->setValue(mat_T0s(1,3));
    ui->r20_T0s->setValue(mat_T0s(2,0));
    ui->r21_T0s->setValue(mat_T0s(2,1));
    ui->r22_T0s->setValue(mat_T0s(2,2));
    ui->rz_T0s->setValue(mat_T0s(2,3));

    outrem.setKeepOrganized(true);

}

PCLViewer_1::~PCLViewer_1()
{
    delete ui;
}

void PCLViewer_1::on_r00_T0i_valueChanged(double arg1)
{
    double r00 = arg1;
    mat_T0i(0,0) = r00;
    pcl::transformPointCloud(*cloudI,*T_cloudI,mat_T0i);
    C_cloudI.setInputCloud(T_cloudI);
    viewer->updatePointCloud(T_cloudI,C_cloudI,"posi");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_r01_T0i_valueChanged(double arg1)
{
    double r01 = arg1;
    mat_T0i(0,1) = r01;
    pcl::transformPointCloud(*cloudI,*T_cloudI,mat_T0i);
    C_cloudI.setInputCloud(T_cloudI);
    viewer->updatePointCloud(T_cloudI,C_cloudI,"posi");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_r02_T0i_valueChanged(double arg1)
{
    double r02 = arg1;
    mat_T0i(0,2) = r02;
    pcl::transformPointCloud(*cloudI,*T_cloudI,mat_T0i);
    C_cloudI.setInputCloud(T_cloudI);
    viewer->updatePointCloud(T_cloudI,C_cloudI,"posi");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_rx_T0i_valueChanged(double arg1)
{
    double rx = arg1;
    mat_T0i(0,3) = rx;
    pcl::transformPointCloud(*cloudI,*T_cloudI,mat_T0i);
    C_cloudI.setInputCloud(T_cloudI);
    viewer->updatePointCloud(T_cloudI,C_cloudI,"posi");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_r10_T0i_valueChanged(double arg1)
{
    double r10 = arg1;
    mat_T0i(1,0) = r10;
    pcl::transformPointCloud(*cloudI,*T_cloudI,mat_T0i);
    C_cloudI.setInputCloud(T_cloudI);
    viewer->updatePointCloud(T_cloudI,C_cloudI,"posi");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_r11_T0i_valueChanged(double arg1)
{
    double r11 = arg1;
    mat_T0i(1,1) = r11;
    pcl::transformPointCloud(*cloudI,*T_cloudI,mat_T0i);
    C_cloudI.setInputCloud(T_cloudI);
    viewer->updatePointCloud(T_cloudI,C_cloudI,"posi");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_r12_T0i_valueChanged(double arg1)
{
    double r12 = arg1;
    mat_T0i(1,2) = r12;
    pcl::transformPointCloud(*cloudI,*T_cloudI,mat_T0i);
    C_cloudI.setInputCloud(T_cloudI);
    viewer->updatePointCloud(T_cloudI,C_cloudI,"posi");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_ry_T0i_valueChanged(double arg1)
{
    double ry = arg1;
    mat_T0i(1,3) = ry;
    pcl::transformPointCloud(*cloudI,*T_cloudI,mat_T0i);
    C_cloudI.setInputCloud(T_cloudI);
    viewer->updatePointCloud(T_cloudI,C_cloudI,"posi");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_r20_T0i_valueChanged(double arg1)
{
    double r20 = arg1;
    mat_T0i(2,0) = r20;
    pcl::transformPointCloud(*cloudI,*T_cloudI,mat_T0i);
    C_cloudI.setInputCloud(T_cloudI);
    viewer->updatePointCloud(T_cloudI,C_cloudI,"posi");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_r21_T0i_valueChanged(double arg1)
{
    double r21 = arg1;
    mat_T0i(2,1) = r21;
    pcl::transformPointCloud(*cloudI,*T_cloudI,mat_T0i);
    C_cloudI.setInputCloud(T_cloudI);
    viewer->updatePointCloud(T_cloudI,C_cloudI,"posi");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_r22_T0i_valueChanged(double arg1)
{
    double r22 = arg1;
    mat_T0i(2,2) = r22;
    pcl::transformPointCloud(*cloudI,*T_cloudI,mat_T0i);
    C_cloudI.setInputCloud(T_cloudI);
    viewer->updatePointCloud(T_cloudI,C_cloudI,"posi");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_rz_T0i_valueChanged(double arg1)
{
    double rz = arg1;
    mat_T0i(2,3) = rz;
    pcl::transformPointCloud(*cloudI,*T_cloudI,mat_T0i);
    C_cloudI.setInputCloud(T_cloudI);
    viewer->updatePointCloud(T_cloudI,C_cloudI,"posi");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_r00_T0d_valueChanged(double arg1)
{
    double r00 = arg1;
    mat_T0d(0,0) = r00;
    pcl::transformPointCloud(*cloudD,*T_cloudD,mat_T0d);
    C_cloudD.setInputCloud(T_cloudD);
    viewer->updatePointCloud(T_cloudD,C_cloudD,"posd");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_r01_T0d_valueChanged(double arg1)
{
    double r01 = arg1;
    mat_T0d(0,1) = r01;
    pcl::transformPointCloud(*cloudD,*T_cloudD,mat_T0d);
    C_cloudD.setInputCloud(T_cloudD);
    viewer->updatePointCloud(T_cloudD,C_cloudD,"posd");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_r02_T0d_valueChanged(double arg1)
{
    double r02 = arg1;
    mat_T0d(0,2) = r02;
    pcl::transformPointCloud(*cloudD,*T_cloudD,mat_T0d);
    C_cloudD.setInputCloud(T_cloudD);
    viewer->updatePointCloud(T_cloudD,C_cloudD,"posd");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_rx_T0d_valueChanged(double arg1)
{
    double rx = arg1;
    mat_T0d(0,3) = rx;
    pcl::transformPointCloud(*cloudD,*T_cloudD,mat_T0d);
    C_cloudD.setInputCloud(T_cloudD);
    viewer->updatePointCloud(T_cloudD,C_cloudD,"posd");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_r10_T0d_valueChanged(double arg1)
{
    double r10 = arg1;
    mat_T0d(1,0) = r10;
    pcl::transformPointCloud(*cloudD,*T_cloudD,mat_T0d);
    C_cloudD.setInputCloud(T_cloudD);
    viewer->updatePointCloud(T_cloudD,C_cloudD,"posd");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_r11_T0d_valueChanged(double arg1)
{
    double r11 = arg1;
    mat_T0d(1,1) = r11;
    pcl::transformPointCloud(*cloudD,*T_cloudD,mat_T0d);
    C_cloudD.setInputCloud(T_cloudD);
    viewer->updatePointCloud(T_cloudD,C_cloudD,"posd");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_r12_T0d_valueChanged(double arg1)
{
    double r12 = arg1;
    mat_T0d(1,2) = r12;
    pcl::transformPointCloud(*cloudD,*T_cloudD,mat_T0d);
    C_cloudD.setInputCloud(T_cloudD);
    viewer->updatePointCloud(T_cloudD,C_cloudD,"posd");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_ry_T0d_valueChanged(double arg1)
{
    double ry = arg1;
    mat_T0d(1,3) = ry;
    pcl::transformPointCloud(*cloudD,*T_cloudD,mat_T0d);
    C_cloudD.setInputCloud(T_cloudD);
    viewer->updatePointCloud(T_cloudD,C_cloudD,"posd");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_r20_T0d_valueChanged(double arg1)
{
    double r20 = arg1;
    mat_T0d(2,0) = r20;
    pcl::transformPointCloud(*cloudD,*T_cloudD,mat_T0d);
    C_cloudD.setInputCloud(T_cloudD);
    viewer->updatePointCloud(T_cloudD,C_cloudD,"posd");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_r21_T0d_valueChanged(double arg1)
{
    double r21 = arg1;
    mat_T0d(2,1) = r21;
    pcl::transformPointCloud(*cloudD,*T_cloudD,mat_T0d);
    C_cloudD.setInputCloud(T_cloudD);
    viewer->updatePointCloud(T_cloudD,C_cloudD,"posd");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_r22_T0d_valueChanged(double arg1)
{
    double r22 = arg1;
    mat_T0d(2,2) = r22;
    pcl::transformPointCloud(*cloudD,*T_cloudD,mat_T0d);
    C_cloudD.setInputCloud(T_cloudD);
    viewer->updatePointCloud(T_cloudD,C_cloudD,"posd");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_rz_T0d_valueChanged(double arg1)
{
    double rz = arg1;
    mat_T0d(2,3) = rz;
    pcl::transformPointCloud(*cloudD,*T_cloudD,mat_T0d);
    C_cloudD.setInputCloud(T_cloudD);
    viewer->updatePointCloud(T_cloudD,C_cloudD,"posd");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_r00_T0s_valueChanged(double arg1)
{
    double r00 = arg1;
    mat_T0s(0,0) = r00;
    pcl::transformPointCloud(*cloudS,*T_cloudS,mat_T0s);
    C_cloudS.setInputCloud(T_cloudS);
    viewer->updatePointCloud(T_cloudS,C_cloudS,"poss");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_r01_T0s_valueChanged(double arg1)
{
    double r01 = arg1;
    mat_T0s(0,1) = r01;
    pcl::transformPointCloud(*cloudS,*T_cloudS,mat_T0s);
    C_cloudS.setInputCloud(T_cloudS);
    viewer->updatePointCloud(T_cloudS,C_cloudS,"poss");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_r02_T0s_valueChanged(double arg1)
{
    double r02 = arg1;
    mat_T0s(0,2) = r02;
    pcl::transformPointCloud(*cloudS,*T_cloudS,mat_T0s);
    C_cloudS.setInputCloud(T_cloudS);
    viewer->updatePointCloud(T_cloudS,C_cloudS,"poss");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_rx_T0s_valueChanged(double arg1)
{
    double rx = arg1;
    mat_T0s(0,3) = rx;
    pcl::transformPointCloud(*cloudS,*T_cloudS,mat_T0s);
    C_cloudS.setInputCloud(T_cloudS);
    viewer->updatePointCloud(T_cloudS,C_cloudS,"poss");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_r10_T0s_valueChanged(double arg1)
{
    double r10 = arg1;
    mat_T0s(1,0) = r10;
    pcl::transformPointCloud(*cloudS,*T_cloudS,mat_T0s);
    C_cloudS.setInputCloud(T_cloudS);
    viewer->updatePointCloud(T_cloudS,C_cloudS,"poss");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_r11_T0s_valueChanged(double arg1)
{
    double r11 = arg1;
    mat_T0s(1,1) = r11;
    pcl::transformPointCloud(*cloudS,*T_cloudS,mat_T0s);
    C_cloudS.setInputCloud(T_cloudS);
    viewer->updatePointCloud(T_cloudS,C_cloudS,"poss");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_r12_T0s_valueChanged(double arg1)
{
    double r12 = arg1;
    mat_T0s(1,2) = r12;
    pcl::transformPointCloud(*cloudS,*T_cloudS,mat_T0s);
    C_cloudS.setInputCloud(T_cloudS);
    viewer->updatePointCloud(T_cloudS,C_cloudS,"poss");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_ry_T0s_valueChanged(double arg1)
{
    double ry = arg1;
    mat_T0s(1,3) = ry;
    pcl::transformPointCloud(*cloudS,*T_cloudS,mat_T0s);
    C_cloudS.setInputCloud(T_cloudS);
    viewer->updatePointCloud(T_cloudS,C_cloudS,"poss");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_r20_T0s_valueChanged(double arg1)
{
    double r20 = arg1;
    mat_T0s(2,0) = r20;
    pcl::transformPointCloud(*cloudS,*T_cloudS,mat_T0s);
    C_cloudS.setInputCloud(T_cloudS);
    viewer->updatePointCloud(T_cloudS,C_cloudS,"poss");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_r21_T0s_valueChanged(double arg1)
{
    double r21 = arg1;
    mat_T0s(2,1) = r21;
    pcl::transformPointCloud(*cloudS,*T_cloudS,mat_T0s);
    C_cloudS.setInputCloud(T_cloudS);
    viewer->updatePointCloud(T_cloudS,C_cloudS,"poss");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_r22_T0s_valueChanged(double arg1)
{
    double r22 = arg1;
    mat_T0s(2,2) = r22;
    pcl::transformPointCloud(*cloudS,*T_cloudS,mat_T0s);
    C_cloudS.setInputCloud(T_cloudS);
    viewer->updatePointCloud(T_cloudS,C_cloudS,"poss");
    ui->qvtkWidget->update();
}

void PCLViewer_1::on_rz_T0s_valueChanged(double arg1)
{
    double rz = arg1;
    mat_T0s(2,3) = rz;
    pcl::transformPointCloud(*cloudS,*T_cloudS,mat_T0s);
    C_cloudS.setInputCloud(T_cloudS);
    viewer->updatePointCloud(T_cloudS,C_cloudS,"poss");
    ui->qvtkWidget->update();
}


void PCLViewer_1::on_radio_filtro_valueChanged(double arg1)
{
    //CAMBIAR RADIO DE BUSQUEDA DE FILTRO
    double radio = arg1;
    double vecinos = ui->vecinos_filtro->value();
    outrem.setRadiusSearch(radio);
    outrem.setMinNeighborsInRadius(vecinos);
    outrem.setInputCloud(cloud0);
    outrem.filter(*cloud0);
    C_cloud0.setInputCloud(cloud0);
    outrem.setInputCloud(T_cloudD);
    outrem.filter(*T_cloudD);
    C_cloudD.setInputCloud(T_cloudD);
    outrem.setInputCloud(T_cloudI);
    outrem.filter(*T_cloudI);
    C_cloudI.setInputCloud(T_cloudI);
    outrem.setInputCloud(T_cloudS);
    outrem.filter(*T_cloudS);
    C_cloudS.setInputCloud(T_cloudS);

    viewer->updatePointCloud(cloud0, C_cloud0, "pos0");
    viewer->updatePointCloud(T_cloudD, C_cloudD, "posd");
    viewer->updatePointCloud(T_cloudI, C_cloudI, "posi");
    viewer->updatePointCloud(T_cloudS, C_cloudS, "poss");
    ui->qvtkWidget->update();

}

void PCLViewer_1::on_vecinos_filtro_valueChanged(double arg1)
{
    //CAMBIAR NUMERO DE VECINOS EN FILTRO

    double vecinos = arg1;
    double radio = ui->radio_filtro->value();
    outrem.setRadiusSearch(radio);
    outrem.setMinNeighborsInRadius(vecinos);
    outrem.setInputCloud(cloud0);
    outrem.filter(*cloud0);
    C_cloud0.setInputCloud(cloud0);
    outrem.setInputCloud(T_cloudD);
    outrem.filter(*T_cloudD);
    C_cloudD.setInputCloud(T_cloudD);
    outrem.setInputCloud(T_cloudI);
    outrem.filter(*T_cloudI);
    C_cloudI.setInputCloud(T_cloudI);
    outrem.setInputCloud(T_cloudS);
    outrem.filter(*T_cloudS);
    C_cloudS.setInputCloud(T_cloudS);

    viewer->updatePointCloud(cloud0, C_cloud0, "pos0");
    viewer->updatePointCloud(T_cloudD, C_cloudD, "posd");
    viewer->updatePointCloud(T_cloudI, C_cloudI, "posi");
    viewer->updatePointCloud(T_cloudS, C_cloudS, "poss");
    ui->qvtkWidget->update();
}
