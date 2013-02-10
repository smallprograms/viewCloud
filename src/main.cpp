#include <iostream>
#include <sstream>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transforms.h>
#include <boost/thread/thread.hpp>
#include <pcl/PolygonMesh.h>
#include <pcl/Vertices.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/gp3.h>
#include <pcl/features/principal_curvatures.h>

pcl::PolygonMesh getMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

//    char triangleVertex[cloud->size()];
//    for(int k=0; k < cloud->size(); k++) {
//        triangleVertex[k] = 0;
//    }
//    pcl::Vertices triangle;
//    pcl::PolygonMesh mesh;
//    sensor_msgs::PointCloud2 msg;
//    pcl::toROSMsg(*cloud,msg);
//    mesh.cloud = msg;
//    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//    kdtree.setInputCloud(cloud);
//    std::vector<int> closestPoint;
//    std::vector<float> closestDist;
//    int k=3;

//    for(int i=0; i < cloud->size(); i++) {

//        //this vertex isn't from a triangle
//        if( triangleVertex[i] == 0) {
//            closestPoint.clear();
//            kdtree.nearestKSearch(cloud->points[i],k,closestPoint,closestDist );
//            if(closestPoint.size() > 1) {

//                triangle.vertices.push_back(i);
//                triangle.vertices.push_back(closestPoint[1]);
//                triangle.vertices.push_back(closestPoint[2]);
//                mesh.polygons.push_back(triangle);
//                triangleVertex[closestPoint[1]] = 1;
//                triangleVertex[closestPoint[2]] = 1;
//                triangleVertex[i] = 1;
//                triangle.vertices.clear();
//                if(i==1) {
//                    std::cout << "p1: " << closestPoint[0] << "p2: " << closestPoint[1] << "p3" << closestPoint[2] << "\n";
//                }

//            }
//        }

//    }
//    std::cout << "MESH SIZE: " << mesh.polygons.size() << "\n";
//    return mesh;
    typedef pcl::GreedyProjectionTriangulation<pcl::PointNormal> MeshGen;

    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> n;
    pcl::PointCloud<pcl::PointNormal> normals;
    n.setInputCloud(boost::make_shared <const pcl::PointCloud<pcl::PointXYZ> > (*cloud));
    n.setKSearch(40);    // Use k nearest neighbors to estimate the normals
    n.compute(normals);

     for (size_t i = 0; i < normals.points.size (); ++i) {
         normals.points[i].x = cloud->points[i].x;
         normals.points[i].y = cloud->points[i].y;
         normals.points[i].z = cloud->points[i].z;
     }

     pcl::PolygonMesh output;
     MeshGen meshGen;
     meshGen.setInputCloud(normals.makeShared());


//     meshGen.setSearchMethod(boost::make_shared<pcl::KdTreeFLANN<pcl::PointNormal>
//   >());
     meshGen.setNormalConsistency(false);
     meshGen.setSearchRadius(10);
     meshGen.setMu(3);
     meshGen.setMaximumNearestNeighbors(10000);
     meshGen.setMaximumAngle(1);
     meshGen.setMaximumSurfaceAngle(0.2);
     meshGen.reconstruct(output);

     return output;
}

void drawNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                             pcl::visualization::PCLVisualizer& vis) {


    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::PointNormal> pnormals;
    pcl::PointCloud<pcl::Normal> normals;
    pcl::PointCloud<pcl::PrincipalCurvatures> curv;
    n.setInputCloud(boost::make_shared <const pcl::PointCloud<pcl::PointXYZ> > (*cloud));
    n.setKSearch(40);    // Use k nearest neighbors to estimate the normals
    n.compute(normals);
    pcl::concatenateFields(*cloud,normals,pnormals);
   //  vis.addPointCloud(cloud);
//     vis.addPointCloudNormals<pcl::PointNormal>(pnormals.makeShared(),100, 0.02f, "cloudNormals");
//     vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0,"cloudNormals");

     pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ,pcl::PointNormal,pcl::PrincipalCurvatures> pce;
     pce.setInputCloud(cloud);
     pce.setInputNormals(pnormals.makeShared());
     pce.setRadiusSearch(0.01);
     pce.compute(curv);
     //vis.addPointCloudPrincipalCurvatures(cloud,normals.makeShared(),curv.makeShared(),100,1.0f,"cloudnorm");
     pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointNormal> color_handler (pnormals.makeShared(), "curvature");
     if (!color_handler.isCapable ()) {
         PCL_WARN ("Cannot create curvature color handler!");
     }
     vis.addPointCloud(pnormals.makeShared(),color_handler,"myCLooud");
}



void customFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

}

int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ> currCloud(640,480);
    pcl::visualization::PCLVisualizer viewer("Dots");
    pcl::PointCloud<pcl::PointXYZ> finalCloud;
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;



    if(argc < 2) {

        std::cout << "Usage:\n " << argv[0] << " " << "myCloud.pcd" << "\n";
        return 0;
    }

    if( pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], currCloud) == -1 ) {

        std::cout << "Error reading : " << argv[1] << "\n";
        return 0;

    }


    voxelGrid.setInputCloud(currCloud.makeShared());
    voxelGrid.setLeafSize(0.01,0.01,0.01);
    voxelGrid.filter(finalCloud);

    viewer.setBackgroundColor (0, 0, 0);
    viewer.addCoordinateSystem (1.0);
    //viewer.addPointCloud<pcl::PointXYZ>(finalCloud.makeShared(),argv[1]);
    //pcl::PolygonMesh mesh = getMesh(finalCloud.makeShared());
    //viewer.addPolygonMesh(mesh,"polygon");
    drawNormals(finalCloud.makeShared(),viewer);
    //viewer.updatePolygonMesh(mesh,"polygon");
    while( !viewer.wasStopped() ) {
        viewer.spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

}
