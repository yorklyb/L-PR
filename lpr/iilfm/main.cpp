#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
// #include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <pcl/io/png_io.h>
#include <pcl/visualization/common/float_image_utils.h>

#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
// #include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>

#include <pcl/range_image/impl/range_image.hpp>
#include <fstream>

#include <limits>
#include <float.h>
#include "yaml-cpp/yaml.h"
#include <algorithm>
#include <vector>
#include <iterator>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>




using namespace cv;
using namespace std;



// void pose_estimation_3d3d (
float pose_estimation_3d3d (
    const vector<Point3f>& pts1,
    const vector<Point3f>& pts2,
    Eigen::Matrix3d & R_ ,
    Eigen::Vector3d & t_

)
{
    Point3f p1, p2;     // center of mass
    int N = pts1.size();
    for ( int i=0; i<N; i++ )
    {
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 = Point3f( Vec3f(p1) /  N);
    p2 = Point3f( Vec3f(p2) / N);
    vector<Point3f>     q1 ( N ), q2 ( N ); // remove the center
    for ( int i=0; i<N; i++ )
    {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for ( int i=0; i<N; i++ )
    {
        W += Eigen::Vector3d ( q1[i].x, q1[i].y, q1[i].z ) * Eigen::Vector3d ( q2[i].x, q2[i].y, q2[i].z ).transpose();
    }
    // cout<<"W="<<W<<endl;

    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd ( W, Eigen::ComputeFullU|Eigen::ComputeFullV );
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    
    if (U.determinant() * V.determinant() < 0)
	{
        for (int x = 0; x < 3; ++x)
        {
            U(x, 2) *= -1;
        }
	}
    



   R_ = U* ( V.transpose() );
   t_ = Eigen::Vector3d ( p1.x, p1.y, p1.z ) - R_ * Eigen::Vector3d ( p2.x, p2.y, p2.z );


   Eigen::Vector3d p3;
   float e = 0;

  
     for ( int i=0; i<N; i++ )
    {
        


        p3 = R_ *Eigen::Vector3d ( pts2[i].x, pts2[i].y, pts2[i].z )+ t_;
        // cout<<p3[0]<<endl;
        Point3f p(p3[0],p3[1],p3[2]);

        e += sqrt((p.x - pts1[i].x)*(p.x - pts1[i].x)+(p.y - pts1[i].y)*(p.y - pts1[i].y)+(p.z - pts1[i].z)*(p.z - pts1[i].z));
    //    cout<<"error "<<e<<endl;

       
    }

    //  cout<<"errorhere "<<e<<endl;
    
    
    

    // cout<<"p3 "<<p3<<"p2"<<p1<<endl;

    return e;
}




// --------------
// -----Main-----
// --------------
int 
main (int argc, char** argv)
{

std::string path = "./this.pcd";
vector<int>::iterator iter;
std::vector<int> tag_id;
vector<Point3f> pts_tag;
std::vector<std::vector<float>> PointsL;
std::vector<float> PointL;

vector<Point3f> pts0;
float m = 0.164; //marker_size in cm
//float m = 0.692; //marker_size in cm

pts0.push_back ( Point3f ( -m/2,-m/2,0 ) ); //  left-bottom    
pts0.push_back ( Point3f ( m/2,-m/2,0) );   //   right-bottom 
pts0.push_back ( Point3f ( m/2,m/2,0 ) );       
pts0.push_back ( Point3f ( -m/2,m/2,0 ) );



//init
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>& point_cloud = *cloud;
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_i(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI> & point_cloud_i = *cloud_i;
//load the pcd file in XYZ and XYZI formats
pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud);
pcl::io::loadPCDFile<pcl::PointXYZI>(path, *cloud_i);


pcl::PointCloud<pcl::PointXYZI>::Ptr unique_i(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI> & unique_cloud_i = *unique_i;

std::vector<std::vector<float>> unique_points_i;
std::vector<float> unique_point_i;

unique_cloud_i.width = cloud->width;
unique_cloud_i.height = 1;
unique_cloud_i.is_dense = false;
unique_cloud_i.points.resize(unique_cloud_i.width * unique_cloud_i.height);

for(size_t p = 0; p < unique_cloud_i.width; ++p){
    
  unique_cloud_i.points[p].x = point_cloud_i.points[p].x*point_cloud_i.points[p].intensity;
		unique_cloud_i.points[p].y = point_cloud_i.points[p].y*point_cloud_i.points[p].intensity;
		unique_cloud_i.points[p].z = point_cloud_i.points[p].z*point_cloud_i.points[p].intensity;
		
//		    unique_cloud_i.points[p].x = point_cloud_i.points[p].intensity;
	//	unique_cloud_i.points[p].y = point_cloud_i.points[p].intensity;
	//	unique_cloud_i.points[p].z = point_cloud_i.points[p].intensity;
    unique_cloud_i.points[p].intensity = 0.001;
  
	}

//generation of the intensity and ranging image

pcl::RangeImage::Ptr range_image_ptr(new pcl::RangeImage);
pcl::RangeImage& range_image = *range_image_ptr;   

pcl::RangeImage::Ptr range_image_i_ptr(new pcl::RangeImage);
pcl::RangeImage& range_image_i = *range_image_i_ptr; 

float angularResolution = (float) (  0.08f * (M_PI/180.0f));  //   1.0 degree in radians
float maxAngleWidth     = (float) (39.0f * (M_PI/180.0f));  // 360.0 degree in radians
float maxAngleHeight    = (float) (39.0f * (M_PI/180.0f));  // 180.0 degree in radians
Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;
float noiseLevel=0.00;
float minRange = 0.0f;
int borderSize = 1;
	

range_image_i.createFromPointCloud(unique_cloud_i, angularResolution, maxAngleWidth, maxAngleHeight,
	                                sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

                                   
   
    

range_image.createFromPointCloud(point_cloud, angularResolution, maxAngleWidth, maxAngleHeight,
	                                sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);


float* ranges = range_image_i.getRangesArray();//it is the intensity array actually
float max = 0;
float val;


  //normalize the intensity, otherwise we cannot transfer it into CV Mat
for(int i = 0; i < range_image_i.width*range_image_i.height; ++i){
    val = *(ranges + i);
    //std::cout << val << std::endl;

    if(val < -FLT_MAX || val > FLT_MAX){
      //std::cout << "is nan or inf" << std::endl;
    }
    else{
      max = (val > max) ? val : max;
    }
  }



    // Create cv::Mat
  Mat image = Mat(range_image_i.height, range_image_i.width, CV_8UC4);
  Mat image_2 = Mat(range_image_i.height, range_image_i.width, CV_8UC4);
  unsigned char r, g, b;

    // pcl::PointCloud to cv::Mat
  #pragma omp parallel for
  for(int y = 0; y < range_image_i.height; y++ ) {
      for(int x = 0; x < range_image_i.width; x++ ) {


            pcl::PointWithRange rangePt = range_image_i.getPoint(x, y);
	 		      float value = rangePt.range / max;

            pcl::visualization::FloatImageUtils::getColorForFloat(value, r, g, b);

            double beta = 0;
            double alpha = 1.0;
            image.at<cv::Vec4b>( y, x )[0] = alpha*b+beta;
            image.at<cv::Vec4b>( y, x )[1] = alpha*g+beta;
            image.at<cv::Vec4b>( y, x )[2] = alpha*r+beta;
            image.at<cv::Vec4b>( y, x )[3] = 0;
        }
    }

    
     for(int y = 0; y < range_image.height; y++ ) {
        for(int x = 0; x < range_image.width; x++ ) {


            pcl::PointWithRange rangePt_2 = range_image.getPoint(x, y);
	 		      float value = rangePt_2.range / max;

            pcl::visualization::FloatImageUtils::getColorForFloat(value, r, g, b);


            image_2.at<cv::Vec4b>( y, x )[0] = b;
            image_2.at<cv::Vec4b>( y, x )[1] = g;
            image_2.at<cv::Vec4b>( y, x )[2] = r;
            image_2.at<cv::Vec4b>( y, x )[3] = 0;//255
        }
    }
  
	

		Mat gray;
     //    imwrite("this1.png",image);
    //cvtColor(frame, gray, COLOR_BGR2GRAY);
    cvtColor(image, gray, COLOR_BGR2GRAY);
    //threshold(gray,gray,20,255,cv::THRESH_BINARY);
    imwrite("this.png",gray);
    //system("python3 detect_aruco.py");
	system("python3 detect_apriltag.py");

    pcl::PointWithRange rangePt_ap;
    pcl::PointWithRange point_up;
    pcl::PointWithRange point_down;
    float ratio;

    ofstream outfile;
    std::string txtpath = "pose.txt";
    outfile.open(txtpath);

    ofstream outfile2;
    std::string txtpath_2 = "points.txt";
    outfile2.open(txtpath_2);

      int test0 = 0;
    vector<Point3f> pts3;
    vector<Point3f> pts4;

    std::ifstream file("this.txt");
    std::string str; 




    while (std::getline(file, str))
    {
        // Process str
        // cout<<str<<endl;

        stringstream ss(str);
        string word;
        int counter = 0;
        int id;
        float v[8];
        float vv[4][2]; 
        while (!ss.eof()) {
        getline(ss, word, ',');
        
        if (counter >0){
        counter = counter -1;
        // v[counter] = float(word);
        v[counter]=stoi(word);
        counter = counter+2;}

        if (counter ==0){
         id = stoi(word);
         counter = counter+1;}



        // cout << "counter"<<counter<<"content"<<word << endl;


        }
    // cout << "this is v"<<v[2] << endl;
    
    vv[0][0] = v[0];
    vv[0][1] = v[1];
    vv[1][0] = v[2];
    vv[1][1] = v[3];
    vv[2][0] = v[4];
    vv[2][1] = v[5];
    vv[3][0] = v[6];
    vv[3][1] = v[7];
    vector<Point3f> pts1;
           for(int j = 0;j<4;j++)
            {//go over the four vertices

             if (!range_image.isObserved (vv[j][0],vv[j][1]))  {//if the vertex is unobserved.
             
                
         
                  for (int m =0; m <10; m++) {
          // check if there exists a pair of neighbor points that is symmetric to each other with respect to the unobserved point 
                        if (range_image.isObserved (vv[j][0],vv[j][1]+m)) //fix the azimuth
                                  {
                                    if (range_image.isObserved (vv[j][0],vv[j][1]-m)){
                                          range_image.calculate3DPoint(vv[j][0],vv[j][1]+m,point_up);
                                          range_image.calculate3DPoint(vv[j][0],vv[j][1]-m, point_down);
                                          PointL = {(point_up.x+point_down.x)/2, (point_up.y+point_down.y)/2, (point_up.z+point_down.z)/2};
                                          PointsL.push_back(PointL);
                                          ratio = point_down.range/point_up.range;
                                          // pts1.push_back ( Point3f ( (point_up.x+point_down.x)/2, (point_up.y+point_down.y)/2, (point_up.z+point_down.z)/2 ) );
                                          pts1.push_back ( Point3f ( point_up.x*(1/(1+ratio))+point_down.x*(ratio/(1+ratio)), 
                                          point_up.y*(1/(1+ratio))+point_down.y*(ratio/(1+ratio)), point_up.z*(1/(1+ratio))+point_down.z*(ratio/(1+ratio))));

                                          break;
                                    }
                                            }


         
           
                                                                                      }
            }


               if (range_image.isObserved (vv[j][0],vv[j][1])) {//if the vertex is observed

                    range_image.calculate3DPoint(vv[j][0], vv[j][1], rangePt_ap);
                    
                    PointL = {rangePt_ap.x,rangePt_ap.y,rangePt_ap.z};               
				            PointsL.push_back(PointL);

                    pts1.push_back ( Point3f ( rangePt_ap.x,rangePt_ap.y,rangePt_ap.z ) );
                  
                                                                                      }
     
            

        }
        
    //           std::cout <<  "Apriltag is detected."<<"\n"<<det->id<<"First vertex:"<<Point(det->p[0][0], det->p[0][1])<<
    //  "\n"<< "Second vertex:"<<Point(det->p[1][0], det->p[1][1])<<"\n"<<"Third vertex:"<<Point(det->p[2][0], det->p[2][1])<<
    //   "\n"<<"Fourth vertex:"<<Point(det->p[3][0], det->p[3][1])<<"\n";
    //    cout<<"3D coordinates of the vertices w.r.t. to the Lidar frame"<<"\n"<<pts1<<"\n"; 

    Mat R(3,3,CV_32FC1);
    Mat t(3,1,CV_32FC1);
    Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>> eigen_R(R.ptr<double>(),R.rows,R.cols);
    // cv::Mat t(3,1,CV_32FC1);

    // std::cout <<"3D coordinates of the vertices w.r.t. to the world frame"<<pts0<<"\n";

    Eigen::Matrix3d R_ ;
    Eigen::Vector3d t_ ;
    float e;
    e = pose_estimation_3d3d ( pts1, pts0, R_, t_);
    if (test0 == 0){
        pts3 = pts1;
        }
    if (test0 ==1){
        pts4 =pts1;
        test0 = 2;}

    test0 = 1;

    outfile<<id<<","<<R_( 0,0 )<<","<<R_( 0,1 )<<","<<R_( 0,2 )<<","<<R_( 1,0 )<<","<<R_( 1,1 )<<","<<R_( 1,2 )<<","<<R_( 2,0 )<<","
    <<R_( 2,1 )<<","<<R_( 2,2 )<<","<<
    t_ ( 0,0 )<<","<< t_ ( 1,0 )<<","<< t_ ( 2,0 )<<","<<e<<endl;

    for ( int i=0; i<int(pts1.size()); i++ )
    {
        outfile2<<id<<","<<i<<","<<pts1[i].x<<","<<pts1[i].y<<","<<pts1[i].z<<endl;
        // outfile2<<det->id<<","<<pts1[i].x<<endl;
    }
    


   
      } 

    
   
       

      
      outfile2.close();   

      outfile.close();  




      return 0;


    }

