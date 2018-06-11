/* 
 * $Id: raytrix.cpp 8800 07-15-2014 ashadema $
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Shiekh Zayed Institute for Pediatric Surgical
 *  Innovation. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <tf_conversions/tf_eigen.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/fill_image.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <stdio.h>
#include <stdlib.h>
#include <netdb.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <iostream>
#include <fstream>
#include <valarray> 
#include <Eigen/Dense>

using namespace std;

/**
  \author Azad Shademan, Ryan Decker, Simon Leonard 

  @b raytrix is a ros node that listens to the raw data on the socket from the raytrix machine.
  The raw data includes the virtual depth in raytrix frame (z axis pointing towards the sensor) and
  raw (unrectified) focused images. The launch file engages the image_proc, which looks for intrinsic
  calibration (found independently through cameracalibrator.py) and publishes the rectified image_rect
  topic. The metric point cloud is published in the /raytrix/pcl topic. 

 **/

int main( int argc, char** argv ){

  ros::init( argc, argv, "raytrix" );

  int sockfd, portno;
  struct sockaddr_in serv_addr;
  struct hostent *server;

  if( (argc != 3) && (argc != 4) ){
    std::cerr << "usage: " << argv[0] << " hostname port [scale file]"
      << std::endl
      << "Try 192.168.0.43 for hostname and 10000 for port /AS"
      << std::endl;
    return -1;
  }

  portno = atoi(argv[2]);

  // Configure the network connection
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if( sockfd < 0 ){
    perror("ERROR opening socket");
    return -1;
  }

  server = gethostbyname(argv[1]);
  if( server == NULL ) {
    std::cerr << "ERROR, no such host" << std::endl;
    return -1;
  }

  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  bcopy((char *)server->h_addr,
      (char *)&serv_addr.sin_addr.s_addr,
      server->h_length);
  serv_addr.sin_port = htons(portno);

  if( connect( sockfd, (sockaddr*)&serv_addr, sizeof( serv_addr ) ) < 0 ){
    perror("ERROR connecting");
    return -1;
  }


  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);
  camera_info_manager::CameraInfoManager camera_info_manager(nh);
  camera_info_manager.setCameraName( "raytrix" );

  // Configure the ROS publishers
  image_transport::CameraPublisher pub_image;
  const std::string image_topic( "/raytrix/image_raw" );
  pub_image = it.advertiseCamera( image_topic, 1 );

  ros::Publisher pub_depth;
  const std::string depth_topic( "/raytrix/depth" );
  pub_depth = nh.advertise<sensor_msgs::Image>( depth_topic, 1 );

  ros::Publisher pub_pcl;
  const std::string pcl_topic( "/raytrix/pcl" );
  pub_pcl = nh.advertise< pcl::PointCloud<pcl::PointXYZI> >( pcl_topic, 1 );

  // Read the scale file
  double scalex=1, scaley=1, scalez=1;

  if( argc == 4 ){
    std::ifstream ifs;
    ifs.open( argv[3] );
    ifs >> scalex >> scaley >> scalez;
    ifs.close();
  }

  tf::TransformListener listener;

  // main while loop
  unsigned int seq = 0;
  while( nh.ok() ){

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.seq = seq++;
    header.frame_id = std::string( "/raytrix" );

    // receive the intensity image
    int imgwidth, imgheight;
    recv( sockfd, &imgwidth, sizeof(imgwidth), MSG_WAITALL );
    recv( sockfd, &imgheight, sizeof(imgheight), MSG_WAITALL );

    unsigned char* img = new unsigned char[ imgwidth*imgheight ];
    recv( sockfd, img, sizeof(unsigned char)*imgwidth*imgheight, MSG_WAITALL );

    sensor_msgs::CameraInfoPtr cinfo;
    cinfo.reset(new sensor_msgs::CameraInfo(camera_info_manager.getCameraInfo()));
    cinfo->header = header;

    // create a ROS image and publish
    sensor_msgs::ImagePtr msgimg( new sensor_msgs::Image() );
    msgimg->header = header;
    sensor_msgs::fillImage( *msgimg,
        sensor_msgs::image_encodings::MONO8,
        imgheight,
        imgwidth,
        imgwidth*sizeof(unsigned char),
        img );
    pub_image.publish( msgimg, cinfo );

    // receive the depth map
    int dptwidth, dptheight;
    recv( sockfd, &dptwidth, sizeof(dptwidth), MSG_WAITALL );
    recv( sockfd, &dptheight, sizeof(dptheight), MSG_WAITALL );

    float* dpt = new float[ dptwidth*dptheight ] ;
    recv( sockfd, dpt, sizeof(float)*dptwidth*dptheight, MSG_WAITALL );

    // create a ROS depth image and publish
    sensor_msgs::Image msgdpt;
    msgdpt.header = header;
    sensor_msgs::fillImage( msgdpt,
        sensor_msgs::image_encodings::TYPE_32FC1,
        dptheight,
        dptwidth,
        dptwidth*sizeof(float),
        dpt );
    pub_depth.publish( msgdpt );

    // create and publish a point cloud for all points obtained from RxLive
    pcl::PointCloud<pcl::PointXYZI>::Ptr
      pcl_msg( new pcl::PointCloud<pcl::PointXYZI>() );

    // pcl data structure initialization
    pcl_msg->header = header; //pcl_conversions::toPCL( header );
    pcl_msg->height = dptheight;
    pcl_msg->width = dptwidth;
    //pcl_msg->is_dense = false;

    pcl_msg->points.resize( pcl_msg->height * pcl_msg->width );
    pcl::PointCloud<pcl::PointXYZI>::iterator pt_iter = pcl_msg->begin();

    // convert
    float f_L = 25.6736073634001; //focal length in mm
    float T_L = 913.563818145304; // focus distance in mm
    float B = 0.94165; // Internal camera offset - distance between micro lens array and image plane

    int v_TCP = 2; //virtual depth of TCP
    // float s_P = 2 * 0.0047; //pixel size in mm, times two because lightfield cameras have reduced resolution
    float s_P = 2 * 0.0047; //pixel size in mm, times two because lightfield cameras have reduced resolution
    float B_L = T_L/2 * (1-sqrt(1-(4*f_L)/(T_L))); //calculate distance from TCP to main lens center
    //loop through all pixels, after defining some things

    float xMetric;
    float yMetric;
    float zMetric;
    using namespace Eigen;
    using namespace std;
    VectorXd acLHS; //for distortion correction
    VectorXd acRHS;
    MatrixXd a(6,1);
    a(0,0) = -379.54; //from matlab note a(0,0) is the z distance, you should change it
    a(1,0) = -0.1128;
    a(2,0) = 0.0036;
    a(3,0) = 0.0;
    a(4,0) = 0.0235;
    a(5,0) = 0.0026;

    for( int dptu=0; dptu<(int)pcl_msg->width; ++dptu ){
      for( int dptv=0; dptv<(int)pcl_msg->height; ++dptv, ++pt_iter){
        // Fill in XYZ
        double x = dptu;
        double y = dptv;
        double z = dpt[dptv*dptwidth+dptu];
        //transform from virtual depth to lens csys
        if (z!=0) {
          z = 1/(1-z); // transform depth values to space II: virtual depth units, lateral coordinates in pixels
          z = (z - v_TCP)*B; //to space III': metric depth units from TCP

          float b_L = z + B_L; //to space III: metric depth units from lens center
          //project through main lens: depth
          float a_L = (b_L*f_L)/(b_L-f_L); // to space V': metric depth values from lens center

          //project through main lens: lateral coordinates
          //convert to metric coordinates on sensor with pixel size
          xMetric = (x-(int)pcl_msg->width / 2) * s_P;
          yMetric = (y-(int)pcl_msg->height / 2) * s_P;
          //project with pinhole model
          xMetric = xMetric * a_L/b_L;
          yMetric = yMetric * a_L/b_L;
          zMetric = -(a_L +B_L) ; //flip z here? wrong directions!!
        }
        else
        {
          xMetric = 0.0;
          yMetric = 0.0;
          zMetric = 0.0;
        }

        //pcl::PointXYZI& pt = *pt_iter; //put in point cloud
        pcl::PointXYZI& pt = pcl_msg->at( dptu, dptv );

        pt.x = (-xMetric)/1000; //flip coordinates, put to meters
        pt.y = (-yMetric)/1000;
        pt.z = zMetric; //correct z later

        //previous stuff for intensity of image
        int imgu = dptu * ( (float)(imgwidth) ) / ( (float)(dptwidth) );
        int imgv = dptv * ( (float)(imgheight) ) / ( (float)(dptheight) );
        pt.intensity = img[imgv*imgwidth+imgu];

        // do distortion correction 
        MatrixXd m(6,2) ;//assign
        m(0,0) = 0; m(0,1) = 0;
        m(1,0) = 1; m(1,1) = 0;
        m(2,0) = 2; m(2,1) = 0;
        m(3,0) = 1; m(3,1) = 1;
        m(4,0) = 0; m(4,1) = 1;
        m(5,0) = 0; m(5,1) = 2;

        a(0,0) = zMetric; //really should be height of ideal plane
        double fit = a(0,0) + a(1,0)*pt.x + a(2,0)*pow(pt.x,2) + a(3,0)*pt.x*pt.y + a(4,0)*pt.y + a(5,0)*pow(pt.y,2); //a known from previous plane-fitting experiments
        double error = zMetric - fit;
        double undistortedZ = zMetric + error;
        pt.z = undistortedZ;

        //now correct for the nonlinear Z scaling to give expanded depth FOV
        //double PCLdiff = 300 - pt.z; //403 is our 0, our 'sweet spot'
        //double Zerror = 1.666*PCLdiff; //error to the CamPose estimated CSYS in Z //was 0.89
        //pt.z = (pt.z - Zerror); //subtract the predicted error to get good Z value

        pt.z = (-pt.z + 680); //this is to get on the right side, with right motion
        pt.z = (pt.z)/1000; // convert to m

        bool CALIBRATED = false; // change to true after rx calibration
        bool NIR = 0;
        bool NIR_LED = 0;
        bool rotate_around_anchor = 1;

        double sx, // scale x 
               sy, // scale y
               sz, // scale z
               vz, // vanishing point
               mx, //slope of fitted lines
               my, 
               cx, // y intercept of fitted lines
               cy,
               z_offset; 


        // translation found from chessboard evaluation
        double dx, dy, dz;

        double AnchorPtX, AnchorPtY, AnchorPtZ; // anchor point for rotation of cloud
        double alpha, beta; // rotation angles around anchor point

        // rotate raw, then calibrate pyramid
        if (!CALIBRATED) {
          if (rotate_around_anchor) {

            //center before z scale correction
            AnchorPtX = 0.00132025382482;
            AnchorPtY = -0.0000318735837936;
            AnchorPtZ = 0.294679701328;

            // rotate y 4 degrees
            alpha =  0.997564050259824; //5 deg:0.996565502497761; 
            beta =   0.0697564737441253;// 5 deg:0.0871557427476582;
            pt.x = (pt.x-AnchorPtX)*alpha   + (pt.y-AnchorPtY)*0.0 + (pt.z-AnchorPtZ)*(+beta);
            pt.y = (pt.x-AnchorPtX)*0.0     + (pt.y-AnchorPtY)*1.0 + (pt.z-AnchorPtZ)*0.0;
            pt.z = (pt.x-AnchorPtX)*(-beta) + (pt.y-AnchorPtY)*0.0 + (pt.z-AnchorPtZ)*alpha;
          }
          else {
            AnchorPtX = 0.0;
            AnchorPtY = 0.0;
            AnchorPtZ = 0.0;
          }
          pt.x = pt.x + AnchorPtX;// - .010;
          pt.y = pt.y + AnchorPtY;// + .0075;
          pt.z = pt.z + AnchorPtZ;
        }
        /////////////////////////////////////////////////////
        // after calibration
        mx = 4.55835764969036;
        my = 4.48606916689838;
        cx = -0.0239162014513492;
        cy = -0.0131908319340899;
        sz = 2.29493383731201;
        vz = 0.911462803607419;

        ///////////////////////////
        mx = 5.041202;
        my = 4.980367;
        cx = -0.020312;
        cy = -0.013376;
        sz = 2.067463;
        vz = 0.821120;

        ///////////////////////////////////////////
        pt.z = sz*pt.z;
        sx = 1/ ( mx*(vz - pt.z) + cx); // x scale
        sy = 1/ ( my*(vz - pt.z) + cy); // y scale

        pt.x = sx*pt.x ;
        pt.y = sy*pt.y ;

        ///////////////////////////////////////////
        if (1) {
          dx = 0.008653;// + (0.0005) ;
          dy = -0.005524;// + (-0.0003) ;
          dz = -0.319190;// + (0.003);
        }
        else
        {
          dx = 0;
          dy = 0;
          dz = 0;
        }



        pt.x = pt.x + dx ;
        pt.y = pt.y + dy ;
        pt.z = pt.z + dz ;


        //rsd - crop the point cloud in z 6.12.15
        bool cropPCL = false;//true;
        double zThresh = 0.3;
        if(cropPCL){
          if(pt.z > zThresh){
            pt.z = 0;
          }
        }

        if (NIR_LED) // NIR ring light and Cold light ring light
        {
          sz = 2.81691290030597;
          sz = 2.68656556992808;
          sz = 2.48386782062331;
          pt.z = sz*pt.z;

          mx = 3.128231772253;
          cx = 0.0156108462123228;
          my = 3.11743968598486;
          cy = 0.0144976960730944;
          vz = 1.15833073483936;

          // 22-26cm
          mx = 3.91120090860203;
          my = 3.99223784809993;
          cx = 0.026714225007006;
          cy = -0.0116350959329146;
          vz = 1.0529650868746;

          // 23-27
          mx = 3.82114462877084;
          my = 3.97053508065774;
          cx = 0.0212253552538907;
          cy = -0.0293473297491343;
          vz = 1.0018796214851;

          sx = 1/ ( mx*(vz - pt.z) + cx); // x scale
          sy = 1/ ( my*(vz - pt.z) + cy); // y scale

          pt.x = sx*pt.x ;
          pt.y = sy*pt.y ;

          z_offset = 0.294; 
          z_offset = 0.0;

          pt.z = pt.z - z_offset ; // translate back - found experimentally keep 0.294, tune dz


          if (rotate_around_anchor) {

            //center
            AnchorPtX = -0.000700328268185031; 
            AnchorPtY =  0.000522500929330138;
            AnchorPtZ =   0.727096047681334;

            // rotate y +2.5 degrees
            // 0.99144486137381                         0         0.130526192220052
            alpha = 0.99144486137381; // 0.996565502497761; //0.997858923238603  ;//0.999048221581858; //0.994521895368273; 
            beta =  0.130526192220052; // 0.0828082075122043;//0.0654031292301431; //0.043619387365336; //0.104528463267653;
            pt.x = (pt.x-AnchorPtX)*alpha   + (pt.y-AnchorPtY)*0.0 + (pt.z-AnchorPtZ)*(+beta);
            pt.y = (pt.x-AnchorPtX)*0.0     + (pt.y-AnchorPtY)*1.0 + (pt.z-AnchorPtZ)*0.0;
            pt.z = (pt.x-AnchorPtX)*(-beta) + (pt.y-AnchorPtY)*0.0 + (pt.z-AnchorPtZ)*alpha;

          }
          else {
            AnchorPtX = 0.0;
            AnchorPtY = 0.0;
            AnchorPtZ = 0.0;
          }

          // adjust for origin alignment -- rsd - adjusted for 2 arms after dubai
          dx =  0.008569937;      //0.0069444259;//-0.017615050 - (-0.0372729189694);
          dy =  -0.006125441;     //-0.00612910115;//-0.029974736 - (-0.0237121284008);
          dz = -0.4401276175;     //-0.436090138; //0.274605658 - (0.472421348095); // -0.247375590278 + 0.00371498657142855;

          pt.x = (pt.x+AnchorPtX) + dx ;
          pt.y = (pt.y+AnchorPtY) + dy ;
          pt.z = (pt.z+AnchorPtZ) + dz ;

        }

        if (NIR) //NIR calibration with ring light
        {
          sz = 2.12602613575506;
          pt.z = sz*pt.z;

          mx = 3.89477699103999;
          my = 3.93197244238446;
          cx = 0.0354391010221112;
          cy = 0.020471647114379;
          vz = 0.880701777716373;

          sx = 1/ ( mx*(vz - pt.z) + cx); // x scale
          sy = 1/ ( my*(vz - pt.z) + cy); // y scale

          dx = 0.0;
          dy = 0.0;
          dz = 0.0;

          dx = 0.00844606575 + 0.0005;//for NIR + COLD
          dy = -0.00598593165;// + 0.001;//for NIR + COLD
          dz = -0.0346045928000001-0.0064; //for NIR + COLD

          pt.x = sx*pt.x + dx;
          pt.y = sy*pt.y + dy;
          pt.z = pt.z - 0.294 + dz; // translate back - found experimentally keep 0.294, tune dz

          if (rotate_around_anchor) {

            //center
            AnchorPtX = 0.0009583;
            AnchorPtY = -0.0061;
            AnchorPtZ = 0.301275;

            // rotate y +2.5 degrees
            // 0.99144486137381                         0         0.130526192220052
            alpha = 0.99144486137381; // 0.996565502497761; //0.997858923238603  ;//0.999048221581858; //0.994521895368273
            beta =  0.130526192220052; // 0.0828082075122043;//0.0654031292301431; //0.043619387365336; //0.10452846326765
            pt.x = (pt.x-AnchorPtX)*alpha   + (pt.y-AnchorPtY)*0.0 + (pt.z-AnchorPtZ)*(+beta);
            pt.y = (pt.x-AnchorPtX)*0.0     + (pt.y-AnchorPtY)*1.0 + (pt.z-AnchorPtZ)*0.0;
            pt.z = (pt.x-AnchorPtX)*(-beta) + (pt.y-AnchorPtY)*0.0 + (pt.z-AnchorPtZ)*alpha;

          }
          else {
            AnchorPtX = 0.0;
            AnchorPtY = 0.0;
            AnchorPtZ = 0.0;
          }

          // adjust for origin alignment
          dx = 0.0;
          dy = 0.0;//-0.00612910115;//-0.029974736 - (-0.0237121284008);
          dz = 0.0;//-0.436090138; //0.274605658 - (0.472421348095); // -0.247375590278 + 0.00371498657142855;

          pt.x = (pt.x+AnchorPtX) + dx ;
          pt.y = (pt.y+AnchorPtY) + dy ;
          pt.z = (pt.z+AnchorPtZ) + dz ;



        }

        //! Last tested calibration for the Halogen light
        //  ---------------------------------------------
        if (CALIBRATED) {
          bool tested = true; // to keep track of last tested calibration params

          //new
          sz = 2.70273420315005;
          if (tested) { sz = 2.09269325903253;}
          pt.z = sz*pt.z;

          //new
          vz = 1.0828561118703; // vanishing point
          mx = 3.69155578707445; //slope of fitted lines
          my =  3.46606756703239; 
          cx = -0.0166923375482605; // y intercept
          cy = 0.0714287238314738; 

          if (tested) {
            vz = 0.861671150311261; // vanishing point
            mx = 4.29516304990163; //slope of fitted lines
            my = 4.32005954552447;
            cx = -0.00946500453024501; // y intercept
            cy = -0.0101878637059607;
          }

          sx = 1/ ( mx*(vz - pt.z) + cx); // x scale
          sy = 1/ ( my*(vz - pt.z) + cy); // y scale

          // new
          dx = 0.0082421211;
          dy= -0.0060866720;
          dz=-0.2087666900;

          dx = -0.001;
          dy = 0.0008;
          dz = -0.03;

          if (tested) {
            dx = 0.0086405749;
            dy= -0.0058957480;
            dz= -0.0340475444;
          }
          pt.x = sx*pt.x + dx;
          pt.y = sy*pt.y + dy;
          pt.z = pt.z - 0.294 + dz; // translate back - found experimentally keep 0.294, tune dz
        }
      }
    }

    pub_pcl.publish( pcl_msg);

    delete[] img;
    delete[] dpt;

    tf::StampedTransform tfRt0r;
    try{
      listener.lookupTransform( "LWR_0", "raytrix", ros::Time(0), tfRt0r );
    }
    catch(tf::TransformException ex ){}

    /*
    // code sample to transform cloud to a different frame 
    try{
    tf::StampedTransform tfRt0r;
    listener.lookupTransform( "LWR_0", "raytrix", ros::Time(0), tfRt0r );
    //listener.lookupTransform( "raytrix", "LWR_0", ros::Time(0), tfRt0r );

    Eigen::Affine3d eigdRt0r;
    tf::TransformTFToEigen( tfRt0r, eigdRt0r );

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_rtmsg( new pcl::PointCloud<pcl::PointXYZI>() );
    Eigen::Affine3f eigfRt0r( eigdRt0r );
    pcl::transformPointCloud( *pcl_msg, *pcl_rtmsg, eigfRt0r );
    pcl_rtmsg->header.frame_id = std::string( "LWR_0" );

    //pub_pcl.publish( pcl_rtmsg );

    } catch(tf::TransformException ex ){
    pub_pcl.publish( pcl_msg );
    } 
     */
  }
  return 0;
}
