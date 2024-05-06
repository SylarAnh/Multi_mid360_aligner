// The key objective of this code is to align the spinning lidar
// and livox code both on position and time side.
// and publish to a new topic, which the timestamp
// are same and under the same coordinate as livox_frame;
//  Qingqing Li @uTU

#include <time.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include "livox_ros_driver/CustomMsg.h"
#include "lidars_extrinsic_cali.h"

#include <fstream>
#include <iostream>
#include <chrono>
#include <string>
#include <Eigen/Dense>
#include <mutex>
#include <tf_conversions/tf_eigen.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/registration/gicp.h>


#define PI 3.1415926
// #define BACKWARD_HAS_DW 1
// #include "backward.hpp"
// namespace backward {
// backward::SignalHandling sh;
// }
typedef uint64_t uint64;
typedef  pcl::PointXYZI   PointType;

bool finished_flag = false;

class LidarsParamEstimator{
private:
  ros::NodeHandle nh;
  // subscribe raw data
  ros::Subscriber sub_mid1;
  ros::Subscriber sub_mid2;
  ros::Subscriber sub_mid3;

  // Hori TF
  int                         _hori_itegrate_frames = 10;
  int                         _hori_itegrate_frames2 = 10;
  int                         _hori_itegrate_frames3 = 10;
  pcl::PointCloud<PointType>  _hori_igcloud;
  pcl::PointCloud<PointType>  _hori_igcloud2;
  pcl::PointCloud<PointType>  _hori_igcloud3;
  Eigen::Matrix4f             _velo_hori_tf_matrix;
  Eigen::Matrix4f             _mid1_ouster_tf_init;
  Eigen::Matrix4f             _mid2_ouster_tf_init;
  Eigen::Matrix4f             _mid3_ouster_tf_init;
  Eigen::Matrix4f             _mid2_mid1_tf_init;
  Eigen::Matrix4f             _mid3_mid1_tf_init;
  bool                        _hori_tf_initd         = false;
  bool                        _first_velo_reveived= false;
  int                         _cut_raw_message_pieces = 2;
  std::string                 _mid_lidar = "/livox/lidar_192_168_1_180";
  std::string                 _rear_lidar = "/livox/lidar_192_168_1_122";
  std::string                 _front_lidar = "/livox/lidar_192_168_1_102";

  // real time angular yaw speed
  double                      _yaw_velocity;

  pcl::PointCloud<PointType> _velo_new_cloud;
  pcl::PointCloud<PointType> _ouster_new_cloud;

  // raw message queue for time_offset
  std::queue< livox_ros_driver::CustomMsg >       _hori_queue;
  std::queue< livox_ros_driver::CustomMsg >       _hori_queue2;
  std::vector<float>                              _hori_msg_yaw_vec;
  std::mutex _mutexHoriQueue;

  uint64                                      _hori_start_stamp ;
  uint64                                      _hori_start_stamp2 ;
  uint64                                      _hori_start_stamp3 ;
  bool                                        _first_hori = true;
  bool                                        _first_hori2 = true;
  bool                                        _first_hori3 = true;


  double      _time_esti_error_th   = 400.0;
  uint64      _time_offset          = 0; //secs (velo_stamp - hori_stamp)


  // Parameter
  bool    en_timeoffset_esti         = true;   // enable time_offset estimation
  bool    en_extrinsic_esti          = true;   // enable extrinsic estimation
  bool    _use_given_extrinsic_lidars  = false;  // using given extrinsic parameter
  bool    _use_given_timeoffset       = true;   // using given timeoffset estimation
  float   _time_start_yaw_velocity   = 0.5;    // the angular speed when time offset estimation triggered
  float   _mid_y_axis   = 0.0;    //
  float   _mid_z_axis   = 0.0;    //
  float   _rear_y_axis   = 0.0;    //
  float   _rear_z_axis   = 0.0;    //
  float   _front_y_axis   = 0.0;    //
  float   _front_z_axis   = 0.0;    //
  int     _offset_search_resolution    = 30;     // points
  int     _offset_search_sliced_points = 12000;     // points
  float   given_timeoffset          = 0;      // the given time-offset value

  // Distortion
  Eigen::Quaterniond _delta_q;

public:
  bool _hori_tf_opted = false;
  LidarsParamEstimator()
  {
    // Get parameters
    ros::NodeHandle private_nh_("~");
    if (!private_nh_.getParam("enable_extrinsic_estimation",  en_extrinsic_esti))       en_extrinsic_esti = true;
    if (!private_nh_.getParam("enable_timeoffset_estimation", en_timeoffset_esti))      en_timeoffset_esti = false;
    if (!private_nh_.getParam("extri_esti_hori_integ_frames", _hori_itegrate_frames))    _hori_itegrate_frames = 10;
    if (!private_nh_.getParam("give_extrinsic_Velo_to_Hori",  _use_given_extrinsic_lidars))   _use_given_extrinsic_lidars = false;
    if (!private_nh_.getParam("give_timeoffset_Velo_to_Hori", _use_given_timeoffset))        _use_given_timeoffset = false;
    if (!private_nh_.getParam("time_esti_error_threshold",    _time_esti_error_th))     _time_esti_error_th = 35000.0;
    if (!private_nh_.getParam("time_esti_start_yaw_velocity", _time_start_yaw_velocity))     _time_start_yaw_velocity = 0.6;
    if (!private_nh_.getParam("timeoffset_Velo_to_Hori",      given_timeoffset))            given_timeoffset = 0.070;
    if (!private_nh_.getParam("timeoffset_search_resolution", _offset_search_resolution))    _offset_search_resolution = 10;
    if (!private_nh_.getParam("timeoffset_search_sliced_points", _offset_search_sliced_points)) _offset_search_sliced_points = 24000;
    if (!private_nh_.getParam("cut_raw_Hori_message_pieces", _cut_raw_message_pieces)) _cut_raw_message_pieces = 1;
    if (!private_nh_.getParam("front_y_axis", _front_y_axis))     _front_y_axis = 0.44444;
    if (!private_nh_.getParam("front_z_axis", _front_z_axis))     _front_z_axis = 0.75;
    if (!private_nh_.getParam("rear_y_axis", _rear_y_axis))     _rear_y_axis = -0.33333;
    if (!private_nh_.getParam("rear_z_axis", _rear_z_axis))     _rear_z_axis = 0.25;
    if (!private_nh_.getParam("mid_y_axis", _mid_y_axis))     _mid_y_axis = 0.0;
    if (!private_nh_.getParam("mid_z_axis", _mid_z_axis))     _mid_z_axis = 0.25;
    if (!private_nh_.getParam("mid_lidar", _mid_lidar)) _mid_lidar = "/livox/lidar_192_168_1_180";
    if (!private_nh_.getParam("rear_lidar", _rear_lidar)) _rear_lidar = "/livox/lidar_192_168_1_122";
    if (!private_nh_.getParam("front_lidar", _front_lidar)) _front_lidar = "/livox/lidar_192_168_1_102";

    ROS_INFO_STREAM( "front_y_axis                       : " <<  _front_y_axis );
    ROS_INFO_STREAM( "front_z_axis                       : " <<  _front_z_axis );
    ROS_INFO_STREAM( "rear_y_axis                       : " <<  _rear_y_axis );
    ROS_INFO_STREAM( "rear_z_axis                       : " <<  _rear_z_axis );
    ROS_INFO_STREAM( "mid_y_axis                       : " <<  _mid_y_axis );
    ROS_INFO_STREAM( "mid_z_axis                       : " <<  _mid_z_axis );
    ROS_INFO_STREAM( "mid_lidar                       : " <<  _mid_lidar );
    ROS_INFO_STREAM( "rear_lidar                      : " <<  _rear_lidar );
    ROS_INFO_STREAM( "front_lidar                     : " <<  _front_lidar );

    //mid1是前面的激光雷达，mid2是后面的激光雷达，mid3是中间的激光雷达
    sub_mid1 = nh.subscribe<livox_ros_driver::CustomMsg>(_front_lidar, 100, &LidarsParamEstimator::hori_cloud_handler, this);//前
    sub_mid2 = nh.subscribe<livox_ros_driver::CustomMsg>(_rear_lidar, 100, &LidarsParamEstimator::hori_cloud_handler2, this);//后
    sub_mid3 = nh.subscribe<livox_ros_driver::CustomMsg>(_mid_lidar, 100, &LidarsParamEstimator::hori_cloud_handler3, this);//中

    std::vector<double> vecVeloHoriExtri;
    if ( _use_given_extrinsic_lidars && private_nh_.getParam("Extrinsic_Velohori", vecVeloHoriExtri )){
      _velo_hori_tf_matrix <<    vecVeloHoriExtri[0], vecVeloHoriExtri[1], vecVeloHoriExtri[2], vecVeloHoriExtri[3],
          vecVeloHoriExtri[4], vecVeloHoriExtri[5], vecVeloHoriExtri[6], vecVeloHoriExtri[7],
          vecVeloHoriExtri[8], vecVeloHoriExtri[9], vecVeloHoriExtri[10], vecVeloHoriExtri[11],
          vecVeloHoriExtri[12], vecVeloHoriExtri[13], vecVeloHoriExtri[14], vecVeloHoriExtri[15];
      _hori_tf_initd = true;
      ROS_INFO_STREAM("Reveived transformation_matrix Velo-> Hori: \n" << _velo_hori_tf_matrix );
    }

    std::vector<double> vecL12OExtri;
    std::vector<double> vecL22OExtri;
    std::vector<double> vecL32OExtri;
    if(!ros::param::get("mm_PoseEstimation/Extrinsic_TM12O",vecL12OExtri )){
      vecL12OExtri = {0.460857, 0.455947, 0.760784, 0.223312,
                      -0.703766, 0.71022, 0.000669762, 0.00225066,
                      -0.540402, -0.536106, 0.648971, 0.120166,
                      0, 0, 0, 1}; // 前mid360对ouster

      ROS_WARN_STREAM("Extrinsic_Tlb unavailable ! Using default param");
    }
    if(!ros::param::get("mm_PoseEstimation/Extrinsic_TM22O",vecL22OExtri )){

      vecL22OExtri = {0.351089, 0.344676, -0.870606, -0.35431,
                      -0.710375, 0.703766, -0.00784904, -0.00602132,
                      0.610049, 0.621264, 0.491873, 0.269782,
                      0, 0, 0, 1}; // 后mid360对ouster

      ROS_WARN_STREAM("Extrinsic_Tlb unavailable ! Using default param");
    }
    if(!ros::param::get("mm_PoseEstimation/Extrinsic_TM32O",vecL32OExtri )){

      vecL32OExtri = {-0.707, 0.707,  0.0,   -0.0,
                      -0.707, -0.707,  0.0,   0.0,
                      0.0, 0.0,  1.0,   0.347,
                      0.0, 0.0,  0.0,   1.0}; // 后mid360对ouster

      ROS_WARN_STREAM("Extrinsic_Tlb unavailable ! Using default param");
    }
    Eigen::Matrix3f rotate_axia;
    Eigen::Matrix3f rotate_init = Eigen::Matrix3f::Identity();
    _mid1_ouster_tf_init = Eigen::Matrix4f::Identity();
    _mid2_ouster_tf_init = Eigen::Matrix4f::Identity();
    _mid3_ouster_tf_init = Eigen::Matrix4f::Identity();

    // mid360-1(front)
    rotate_axia = Eigen::AngleAxisf(PI * _front_y_axis, Eigen::Vector3f::UnitY()).toRotationMatrix();
    rotate_init = rotate_axia;
    rotate_axia = Eigen::AngleAxisf(PI * _front_z_axis, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    rotate_init = rotate_init * rotate_axia;
    _mid1_ouster_tf_init.block(0,0,3,3) = rotate_init;
    _mid1_ouster_tf_init.block(0,3,3,1) << 0.223312, 0.00225066, 0.120166;
    // mid360-2(rear)
    rotate_axia = Eigen::AngleAxisf(PI * _rear_y_axis, Eigen::Vector3f::UnitY()).toRotationMatrix();
    rotate_init = rotate_axia;
    rotate_axia = Eigen::AngleAxisf(PI * _rear_z_axis, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    rotate_init = rotate_init * rotate_axia;
    _mid2_ouster_tf_init.block(0,0,3,3) = rotate_init;
    _mid2_ouster_tf_init.block(0,3,3,1) << -0.35431, -0.00602132, 0.269782;
    // mid360-3(mid)
    rotate_axia = Eigen::AngleAxisf(PI * _mid_y_axis, Eigen::Vector3f::UnitY()).toRotationMatrix();
    rotate_init = rotate_axia;
    rotate_axia = Eigen::AngleAxisf(PI * _mid_z_axis, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    rotate_init = rotate_init * rotate_axia;
    _mid3_ouster_tf_init.block(0,0,3,3) = rotate_init;
    _mid3_ouster_tf_init.block(0,3,3,1) << 0.0, 0.0, 0.500;

    // _mid1_ouster_tf_init <<    vecL12OExtri[0], vecL12OExtri[1], vecL12OExtri[2], vecL12OExtri[3],
    //         vecL12OExtri[4], vecL12OExtri[5], vecL12OExtri[6], vecL12OExtri[7],
    //         vecL12OExtri[8], vecL12OExtri[9], vecL12OExtri[10], vecL12OExtri[11],
    //         vecL12OExtri[12], vecL12OExtri[13], vecL12OExtri[14], vecL12OExtri[15];
    // _mid2_ouster_tf_init <<    vecL22OExtri[0], vecL22OExtri[1], vecL22OExtri[2], vecL22OExtri[3],
    //         vecL22OExtri[4], vecL22OExtri[5], vecL22OExtri[6], vecL22OExtri[7],
    //         vecL22OExtri[8], vecL22OExtri[9], vecL22OExtri[10], vecL22OExtri[11],
    //         vecL22OExtri[12], vecL22OExtri[13], vecL22OExtri[14], vecL22OExtri[15];
    // _mid3_ouster_tf_init <<    vecL32OExtri[0], vecL32OExtri[1], vecL32OExtri[2], vecL32OExtri[3],
    //         vecL32OExtri[4], vecL32OExtri[5], vecL32OExtri[6], vecL32OExtri[7],
    //         vecL32OExtri[8], vecL32OExtri[9], vecL32OExtri[10], vecL32OExtri[11],
    //         vecL32OExtri[12], vecL32OExtri[13], vecL32OExtri[14], vecL32OExtri[15];

    ROS_WARN_STREAM("Reveived transformation_matrix mid1 -> base: \n" << _mid1_ouster_tf_init );
    ROS_WARN_STREAM("Reveived transformation_matrix mid2 -> base: \n" << _mid2_ouster_tf_init );
    ROS_WARN_STREAM("Reveived transformation_matrix mid3 -> base: \n" << _mid3_ouster_tf_init );
    _mid2_mid1_tf_init = _mid1_ouster_tf_init.inverse() * _mid2_ouster_tf_init;
    ROS_WARN_STREAM("Reveived transformation_matrix mid2 -> mid1: \n" << _mid2_mid1_tf_init );
    if(_use_given_timeoffset)
    {
      ROS_INFO_STREAM("Given time offset " << given_timeoffset);
      ros::Time tmp_stamp;
      _time_offset = tmp_stamp.fromSec(given_timeoffset).toNSec();
      _time_offset_initd = true;
    }
  };

  ~LidarsParamEstimator(){};

  /**
   * @brief subscribe raw pointcloud message from Livox lidar and process the data.
   * - save the first timestamp of first message to init the timestamp
   * - Undistort pointcloud based on rotation from IMU
   * - If TF is not initlized,  Push the current undistorted message and yaw to queue;
   * - If TF is not intilized,  align two pointclouds with ICP after integrating enough frames
   * - If TF has been initized, publish aligned cloud in Horizon frame-id
   */
  void hori_cloud_handler(const livox_ros_driver::CustomMsgConstPtr& livox_msg_in)
  {
    auto tick = std::chrono::high_resolution_clock::now();
    if(!_first_velo_reveived) return; // to make sure we have velo cloud to match

    if(_first_hori){
      _hori_start_stamp = livox_msg_in->timebase; // global hori message time_base
      ROS_INFO_STREAM("Update _hori_start_stamp :" << _hori_start_stamp);
      _first_hori = false;
    }

    livox_ros_driver::CustomMsg livox_msg_in_distort(*livox_msg_in);

    //#### push to queue to aligh timestamp
    std::unique_lock<std::mutex> lock_hori(_mutexHoriQueue);
    _hori_queue.push(livox_msg_in_distort);
    _hori_msg_yaw_vec.push_back(_yaw_velocity);
    lock_hori.unlock();

    // ###$ integrate more msgs to get extrinsic transform
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudIn(new  pcl::PointCloud<pcl::PointXYZI>);
    livoxToPCLCloud(livox_msg_in_distort, *pointCloudIn, _cut_raw_message_pieces);
    if(_hori_itegrate_frames > 0 && !_hori_tf_initd )
    {
      _hori_igcloud += *pointCloudIn;
      _hori_itegrate_frames--;
      ROS_INFO_STREAM("hori cloud integrating: " << _hori_itegrate_frames);
      return;
    }
    else
    {
      // Calibrate the Lidar first
      if(!_hori_tf_initd && en_extrinsic_esti){
        Eigen::AngleAxisf init_rot_x( 0.0 , Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf init_rot_y( 0.0 , Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf init_rot_z( 0.0 , Eigen::Vector3f::UnitZ());

        Eigen::Translation3f init_trans(0.0,0.0,0.0);
        Eigen::Matrix4f init_tf = (init_trans * init_rot_z * init_rot_y * init_rot_x).matrix();
        Eigen::Matrix4f mid12ouster_tf_matrix, mid22ouster_tf_matrix, mid22mid1_tg_matrix;
        ROS_INFO("\n\n\n  Calibrate Horizon ...");

        pcl::PointCloud<PointType> hori_temp1, hori_temp2, hori_temp3;


        pcl::transformPointCloud (_hori_igcloud, hori_temp1, _mid1_ouster_tf_init);
        pcl::transformPointCloud (_hori_igcloud2, hori_temp2, _mid2_ouster_tf_init);
        pcl::transformPointCloud (_hori_igcloud3, hori_temp3, _mid3_ouster_tf_init);

        pcl::copyPointCloud(hori_temp3, _ouster_new_cloud);

        calibratePCLICP(hori_temp1.makeShared(), _ouster_new_cloud.makeShared(), mid12ouster_tf_matrix, true, "mid12ouster");
        calibratePCLICP(hori_temp2.makeShared(), _ouster_new_cloud.makeShared(), mid22ouster_tf_matrix, true, "mid22ouster");
        calibratePCLICP(hori_temp2.makeShared(), hori_temp1.makeShared(), mid22mid1_tg_matrix, true, "mid22mid1");

        mid12ouster_tf_matrix = mid12ouster_tf_matrix * _mid1_ouster_tf_init;
        mid22ouster_tf_matrix = mid22ouster_tf_matrix * _mid2_ouster_tf_init;
        mid22mid1_tg_matrix = _mid1_ouster_tf_init.inverse() * mid22mid1_tg_matrix * _mid2_ouster_tf_init;

        pcl::transformPointCloud (_hori_igcloud, hori_temp1, mid12ouster_tf_matrix);
        pcl::transformPointCloud (_hori_igcloud2, hori_temp2, mid22ouster_tf_matrix);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudALL (new pcl::PointCloud<pcl::PointXYZRGB>(_ouster_new_cloud.size() + hori_temp1.size() + hori_temp2.size(),1));

        // Fill in the CloudIn data
        for (int i = 0; i < _ouster_new_cloud.size(); i++)
        {
          pcl::PointXYZRGB pointin;
          pointin.x = (_ouster_new_cloud)[i].x;
          pointin.y = (_ouster_new_cloud)[i].y;
          pointin.z = (_ouster_new_cloud)[i].z;
          pointin.r = 255;
          pointin.g = 0;
          pointin.b = 0;
          (*cloudALL)[i] = pointin;
        }
        for (int i = 0; i < hori_temp1.size(); i++)
        {
          pcl::PointXYZRGB pointout;
          pointout.x = (hori_temp1)[i].x;
          pointout.y = (hori_temp1)[i].y;
          pointout.z = (hori_temp1)[i].z;
          pointout.r = 0;
          pointout.g = 255;
          pointout.b = 0;
          (*cloudALL)[i+_ouster_new_cloud.size()] = pointout;
        }
        for (int i = 0; i < hori_temp2.size(); i++)
        {
          pcl::PointXYZRGB pointout;
          pointout.x = (hori_temp2)[i].x;
          pointout.y = (hori_temp2)[i].y;
          pointout.z = (hori_temp2)[i].z;
          pointout.r = 0;
          pointout.g = 0;
          pointout.b = 255;
          (*cloudALL)[i+_ouster_new_cloud.size() + hori_temp1.size()] = pointout;
        }

        pcl::io::savePCDFile<pcl::PointXYZRGB> (std::string(ROOT_DIR) + "PCD/icp_ICP_all.pcd", *cloudALL);

        // mid3 is mid1（mid） in fastlio yml; mid1 is mid2（front） in fastlio yml; mid2 is mid3(rear) in fastlio yml
        // ROS_WARN_STREAM("transformation_matrix Mid2-> Mid1: \n"<< _mid3_ouster_tf_init.inverse() * mid12ouster_tf_matrix);
        // ROS_WARN_STREAM("transformation_matrix Mid3-> Mid1: \n"<< _mid3_ouster_tf_init.inverse() * mid22ouster_tf_matrix);
        // ROS_WARN_STREAM("transformation_matrix Mid2-> base: \n"<< mid12ouster_tf_matrix);
        // ROS_WARN_STREAM("transformation_matrix Mid3-> base: \n"<< mid22ouster_tf_matrix);
        // ROS_WARN_STREAM("transformation_matrix Mid3-> Mid2: \n"<< mid22mid1_tg_matrix);

        Eigen::Matrix4f mid1_tf_matrix, mid2_tf_matrix;
        mid1_tf_matrix = _mid3_ouster_tf_init.inverse() * mid12ouster_tf_matrix;
        mid2_tf_matrix = _mid3_ouster_tf_init.inverse() * mid22ouster_tf_matrix;

        // 打开一个文件用于写入
        std::ofstream file(std::string(ROOT_DIR) + "PCD/result.txt");
        // 检查文件是否成功打开
        if (!file.is_open()) {
          std::cerr << "无法打开文件" << std::endl;
        }
        else {
          // 平移参数
          file << "extrinsic_T2: ["; //
          std::cout << "extrinsic_T2: ["; //
          for (int i = 0; i < 3; ++i) {
            file << mid1_tf_matrix(i, 3);
            if (i < 2) file << ", "; // 在每个元素后面添加逗号，除了每行的最后一个元素
            std::cout << mid1_tf_matrix(i, 3);
            if (i < 2) std::cout << ", "; // 在每个元素后面添加逗号，除了每行的最后一个元素
          }

          // 旋转参数
          file << "] \nextrinsic_R2: ["; //
          std::cout << "] \nextrinsic_R2: ["; //

          // 遍历矩阵并将每个元素写入文件
          for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
              file << mid1_tf_matrix(i, j);
              if (j < 2 || i < 2) file << ", "; // 在每个元素后面添加逗号，除了每行的最后一个元素
              std::cout << mid1_tf_matrix(i, j);
              if (j < 2 || i < 2) std::cout << ", "; // 在每个元素后面添加逗号，除了每行的最后一个元素
            }
            if (i == 2)  {
              file << "]"; //
              std::cout << "]"; //
            }
            file << "\n"; // 每行结束后换行
            std::cout << "\n"; // 每行结束后换行
          }

          // 平移参数
          file << "\nextrinsic_T3: ["; // 每行结束后换行
          std::cout << "\nextrinsic_T3: ["; // 每行结束后换行

          for (int i = 0; i < 3; ++i) {
            file << mid2_tf_matrix(i, 3);
            std::cout << mid2_tf_matrix(i, 3);
            if (i < 2) {
              file << ", "; // 在每个元素后面添加逗号，除了每行的最后一个元素
              std::cout << ", "; // 在每个元素后面添加逗号，除了每行的最后一个元素
            }
          }

          // 旋转参数
          file << "] \nextrinsic_R3: ["; // 每行结束后换行
          std::cout << "] \nextrinsic_R3: ["; // 每行结束后换行

          // 遍历矩阵并将每个元素写入文件
          for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
              file << mid2_tf_matrix(i, j);
              std::cout << mid2_tf_matrix(i, j);
              if (j < 2 || i < 2) {
                file << ", "; // 在每个元素后面添加逗号，除了每行的最后一个元素
                std::cout << ", "; // 在每个元素后面添加逗号，除了每行的最后一个元素
              }
            }
            if (i == 2)  {
              file << "]"; //
              std::cout << "]"; //
            }
            file << "\n"; // 每行结束后换行
            std::cout << "\n"; // 每行结束后换行
          }

          _hori_tf_opted = true;
        }
      }
    }
  }
  /**
   * @brief subscribe raw pointcloud message from Livox lidar and process the data.
   * - save the first timestamp of first message to init the timestamp
   * - Undistort pointcloud based on rotation from IMU
   * - If TF is not initlized,  Push the current undistorted message and yaw to queue;
   * - If TF is not intilized,  align two pointclouds with ICP after integrating enough frames
   * - If TF has been initized, publish aligned cloud in Horizon frame-id
   */
  void hori_cloud_handler3(const livox_ros_driver::CustomMsgConstPtr& livox_msg_in)
  {
    auto tick = std::chrono::high_resolution_clock::now();
    if(!_first_velo_reveived) _first_velo_reveived = true; // to make sure we have velo cloud to match

    if(_first_hori3){
      _hori_start_stamp3 = livox_msg_in->timebase; // global hori message time_base
      ROS_INFO_STREAM("Update _hori_start_stamp3 :" << _hori_start_stamp3);
      _first_hori3 = false;
    }

    livox_ros_driver::CustomMsg livox_msg_in_distort(*livox_msg_in);
    // RemoveLidarDistortion( livox_msg_in, livox_msg_in_distort);
    // ###$ integrate more msgs to get extrinsic transform
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudIn(new  pcl::PointCloud<pcl::PointXYZI>);
    livoxToPCLCloud(livox_msg_in_distort, *pointCloudIn, _cut_raw_message_pieces);
    if(_hori_itegrate_frames3 > 0 && !_hori_tf_initd )
    {
      _hori_igcloud3 += *pointCloudIn;
      _hori_itegrate_frames3--;
      ROS_INFO_STREAM("hori cloud integrating: " << _hori_itegrate_frames3);
      return;
    }
  }

  /**
   * @brief subscribe raw pointcloud message from Livox lidar and process the data.
   * - save the first timestamp of first message to init the timestamp
   * - Undistort pointcloud based on rotation from IMU
   * - If TF is not initlized,  Push the current undistorted message and yaw to queue;
   * - If TF is not intilized,  align two pointclouds with ICP after integrating enough frames
   * - If TF has been initized, publish aligned cloud in Horizon frame-id
   */
  void hori_cloud_handler2(const livox_ros_driver::CustomMsgConstPtr& livox_msg_in)
  {
    auto tick = std::chrono::high_resolution_clock::now();
    if(!_first_velo_reveived) return; // to make sure we have velo cloud to match

    if(_first_hori2){
      _hori_start_stamp2 = livox_msg_in->timebase; // global hori message time_base
      ROS_INFO_STREAM("Update _hori_start_stamp2 :" << _hori_start_stamp2);
      _first_hori2 = false;
    }

    livox_ros_driver::CustomMsg livox_msg_in_distort(*livox_msg_in);
    // RemoveLidarDistortion( livox_msg_in, livox_msg_in_distort);
    // ###$ integrate more msgs to get extrinsic transform
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudIn(new  pcl::PointCloud<pcl::PointXYZI>);
    livoxToPCLCloud(livox_msg_in_distort, *pointCloudIn, _cut_raw_message_pieces);
    if(_hori_itegrate_frames2 > 0 && !_hori_tf_initd )
    {
      _hori_igcloud2 += *pointCloudIn;
      _hori_itegrate_frames2--;
      ROS_INFO_STREAM("hori cloud integrating: " << _hori_itegrate_frames2);
      return;
    }
  }
  void removeFarPointCloud(const pcl::PointCloud<pcl::PointXYZI> &cloud_in, pcl::PointCloud<pcl::PointXYZI> &cloud_out, float thres)
  {
    if (&cloud_in != &cloud_out) {
      cloud_out.header = cloud_in.header;
      cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i) {
      if (cloud_in.points[i].x * cloud_in.points[i].x +
          cloud_in.points[i].y * cloud_in.points[i].y +
          cloud_in.points[i].z * cloud_in.points[i].z > thres * thres)
        continue;
      cloud_out.points[j] = cloud_in.points[i];
      j++;
    }
    if (j != cloud_in.points.size()) {
      cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
  }


  void livoxToPCLCloud( livox_ros_driver::CustomMsg& livox_msg_in, pcl::PointCloud<pcl::PointXYZI>& out_cloud, int ratio = 1)
  {
    out_cloud.clear();
    for (unsigned int i = 0; i < livox_msg_in.point_num / ratio; ++i) {
      pcl::PointXYZI pt;
      pt.x = livox_msg_in.points[i].x;
      pt.y = livox_msg_in.points[i].y;
      pt.z = livox_msg_in.points[i].z;
      pt.intensity =  livox_msg_in.points[i].reflectivity;
      // pt.intensity = livox_msg_in.timebase + livox_msg_in.points[i].offset_time;
      out_cloud.push_back(pt);
    }
  }

  void calibratePCLICP(pcl::PointCloud<PointType>::Ptr source_cloud,
                       pcl::PointCloud<PointType>::Ptr target_cloud, Eigen::Matrix4f &tf_marix, bool save_pcd =true, std::string hw = "mid2ouster")
  {
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::ratio<1, 1000>> time_span =
        std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1000>>>(t2 - t1);

    std::cout << "------------checking PCL GICP---------------- "<< std::endl;
    int pCount = source_cloud->size();

    pcl::PointCloud<PointType>::Ptr ds_cloud_in (new pcl::PointCloud<PointType> );
    pcl::PointCloud<PointType>::Ptr dstf_cloud_in (new pcl::PointCloud<PointType> );
    pcl::PointCloud<PointType>::Ptr ds_cloud_out (new pcl::PointCloud<PointType> );

    // // Remove the noise points
    pcl::PointCloud<PointType>::Ptr cloudin_filtered (new pcl::PointCloud<PointType>);
    // pcl::StatisticalOutlierRemoval<PointType> sor;
    // sor.setInputCloud (source_cloud);
    // sor.setMeanK (50);
    // sor.setStddevMulThresh (1.0);
    // sor.filter (*cloudin_filtered);
    // ROS_INFO_STREAM("CloudIn-> Removing noise clouds "<< source_cloud->size() << " "<<  cloudin_filtered->size() );
    removeFarPointCloud(*source_cloud, *cloudin_filtered, 50);
    ROS_INFO_STREAM("CloudIn-> Removing noise clouds "<< source_cloud->size() << " "<<  cloudin_filtered->size() );

    // Remove the noise points
    pcl::PointCloud<PointType>::Ptr cloudout_filtered (new pcl::PointCloud<PointType>);
    // sor.setInputCloud (target_cloud);
    // sor.setMeanK (50);
    // sor.setStddevMulThresh (1.0);
    // sor.filter (*cloudout_filtered);
    // ROS_INFO_STREAM("CloudOut-> Removing noise clouds "<< target_cloud->size() << " "<<  cloudout_filtered->size() );
    removeFarPointCloud(*target_cloud, *cloudout_filtered, 50);
    ROS_INFO_STREAM("CloudOut-> Removing noise clouds "<< target_cloud->size() << " "<<  cloudout_filtered->size() );

    // Create the filtering object
    pcl::VoxelGrid< PointType> vox;
    vox.setInputCloud (cloudin_filtered);
    vox.setLeafSize (0.05f, 0.05f, 0.05f);
    vox.filter(*ds_cloud_in);
//    *ds_cloud_in = *cloudin_filtered;
    // std::cout << "Source DS: " << source_cloud->size() << " ->  " << ds_cloud_in->size()<< std::endl;

    // Create the filtering object
    vox.setInputCloud (cloudout_filtered);
    vox.setLeafSize (0.05f, 0.05f, 0.05f);
    vox.filter(*ds_cloud_out);
    // std::cout << "Target DS: " << target_cloud->size() << " -> " << ds_cloud_out->size()<< std::endl;

    *dstf_cloud_in =  *dstf_cloud_in + *ds_cloud_in;

    std::cout << "GICP start  .... " << ds_cloud_in->size() << " to "<< ds_cloud_out->size()<< std::endl;
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;
    gicp.setTransformationEpsilon(0.001);
    gicp.setMaxCorrespondenceDistance(0.5);
    gicp.setMaximumIterations(500);
    gicp.setRANSACIterations(12);
    gicp.setInputSource(dstf_cloud_in);
    gicp.setInputTarget(ds_cloud_out);

    pcl::PointCloud<PointType>::Ptr transformedP (new pcl::PointCloud<PointType>);

    t1 = std::chrono::steady_clock::now();
    gicp.align(*transformedP);
    t2 = std::chrono::steady_clock::now();
    time_span = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1000>>>(t2 - t1);
    // std::cout << "PCL gicp.align Time: " << time_span.count() << " ms."<< std::endl;
    std::cout << "====>>>>> has converged: " << gicp.hasConverged() << " ====>>>>> score: " <<
              gicp.getFitnessScore() << std::endl;

    auto transformation_matrix =  gicp.getFinalTransformation ();
    std::cout << "transformation_matrix:\n"<<transformation_matrix << std::endl;
    std::cout << std::endl;

    auto cloudSrc = dstf_cloud_in;
    auto cloudDst = ds_cloud_out;

    pcl::PointCloud<PointType> input_transformed;
    pcl::transformPointCloud (*cloudSrc, input_transformed, transformation_matrix);

    Eigen::Matrix3f rot_matrix = transformation_matrix.block<3,3>(0,0);
    Eigen::Vector3f euler = rot_matrix.eulerAngles(2, 1, 0);
    // std::cout << "Rotation    : "<<  euler.x() << " " << euler.y() << " " << euler.z()<< std::endl;
    // std::cout << "Rotation_matrix:\n"<<rot_matrix << std::endl;

    tf_marix = transformation_matrix;

    if(save_pcd)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSrcRGB (new pcl::PointCloud<pcl::PointXYZRGB>(cloudSrc->size(),1));
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudDstRGB (new pcl::PointCloud<pcl::PointXYZRGB>(cloudDst->size(),1));
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudALL (new pcl::PointCloud<pcl::PointXYZRGB>(cloudSrc->size() + cloudDst->size(),1));

      // Fill in the CloudIn data
      for (int i = 0; i < cloudSrc->size(); i++)
      {
        pcl::PointXYZRGB &pointin = (*cloudSrcRGB)[i];
        pointin.x = (*cloudSrc)[i].x;
        pointin.y = (*cloudSrc)[i].y;
        pointin.z = (*cloudSrc)[i].z;
        pointin.r = 255;
        pointin.g = 0;
        pointin.b = 0;
        (*cloudALL)[i] = pointin;
      }
      for (int i = 0; i < cloudDst->size(); i++)
      {
        pcl::PointXYZRGB &pointout = (*cloudDstRGB)[i];
        pointout.x = (*cloudDst)[i].x;
        pointout.y = (*cloudDst)[i].y;
        pointout.z = (*cloudDst)[i].z;
        pointout.r = 0;
        pointout.g = 255;
        pointout.b = 255;
        (*cloudALL)[i+cloudSrc->size()] = pointout;
      }
      pcl::io::savePCDFile<pcl::PointXYZRGB> (std::string(ROOT_DIR) + "PCD/icp_ICP" + hw + ".pcd", *cloudALL);
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mm_lidarsAligner");

  LidarsParamEstimator mma;
  ROS_INFO("\033[1;32m---->\033[0m Lidar Calibrate Started.");

  ros::Rate r(10);
  while(ros::ok() && !mma._hori_tf_opted){

    ros::spinOnce();
    r.sleep();

  }

  return 0;
}
