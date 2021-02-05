#ifdef USE_PANGOLIN_VIEWER
#include <pangolin_viewer/viewer.h>
#elif USE_SOCKET_PUBLISHER
#include <socket_publisher/publisher.h>
#endif

#include <openvslam/system.h>
#include <openvslam/config.h>
#include "openvslam/data/landmark.h"
#include "openvslam/data/keyframe.h"
#include "openvslam/data/map_database.h"
#include "openvslam/io/map_database_io.h"
#include "openvslam/util/stereo_rectifier.h"
// #include "run_slam.h"

#include <iostream>
#include <chrono>
#include <numeric>

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <sensor_msgs/Image.h>

#include "std_msgs/Bool.h"
#include <std_srvs/Trigger.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl_ros/point_cloud.h>

#include <spdlog/spdlog.h>
#include <popl.hpp>

#include <tf/transform_broadcaster.h>

#ifdef USE_STACK_TRACE_LOGGER
#include <glog/logging.h>
#endif

#ifdef USE_GOOGLE_PERFTOOLS
#include <gperftools/profiler.h>
#endif

using namespace openvslam;

openvslam::system *SLAM=nullptr;


//------------------------------------------------- ROS service for reseting SLAM ---------------------------------------------------
bool callback_reset_SLAM(std_srvs::TriggerRequest& request,
                        std_srvs::TriggerResponse& response)
{
    ROS_INFO("Got signal from service /OpenVSLAM/reset_SLAM to reset SLAM");
    response.success = true;
    SLAM->request_reset();
    return true;
}
//------------------------------------------------ end of service reseting SLAM ----------------------------------------------------

class tracking {
    private:
        std::vector<double> track_times;
        std::chrono::time_point<std::chrono::steady_clock> tp_0;
        std::chrono::time_point<std::chrono::steady_clock> tp_1;
        std::chrono::time_point<std::chrono::steady_clock> tp_2;
        // openvslam::system *SLAM=nullptr;
        openvslam::data::map_database *map_db_;
        cv::Mat mask;
        unsigned int num_frame_rgbd = 0;
        unsigned int num_frame_stereo = 0;
        ros::NodeHandle nh;

        ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/OpenVSLAM/odom", 50);
        ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/OpenVSLAM/cloud", 50);
        ros::Publisher path_pub =  nh.advertise<nav_msgs::Path>("/OpenVSLAM/path", 1000);

        ros::ServiceServer service = nh.advertiseService("/OpenVSLAM/reset_SLAM", callback_reset_SLAM);

        sensor_msgs::CameraInfo info_left, info_right;
        sensor_msgs::PointCloud2 cloud_message;
        
        nav_msgs::Path msg_path;
        std::deque<nav_msgs::Odometry> history_poses;
        
        tf::Matrix3x3 rotation_matrix;
        double focal_x_baseline;
        FILE* file_timestamps;
        openvslam::Mat44_t camera_pose_cw_;

        openvslam::MatX_t covariance_;

        openvslam::Mat33_t rot_cw_;
        openvslam::Mat33_t rot_wc_;
        openvslam::Vec3_t trans_cw_;
        openvslam::Vec3_t cam_center_;

        int n_kf_for_cloud_;

//-------------------------------------------------------- odomCB ---------------------------------------------------------------------------
	void poseTimeSequenceRegression(geometry_msgs::Twist &twist) {
        if (history_poses.size() < 3)
            ROS_WARN_STREAM("There is no " << 3 << " sequential frames, current (" << history_poses.size() << ").");
        // filling trajectory data
        Eigen::MatrixXd values(8, history_poses.size());
        for (unsigned int i = 0; i < history_poses.size(); i++) {
            values(0, i) = history_poses[i].pose.pose.position.x;
            values(1, i) = history_poses[i].pose.pose.position.y;
            values(2, i) = history_poses[i].pose.pose.position.z;
            values(3, i) = history_poses[i].pose.pose.orientation.x;
            values(4, i) = history_poses[i].pose.pose.orientation.y;
            values(5, i) = history_poses[i].pose.pose.orientation.z;
            values(6, i) = history_poses[i].pose.pose.orientation.w;
            values(7, i) = history_poses[i].header.stamp.toSec(); 
        }
        // position xyz
        double v[6] = {0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 3; i++)
            motionRegression1D(values.row(i), values.row(7), v[i], v[i + 3]);
        // orientation quaternion xyzw
        double w[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 4; i++)
            motionRegression1D(values.row(3 + i), values.row(7), w[i], w[i + 4]);
        // transform to the body-fixed frame ?!?!?
        Eigen::Quaterniond q, q_adj, qv, qva, qw, qwa;
        q.w() = history_poses[history_poses.size() - 1].pose.pose.orientation.w;
        q.x() = history_poses[history_poses.size() - 1].pose.pose.orientation.x;
        q.y() = history_poses[history_poses.size() - 1].pose.pose.orientation.y;
        q.z() = history_poses[history_poses.size() - 1].pose.pose.orientation.z;
        q_adj = q;
        q_adj.x() *= -1;
        q_adj.y() *= -1;
        q_adj.z() *= -1;
  
        qv = q_adj * Eigen::Quaterniond(0.0, v[0], v[1], v[2]) * q;
        qva = q_adj * Eigen::Quaterniond(0.0, v[3], v[4], v[5]) * q;  
        q_adj.w() *= 2;
        q_adj.x() *= 2;
        q_adj.y() *= 2;
        q_adj.z() *= 2;
        qw = q_adj * Eigen::Quaterniond(w[3], w[0], w[1], w[2]);
        qwa = q_adj * Eigen::Quaterniond(w[7], w[4], w[5], w[6]);
        qva.vec() = qva.vec() - qw.vec().cross(qv.vec());
        twist.linear.x = qv.x();
        twist.linear.y = qv.y();
        twist.linear.z = qv.z();
        twist.angular.x = qw.x();
        twist.angular.y = qw.y();
        twist.angular.z = qw.z();
    }
//-------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------- motionRegression1D --------------------------------------------------------------------
    void motionRegression1D(Eigen::VectorXd values, Eigen::VectorXd times, double &v, double &a) {
        double s_x = 0;
        double s_tx = 0;
        double s_t2x = 0;
        double s_t = 0;
        double s_t2 = 0;
        double s_t3 = 0;
        double s_t4 = 0;
        double target_time = times[values.size() - 1];
        for (int i = 0; i < values.size(); i++) {
            double t_i = times[i] - target_time;
            s_x += values[i];
            s_tx += values[i] * t_i;
            double t2 = t_i * t_i;
            s_t2x += values[i] * t2;
            s_t += t_i;
            s_t2 += t2;
            s_t3 += t2 * t_i;
            s_t4 += t2 * t2;
        }
        double det_1 = 1 / (values.size() * (s_t3 * s_t3 - s_t2 * s_t4) +
        s_t * (s_t * s_t4 - s_t2 * s_t3) + s_t2 * (s_t2 * s_t2 - s_t * s_t3));
  
        v = det_1 * (s_x * (s_t * s_t4 - s_t2 * s_t3) + s_tx * (s_t2 * s_t2 - values.size() * s_t4) + s_t2x * (values.size() * s_t3 - s_t * s_t2));
  
        a = 2 * det_1 * (s_x * (s_t2 * s_t2 - s_t * s_t3) + s_tx * (values.size() * s_t3 - s_t * s_t2) + s_t2x * (s_t * s_t - values.size() * s_t2));

    }

//------------------------------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------- updating pose parameters ------------------------------------------------------
        void update_pose_params() {
            rot_cw_ = camera_pose_cw_.block<3, 3>(0, 0);
            rot_wc_ = rot_cw_.transpose();
            trans_cw_ = camera_pose_cw_.block<3, 1>(0, 3);
            cam_center_ = -rot_cw_.transpose() * trans_cw_;
}
//-----------------------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------- Odometry Publisher -------------------------------------------------------------
        void publish_odom_and_path(const std_msgs::Header& header) {
            Eigen::Matrix3d rotation_matrix = camera_pose_cw_.block(0, 0, 3, 3);
            Eigen::Vector3d translation_vector = camera_pose_cw_.block(0, 3, 3, 1);

            tf::Matrix3x3 tf_rotation_matrix(rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2),
                                      rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2),
                                      rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2));
    
            tf::Vector3 tf_translation_vector(translation_vector(0), translation_vector(1), translation_vector(2));

            tf_rotation_matrix = tf_rotation_matrix.inverse();
            tf_translation_vector = -(tf_rotation_matrix * tf_translation_vector);

            tf::Transform transform_tf(tf_rotation_matrix, tf_translation_vector);

            tf::Matrix3x3 rot_open_to_ros (0, 0, 1,
                                  -1, 0, 0,
                                   0,-1, 0);

            tf::Transform transformA(rot_open_to_ros, tf::Vector3(0.0, 0.0, 0.0));
            tf::Transform transformB(rot_open_to_ros.inverse(), tf::Vector3(0.0, 0.0, 0.0));
            tf::Transform transform_6_track(tf::Quaternion(-0.47545, -0.50846, 0.51558, 0.499572).normalize(), tf::Vector3(20.84356, 0.24199, -0.519));

            transform_tf = transformA * transform_tf * transformB * transform_6_track;
            // setting odometry
            nav_msgs::Odometry msg_odom;
            msg_odom.header = header;
            msg_odom.header.frame_id = "map";
            msg_odom.child_frame_id = "map";
            msg_odom.pose.pose.orientation.x = transform_tf.getRotation().getX();
            msg_odom.pose.pose.orientation.y = transform_tf.getRotation().getY();
            msg_odom.pose.pose.orientation.z = transform_tf.getRotation().getZ();
            msg_odom.pose.pose.orientation.w = transform_tf.getRotation().getW();
            msg_odom.pose.pose.position.x = transform_tf.getOrigin().getX();
            msg_odom.pose.pose.position.y = transform_tf.getOrigin().getY();
            msg_odom.pose.pose.position.z = transform_tf.getOrigin().getZ();
            odom_pub.publish(msg_odom);

            // publishing path
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header = header;
            pose_stamped.pose = msg_odom.pose.pose;
            msg_path.header = header;
            msg_path.header.frame_id = "map";
            // !!!only 6 track!!! filter of nan and inf value during openvslam initialization
            if (header.stamp.sec >= 1584445690){
                msg_path.poses.push_back(pose_stamped);
                path_pub.publish(msg_path);
            }
        }
//------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------- Cloud Publisher -----------------------------------------------------------------
        void publish_cloud(const std_msgs::Header& header) {
            // number_of_previous_keyframes:
            // 0  -- local landmarks - TODO
            // -1 -- all landmarks
            // 1 up to inf -- a number of the previous keyframes
            map_db_  = SLAM->get_reference_to_map_database();
            pcl::PointCloud<pcl::PointXYZ> cloud_of_keypoints;
            std::vector<openvslam::data::landmark*> landmarks;
            update_pose_params();
            if (n_kf_for_cloud_ == -1) {
                landmarks = map_db_->get_all_landmarks();
            }
            else if (n_kf_for_cloud_ > 0) {
                std::vector<openvslam::data::keyframe*> all_keyframes = map_db_->get_all_keyframes();
                // int number_of_keyframes = all_keyframes.size();
                for (auto number_of_keyframe = 1; 
                    (number_of_keyframe <= static_cast<int>(all_keyframes.size())) && (number_of_keyframe <= n_kf_for_cloud_); 
                    number_of_keyframe++) 
                {
                    openvslam::data::keyframe* kframe = all_keyframes.at(number_of_keyframe-1);
                    std::vector<openvslam::data::landmark*> landmarks_from_keyframe = kframe->get_landmarks();
                    landmarks.reserve(landmarks.size() + distance(landmarks_from_keyframe.begin(),landmarks_from_keyframe.end()));
                    landmarks.insert(landmarks.end(),landmarks_from_keyframe.begin(),landmarks_from_keyframe.end());
                }
            }
            for (const auto& landmark : landmarks) {
                if (!landmark) {
                    continue;
                }
                if (landmark->will_be_erased()) {
                    continue;
                }
                pcl::PointXYZ newPoint;
                auto xyz_world = landmark->get_pos_in_world();
                auto xyz_camera = rot_wc_.inverse()*(xyz_world - cam_center_);
                newPoint.x = xyz_camera(0);
                newPoint.y = xyz_camera(1);
                newPoint.z = xyz_camera(2);
                cloud_of_keypoints.push_back(newPoint);
            }
            pcl::toROSMsg(cloud_of_keypoints, cloud_message);
            cloud_message.header = header;
            cloud_message.is_dense = false;
            cloud_pub.publish(cloud_message);
        }
//------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------- callback_stereo method -----------------------------------------------------------------
        void callback_stereo(const sensor_msgs::ImageConstPtr& left_msg, const sensor_msgs::ImageConstPtr& right_msg) {

            tp_1 = std::chrono::steady_clock::now();
            cv::Mat left_image_rectified = cv_bridge::toCvShare(left_msg, "bgr8")->image;
            cv::Mat right_image_rectified = cv_bridge::toCvShare(right_msg, "bgr8")->image;
            const double timestamp = left_msg->header.stamp.sec + left_msg->header.stamp.nsec/1000000000.0;
            std::pair<Mat44_t, MatX_t> camera_pose_cw__with_covariance = SLAM->feed_stereo_frame(left_image_rectified, right_image_rectified, timestamp, mask);
            camera_pose_cw_ = camera_pose_cw__with_covariance.first;
            covariance_ = camera_pose_cw__with_covariance.second;

            publish_odom_and_path(left_msg->header);
            publish_cloud(left_msg->header);

            tp_2 = std::chrono::steady_clock::now();
            const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
            track_times.push_back(track_time);
        } 
//-------------------------------------------------- end of method callback_stereo -----------------------------------------------------

//--------------------------------------------------------- callback_RGBD_SLAM -------------------------------------------------------
        void callback_RGBD_SLAM(const sensor_msgs::ImageConstPtr& input_left_message, const sensor_msgs::ImageConstPtr& input_depth_map_message) {
            
            tp_1 = std::chrono::steady_clock::now();

            cv::Mat left_image = cv_bridge::toCvShare(input_left_message, "bgr8")->image;
            cv::Mat depth_map = cv_bridge::toCvShare(input_depth_map_message, "32FC1")->image;

            const double timestamp = input_left_message->header.stamp.sec + input_left_message->header.stamp.nsec/1000000000.0; 
            std::pair<Mat44_t, MatX_t> camera_pose_cw__with_covariance = SLAM->feed_RGBD_frame(left_image, depth_map, timestamp, mask);
            camera_pose_cw_ = camera_pose_cw__with_covariance.first;
            covariance_ = camera_pose_cw__with_covariance.second;

            publish_odom_and_path(input_left_message->header);
            publish_cloud(input_left_message->header);

            tp_2 = std::chrono::steady_clock::now();
            const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
            track_times.push_back(track_time);

        }
//------------------------------------------------------------------------------------------------------------------------------------

//----------------------------------------converting cv_Mat to vector of double ------------------------------------------------------
    std::vector<float> convert_Mat_to_vector(cv::Mat mat) {
        std::vector<float> array;
        if (mat.isContinuous()) {
            array.assign((float*)mat.data, (float*)mat.data + mat.total()*mat.channels());
        } 
        else {
            for (int i = 0; i < mat.rows; ++i) {
                array.insert(array.end(), mat.ptr<float>(i), mat.ptr<float>(i)+mat.cols*mat.channels());
            }
        }
        return array;
    }
//------------------------------------------------------------------------------------------------------------------------------------

    public:

//------------------------------------------------- Method of tracking initialization -----------------------------------------------------------------
        tracking(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path,
                 const std::string& mask_img_path, const unsigned& n_kf_for_cloud)       
        {
            SLAM = new openvslam::system(cfg, vocab_file_path);
            // load the mask image
            mask = mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);
            // startup the SLAM process
            //file_timestamps = fopen(("/media/cds-s/data/Datasets/Husky-NKBVS/00_map_half_2020-03-17-14-21-57/RGBD_run_slam_ros/timestamps.txt"),"w");
            SLAM->startup();
            n_kf_for_cloud_ = n_kf_for_cloud;
        }
//------------------------------------------------- end of tracking initialization -------------------------------------------------------------------

//----------------------------------------------------------- mono_tracking method -----------------------------------------------------------------------
        void mono_tracking(const std::shared_ptr<openvslam::config>& cfg, const bool eval_log, const std::string& map_db_path, const std::string& topic) {
            
        #ifdef USE_PANGOLIN_VIEWER
            pangolin_viewer::viewer viewer(cfg, SLAM, SLAM->get_frame_publisher(), SLAM->get_map_publisher());
        #elif USE_SOCKET_PUBLISHER
            socket_publisher::publisher publisher(cfg, &SLAM, SLAM->get_frame_publisher(), SLAM->get_map_publisher());
        #endif            

            tp_0 = std::chrono::steady_clock::now();

            image_transport::ImageTransport it(nh);

            // run the SLAM as subscriber
            image_transport::Subscriber sub = it.subscribe(topic, 1, [&](const sensor_msgs::ImageConstPtr& msg) {
                auto tp_1 = std::chrono::steady_clock::now();
                const auto timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - tp_0).count();

                // input the current frame and estimate the camera pose
                std::pair<Mat44_t, MatX_t> camera_pose_cw__with_covariance = SLAM->feed_monocular_frame(cv_bridge::toCvShare(msg, "bgr8")->image, timestamp, mask);
                
                camera_pose_cw_ = camera_pose_cw__with_covariance.first;
                //covariance_ = camera_pose_cw__with_covariance.second;

                publish_odom_and_path(msg->header);
                publish_cloud(msg->header);

                
                const auto tp_2 = std::chrono::steady_clock::now();

                const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
                track_times.push_back(track_time);
            });

            // run the viewer in another thread
        #ifdef USE_PANGOLIN_VIEWER
            std::thread thread([&]() {
                viewer.run();
                if (SLAM->terminate_is_requested()) {
                    // wait until the loop BA is finished
                    while (SLAM->loop_BA_is_running()) {
                        std::this_thread::sleep_for(std::chrono::microseconds(5000));
                    }
                    ros::shutdown();
                }
            });
        #elif USE_SOCKET_PUBLISHER
            std::thread thread([&]() {
                publisher.run();
                if (SLAM->terminate_is_requested()) {
                    // wait until the loop BA is finished
                    while (SLAM->loop_BA_is_running()) {
                        std::this_thread::sleep_for(std::chrono::microseconds(5000));
                    }
                    ros::shutdown();
                }
            });
        #endif

            ros::spin();

            // automatically close the viewer
        #ifdef USE_PANGOLIN_VIEWER
            viewer.request_terminate();
            thread.join();
        #elif USE_SOCKET_PUBLISHER
            publisher.request_terminate();
            thread.join();
        #endif

            // shutdown the SLAM process
            SLAM->shutdown();

            if (eval_log) {
                // output the trajectories for evaluation
                SLAM->save_frame_trajectory("frame_trajectory.txt", "TUM");
                SLAM->save_keyframe_trajectory("keyframe_trajectory.txt", "TUM");
                // output the tracking times for evaluation
                std::ofstream ofs("track_times.txt", std::ios::out);
                if (ofs.is_open()) {
                    for (const auto track_time : track_times) {
                        ofs << track_time << std::endl;
                    }
                    ofs.close();
                }
            }

            if (!map_db_path.empty()) {
                // output the map database
                SLAM->save_map_database(map_db_path);
            }

            if (track_times.size()) {
                std::sort(track_times.begin(), track_times.end());
                const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
                std::cout << "median tracking time: " << track_times.at(track_times.size() / 2) << "[s]" << std::endl;
                std::cout << "mean tracking time: " << total_track_time / track_times.size() << "[s]" << std::endl;
            }
        } 
//--------------------------------------------------- end of method mono_tracking --------------------------------------------------------

//--------------------------------------------------- stereo_tracking method -------------------------------------------------------------
        void stereo_tracking(const std::shared_ptr<openvslam::config>& cfg, const bool eval_log, const std::string& map_db_path, 
                             const std::string& left_topic, const std::string& right_topic) {

        #ifdef USE_PANGOLIN_VIEWER
            pangolin_viewer::viewer viewer(cfg, SLAM, SLAM->get_frame_publisher(), SLAM->get_map_publisher());
        #elif USE_SOCKET_PUBLISHER
            socket_publisher::publisher publisher(cfg, &SLAM, SLAM->get_frame_publisher(), SLAM->get_map_publisher());
        #endif

            tp_0 = std::chrono::steady_clock::now();

            message_filters::Subscriber<sensor_msgs::Image> image_left_sub(nh, left_topic, 1);
            message_filters::Subscriber<sensor_msgs::Image> image_right_sub(nh, right_topic, 1);
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
            MySyncPolicy sync_policy(10);
            sync_policy.setMaxIntervalDuration(ros::Duration(0.05));
            message_filters::Synchronizer<MySyncPolicy> sync(static_cast<const MySyncPolicy>(sync_policy), image_left_sub, image_right_sub);
            sync.registerCallback(boost::bind(&tracking::callback_stereo, this, _1, _2));

            // run the viewer in another thread
        #ifdef USE_PANGOLIN_VIEWER
            std::thread thread([&]() {
                viewer.run();
                if (SLAM->terminate_is_requested()) {
                    // wait until the loop BA is finished
                    while (SLAM->loop_BA_is_running()) {
                        std::this_thread::sleep_for(std::chrono::microseconds(5000));
                    }
                    ros::shutdown();
                }
            });
        #elif USE_SOCKET_PUBLISHER
            std::thread thread([&]() {
                publisher.run();
                if (SLAM->terminate_is_requested()) {
                    // wait until the loop BA is finished
                    while (SLAM->loop_BA_is_running()) {
                        std::this_thread::sleep_for(std::chrono::microseconds(5000));
                    }
                    ros::shutdown();
                }
            });
        #endif

            ros::spin();

            // automatically close the viewer
        #ifdef USE_PANGOLIN_VIEWER
            viewer.request_terminate();
            thread.join();
        #elif USE_SOCKET_PUBLISHER
            publisher.request_terminate();
            thread.join();
        #endif

            // shutdown the SLAM process
            SLAM->shutdown();

            if (eval_log) {
                // output the trajectories for evaluation
                SLAM->save_frame_trajectory("frame_trajectory.txt", "TUM");
                SLAM->save_keyframe_trajectory("keyframe_trajectory.txt", "TUM");
                // output the tracking times for evaluation
                std::ofstream ofs("track_times.txt", std::ios::out);
                if (ofs.is_open()) {
                    for (const auto track_time : track_times) {
                        ofs << track_time << std::endl;
                    }
                    ofs.close();
                }
            }

            if (!map_db_path.empty()) {
                // output the map database
                SLAM->save_map_database(map_db_path);
            }

            if (track_times.size()) {
                std::sort(track_times.begin(), track_times.end());
                const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
                std::cout << "median tracking time: " << track_times.at(track_times.size() / 2) << "[s]" << std::endl;
                std::cout << "mean tracking time: " << total_track_time / track_times.size() << "[s]" << std::endl;
            }
        } 
//-------------------------------------------------- end of stereo tracking method ------------------------------------------------------------

//------------------------------------------------- RGBD tracking -------------------------------------------------------------------------------------------------------------------------
        void RGBD_tracking(const std::shared_ptr<openvslam::config>& cfg, const bool eval_log, const std::string& map_db_path, 
                       const std::string& topic1, const std::string& topic2) {

            #ifdef USE_PANGOLIN_VIEWER
                pangolin_viewer::viewer viewer(cfg, SLAM, SLAM->get_frame_publisher(), SLAM->get_map_publisher());
            #elif USE_SOCKET_PUBLISHER
                socket_publisher::publisher publisher(cfg, &SLAM, SLAM->get_frame_publisher(), SLAM->get_map_publisher());
            #endif  

            tp_0 = std::chrono::steady_clock::now();

            message_filters::Subscriber<sensor_msgs::Image> left_image_rect_sub(nh, topic1, 1);
            message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, topic2, 1);

            typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> ExactSyncPolicy;
            ExactSyncPolicy exact_sync_policy(10);
            message_filters::Synchronizer<ExactSyncPolicy> sync_rgbd(static_cast<const ExactSyncPolicy>(exact_sync_policy), left_image_rect_sub, depth_sub);
            sync_rgbd.registerCallback(boost::bind(&tracking::callback_RGBD_SLAM, this, _1, _2));

            // run the viewer in another thread
            #ifdef USE_PANGOLIN_VIEWER
                std::thread thread([&]() {
                    viewer.run();
                    if (SLAM->terminate_is_requested()) {
                        // wait until the loop BA is finished
                        while (SLAM->loop_BA_is_running()) {
                            std::this_thread::sleep_for(std::chrono::microseconds(5000));
                        }
                        ros::shutdown();
                    }
                });
            #elif USE_SOCKET_PUBLISHER
                std::thread thread([&]() {
                    publisher.run();
                    if (SLAM->terminate_is_requested()) {
                        // wait until the loop BA is finished
                        while (SLAM->loop_BA_is_running()) {
                            std::this_thread::sleep_for(std::chrono::microseconds(5000));
                        }       
                        ros::shutdown();
                    }
                });
            #endif

            ros::spin();

            // automatically close the viewer
            #ifdef USE_PANGOLIN_VIEWER
                viewer.request_terminate();
                thread.join();
            #elif USE_SOCKET_PUBLISHER
                publisher.request_terminate();
                thread.join();
            #endif

            // shutdown the SLAM process
            SLAM->shutdown();

            if (eval_log) {
                // output the trajectories for evaluation
                SLAM->save_frame_trajectory("frame_trajectory.txt", "TUM");
                SLAM->save_keyframe_trajectory("keyframe_trajectory.txt", "TUM");
                // output the tracking times for evaluation
                std::ofstream ofs("track_times.txt", std::ios::out);
                if (ofs.is_open()) {
                    for (const auto track_time : track_times) {
                        ofs << track_time << std::endl;
                    }
                    ofs.close();
                }
            }

            if (!map_db_path.empty()) {
                // output the map database
                SLAM->save_map_database(map_db_path);
            }

            if (track_times.size()) {
                std::sort(track_times.begin(), track_times.end());
                const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
                std::cout << "median tracking time: " << track_times.at(track_times.size() / 2) << "[s]" << std::endl;
                std::cout << "mean tracking time: " << total_track_time / track_times.size() << "[s]" << std::endl;
            }
            if (file_timestamps != NULL)
                fclose (file_timestamps);
        }
            
//------------------------------------------------- end of RGBD tracking -----------------------------------------------------------------------------------------------------------------
};

int main(int argc, char* argv[]) {
#ifdef USE_STACK_TRACE_LOGGER
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
#endif
    ros::init(argc, argv, "OpenVSLAM");

    // create options
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto vocab_file_path = op.add<popl::Value<std::string>>("v", "vocab", "vocabulary file path");
    auto setting_file_path = op.add<popl::Value<std::string>>("c", "config", "setting file path");
    auto topic_1 = op.add<popl::Value<std::string>>("", "topic1", "topic of images for mono SLAM, or of left camera images for stereo SLAM, or of rectified left images for RGBD SLAM", "");
    auto topic_2 = op.add<popl::Value<std::string>>("", "topic2", "topic of depth maps for RGBD SLAM. For Mono SLAM don't fill argument, it'll be ignored", "");
    auto n_kf_for_cloud = op.add<popl::Value<int>>("", "n_kf_for_cloud", "Number of previous keyframes with landmarks for publishing keypoints cloud", 10);    
    auto eval_log = op.add<popl::Switch>("", "eval-log", "store trajectory and tracking times for evaluation");
    auto map_db_path = op.add<popl::Value<std::string>>("", "map-db", "store a map database at this path after SLAM", "");
    auto mask_img_path = op.add<popl::Value<std::string>>("", "mask", "mask image path", "");
    auto debug_mode = op.add<popl::Switch>("", "debug", "debug mode");    
    
    try {
        op.parse(argc, argv);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // check validness of options
    if (help->is_set()) {
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    if (!vocab_file_path->is_set() || !setting_file_path->is_set()) {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    // setup logger
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
    if (debug_mode->is_set()) {
        spdlog::set_level(spdlog::level::debug);
    }
    else {
        spdlog::set_level(spdlog::level::info);
    }

    // load configuration
    std::shared_ptr<openvslam::config> cfg;
    try {
        cfg = std::make_shared<openvslam::config>(setting_file_path->value());
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStart("slam.prof");
#endif
    tracking track(cfg, vocab_file_path->value(), mask_img_path->value(), n_kf_for_cloud->value());
    // run tracking
    if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::Monocular) {
        track.mono_tracking(cfg, eval_log->is_set(), map_db_path->value(), topic_1->value());
    }
    else if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::Stereo) {
        track.stereo_tracking(cfg, eval_log->is_set(), map_db_path->value(), topic_1->value(), topic_2->value());
    }
    else if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::RGBD) {
        track.RGBD_tracking(cfg, eval_log->is_set(), map_db_path->value(), topic_1->value(), topic_2->value());       
    }
    else {
        throw std::runtime_error("Invalid setup type: " + cfg->camera_->get_setup_type_string());
    }

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStop();
#endif

    return EXIT_SUCCESS;
}
