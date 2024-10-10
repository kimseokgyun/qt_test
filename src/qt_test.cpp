
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "qt_test.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;


namespace qt_test
{
    qt_node::qt_node(const rclcpp::NodeOptions& options)
    : Node("qt_node",options)
    {
        //RCLCPP_INFO(get_logger(),"qt_node starting");

        // 로그 파일 열기
        icp_log_file_.open("frm_icp_log.txt", std::ios::out | std::ios::app);

        if (!icp_log_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open ICP log file.");
        }

        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
        marker_publisher_2 = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker_frame", 10);

        // live_cloud = get_vmark_cloud();
        // live_tree = new KD_TREE_XYZR(3, live_cloud, nanoflann::KDTreeSingleIndexAdaptorParams(3)); //REAF10
        // live_tree->buildIndex();

        c_point T1 = c_point(3, 0);
        c_point T2 = c_point(3 -2*DOCK_SIZE_X[1], 0 + 1*DOCK_SIZE_Y[1]);
        c_point T3 = c_point(3 -2*DOCK_SIZE_X[1], 0 - 1*DOCK_SIZE_Y[1]);



        // FRAME frm = generateSampleFrame(T1,T2, T3);
        // RCLCPP_INFO(this->get_logger(), "KD Tree built with %zu points", live_cloud.pts.size());

        timer_ = this->create_wall_timer(50ms, std::bind(&qt_node::timerCallback, this));

        dock_tf = Eigen::Matrix4d::Identity();

        // double err = frm_icp(*live_tree, live_cloud, frm, dock_tf);
        // RCLCPP_INFO(get_logger(), "ICP error: %f", err);
        // RCLCPP_INFO(this->get_logger(), "dock_tf: \n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f", dock_tf(0,0), dock_tf(0,1), dock_tf(0,2), dock_tf(0,3), dock_tf(1,0), dock_tf(1,1), dock_tf(1,2), dock_tf(1,3), dock_tf(2,0), dock_tf(2,1), dock_tf(2,2), dock_tf(2,3), dock_tf(3,0), dock_tf(3,1), dock_tf(3,2), dock_tf(3,3));


        KFRAME frm0 = generateSampleKFrame(T1,T2, T3);
        KFRAME target = generateVKFrame();

        double err2 = kfrm_icp(frm0, target, dock_tf);
        RCLCPP_INFO(get_logger(), "ICP error: %f", err2);
        RCLCPP_INFO(get_logger(), "dock_tf: \n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f", dock_tf(0,0), dock_tf(0,1), dock_tf(0,2), dock_tf(0,3), dock_tf(1,0), dock_tf(1,1), dock_tf(1,2), dock_tf(1,3), dock_tf(2,0), dock_tf(2,1), dock_tf(2,2), dock_tf(2,3), dock_tf(3,0), dock_tf(3,1), dock_tf(3,2), dock_tf(3,3));
    }


    void qt_node::timerCallback() {


        // 샘플 KFRAME 퍼블리시
        c_point T1 = c_point(3, 0);
        c_point T2 = c_point(3 - 2 * DOCK_SIZE_X[1], 0 + 1 * DOCK_SIZE_Y[1]);
        c_point T3 = c_point(3 - 2 * DOCK_SIZE_X[1], 0 - 1 * DOCK_SIZE_Y[1]);

        KFRAME kframe = generateSampleKFrame(T1, T2, T3);
        publishKFrameMarker(kframe, 0, "sample_kframe", 1.0, 0.0, 0.0);
        KFRAME target = generateVKFrame();
        publishKFrameMarker(target, 1, "v_kframe", 0.0, 1.0, 0.0);

        // //RCLCPP_INFO(get_logger(), "Publishing marker");
        // visualization_msgs::msg::Marker marker_msg;
        // marker_msg.header.frame_id = "map";       // RViz에서 참조할 프레임 ID
        // marker_msg.header.stamp = this->now();
        // marker_msg.ns = "vmark_points";
        // marker_msg.id = 0;
        // marker_msg.type = visualization_msgs::msg::Marker::POINTS;
        // marker_msg.action = visualization_msgs::msg::Marker::ADD;
        // marker_msg.scale.x = 0.05; // 포인트 크기
        // marker_msg.scale.y = 0.05;
        // marker_msg.color.r = 1.0f; // 빨간색
        // marker_msg.color.g = 0.0f;
        // marker_msg.color.b = 0.0f;
        // marker_msg.color.a = 1.0f; // 투명도

        // // Marker에 포인트 데이터 추가
        // for (const auto& pt : live_cloud.pts) {
        //     geometry_msgs::msg::Point p;
        //     p.x = pt.x;
        //     p.y = pt.y;
        //     p.z = pt.z;
        //     marker_msg.points.push_back(p);
        // }

        // // 메시지 퍼블리시
        // marker_publisher_->publish(marker_msg);

        // c_point T1 = c_point(3, 0);
        // c_point T2 = c_point(3 -2*DOCK_SIZE_X[1], 0 + 1*DOCK_SIZE_Y[1]);
        // c_point T3 = c_point(3 -2*DOCK_SIZE_X[1], 0 - 1*DOCK_SIZE_Y[1]);



        // FRAME frm = generateSampleFrame(T1,T2, T3);

        //         // 새로운 Marker 메시지로 FRAME 퍼블리시
        // visualization_msgs::msg::Marker marker_msg_2;
        // marker_msg_2.header.frame_id = "map";
        // marker_msg_2.header.stamp = this->now();
        // marker_msg_2.ns = "frame_points";
        // marker_msg_2.id = 1;
        // marker_msg_2.type = visualization_msgs::msg::Marker::POINTS;
        // marker_msg_2.action = visualization_msgs::msg::Marker::ADD;
        // marker_msg_2.scale.x = 0.03;
        // marker_msg_2.scale.y = 0.03;
        // marker_msg_2.color.r = 0.0f;
        // marker_msg_2.color.g = 0.0f;
        // marker_msg_2.color.b = 1.0f;
        // marker_msg_2.color.a = 1.0f;

        // for (const auto& pt : frm.pts) {
        //     geometry_msgs::msg::Point p;
        //     p.x = pt.x();
        //     p.y = pt.y();
        //     p.z = pt.z();
        //     marker_msg_2.points.push_back(p);
        // }

        // marker_publisher_2->publish(marker_msg_2);
        //RCLCPP_INFO(get_logger(), "now");
        // double err = frm_icp(*live_tree, live_cloud, frm, dock_tf);
    }


    XYZR_CLOUD qt_node::get_vmark_cloud()
    {
        XYZR_CLOUD res ;
        XYZR_CLOUD res_l;
        XYZR_CLOUD res_r;

        //p1 is middle of the v marker
        //p2 is the left of the v marker
        //p3 is the right of the v marker

        // c_point p1 = c_point(robot_size_x + 2*dock_size_x, 0.0);
        // c_point p2 = c_point(robot_size_x, dock_size_y);
        // c_point p3 = c_point(robot_size_x, -dock_size_y);


        c_point p1 = c_point(0.0 ,0.0);
        c_point p2 = c_point(0.0 -2*DOCK_SIZE_X[1], 0.0 + 1*DOCK_SIZE_Y[1]);
        c_point p3 = c_point(0.0 -2*DOCK_SIZE_X[1], 0.0- 1*DOCK_SIZE_Y[1]);

        res_l = generateSamplePoints(p1, p2, 30);
        res_r = generateSamplePoints(p1, p3, 30);

        res.pts.insert(res.pts.end(), res_l.pts.begin(), res_l.pts.end());
        res.pts.insert(res.pts.end(), res_r.pts.begin(), res_r.pts.end());

        return res;
    }

    XYZR_CLOUD qt_node::generateSamplePoints(const c_point& p1, const c_point& p2, int n){

    XYZR_CLOUD cloud;

    if (n < 2) {
        return cloud;
    }

    float dx = (p2.first - p1.first) / (n - 1);
    float dy = (p2.second - p1.second) / (n - 1);

    for (int i = 0; i < n; ++i) {
        PT_XYZR pt;
        pt.x = p1.first + i * dx;
        pt.y = p1.second + i * dy;
        pt.z = 0.0;  
        pt.vx = 0.0; 
        pt.vy = 0.0; 
        pt.vz = 1.0; 
        pt.r = 0.0;  
        pt.k0 = 0;  
        pt.k = 0;    
        pt.do_cnt = 0; 
        cloud.pts.push_back(pt);
    }

    return cloud;
}
        
void qt_node::log_icp(const std::string &message) {
        // 메시지를 파일로 저장
        if (icp_log_file_.is_open()) {
            icp_log_file_ << message << std::endl;
        }
}

double qt_node::frm_icp(KD_TREE_XYZR& tree, XYZR_CLOUD& cloud, FRAME& frm, Eigen::Matrix4d& G)
{
    // for processing time
    double t_st = rclcpp::Clock().now().seconds();

    // for random selection
    std::vector<int> idx_list;
    std::vector<Eigen::Vector3d> pts;
    for(size_t p = 0; p < frm.pts.size(); p++)
    {
        Eigen::Vector3d _P = frm.pts[p];
        idx_list.push_back(p);
        Eigen::Vector3d P = G.block(0,0,3,3)*_P + G.block(0,3,3,1);
        pts.push_back(P);
    }

    // solution
    Eigen::Matrix4d _G = Eigen::Matrix4d::Identity();

    // optimization param
    const int max_iter = 300; //100
    double lambda = 0.01;
    double first_err = 9999;
    double last_err = 9999;
    double convergence = 9999;
    int num_correspondence = 0;

    const double cost_threshold = 0.5 * 0.5;//*18.5;
    const int num_feature = std::min<int>(idx_list.size(), 1000);

    // for rmt
    double tm0 = 0;
    double tm1 = 0;
                                 
    int iter = 0;
    for(iter = 0; iter < max_iter; iter++)
    {
            RCLCPP_INFO(get_logger(), "iter: %d", iter);
            log_icp("Iteration: " + std::to_string(iter));
            //RCLCPP_INFO(get_logger(), "now4");
        std::vector<double> costs;
        std::vector<COST_JACOBIAN> cj_set;

        Eigen::Matrix4d cur_G = _G*G;
    
        for(size_t p = 0; p < idx_list.size(); p++)
        {
            RCLCPP_INFO(get_logger(), "p: %d", p);
            icp_log_file_ << "\n";
            log_icp("p: " + std::to_string(p));
                  //RCLCPP_INFO(get_logger(), "now10");
            // get index
            int i = idx_list[p];
                                                                                                                                                                             
            // local to global
            Eigen::Vector3d P1 = pts[i];
            log_icp("P1: " + std::to_string(P1[0]) + " " + std::to_string(P1[1]) + " " + std::to_string(P1[2]));
            Eigen::Vector3d _P1 = _G.block(0,0,3,3)*P1 + _G.block(0,3,3,1);
            Eigen::Vector3d V1 = (_P1 - cur_G.block(0,3,3,1)).normalized();
            // _G의 3x3 블록과 마지막 열 출력
            icp_log_file_ << "[";
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    icp_log_file_ << _G(i, j);
                    if (j < 2) icp_log_file_ << ", ";
                }
                icp_log_file_ << " | " << _G(i, 3) << "\n";
            }
            icp_log_file_ << "] * [" << P1[0] << ", " << P1[1] << ", " << P1[2] << "]\n";
            icp_log_file_ << "\n";
            log_icp("V1: " + std::to_string(V1[0]) + " " + std::to_string(V1[1]) + " " + std::to_string(V1[2]));
                //RCLCPP_INFO(get_logger(), "now19");
            // knn points            
            int nn_idx = 0;
            Eigen::Vector3d V0;
            Eigen::Vector3d P0(0, 0, 0);

                //RCLCPP_INFO(get_logger(), "now20");
            {
                const int pt_num = 5;
                std::vector<unsigned int> ret_near_idxs(pt_num);
                std::vector<double> ret_near_sq_dists(pt_num);

                double near_query_pt[3] = {_P1[0], _P1[1], _P1[2]};
                tree.knnSearch(&near_query_pt[0], pt_num, &ret_near_idxs[0], &ret_near_sq_dists[0]);

                nn_idx = ret_near_idxs[0];
                               //RCLCPP_INFO(get_logger(), "now21");
                V0 = Eigen::Vector3d(cloud.pts[nn_idx].vx, cloud.pts[nn_idx].vy, cloud.pts[nn_idx].vz);
                RCLCPP_INFO(get_logger(), "V0: [%f, %f, %f]", V0[0], V0[1], V0[2]);
                log_icp("V0: " + std::to_string(V0[0]) + " " + std::to_string(V0[1]) + " " + std::to_string(V0[2]));
                //RCLCPP_INFO(get_logger(), "now11");
                for(int q = 0; q < pt_num; q++)
                {
                    //RCLCPP_INFO(get_logger(), "now12");
                    int idx = ret_near_idxs[q];
                    P0 += Eigen::Vector3d(cloud.pts[idx].x, cloud.pts[idx].y, cloud.pts[idx].z);
                }
                P0 /= pt_num;
  
                RCLCPP_INFO(get_logger(), "P0: [%f, %f, %f]", P0[0], P0[1], P0[2]);
            }


            // view filter
            if(compare_view_vector(V0, V1, 150.0*0.0174533))
            {
                log_icp("View filter");
                RCLCPP_INFO(get_logger(), "view filter");
                continue;
            }

            // rmt
            double rmt = 1.0;
            if(iter >= 1)
            {
                rmt = tm1/tm0;
                if(rmt > 1.0)
                {
                    rmt = 1.0;
                }
            }
            log_icp("_P1: " + std::to_string(_P1[0]) + " " + std::to_string(_P1[1]) + " " + std::to_string(_P1[2]));
            log_icp("P0: " + std::to_string(P0[0]) + " " + std::to_string(P0[1]) + " " + std::to_string(P0[2]));
            RCLCPP_INFO(get_logger(), "P1: [%f, %f, %f]", _P1[0], _P1[1], _P1[2]);
            RCLCPP_INFO(get_logger(), "P0: [%f, %f, %f]", P0[0], P0[1], P0[2]);
            // point to point distance
            double cost = (_P1 - P0).squaredNorm();
            double cost_root = std::sqrt(cost);
            log_icp("Cost: " + std::to_string(cost));
            RCLCPP_INFO(get_logger(), "norm costroot: %f", cost_root);
            // if(cost > cost_threshold || std::abs(cost) > rmt*std::abs(cost) + 100.0) //0.01
            // {
            //     RCLCPP_INFO(get_logger(), "cost root: %f", cost_root);
            //     continue;
            // }
            if(cost > cost_threshold ) //0.01
            {
                log_icp("Cost threshold");
                RCLCPP_INFO(get_logger(), "cost root: %f", cost_root);
                continue;
            }
            // jacobian
            Eigen::Vector3d xi = TF_to_se2(_G);

            double J[3] = {0,};
            J[0] = 2.0 * (_P1[0] - P0[0]);
            J[1] = 2.0 * (_P1[1] - P0[1]);
            J[2] = 2.0 * ((_P1[0] - P0[0]) * (-std::sin(xi[2]) * P1[0] - std::cos(xi[2]) * P1[1]) + (_P1[1] - P0[1]) * (std::cos(xi[2]) * P1[0] - std::sin(xi[2]) * P1[1]));
            if(!std::isfinite(J[0]) || !std::isfinite(J[1]) || !std::isfinite(J[2]))
            {
                continue;
            }

            // // dynamic object filter
            // if(cloud.pts[nn_idx].do_cnt > 0 && cloud.pts[nn_idx].do_cnt < 3)
            // {
            //     continue;
            // }

            // additional weight
            double dist = calc_dist_2d(pts[i] - cur_G.block(0,3,3,1));
            double weight = 1.0 + 0.01*dist;

            // storing cost jacobian
            COST_JACOBIAN cj;
            cj.c = cost;
            cj.w = weight;
            memcpy(cj.J, J, sizeof(double)*3);

            cj_set.push_back(cj);
            costs.push_back(cost);

            // check num
            if((int)cj_set.size() == num_feature)
            {
                break;
            }
        }

        // num of correspondence
        num_correspondence = cj_set.size();
        log_icp("Num correspondence: " + std::to_string(num_correspondence));
        RCLCPP_INFO(get_logger(), "num_correspondence: %d", num_correspondence);
        
        if(num_correspondence < 1) //30
        {
            log_icp("Not enough correspondences");
            printf("[frm_icp] not enough correspondences, %d!!\n", num_correspondence);
            return 9999;
        }

        // calc mu
        std::nth_element(costs.begin(), costs.begin() + costs.size() / 2, costs.end());
        double mu = costs[costs.size() / 2];

        // calc sigma
        std::vector<double> vars(costs.size());
        for (size_t p = 0; p < costs.size(); p++)
        {
            vars[p] = std::abs(costs[p] - mu);
        }
        std::nth_element(vars.begin(), vars.begin() + vars.size() / 2, vars.end());
        double sigma = vars[vars.size() / 2];
        if(sigma < 0.001)
        {
            sigma = 0.001;
        }

        // calc weight
        for (size_t p = 0; p < cj_set.size(); p++)
        {
            double c = cj_set[p].c;
            double V0 = 30;
            double w = (V0 + 1.0) / (V0 + ((c - mu) / sigma) * ((c - mu) / sigma));
            cj_set[p].w *= w;
        }

        // make matrix
        double _A[3*3] = { 0, };        
        double _b[3] = { 0, };
        double err = 0;
        double err_cnt = 0;
        for (size_t p = 0; p < cj_set.size(); p++)
        {
            double c = cj_set[p].c;
            double* J = cj_set[p].J;
            double w = cj_set[p].w;

            // set tempolar matrix(col major)
            for (int y = 0; y < 3; y++)
            {
                for (int x = 0; x < 3; x++)
                {
                    _A[y * 3 + x] += w * J[y] * J[x];
                }
                _b[y] += w * c * J[y];
            }

            // error
            //err += std::abs(w*c);
            err += std::sqrt(std::abs(c));
            err_cnt += w;
        }

        //RCLCPP_INFO(get_logger(), "now3");
        err /= err_cnt;

        // set first error
        if(iter == 0)
        {
            first_err = err;
        }

        // solve
        Eigen::Matrix<double, 3, 3> A(_A);
        A += 1e-6*Eigen::Matrix<double, 3, 3>::Identity();

        Eigen::Matrix<double, 3, 1> b(_b);
        Eigen::Matrix<double, 3, 3> diag_A = A.diagonal().asDiagonal();
        Eigen::Matrix<double, 3, 1> X = (-(A + lambda * diag_A)).ldlt().solve(b);

        // lambda update
        if(err < last_err)
        {
            lambda *= 0.1;
        }
        else
        {
            lambda *= std::pow(2, iter);
        }
        last_err = err;

        // pose update
        Eigen::Vector3d xi;
        xi[0] = X(0, 0);
        xi[1] = X(1, 0);
        xi[2] = X(2, 0);

        _G = se2_to_TF(xi)*_G;
        refine_pose(_G);

        // for rmt
        tm0 = tm1;
        tm1 = _G.block(0,3,3,1).norm();

        // convergence check
        convergence = X.cwiseAbs().maxCoeff();
        if(convergence < 1e-7)
        {
            break;
        }

           // 현재 변환 행렬에 따른 cur_frame 포인트 변환 및 시각화
            visualization_msgs::msg::Marker marker_msg;
            marker_msg.header.frame_id = "map";
            marker_msg.header.stamp = this->now();
            marker_msg.ns = "icp_iterations";
            marker_msg.id = iter;  // 각 반복에 고유한 ID 부여
            marker_msg.type = visualization_msgs::msg::Marker::POINTS;
            marker_msg.action = visualization_msgs::msg::Marker::ADD;
            marker_msg.scale.x = 0.02;
            marker_msg.scale.y = 0.02;
            marker_msg.color.r = 0.0f;
            marker_msg.color.g = 1.0f - (float)iter / max_iter;
            marker_msg.color.b = (float)iter / max_iter;
            marker_msg.color.a = 1.0f;

            // 현재 반복에서 포인트 변환
            Eigen::Matrix4d TEMP_G = _G * G;
            for(const auto& pt : frm.pts)
            {
                // 3D 포인트를 변환
                Eigen::Vector4d pt_h(pt.x(), pt.y(), pt.z(), 1.0);
                Eigen::Vector4d pt_transformed = TEMP_G * pt_h;

                geometry_msgs::msg::Point p;
                p.x = pt_transformed[0];
                p.y = pt_transformed[1];
                p.z = pt_transformed[2];
                marker_msg.points.push_back(p);
            }

            // Marker 퍼블리시
            marker_publisher_->publish(marker_msg);


    }

    // update
    G = _G*G;

    if(last_err > first_err+0.01 || last_err > 0.2)
    {

        return 9999;
    }

    return last_err;
}


bool qt_node::compare_view_vector(Eigen::Vector3d V0, const Eigen::Vector3d V1, double threshold)
{
    double angle = std::acos(V0.dot(V1));
    if (angle > threshold)
    {
        return true;
    }
    return false;
}

double qt_node::calc_dist_2d(Eigen::Vector3d P)
{
    return std::sqrt(P[0]*P[0] + P[1]*P[1]);
}

Eigen::Matrix4d qt_node::se2_to_TF(Eigen::Vector3d xi)
{
    Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();

    tf(0, 3) = xi[0];
    tf(1, 3) = xi[1];

    double rz = xi[2];
    tf(0, 0) = std::cos(rz);
    tf(0, 1) = -std::sin(rz);
    tf(1, 0) = std::sin(rz);
    tf(1, 1) = std::cos(rz);

    return tf;
}


void qt_node::refine_pose(Eigen::Matrix4d& G)
{

    // Extract the rotation part and use Singular Value Decomposition (SVD) to ensure it's orthonormal
    Eigen::Matrix3d R = G.block<3,3>(0,0);

    // Use SVD to find the nearest orthogonal matrix
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    R = svd.matrixU() * svd.matrixV().transpose();

    // Ensure the rotation matrix has a positive determinant (to avoid reflections)
    if (R.determinant() < 0)
    {
        R = -R;
    }

    // Update G's rotation part
    G.block<3,3>(0,0) = R;

    //G = Sophus::SE3d::fitToSE3(G).matrix();
}

Eigen::Vector3d qt_node::TF_to_se2(Eigen::Matrix4d tf)
{
    Eigen::Vector3d t = tf.block<3,1>(0,3);
    double rz = std::atan2(tf(1, 0), tf(0, 0));

    Eigen::Vector3d res;
    res[0] = t[0];
    res[1] = t[1];
    res[2] = rz;
    return res;
}


double qt_node::kfrm_icp(KFRAME& frm0, KFRAME& frm1, Eigen::Matrix4d& dG)
{
    // for processing time
    double t_st = rclcpp::Clock().now().seconds();

    // build kd_tree
    XYZR_CLOUD cloud;
    for(size_t p = 0; p < frm0.pts.size(); p++)
    {
        cloud.pts.push_back(frm0.pts[p]);
    }

    KD_TREE_XYZR tree(3, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    tree.buildIndex();

    // points random selection
    std::vector<int> idx_list;
    std::vector<PT_XYZR> pts;
    for(size_t p = 0; p < frm1.pts.size(); p++)
    {
        idx_list.push_back(p);

        Eigen::Vector3d P(frm1.pts[p].x, frm1.pts[p].y, frm1.pts[p].z);
        Eigen::Vector3d _P = dG.block(0,0,3,3)*P + dG.block(0,3,3,1);

        PT_XYZR pt = frm1.pts[p];
        pt.x = _P[0];
        pt.y = _P[1];
        pt.z = _P[2];
        pts.push_back(pt);
    }

    // solution
    Eigen::Matrix4d _dG = Eigen::Matrix4d::Identity();

    // optimization param
    const int max_iter = 100;
    double lambda = 0.05; //0.01;
    double first_err = 9999;
    double last_err = 9999;
    double convergence = 9999;
    int num_correspondence = 0;

    const double cost_threshold = 3.0*3.0;//0.5 * 0.5;//00000.0;
    const int num_feature = std::min<int>(idx_list.size(),1000);

    // for rmt
    double tm0 = 0;
    double tm1 = 0;

    int iter = 0;
    for(iter = 0; iter < max_iter; iter++)
    {
        RCLCPP_INFO(get_logger(), "iter: %d", iter);
        log_icp("Iteration: " + std::to_string(iter));

        std::shuffle(idx_list.begin(), idx_list.end(), std::default_random_engine());

        std::vector<double> costs;
        std::vector<COST_JACOBIAN> cj_set;

        for(size_t p = 0; p < idx_list.size(); p++)
        {
            RCLCPP_INFO(get_logger(), "p: %d", p);
            icp_log_file_ << "\n";
            log_icp("p: " + std::to_string(p));
            // get index
            int i = idx_list[p];

            // local to global
            Eigen::Vector3d P1(pts[i].x, pts[i].y, pts[i].z);
            //log_icp("P1: " + std::to_string(P1[0]) + " " + std::to_string(P1[1]) + " " + std::to_string(P1[2]));
            Eigen::Vector3d _P1 = _dG.block(0,0,3,3)*P1 + _dG.block(0,3,3,1);
            // log_icp("_P1: "+ std::to_string(_P1[0]) + " " + std::to_string(_P1[1]) + " " + std::to_string(_P1[2]));

            // icp_log_file_ << "[";
            // for (int i = 0; i < 3; ++i) {
            //     for (int j = 0; j < 3; ++j) {
            //         icp_log_file_ << _dG(i, j);
            //         if (j < 2) icp_log_file_ << ", ";
            //     }
            //     icp_log_file_ << " | " << _dG(i, 3) << "\n";
            // }
            // icp_log_file_ << "] * [" << P1[0] << ", " << P1[1] << ", " << P1[2] << "]\n";
            // icp_log_file_ << "\n";
            // knn points
            int nn_idx = 0;
            Eigen::Vector3d P0(0, 0, 0);
            {
                const int pt_num = 5;
                std::vector<unsigned int> ret_near_idxs(pt_num);
                std::vector<double> ret_near_sq_dists(pt_num);

                double near_query_pt[3] = {_P1[0], _P1[1], _P1[2]};
                tree.knnSearch(&near_query_pt[0], pt_num, &ret_near_idxs[0], &ret_near_sq_dists[0]);

                nn_idx = ret_near_idxs[0];
                for(int q = 0; q < pt_num; q++)
                {
                    int idx = ret_near_idxs[q];
                    P0 += Eigen::Vector3d(cloud.pts[idx].x, cloud.pts[idx].y, cloud.pts[idx].z);
                }
                P0 /= pt_num;
            }

            // rmt
            double rmt = 1.0;
            if(iter >= 1)
            {
                rmt = tm1/tm0;
                if(rmt > 1.0)
                {
                    rmt = 1.0;
                }
            }

            // point to point distance
            // log_icp("_P1: " + std::to_string(_P1[0]) + " " + std::to_string(_P1[1]) + " " + std::to_string(_P1[2]));
            // log_icp("P0: " + std::to_string(P0[0]) + " " + std::to_string(P0[1]) + " " + std::to_string(P0[2]));
            double cost = (_P1 - P0).squaredNorm();

            log_icp("Cost: " + std::to_string(cost));
            if(cost > cost_threshold)
            {
                log_icp("Cost threshold");
                continue;
            }

            // if(cost > cost_threshold || std::abs(cost) > rmt*std::abs(cost) + 0.01)
            // {
            //     log_icp("Cost threshold");
            //     continue;
            // }

            // jacobian
            Eigen::Vector3d xi = TF_to_se2(_dG);

            double J[3] = {0,};
            J[0] = 2.0 * (_P1[0] - P0[0]);
            J[1] = 2.0 * (_P1[1] - P0[1]);
            J[2] = 2.0 * ((_P1[0] - P0[0]) * (-std::sin(xi[2]) * P1[0] - std::cos(xi[2]) * P1[1]) + (_P1[1] - P0[1]) * (std::cos(xi[2]) * P1[0] - std::sin(xi[2]) * P1[1]));
            if(!std::isfinite(J[0]) || !std::isfinite(J[1]) || !std::isfinite(J[2]))
            {
                log_icp("J is not finite");
                continue;
            }

            //log_icp("J: " + std::to_string(J[0]) + " " + std::to_string(J[1]) + " " + std::to_string(J[2]));
            // // dynamic object filter
            // if(cloud.pts[nn_idx].do_cnt < 3)
            // {
            //     continue;
            // }

            // additional weight
            double weight = 1.0;

            // storing cost jacobian
            COST_JACOBIAN cj;
            cj.c = cost;
            cj.w = weight;
            memcpy(cj.J, J, sizeof(double)*3);

            cj_set.push_back(cj);
            costs.push_back(cost);

            // check num
            if((int)cj_set.size() == num_feature)
            {
                break;
            }
        }

        // num of correspondence
        num_correspondence = cj_set.size();
        log_icp("Num correspondence: " + std::to_string(num_correspondence));
        if(num_correspondence < 1)
        {
            printf("[frm_icp] not enough correspondences, %d!!\n", num_correspondence);
            return 9999;
        }

        // calc mu
        std::nth_element(costs.begin(), costs.begin() + costs.size() / 2, costs.end());
        double mu = costs[costs.size() / 2];

        // calc sigma
        std::vector<double> vars(costs.size());
        for (size_t p = 0; p < costs.size(); p++)
        {
            vars[p] = std::abs(costs[p] - mu);
        }
        std::nth_element(vars.begin(), vars.begin() + vars.size() / 2, vars.end());
        double sigma = vars[vars.size() / 2];
        if(sigma < 0.001)
        {
            sigma = 0.001;
        }

        // calc weight
        for (size_t p = 0; p < cj_set.size(); p++)
        {
            double c = cj_set[p].c;
            double V0 = 30;
            double w = (V0 + 1.0) / (V0 + ((c - mu) / sigma) * ((c - mu) / sigma));
            cj_set[p].w *= w;
        }

        // make matrix
        double _A[3*3] = { 0, };
        double _b[3] = { 0, };
        double err = 0;
        double err_cnt = 0;
        for (size_t p = 0; p < cj_set.size(); p++)
        {
            double c = cj_set[p].c;
            double* J = cj_set[p].J;
            double w = cj_set[p].w;

            // set tempolar matrix(col major)
            for (int y = 0; y < 3; y++)
            {
                for (int x = 0; x < 3; x++)
                {
                    _A[y * 3 + x] += w * J[y] * J[x];
                }
                _b[y] += w * c * J[y];
            }

            // error
            //err += std::abs(w*c);
            err += std::sqrt(std::abs(c));
            err_cnt += w;
        }
        err /= err_cnt;

        // set first error
        if(iter == 0)
        {
            first_err = err;
        }

        // solve
        Eigen::Matrix<double, 3, 3> A(_A);
        A += 1e-6*Eigen::Matrix<double, 3, 3>::Identity();

        Eigen::Matrix<double, 3, 1> b(_b);
        Eigen::Matrix<double, 3, 3> diag_A = A.diagonal().asDiagonal();
        Eigen::Matrix<double, 3, 1> X = (-(A + lambda * diag_A)).ldlt().solve(b);

        // lambda update
        if(err < last_err)
        {
            lambda *= 0.1;
        }
        else
        {
            lambda *= std::pow(2, iter);
        }
        last_err = err;

        // pose update
        Eigen::Vector3d xi;
        xi[0] = X(0, 0);
        xi[1] = X(1, 0);
        xi[2] = X(2, 0);

        _dG = se2_to_TF(xi)*_dG;
        refine_pose(_dG);

        // for rmt
        tm0 = tm1;
        tm1 = _dG.block(0,3,3,1).norm();

        // convergence check
        convergence = X.cwiseAbs().maxCoeff();
        if(convergence < 1e-7)
        {
            break;
        }
    }
           // 현재 변환 행렬에 따른 cur_frame 포인트 변환 및 시각화
            visualization_msgs::msg::Marker marker_msg;
            marker_msg.header.frame_id = "map";
            marker_msg.header.stamp = this->now();
            marker_msg.ns = "icp_iterations";
            marker_msg.id = iter;  // 각 반복에 고유한 ID 부여
            marker_msg.type = visualization_msgs::msg::Marker::POINTS;
            marker_msg.action = visualization_msgs::msg::Marker::ADD;
            marker_msg.scale.x = 0.02;
            marker_msg.scale.y = 0.02;
            marker_msg.color.r = 0.0f;
            marker_msg.color.g = 1.0f - (float)iter / max_iter;
            marker_msg.color.b = (float)iter / max_iter;
            marker_msg.color.a = 1.0f;

            // 현재 반복에서 포인트 변환
            Eigen::Matrix4d TEMP_G = _dG * dG;
            for(const auto& pt : frm1.pts)
            {
                // 3D 포인트를 변환
                Eigen::Vector4d pt_h(pt.x, pt.y, pt.z, 1.0);
                Eigen::Vector4d pt_transformed = TEMP_G * pt_h;

                geometry_msgs::msg::Point p;
                p.x = pt_transformed[0];
                p.y = pt_transformed[1];
                p.z = pt_transformed[2];
                marker_msg.points.push_back(p);
            }

            // Marker 퍼블리시
            marker_publisher_->publish(marker_msg);

    // update
    dG = _dG*dG;

    // check
    if(last_err > first_err+0.01 || last_err > 0.2)
    {
        return 9999;
    }
    return last_err;
}



FRAME qt_node::generateSampleFrame(const c_point& p1, const c_point& p2, const c_point& p3)
{

    // Gaussian noise generator
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0, 0.01);

    FRAME frame;
    // Generate left side points from p1 to p2
    XYZR_CLOUD left_cloud = generateSamplePoints(p1, p2, 30);
    // Generate right side points from p1 to p3
    XYZR_CLOUD right_cloud = generateSamplePoints(p1, p3, 30);

    // Combine points into the FRAME structure
    for (const auto& pt : left_cloud.pts) {
        double noise_x = distribution(generator);
        double noise_y = distribution(generator);
        double noise_z = distribution(generator);
        frame.pts.push_back(Eigen::Vector3d(pt.x + noise_x, pt.y +noise_y, pt.z));
    }
    for (const auto& pt : right_cloud.pts) {
        double noise_x = distribution(generator);
        double noise_y = distribution(generator);
        double noise_z = distribution(generator);
        frame.pts.push_back(Eigen::Vector3d(pt.x + noise_x, pt.y + noise_y, pt.z));
    }

    return frame;
}


KFRAME qt_node::generateSampleKFrame(const c_point& p1, const c_point& p2, const c_point& p3)
{
    // Gaussian noise generator
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0, 0.01);

    KFRAME frame;
    // Generate left side points from p1 to p2
    XYZR_CLOUD left_cloud = generateSamplePoints(p1, p2, 30);
    // Generate right side points from p1 to p3
    XYZR_CLOUD right_cloud = generateSamplePoints(p1, p3, 30);

    // Combine points into the FRAME structure
    for (auto& pt : left_cloud.pts) {
        double noise_x = distribution(generator);
        double noise_y = distribution(generator);
        double noise_z = distribution(generator);
        pt.x += noise_x;
        pt.y += noise_y;
        pt.z = 0.0;
        frame.pts.push_back(pt);
    }
    for (auto& pt : right_cloud.pts) {
        double noise_x = distribution(generator);
        double noise_y = distribution(generator);
        double noise_z = distribution(generator);
        pt.x += noise_x;
        pt.y += noise_y;
        pt.z = 0.0;
        frame.pts.push_back(pt);
    }

    return frame;

}


KFRAME qt_node::generateVKFrame()
{

        KFRAME frame;
        XYZR_CLOUD res ;
        XYZR_CLOUD res_l;
        XYZR_CLOUD res_r;

        //p1 is middle of the v marker
        //p2 is the left of the v marker
        //p3 is the right of the v marker

        // c_point p1 = c_point(robot_size_x + 2*dock_size_x, 0.0);
        // c_point p2 = c_point(robot_size_x, dock_size_y);
        // c_point p3 = c_point(robot_size_x, -dock_size_y);


        c_point p1 = c_point(0.0 ,0.0);
        c_point p2 = c_point(0.0 -2*DOCK_SIZE_X[1], 0.0 + 1*DOCK_SIZE_Y[1]);
        c_point p3 = c_point(0.0 -2*DOCK_SIZE_X[1], 0.0- 1*DOCK_SIZE_Y[1]);

        res_l = generateSamplePoints(p1, p2, 30);
        res_r = generateSamplePoints(p1, p3, 30);

        for (const auto& pt : res_l.pts) {
            frame.pts.push_back(pt);
        }
        for (const auto& pt : res_r.pts) {
            frame.pts.push_back(pt);
        }
        return frame;

}

void qt_node::publishKFrameMarker(const KFRAME& kframe, int marker_id, const std::string& ns, float r, float g, float b)
{
    visualization_msgs::msg::Marker marker_msg;
    marker_msg.header.frame_id = "map";
    marker_msg.header.stamp = this->now();
    marker_msg.ns = ns;
    marker_msg.id = marker_id;
    marker_msg.type = visualization_msgs::msg::Marker::POINTS;
    marker_msg.action = visualization_msgs::msg::Marker::ADD;
    marker_msg.scale.x = 0.03; // 포인트 크기
    marker_msg.scale.y = 0.03;
    marker_msg.color.r = r; 
    marker_msg.color.g = g;
    marker_msg.color.b = b;
    marker_msg.color.a = 1.0f; // 불투명도

    // KFRAME의 포인트 추가
    for (const auto& pt : kframe.pts) {
        geometry_msgs::msg::Point p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        marker_msg.points.push_back(p);
    }

    marker_publisher_2->publish(marker_msg);
}


}  // namespace qt_node