#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"  // 추가
#include "nanoflann.hpp"
#include <Eigen/Dense>
#include <cstring>
#include <cmath>
#include <random>
#include <fstream>
// tree typedef

struct PT_XYZR
{
    double x = 0;    // x coordinates
    double y = 0;    // y coordinates
    double z = 0;    // z coordinates
    double vx = 0;   // view vector x
    double vy = 0;   // view vector y
    double vz = 0;   // view vector z
    double r = 0;    // reflect 0~1
    int k0 = 0;      // original add cnt
    int k = 0;       // add cnt of tree
    int do_cnt = 0;  // dynamic object count
};



struct KFRAME
{
    int id = 0;
    std::vector<PT_XYZR> pts;
    Eigen::Matrix4d G;
    Eigen::Matrix4d opt_G;

    KFRAME()
    {
        id = 0;
        G.setIdentity();
        opt_G.setIdentity();
    }

    KFRAME(const KFRAME& p)
    {
        id = p.id;
        pts = p.pts;
        G = p.G;
        opt_G = p.opt_G;
    }

    KFRAME& operator=(const KFRAME& p)
    {
        id = p.id;
        pts = p.pts;
        G = p.G;
        opt_G = p.opt_G;
        return *this;
    }
};


struct FRAME
{
    double t = 0;    
    std::vector<double> reflects;
    std::vector<Eigen::Vector3d> pts;

    FRAME()
    {
    }
    FRAME(const FRAME& p)
    {
        t = p.t;        
        reflects = p.reflects;
        pts = p.pts;

    }
    FRAME& operator=(const FRAME& p)
    {
        t = p.t;        
        reflects = p.reflects;
        pts = p.pts;

        return *this;
    }
};


struct XYZR_CLOUD
{
    std::vector<PT_XYZR> pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        if (dim == 0) return pts[idx].x;
        else if (dim == 1) return pts[idx].y;
        else return pts[idx].z;
    }

    // Optional: computation of bounding box
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
};

    typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, XYZR_CLOUD>, XYZR_CLOUD, 3> KD_TREE_XYZR;
    
    typedef std::pair<float, float> c_point;
    typedef std::vector<c_point> c_pointList;
    struct Point {
    double x, y, theta; // x, y 좌표와 방향 (theta)
    Point(double _x, double _y) : x(_x), y(_y), theta(0.0) {}
    };


struct COST_JACOBIAN
{
    double c = 0;
    double w = 0;
    double J[12] = {0,};

    COST_JACOBIAN(){}

    COST_JACOBIAN(const COST_JACOBIAN& p)
    {
        c = p.c;
        w = p.w;
        memcpy(J, p.J, sizeof(double)*12);
    }

    COST_JACOBIAN& operator=(const COST_JACOBIAN& p)
    {
        c = p.c;
        w = p.w;
        memcpy(J, p.J, sizeof(double)*12);
        return *this;
    }
};




namespace qt_test
{

    
    class qt_node : public rclcpp::Node{

        public:
            explicit qt_node (const rclcpp::NodeOptions & options);

            double DOCK_SIZE_X[2] = {-0.03, 0.03};
            double DOCK_SIZE_Y[2] = {-0.15, 0.15};
            // double DOCK_SIZE_X[2] = {-2.0, 2.0};
            // double DOCK_SIZE_Y[2] = {-3.0, 3.0};    
            double DOCK_ANGLE = 0.0;
            double DOCK_X = 3.0;
            double DOCK_Y = 0.0;
            //for ICP
            XYZR_CLOUD get_vmark_cloud();
            XYZR_CLOUD live_cloud;
            XYZR_CLOUD vmark_cloud;
            KD_TREE_XYZR *live_tree = NULL;
            Eigen::Matrix4d dock_tf;
            //for utils
            XYZR_CLOUD generateSamplePoints(const c_point&, const c_point&, int n);
            bool compare_view_vector(Eigen::Vector3d V0, const Eigen::Vector3d V1, double threshold);
            double calc_dist_2d(Eigen::Vector3d P);
            Eigen::Matrix4d se2_to_TF(Eigen::Vector3d xi);
            void refine_pose(Eigen::Matrix4d& G);
            Eigen::Vector3d TF_to_se2(Eigen::Matrix4d tf);
            double frm_icp(KD_TREE_XYZR& tree, XYZR_CLOUD& cloud, FRAME& frm, Eigen::Matrix4d& G);
            double kfrm_icp(KFRAME&, KFRAME&, Eigen::Matrix4d&);
            FRAME generateSampleFrame(const c_point& p1, const c_point& p2, const c_point& p3);
            void log_icp(const std::string &message); 
            KFRAME generateSampleKFrame(const c_point& p1, const c_point& p2, const c_point& p3);
            KFRAME generateVKFrame();
            void publishKFrameMarker(const KFRAME& kframe, int marker_id, const std::string& ns, float r, float g, float b);
            std::ofstream icp_log_file_; 
            void icp_go();

            double sample_noise = 0.01;
            int ICP_MAX_FEATURE_NUM = 1000;
            double ICP_COST_THRESHOLD = 3.0;
            int ICP_CORRESPONDENCE_THRESHOLD = 10;
        private:
            void timerCallback();
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_; // Marker 퍼블리셔
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_2; // Marker 퍼블리셔

    
    };




}