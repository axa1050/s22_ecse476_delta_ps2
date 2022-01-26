
#include <ros/ros.h>
#include <double_vec_srv/DblVecSrv.h>
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "heading_test_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<double_vec_srv::DblVecSrv>("stdr_rotation_service");
    double_vec_srv::DblVecSrv srv;
    double desired_heading;
    while(ros::ok()) {
        cout<<"enter a desired heading: ";
        cin>>desired_heading;
        srv.request.vec_of_doubles.resize(1);
        srv.request.vec_of_doubles[0]=desired_heading;
        //alt:
        //srv.request.vec_of_doubles.clear();
        //srv.request.vec_of_doubles.push_back(desired_heading);
        client.call(srv);

    }


    return 0;
}
