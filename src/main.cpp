#include <ecn_sensorbased/pioneer_cam.h>
#include <visp/vpFeaturePoint.h>
#include <ecn_common/vpQuadProg.h>
#include <ecn_common/visp_utils.h>

using namespace std;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_node");
    PioneerCam robot;

    // pose error gain
    const double lv = .5;
    // constraints gain
    const double lc = 2;
    geometry_msgs::Pose2D target;

    int it = 0;
    vpColVector v(4);

    // QP solver
    vpQuadProg qp;
    vpMatrix Q, C(8,4), A(1,4);
    Q.eye(4);
    Q[2][2] = Q[3][3] = 0;
    A[0][2] = A[0][3] = 0;

    vpMatrix J2D(2,4), Jinv_robot(2,2);

    vpColVector r(4, 0.0), b(1, 0.0), s(2,0.0), d(8,0.0), camLim(2,0.0);

    vpFeaturePoint p;


    while(ros::ok())
    {
        it++;
        cout << "-------------" << endl;

        if(robot.ok())
        {
            // get robot and target positions to get position error
            target = robot.getTargetRelativePose();

            // linear velocity
            v[0] = lv*(target.x - .1);
            // angular velocity
            v[1] = 10*lv*(fmod(atan2(target.y, target.x)+M_PI, 2*M_PI) - M_PI);

            cout << "v: " << v.t() << endl;

            //Definition r as in Qx=r
            r[0]    = v[0];
            r[1]    = v[1];
            //Definition of A as in Ax=b
            A[0][0] = v[1];//w*
            A[0][1] =-v[0];//-v*

            //Construcion Q
            robot.getImagePoint(s);
            p.buildFrom(s[0],s[1],1);
            J2D = p.interaction()*robot.getCamJacobian();

            Jinv_robot[0][0] = Jinv_robot[1][0] = 1/robot.radius();
            Jinv_robot[0][1] = robot.base()/robot.radius();
            Jinv_robot[1][1] = - Jinv_robot[0][1];

            ecn::putAt(C,J2D,0,0);
            ecn::putAt(C,J2D,2,0);
            ecn::putAt(C,Jinv_robot,4,0);
            ecn::putAt(C,-Jinv_robot,6,0);

            //Construction d
            camLim = robot.getCamLimits();
            ecn::putAt(d,(lc*(camLim-s)),0);
            ecn::putAt(d,(lc*(s+camLim)),2);
            for(int i=0;i<4;i++)
                d[i+3] = robot.wmax();
            //Solving system
            qp.solveQP(Q,r,A,b,C,d,v);
            //Sending setpoint
            cout << "v: " << v.t() << endl;
            robot.setVelocity(v);

            robot.radius();
            robot.base();
            robot.wmax();
        }
    }
}
