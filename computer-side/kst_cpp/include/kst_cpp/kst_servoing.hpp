#pragma once

struct KstServoingRobotConst{
	// type of the robot
    const int LBR7R800=1;
    const int LBR14R820=2;
    // height of flange (unit meter), taken from iiwa data sheets
    // Manual: 
    // [1] Medien-Flansch
    // Für Produktfamilie LBR iiwa
    // Montage- und Betriebsanleitung
    const double MEDIEN_FLANSCH_ELEKTRISCH= 0.035;
    const double MEDIEN_FLANSCH_PNEUMATISCH= 0.035;
    const double MEDIEN_FLANSCH_IO_PNEUMATISCH= 0.035;
    const double MEDIEN_FLANSCH_TOUCH_PNEUMATISCH= 0.061;
    const double MEDIEN_NONE=0.0;
};

struct KstInertia{
    std::vector<double> m;
    std::vector<std::vector<double>> pcii;
    std::vector<std::vector<std::vector<double>>> I;
};

struct KstDH{
    std::vector<double> alpha;
    std::vector<double> d;
    std::vector<double> a;
};

struct KstForwardKinematics{

};

struct KstInverseKinematics{

};

struct KstForwardDynamics{

};

struct KstInverseDynamics{

};

class KstServoing{

private:

    // connection
    std::string ip_;
    tcp::socket tcp_sock_; // tcpip connection object
    boost::array<char, 128> buf_;

    //robot data
    KstInertia data_I_; // inertial data of the robot
    KstDH data_dh_; // DH parameters of the robot, combination
    std::string robot_type_;
    int flange_type_;

    // tf2::Transform teftool_ = tf2::Transform( // end effector tranform
    // 	tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(0.0, 0.0, 0.0)); 

public:

    // constructor
    KSTServoing(
        std::string robot_ip, 
        int robot_type, 
        double h_flange, 
        boost::asio::io_context& io_context,
        );
    
    // PTP motion
    bool PTPJointSpace(std::vector<double> jpos , double relVel);
    bool PTPLineEEF(std::vector<double> epos, double vel); // vel: mm/sec

    // Smart and direct servo methods
    void ServoDirectCartesianStart();
    void ServoDirectJointStart();
    void ServoSmartCartesianStart();
    void ServoSmartJointStart();
    void ServoStop();

    void ServoSendJoints(std::vector<double> jp);
    std::vector<double> ServoSendJointsGetFeedback(std::vector<double> jp);
    void ServoSendEEF(std::vector<double> eef); // x y z rz ry rx
    std::vector<double> ServoSendEEFGetFeedback(std::vector<double> eef); // x y z rz ry rx

    // getters
    std::vector<double> GetJointPosition();
    std::vector<double> GetEEFPosition();
        
    // networking
    bool NetEstablishConnection();
    void NetTurnoffServer();

    // utility/member methods
    // std::vector<std::vector<double>> utl_centrifugal_matrix();
    // std::vector<std::vector<double>> utl_coriolis_matrix();
    // std::vector<std::vector<double>> utl_mass_matrix();
    // std::vector<std::vector<double>> utl_nullspace_matrix();
    // std::vector<std::vector<double>> utl_jacobian_matrix();
    // std::vector<std::vector<double>> utl_dh_matrix();
    // std::vector<double> utl_gravity_vector();
    // KSTServoingDynamicsForward utl_forward_dynamics();
    // KSTServoingDynamicsInverse utl_inverse_dynamics();
    // KSTServoingKinematicsForward utl_forward_kinematics();
    // KSTServoingKinematicsInverse utl_inverse_kinematics();
    // KstDH utl_dh_parameters(int robot_type);
    // KstInertia utl_inertial_parameters(int robot_type);

    // flags
    bool flg_realtimectl_ = false;
};