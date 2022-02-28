#pragma once
#include <boost/asio.hpp>

// struct KstServoingRobotConst{
// 	// type of the robot
//     const int LBR7R800=1;
//     const int LBR14R820=2;
//     // height of flange (unit meter), taken from iiwa data sheets
//     // Manual: 
//     // [1] Medien-Flansch
//     // Für Produktfamilie LBR iiwa
//     // Montage- und Betriebsanleitung
//     const double MEDIEN_FLANSCH_ELEKTRISCH= 0.035;
//     const double MEDIEN_FLANSCH_PNEUMATISCH= 0.035;
//     const double MEDIEN_FLANSCH_IO_PNEUMATISCH= 0.035;
//     const double MEDIEN_FLANSCH_TOUCH_PNEUMATISCH= 0.061;
//     const double MEDIEN_NONE=0.0;
// };

class KstServoing{

private:

    // connection
    std::string ip_;
    tcp::socket tcp_sock_; // tcpip connection object
    boost::array<char, 128> buf_;

    // robot data
    std::string robot_type_;
    int flange_type_;

    // flags
    bool f_sft_real_time_ = false; // status of soft real time

public:

    // constructor
    KSTServoing(
        std::string robot_ip, 
        boost::asio::io_context& io_context,
        );
    
    // PTP motion
    void PTPJointSpace(std::vector<double> jpos , double relVel);
    void PTPLineEEF(std::vector<double> epos, double vel); // vel: mm/sec

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
    void NetEstablishConnection();
    void NetTurnoffServer();

};