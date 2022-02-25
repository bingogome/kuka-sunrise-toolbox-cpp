#include <kst_servoing.hpp>

KstServoing::KstServoing(
	std::string robot_ip, 
	int robot_type, double h_flange, 
	boost::asio::io_context& io_context)
	: 
	tcp_sock_(io_context)
{
	ip_ = robot_ip;
	tcp::endpoint remote_endpoint = tcp::endpoint(boost::asio::ip::address_v4::from_string(robot_ip), 30001);
}

void KstServoing::NetEstablishConnection()
{
	tcp_sock_.connect(remote_endpoint);


	boost::system::error_code error;

	// Msg is the tool transformation (may need to edit. leave it for now
	// currently just calculate from outside and send the command that only
	// regards to the end-effector of the robot)
	const std::string msg1 = "TFtrans_0.0_0.0_0.0_0.0_0.0_0.0\n";
	size_t lenmsgr1;

	// TODO: success connection when received 
	while()
	{
		boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);
		lenmsgr1 = tcp_sock_.read_some(boost::asio::buffer(buf_));
	}
}

void KstServoing::PTPJointSpace(std::vector<double> jpos , double relVel)
{
	try
	{
		boost::system::error_code error;

		const std::string msg1 = "jRelVel_" + 
			boost::lexical_cast<std::string>(relVel) + "_\n";
		boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);
		size_t lenmsgr1 = tcp_sock_.read_some(boost::asio::buffer(buf_), error);

		const std::string msgjs = "jp_" + 
			formated_double_2_string(jpos[0], 5) + "_" +
			formated_double_2_string(jpos[1], 5) + "_" +
			formated_double_2_string(jpos[2], 5) + "_" +
			formated_double_2_string(jpos[3], 5) + "_" +
			formated_double_2_string(jpos[4], 5) + "_" +
			formated_double_2_string(jpos[5], 5) + "_" +
			formated_double_2_string(jpos[6], 5) + "_\n";
		boost::asio::write(tcp_sock_, boost::asio::buffer(msgjs), error);
		size_t lenmsgrjs = tcp_sock_.read_some(boost::asio::buffer(buf_), error);

		const std::string msg2 = "doPTPinJS\n";
		boost::asio::write(tcp_sock_, boost::asio::buffer(msg2), error);
		size_t lenmsgr2 = tcp_sock_.read_some(boost::asio::buffer(buf_), error);

		size_t lenmsgr3 = tcp_sock_.read_some(boost::asio::buffer(buf_), error);

	}
	catch(std::exception& e)
	{
		throw;
	}
}

void KstServoing::PTPLineEEF(std::vector<double> epos, double vel)
{// vel: mm/sec
	try
	{
		boost::system::error_code error;

		const std::string msg1 = "jRelVel_" + 
			boost::lexical_cast<std::string>(vel) + "_\n";
		boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);
		size_t lenmsgr1 = tcp_sock_.read_some(boost::asio::buffer(buf_), error);

		const std::string msgjs = "cArtixanPosition_" +
			std::to_string(epos[0]) + "_" +
			std::to_string(epos[1]) + "_" +
			std::to_string(epos[2]) + "_" +
			std::to_string(epos[3]) + "_" +
			std::to_string(epos[4]) + "_" +
			std::to_string(epos[5]) + "_\n";
		boost::asio::write(tcp_sock_, boost::asio::buffer(msgjs), error);
		size_t lenmsgrjs = tcp_sock_.read_some(boost::asio::buffer(buf_), error);

		const std::string msg2 = "doPTPinCS\n"; 
		boost::asio::write(tcp_sock_, boost::asio::buffer(msg2), error);
		size_t lenmsgr2 = tcp_sock_.read_some(boost::asio::buffer(buf_), error);

		size_t lenmsgr3 = tcp_sock_.read_some(boost::asio::buffer(buf_), error);
	}
	catch(std::exception& e)
	{
		throw;
	}
}

void KstServoing::ServoDirectCartesianStart()
{
	boost::system::error_code error;
	const std::string msg1 = "stDcEEf_\n";
	boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);
	size_t lenmsgr = tcp_sock_.read_some(boost::asio::buffer(buf_), error);
}

void KstServoing::ServoDirectJointStart()
{
	boost::system::error_code error;
	const std::string msg1 = "startDirectServoJoints\n";
	boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);
	size_t lenmsgr = tcp_sock_.read_some(boost::asio::buffer(buf_), error);
}

void KstServoing::ServoSmartCartesianStart()
{
	boost::system::error_code error;
	const std::string msg1 = "stSmtSEEf_\n";
	boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);
	size_t lenmsgr = tcp_sock_.read_some(boost::asio::buffer(buf_), error);
}

void KstServoing::ServoStop()
{
	boost::system::error_code error;
	const std::string msg1 = "stopDirectServoJoints\n"; // the same to stop direct servo based on KST
	boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);
	size_t lenmsgr = tcp_sock_.read_some(boost::asio::buffer(buf_), error);
}

void KstServoing::ServoSendJoints(std::vector<double> jp)
{
	boost::system::error_code error;
	const std::string msg1 = "jf_"+ 
		formated_double_2_string(jp[0], 5) + "_" +
		formated_double_2_string(jp[1], 5) + "_" +
		formated_double_2_string(jp[2], 5) + "_" +
		formated_double_2_string(jp[3], 5) + "_" +
		formated_double_2_string(jp[4], 5) + "_" +
		formated_double_2_string(jp[5], 5) + "_" +
		formated_double_2_string(jp[6], 5) + "_\n";
	boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);
}

std::vector<double> KstServoing::ServoSendJointsGetFeedback(std::vector<double> jp)
{
	boost::system::error_code error;
	const std::string msg1 = "jpJP_"+ 
		formated_double_2_string(jp[0], 5) + "_" +
		formated_double_2_string(jp[1], 5) + "_" +
		formated_double_2_string(jp[2], 5) + "_" +
		formated_double_2_string(jp[3], 5) + "_" +
		formated_double_2_string(jp[4], 5) + "_" +
		formated_double_2_string(jp[5], 5) + "_" +
		formated_double_2_string(jp[6], 5) + "_\n";
	boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);

	size_t lenmsgr = tcp_sock_.read_some(boost::asio::buffer(buf_), error);
	std::stringstream ssmsgr;
	std::string strmsgr;
	for(int i=0;i<lenmsgr;i++)
		ssmsgr << buf_[i];
	strmsgr = ssmsgr.str();

	std::vector<double> jpbk = parseString2DoubleVec(strmsgr);

	return jpbk;
}

void KstServoing::ServoSendEEF(std::vector<double> eef) // x y z rz ry rx
{
	boost::system::error_code error;
	const std::string msg1 = "DcSeCar_"+ 
		formated_double_2_string(eef[0], 5) + "_" +
		formated_double_2_string(eef[1], 5) + "_" +
		formated_double_2_string(eef[2], 5) + "_" +
		formated_double_2_string(eef[3], 5) + "_" +
		formated_double_2_string(eef[4], 5) + "_" +
		formated_double_2_string(eef[5], 5) + "_\n";
	boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);
}

std::vector<double> KstServoing::ServoSendEEFGetFeedback(std::vector<double> eef)  // x y z rz ry rx
{
	boost::system::error_code error;
	const std::string msg1 = "DcSeCarEEfP_"+ 
		formated_double_2_string(eef[0], 5) + "_" +
		formated_double_2_string(eef[1], 5) + "_" +
		formated_double_2_string(eef[2], 5) + "_" +
		formated_double_2_string(eef[3], 5) + "_" +
		formated_double_2_string(eef[4], 5) + "_" +
		formated_double_2_string(eef[5], 5) + "_\n";
	boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);

	size_t lenmsgr = tcp_sock_.read_some(boost::asio::buffer(buf_), error);
	std::stringstream ssmsgr;
	std::string strmsgr;
	for(int i=0;i<lenmsgr;i++)
		ssmsgr << buf_[i];
	strmsgr = ssmsgr.str();

	std::vector<double> eefbk = parseString2DoubleVec(strmsgr);

	return eefbk;
}

std::vector<double> KstServoing::GetJointPosition()
{
	std::string strmsgr;
	try
	{
		boost::system::error_code error;
		const std::string msg = "getJointsPositions\n";
		boost::asio::write(tcp_sock_, boost::asio::buffer(msg), error);
		size_t lenmsgr = tcp_sock_.read_some(boost::asio::buffer(buf_), error);
		std::stringstream ssmsgr;
		for(int i=0;i<lenmsgr;i++)
			ssmsgr << buf_[i];
		ssmsgr << "_"; // this is for getting jpos only, because the kuka java side miss a _
		strmsgr = ssmsgr.str();
	}
	catch(std::exception& e)
	{
		throw;
	}
	std::vector<double> vec = parseString2DoubleVec(strmsgr);
	// vector format: a1 a2 a3 a4 a5 a6 a7
	
	return vec;
}

std::vector<double> KstServoing::GetEEFPosition()
{
	std::string strmsgr;
	try
	{
		boost::system::error_code error;
		const std::string msg = "Eef_pos\n";
		boost::asio::write(tcp_sock_, boost::asio::buffer(msg), error);
		size_t lenmsgr = tcp_sock_.read_some(boost::asio::buffer(buf_), error);
		std::stringstream ssmsgr;
		for(int i=0;i<lenmsgr;i++)
			ssmsgr << buf_[i];
		strmsgr = ssmsgr.str();
	}
	catch(std::exception& e)
	{
		throw;
	}
	std::vector<double> vec = parseString2DoubleVec(strmsgr);
	// vector format: x y z rz ry rx

	geometry_msgs::TransformStamped eef; 
	eef.transform.translation.x = vec[0];
	eef.transform.translation.y = vec[1];
	eef.transform.translation.z = vec[2];
	std::vector<double> eul(&vec[3],&vec[6]);
	eef.transform.rotation = UtlCalculations::eul2quat(eul);

	return eef;
}

void KstServoing::NetTurnoffServer()
{
	boost::system::error_code error;
	const std::string msg1 = "end\n";
	boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);
	tcp_sock_.close();
}
