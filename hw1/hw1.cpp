#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>
#include <fstream>

#define QUESTION_1   1
#define QUESTION_2   2
#define QUESTION_3   3
#define QUESTION_4   4
#define QUESTION_5   5
#define QUESTION_6   6

// handle ctrl-c nicely
#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm_controller.urdf";
const string robot_name = "PANDA";

// redis keys:
// - read:
const std::string JOINT_ANGLES_KEY = "sai2::cs225a::panda_robot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::cs225a::panda_robot::sensors::dq";
// - write
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::panda_robot::actuators::fgc";
const string CONTROLLER_RUNING_KEY = "sai2::cs225a::controller_running";

unsigned long long controller_counter = 0;

int main() {

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	redis_client.set(CONTROLLER_RUNING_KEY, "1");
	
	ofstream myfile;
	myfile.open("P6.txt");
	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		robot->updateModel();

		// **********************
		// WRITE YOUR CODE AFTER
		// **********************
		int controller_number = QUESTION_6;  // change to the controller of the question you want : QUESTION_1, QUESTION_2, QUESTION_3, QUESTION_4, QUESTION_5


		// ---------------------------  question 1 ---------------------------------------
		if(controller_number == QUESTION_1)
		{
			double kp = 400.0; 
			double kv = 52.0;   

			VectorXd q_desired(dof);  
			q_desired(0) = M_PI/2;
			q_desired(1) = -M_PI/4;
			q_desired(2) = 0.0;
			q_desired(3) = -25.0/36.0*M_PI;
			q_desired(4) = 0.0;
			q_desired(5) = 4.0/9.0*M_PI;
			q_desired(6) = 0.0;

			command_torques = -kp*(robot->_q-q_desired)-kv*robot->_dq;
			
		    // writing into P1.txt q and qd for joint 0,2,3
		    myfile << "joint 0: " << time << " " << robot->_q.coeff(0) << " " << q_desired.coeff(0) << "\n";
		    myfile << "joint 2: " << time << " " << robot->_q.coeff(2) << " " << q_desired.coeff(2) << "\n";
		    myfile << "joint 3: " << time << " " << robot->_q.coeff(3) << " " << q_desired.coeff(3) << "\n";
			
		}

		// ---------------------------  question 2 ---------------------------------------
		if(controller_number == QUESTION_2)
		{

			double kp = 400.0; 
			double kv = 52.0;   

			VectorXd q_desired(dof);  
			q_desired(0) = M_PI/2;
			q_desired(1) = -M_PI/4;
			q_desired(2) = 0.0;
			q_desired(3) = -25.0/36.0*M_PI;
			q_desired(4) = 0.0;
			q_desired(5) = 4.0/9.0*M_PI;
			q_desired(6) = 0.0;
			
			VectorXd g(dof);
			robot->gravityVector(g);
			
			command_torques = -kp*(robot->_q-q_desired)-kv*robot->_dq+g;
			
		    // writing into P1.txt q and qd for joint 0,2,3
		    myfile << "joint 0: " << time << " " << robot->_q.coeff(0) << " " << q_desired.coeff(0) << "\n";
		    myfile << "joint 2: " << time << " " << robot->_q.coeff(2) << " " << q_desired.coeff(2) << "\n";
		    myfile << "joint 3: " << time << " " << robot->_q.coeff(3) << " " << q_desired.coeff(3) << "\n";
		}

		// ---------------------------  question 3 ---------------------------------------
		if(controller_number == QUESTION_3)
		{

			double kp = 400.0; 
			double kv = 40.0;   

			VectorXd q_desired(dof);  
			q_desired(0) = M_PI/2;
			q_desired(1) = -M_PI/4;
			q_desired(2) = 0.0;
			q_desired(3) = -25.0/36.0*M_PI;
			q_desired(4) = 0.0;
			q_desired(5) = 4.0/9.0*M_PI;
			q_desired(6) = 0.0;
			
			VectorXd g(dof);
			robot->gravityVector(g);
			
			command_torques = robot->_M*(-kp*(robot->_q-q_desired)-kv*robot->_dq)+g;
			
		    // writing into P1.txt q and qd for joint 0,2,3
		    myfile << "joint 0: " << time << " " << robot->_q.coeff(0) << " " << q_desired.coeff(0) << "\n";
		    myfile << "joint 2: " << time << " " << robot->_q.coeff(2) << " " << q_desired.coeff(2) << "\n";
		    myfile << "joint 3: " << time << " " << robot->_q.coeff(3) << " " << q_desired.coeff(3) << "\n";
		}

		// ---------------------------  question 4 ---------------------------------------
		if(controller_number == QUESTION_4)
		{

			double kp = 400.0; 
			double kv = 40.0;   

			VectorXd q_desired(dof);  
			q_desired(0) = M_PI/2;
			q_desired(1) = -M_PI/4;
			q_desired(2) = 0.0;
			q_desired(3) = -25.0/36.0*M_PI;
			q_desired(4) = 0.0;
			q_desired(5) = 4.0/9.0*M_PI;
			q_desired(6) = 0.0;
			
			VectorXd g(dof);
			robot->gravityVector(g);
			
			VectorXd coriolis = VectorXd::Zero(dof);
			robot->coriolisForce(coriolis);
			
			command_torques = robot->_M*(-kp*(robot->_q-q_desired)-kv*robot->_dq)+coriolis+g;
			
		    // writing into P1.txt q and qd for joint 0,2,3
		    myfile << "joint 0: " << time << " " << robot->_q.coeff(0) << " " << q_desired.coeff(0) << "\n";
		    myfile << "joint 2: " << time << " " << robot->_q.coeff(2) << " " << q_desired.coeff(2) << "\n";
		    myfile << "joint 3: " << time << " " << robot->_q.coeff(3) << " " << q_desired.coeff(3) << "\n";
		}

		// ---------------------------  question 5 ---------------------------------------
		if(controller_number == QUESTION_5)
		{

			double kp = 400.0; 
			double kv = 40.0;   

			VectorXd q_desired(dof);  
			q_desired(0) = M_PI/2;
			q_desired(1) = -M_PI/4;
			q_desired(2) = 0.0;
			q_desired(3) = -25.0/36.0*M_PI;
			q_desired(4) = 0.0;
			q_desired(5) = 4.0/9.0*M_PI;
			q_desired(6) = 0.0;
			
			VectorXd g(dof);
			robot->gravityVector(g);
			
			VectorXd coriolis = VectorXd::Zero(dof);
			robot->coriolisForce(coriolis);
			
			command_torques = robot->_M*(-kp*(robot->_q-q_desired)-kv*robot->_dq)+coriolis+g;
			
		    // writing into P1.txt q and qd for joint 0,2,3
		    myfile << "joint 0: " << time << " " << robot->_q.coeff(0) << " " << q_desired.coeff(0) << "\n";
		    myfile << "joint 2: " << time << " " << robot->_q.coeff(2) << " " << q_desired.coeff(2) << "\n";
		    myfile << "joint 3: " << time << " " << robot->_q.coeff(3) << " " << q_desired.coeff(3) << "\n";
		}
		
		// ---------------------------  extra credit ---------------------------------------
		if(controller_number == QUESTION_6)
		{	
			
			std::string ee_link_name = "link7";
			MatrixXd ee_jacobian(3,dof);
			Vector3d ee_pos_in_link = Eigen::Vector3d(0.0, 0.0, 0.17); 
			robot->Jv(ee_jacobian,ee_link_name,ee_pos_in_link);
			MatrixXd M_additional = 2.5*ee_jacobian.transpose()*ee_jacobian;
			
			Vector3d g_global = Eigen::Vector3d(0.0, 0.0, 9.81); 
			VectorXd g_additional = 2.5*ee_jacobian.transpose()*g_global;
			
			
			double kp = 400.0; 
			double kv = 40.0;   

			VectorXd q_desired(dof);  
			q_desired(0) = M_PI/2;
			q_desired(1) = -M_PI/4;
			q_desired(2) = 0.0;
			q_desired(3) = -25.0/36.0*M_PI;
			q_desired(4) = 0.0;
			q_desired(5) = 4.0/9.0*M_PI;
			q_desired(6) = 0.0;
			
			VectorXd g(dof);
			robot->gravityVector(g);
			
			VectorXd coriolis = VectorXd::Zero(dof);
			robot->coriolisForce(coriolis);
			
			command_torques = (robot->_M+M_additional)*(-kp*(robot->_q-q_desired)-kv*robot->_dq)+coriolis+g+g_additional;
			
		    // writing into P1.txt q and qd for joint 0,2,3
		    myfile << "joint 0: " << time << " " << robot->_q.coeff(0) << " " << q_desired.coeff(0) << "\n";
		    myfile << "joint 2: " << time << " " << robot->_q.coeff(2) << " " << q_desired.coeff(2) << "\n";
		    myfile << "joint 3: " << time << " " << robot->_q.coeff(3) << " " << q_desired.coeff(3) << "\n";
		}
		// **********************
		// WRITE YOUR CODE BEFORE
		// **********************

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}
	
	myfile.close();

	command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	redis_client.set(CONTROLLER_RUNING_KEY, "0");

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
