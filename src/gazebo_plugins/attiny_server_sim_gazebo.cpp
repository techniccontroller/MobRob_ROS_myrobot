#include <ros/ros.h>
#include "myrobot_model/AttinyCommand.h"
#include <string>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>



namespace gazebo
{
    class GripperUnit : public ModelPlugin
    {
        private:
            ros::NodeHandle n;
            ros::ServiceServer service;
            float currentPositionVertical;
            float currentPositionGripper;
            float currentPositionServo;
            float targetPositionVertical;
            float targetPositionGripper;
            float targetPositionServo;
            float temptargetPositionGripper;
            float temptargetPositionVertical;
            float temptargetPositionServo;
            float velocityVertical;
            float velocityGripper;
            float velocityServo;
            bool isInitializedVERT;
            bool isInitializedGRIP;
            float VELCONSTVertical = 0.02;
            float VELCONSTGripper = 0.1;
            float LIMIT_VERT_UP = 0.142;
            float LIMIT_VERT_DOWN = 0.025;
            float LIMIT_GRIP_UP = 0.153;
            float LIMIT_GRIP_DOWN = 0.0;
            float LIMIT_SERVO_UP = 3.14159;
            float LIMIT_SERVO_DOWN = 0.69;
            float TOLERANCE = 0.001;
            bool isEndGripPressed = false;

            // Pointer to the model
            physics::ModelPtr model;

            physics::JointControllerPtr jointController;
            std::string nameLeftFingerJoint;
            std::string nameRightFingerJoint;
            std::string nameVerticalJoint;
            std::string nameCameraServoJoint;


            // Pointer to the update event connection
            event::ConnectionPtr updateConnection;

            // function to extract int parameters from comma separated string
            void getParameters(std::string str, int* params)
            {
                size_t pos = 0;
                std::string token;
                int i = 0;
                std::cout << "str: " << str << std::endl;
                while ((pos = str.find(",")) != std::string::npos || (pos = str.find(")")) != std::string::npos)
                {
                    std::cout << str << std::endl;
                    token = str.substr(0, pos);
                    std::cout << token << std::endl;
                    params[i] = atoi(token.c_str());
                    i++;
                    str.erase(0, pos + 1);
                }
                
            }

            // set gripper target position
            void setGripperTarget(float target){
                temptargetPositionGripper = currentPositionGripper;
                targetPositionGripper = target;
            }

            // set vertical target position
            void setVerticalTarget(float target){
                temptargetPositionVertical = currentPositionVertical;
                targetPositionVertical = target;
            }

            // set servo target position
            void setServoTarget(float target){
                if(target > LIMIT_SERVO_UP) target = LIMIT_SERVO_UP;
                if(target < LIMIT_SERVO_DOWN) target = LIMIT_SERVO_DOWN;
                temptargetPositionServo = currentPositionServo;
                targetPositionServo = target;
            }

            // move gripper motor to initial position
            void initGripper(){
                setGripperTarget(LIMIT_GRIP_DOWN);
                int timeout = 100;
                while(currentPositionGripper > LIMIT_GRIP_DOWN + 10*TOLERANCE && !timeout){
                    ros::Duration(0.1).sleep();
                    timeout--;
                }
                isInitializedGRIP = timeout > 0;
            }

            // move vertical motor to initial position
            void initVertical(){
                targetPositionVertical = LIMIT_VERT_UP;
                int timeout = 100;
                while(currentPositionVertical < LIMIT_VERT_UP - 10*TOLERANCE && !timeout){
                    ros::Duration(0.1).sleep();
                    timeout--;
                }
                isInitializedVERT = timeout > 0;
            }

            std::string fillOutputBuffer(uint16_t value){
                uint8_t i = 2;           
                uint16_t d = 1000;
                std::string outputbuffer = "000000\n";
                for(; i < 6; i++, d /= 10){
                    outputbuffer[i] = std::to_string(((uint16_t)(value/d))%10)[0];
                }
                return outputbuffer;
            }

            float toVerticalPosition(float height){
                return height - 0.142;
            }

            float toVerticalHeight(float position){
                return position + 0.142;
            }

        public:
            GripperUnit()
            {
                service = n.advertiseService("attiny_command", &GripperUnit::send_to_attiny, this);
                ROS_INFO("Attiny server is ready.");

                // initialize variables
                currentPositionVertical = LIMIT_VERT_UP;
                currentPositionGripper = 0;
                currentPositionServo = LIMIT_SERVO_DOWN;
                setVerticalTarget(LIMIT_VERT_UP);
                setGripperTarget(0);
                setServoTarget(LIMIT_SERVO_DOWN);
                velocityVertical = 0.0;
                velocityGripper = 0.0;
                velocityServo = 0.0;
                isInitializedVERT = false;
                isInitializedGRIP = false;
            }

            std::string evaluate_input(std::string &input)
            {
                // output is the input, but upper case
                std::string output(input.c_str());
                std::transform(output.begin(), output.end(), output.begin(), ::toupper);
                
                // extract first 2 characters
                std::string dev = input.substr(0,2);
                // extract characters 3 and 4
                std::string cmd = input.substr(3,2);
                // extract characters 6 till end
                std::string paramsstr = input.substr(6);

                // print dev and cmd
                //std::cout << "dev: " << dev << std::endl;
                //std::cout << "cmd: " << cmd << std::endl;

                // create if else statements to check if dev is sv, vt, gr, or st
                // if dev is sv, check if cmd is ac, wr, rf, gp
                // if dev is vt, check if cmd is it, sp, ma, mr, st, gp
                // if dev is gr, check if cmd is it, sp, ma, mr, st, gp, gg
                // if dev is st, check if cmd is ac, st
                if(dev == "sv")
                {
                    if(cmd == "ac")
                    {
                        // NO ACTION
                    }
                    else if(cmd == "wr")
                    {
                        int params[1] = {0};
                        getParameters(paramsstr, params);
                        setServoTarget(params[0] * M_PI / 180);
                    }
                    else if(cmd == "rf")
                    {
                        // NO ACTION
                    }
                    else if(cmd == "gp")
                    {
                        // set output the current servo postion with leading zeros and a length of 6 characters
                        output = fillOutputBuffer(currentPositionServo);
                    }

                }
                else if(dev == "vt")
                {
                    if(cmd == "it")
                    {
                        initVertical();
                    }
                    else if(cmd == "sp")
                    {
                        int params[1] = {0};
                        getParameters(paramsstr, params);
                        int rpm = params[0];
                        VELCONSTVertical = rpm/60.0;
                    }
                    else if(cmd == "ma")
                    {
                        int params[1] = {0};
                        getParameters(paramsstr, params);
                        int distAbs = params[0];
                        targetPositionVertical = ((float)distAbs)/1000.0;
                        if(targetPositionVertical < LIMIT_VERT_DOWN){
                            targetPositionVertical = LIMIT_VERT_DOWN;  
                        }
                        else if(targetPositionVertical > LIMIT_VERT_UP){
                            targetPositionVertical = LIMIT_VERT_UP;
                        }
                    }
                    else if(cmd == "mr")
                    {
                        int params[1] = {0};
                        getParameters(paramsstr, params);
                        int distRel = params[0];
                        targetPositionVertical += ((float)distRel)/1000.0;
                        if(targetPositionVertical < LIMIT_VERT_DOWN){
                            targetPositionVertical = LIMIT_VERT_DOWN;  
                        }
                        else if(targetPositionVertical > LIMIT_VERT_UP){
                            targetPositionVertical = LIMIT_VERT_UP;
                        }
                    }
                    else if(cmd == "st")
                    {
                        targetPositionVertical = currentPositionVertical;
                    }
                    else if(cmd == "gp")
                    {
                        if(isInitializedVERT){
                            output = fillOutputBuffer(int(currentPositionVertical * 1000));
                        }else{
                            output = fillOutputBuffer(9999);
                        }
                    }

                }
                else if(dev == "gr")
                {
                    if(cmd == "it")
                    {
                        initGripper();
                    }
                    else if(cmd == "sp")
                    {
                        int params[1] = {0};
                        getParameters(paramsstr, params);
                        int rpm = params[0];
                        VELCONSTGripper = rpm/60.0;
                    }
                    else if(cmd == "ma")
                    {
                        int params[1] = {0};
                        getParameters(paramsstr, params);
                        int distAbs = params[0];
                        targetPositionGripper= ((float)distAbs)/1000.0;
                        if(targetPositionGripper < LIMIT_GRIP_DOWN){
                            setGripperTarget(LIMIT_GRIP_DOWN);  
                        }
                        else if(targetPositionGripper > LIMIT_GRIP_UP){
                            setGripperTarget(LIMIT_GRIP_UP);
                        }
                    }
                    else if(cmd == "mr")
                    {
                        int params[1] = {0};
                        getParameters(paramsstr, params);
                        int distRel = params[0];
                        targetPositionGripper += ((float)distRel)/1000.0;
                        if(targetPositionGripper < LIMIT_GRIP_DOWN){
                            setGripperTarget(LIMIT_GRIP_DOWN);  
                        }
                        else if(targetPositionGripper > LIMIT_GRIP_UP){
                            setGripperTarget(LIMIT_GRIP_UP);
                        }
                    }
                    else if(cmd == "st")
                    {
                        setGripperTarget(currentPositionGripper);
                    }
                    else if(cmd == "gp")
                    {
                        if(isInitializedGRIP){
                            output = fillOutputBuffer(int(currentPositionGripper * 1000));
                        }else{
                            output = fillOutputBuffer(9999);
                        }
                    }
                    else if(cmd == "gg")
                    {
                        output = fillOutputBuffer(isEndGripPressed);
                    }
                }
                else if(dev == "st")
                {
                    if(cmd == "ac")
                    {
                        // NO ACTION
                    }
                    else if(cmd == "st")
                    {
                        setGripperTarget(currentPositionGripper);
                        targetPositionVertical = currentPositionVertical;
                    }
                }

                return output;
            }

            bool send_to_attiny(myrobot_model::AttinyCommand::Request  &req,
                    myrobot_model::AttinyCommand::Response &res)
            {
                ROS_INFO("request: input=%s", req.input.c_str());

                res.output = evaluate_input(req.input);

                ROS_INFO("sending back response: [%s]", res.output.c_str());

                return true;
            }

            void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
            {
                // Make sure the ROS node for Gazebo has already been initialized
                if (!ros::isInitialized())
                {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                    << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
                return;
                }

                ROS_INFO("Hello World!");
                
                // Store the pointer to the model
                this->model = _parent;

                // Init Joint Controller
                this->jointController.reset(new physics::JointController(this->model));

                this->jointController->AddJoint(model->GetJoint("gripper_to_left_finger_joint"));
                this->jointController->AddJoint(model->GetJoint("gripper_to_right_finger_joint"));
                this->jointController->AddJoint(model->GetJoint("gripper_stand_to_gripper_joint"));
                this->jointController->AddJoint(model->GetJoint("camera_servo_to_camera_base_joint"));

                this->nameLeftFingerJoint = model->GetJoint("gripper_to_left_finger_joint")->GetScopedName();
                this->nameRightFingerJoint = model->GetJoint("gripper_to_right_finger_joint")->GetScopedName();
                this->nameVerticalJoint = model->GetJoint("gripper_stand_to_gripper_joint")->GetScopedName();
                this->nameCameraServoJoint = model->GetJoint("camera_servo_to_camera_base_joint")->GetScopedName();

                this->jointController->SetPositionPID(this->nameLeftFingerJoint, common::PID(1000, 0, 0));
                this->jointController->SetPositionPID(this->nameRightFingerJoint, common::PID(1000, 0, 0));
                this->jointController->SetPositionPID(this->nameVerticalJoint, common::PID(1000, 1, 0));
                this->jointController->SetPositionPID(this->nameCameraServoJoint, common::PID(10, 0, 0));

                // Listen to the update event. This event is broadcast every
                // simulation iteration.
                this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&GripperUnit::OnUpdate, this));
            }

            // Called by the world update start event
            void OnUpdate()
            {
                //https://classic.gazebosim.org/tutorials?tut=set_velocity&cat=

                // get current position of gripper
                float currentPositionLeftFinger = this->model->GetJoint("gripper_to_left_finger_joint")->Position(0);
                float currentPositionRightFinger = this->model->GetJoint("gripper_to_right_finger_joint")->Position(0);
                currentPositionGripper = currentPositionLeftFinger - currentPositionRightFinger;
                // get current position of vertical stand
                currentPositionVertical = toVerticalHeight(this->model->GetJoint("gripper_stand_to_gripper_joint")->Position(0));
                // get current position of camera servo
                currentPositionServo = this->model->GetJoint("camera_servo_to_camera_base_joint")->Position(0);
                
                // output current positions
                //std::cout << "gripper position: " << currentPositionGripper << std::endl;
                //std::cout << "vertical position: " << currentPositionVertical << std::endl;
                //std::cout << "vertical position*: " << toVerticalPosition(currentPositionVertical) << std::endl;
                //std::cout << "servo position: " << currentPositionServo << std::endl;
                
                
                if(fabs(temptargetPositionGripper - targetPositionGripper) > TOLERANCE){
                    if(temptargetPositionGripper > targetPositionGripper){
                        temptargetPositionGripper -= TOLERANCE*0.1;
                    }
                    else if(temptargetPositionGripper < targetPositionGripper){
                        temptargetPositionGripper += TOLERANCE*0.1;
                    }
                }

                if(fabs(temptargetPositionVertical - targetPositionVertical) > TOLERANCE){
                    if(temptargetPositionVertical > targetPositionVertical){
                        temptargetPositionVertical -= TOLERANCE*0.1;
                    }
                    else if(temptargetPositionVertical < targetPositionVertical){
                        temptargetPositionVertical += TOLERANCE*0.1;
                    }
                }

                if(fabs(temptargetPositionServo - targetPositionServo) > TOLERANCE){
                    if(temptargetPositionServo > targetPositionServo){
                        temptargetPositionServo -= TOLERANCE*0.1;
                    }
                    else if(temptargetPositionServo < targetPositionServo){
                        temptargetPositionServo += TOLERANCE*0.1;
                    }
                }
 
                // set setpoints
                this->jointController->SetPositionTarget(this->nameLeftFingerJoint, temptargetPositionGripper/2);
                this->jointController->SetPositionTarget(this->nameRightFingerJoint, -temptargetPositionGripper/2);
                this->jointController->SetPositionTarget(this->nameVerticalJoint, toVerticalPosition(temptargetPositionVertical));
                this->jointController->SetPositionTarget(this->nameCameraServoJoint, temptargetPositionServo);

                // print temp target
                //std::cout << "temptargetPositionGripper: " << temptargetPositionGripper << std::endl;
                //std::cout << "temptargetPositionVertical: " << temptargetPositionVertical << std::endl;
                //std::cout << "temptargetPositionVertical (*): " << toVerticalPosition(temptargetPositionVertical) << std::endl;
                //std::cout << "temptargetPositionServo: " << temptargetPositionServo << std::endl << std::endl;

                // update controller
                this->jointController->Update();
            }



    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(GripperUnit)
}