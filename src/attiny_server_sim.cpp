#include "ros/ros.h"
#include "myrobot_model/AttinyCommand.h"
#include <string>

class GripperUnit
{
    private:
        ros::NodeHandle n;
        ros::ServiceServer service;
        int currentPositionVertical;
        int currentPositionGripper;
        int currentPositionServo;
        int targetPositionVertical;
        int targetPositionGripper;
        int targetPositionServo;
        float velocityVertical;
        float velocityGripper;
        float velocityServo;
        bool isInitializedVERT;
        bool isInitializedGRIP;
        float VELCONSTVertical = 0.01;
        float VELCONSTGripper = 0.01;
        float LIMIT_VERT_UP = 0.142;
        float LIMIT_VERT_DOWN = 0.029;
        float LIMIT_GRIP_UP = 0.153;
        float LIMIT_GRIP_DOWN = 0.0;
        bool isEndGripPressed = false;

        // function to extract int parameters from comma separated string
        void getParameters(std::string str, int* params)
        {
            std::string delimiter = ",)";

            size_t pos = 0;
            std::string token;
            int i = 0;
            while ((pos = str.find(delimiter)) != std::string::npos) 
            {
                token = str.substr(0, pos);
                std::cout << token << std::endl;
                params[i] = atoi(token.c_str());
                i++;
                str.erase(0, pos + delimiter.length());
            }
            std::cout << str << std::endl;
        }

        // move gripper motor to initial position
        void initGripper(){
            velocityGripper = 0.01;
            // wait for 2 seconds
            ros::Duration(0.5).sleep(); 
            velocityGripper = 0.0;
            currentPositionGripper = 0;
            targetPositionGripper  = 0;
            isInitializedGRIP = true;
        }

        // move vertical motor to initial position
        void initVertical(){
            velocityVertical = 0.01;
            ros::Duration(0.5).sleep(); 
            velocityVertical = 0.0;
            currentPositionVertical = 0;
            targetPositionVertical  = 0;
            isInitializedVERT = true;
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

    public:
        GripperUnit()
        {
            service = n.advertiseService("attiny_command", &GripperUnit::send_to_attiny, this);
            ROS_INFO("Attiny server is ready.");

            // initialize variables
            currentPositionVertical = 0;
            currentPositionGripper = 0;
            currentPositionServo = 0;
            targetPositionVertical = 0;
            targetPositionGripper = 0;
            targetPositionServo = 0;
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
                    getParameters(cmd.c_str(), params);
                    targetPositionServo = params[0];
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
                    getParameters(cmd.c_str(), params);
                    int rpm = params[0];
                    VELCONSTVertical = rpm/60.0;
                }
                else if(cmd == "ma")
                {
                    int params[1] = {0};
                    getParameters(cmd.c_str(), params);
                    int distAbs = params[0];
                    targetPositionVertical = distAbs/1000.0;
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
                    getParameters(cmd.c_str(), params);
                    int distRel = params[0];
                    targetPositionVertical += distRel/1000.0;
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
                    getParameters(cmd.c_str(), params);
                    int rpm = params[0];
                    VELCONSTGripper = rpm/60.0;
                }
                else if(cmd == "ma")
                {
                    int params[1] = {0};
                    getParameters(cmd.c_str(), params);
                    int distAbs = params[0];
                    targetPositionGripper= distAbs/1000.0;
                    if(targetPositionGripper < LIMIT_GRIP_DOWN){
                        targetPositionGripper = LIMIT_GRIP_DOWN;  
                    }
                    else if(targetPositionGripper > LIMIT_GRIP_UP){
                        targetPositionGripper = LIMIT_GRIP_UP;
                    }
                }
                else if(cmd == "mr")
                {
                    int params[1] = {0};
                    getParameters(cmd.c_str(), params);
                    int distRel = params[0];
                    targetPositionGripper += distRel/1000.0;
                    if(targetPositionGripper < LIMIT_GRIP_DOWN){
                        targetPositionGripper = LIMIT_GRIP_DOWN;  
                    }
                    else if(targetPositionGripper > LIMIT_GRIP_UP){
                        targetPositionGripper = LIMIT_GRIP_UP;
                    }
                }
                else if(cmd == "st")
                {
                    targetPositionGripper = currentPositionGripper;
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
                    targetPositionGripper = currentPositionGripper;
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

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "attiny_server_sim");

    GripperUnit gripper_unit;

    ros::spin();

    return 0;
}