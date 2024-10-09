#ifndef DIFFDRIVE_ARDUINO_CAMERA_GIMBAL_HPP
#define DIFFDRIVE_ARDUINO_CAMERA_GIMBAL_HPP

#include <string>
#include <cmath>


class Camera_joint
{
    public:
        std::string name = "";
        double pos = 0;
        double cmd = 0;
        
        Camera_joint() = default;

        Camera_joint(const std::string &joint_name)
        {
            setup(joint_name);
        }
        void setup(const std::string &joint_name)
        {
            name =joint_name;
        }

};


#endif // DIFFDRIVE_ARDUINO_CAMERA_GIMBAL_HPP
