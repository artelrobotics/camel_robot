#include "ros/ros.h"
//#include "Light_control/LightControl.h"
#include "camel_robot/light.h"

class Light_Service
{
    public:
        bool light_command(camel_robot::light::Request& req,
                            camel_robot::light::Response& res);
};


bool Light_Service::light_command(camel_robot::light::Request& req,
                                    camel_robot::light::Response& res)
{
    if(req.request_string == "light_off")
    {

        res.response_string = "ALL Lights are turned OFF!";
    }

    else
    {
        res.response_string = "Undefined request String";
    }

    return true;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Light_Service_cpp");
  ros::NodeHandle n;

// %Tag(SERVICE_SERVER)%
  Light_Service a;
  
  ros::ServiceServer ss = n.advertiseService("Light_server", &Light_Service::light_command, &a);
// %EndTag(SERVICE_SERVER)%

  ros::spin();

  return 0;
}
