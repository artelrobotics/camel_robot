/*#include <ros/ros.h>
#include <ethernet_remote_io_module/WriteCoilsList.h>
#include <std_msgs/Int64.h>
#include <ethernet_remote_io_module/ReadDigitalInputs.h>


//using namespace std;


class LightControl_cpp {
    public:
    bool ctrl_c = false;
    ros::Publisher coils_pub;
    ros::Subscriber read_inputs_subscriber;
    
    public:
    LightControl_cpp(ros::NodeHandle *nh) {
        ethernet_remote_io_module::WriteCoilsList light;
        coils_pub = nh->advertise<ethernet_remote_io_module::WriteCoilsList>("write_coils", 10);    
        read_inputs_subscriber = nh->subscribe("read_inputs", 1000, &LightControl_cpp::read_digital_callback, this);

    }
    ethernet_remote_io_module::ReadDigitalInputs read_digital_callback(const ethernet_remote_io_module::ReadDigitalInputs& msg) {
        ethernet_remote_io_module::ReadDigitalInputs data;
        data = msg.data;
        return data;
        
    }
    void write_coils(const ethernet_remote_io_module::WriteCoilsList& msg)
    {
        while(ros::ok)
        {
            coils_pub.publish(msg);
        } 

    }

    void light_off()
    {
        light = [0,0,0,0,0,0,0]
        LightControl_cpp::write_coils(light)
    }

    void light_type_1_cpp()
    {
        light = [1,0,0,0,0,0,0]
        LightControl_cpp::write_coils(light)
    }
};

    

class Light_Service
{
    public:
          bool light_command(camel_robot::light::Request& req,
                            camel_robot::light::Response& res);
          
       
};


bool Light_Service::light_command(camel_robot::light::Request& req,
                                    camel_robot::light::Response& res)
{
    
    LightControl_cpp l;
    
    if(req.request_string == "light_off")
    {
        l.light_off()
        res.response_string = "ALL Lights are turned OFF!";

    }

    else if(req.request_string == "light_type_1")
    {
        //l::light_type_1();
        res.response_string = "Lighting_type_1 is on!";
    }

    else if(req.request_string == "light_type_2")
    {
        //l::light_type_2();
        res.response_string = "Lighting_type_2 is on!";
    }

    else if(req.request_string == "light_type_3")
    {
        //l::light_type_3();
        res.response_string = "Lighting_type_3 is on!";
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
  ros::NodeHandle nh;
  LightControl_cpp nc = LightControl_cpp(&n);
  
  Light_Service a;

  
  ros::Publisher write_coils_publisher = n.advertise<ethernet_remote_io_module::WriteCoilsList>("write_coils", 10);
  ros::ServiceServer ss = n.advertiseService("Light_server", &Light_Service::light_command, &a);
// %EndTag(SERVICE_SERVER)%

  ros::spin();

  return 0;
}

*/
#include <iostream>

int main(){

    return 0;
}
