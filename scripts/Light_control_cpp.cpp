#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <ethernet_remote_io_module/ReadDigitalInputs.h>
#include <ethernet_remote_io_module/WriteCoilsList.h>
class LightControl_cpp {
    public:
    bool ctrl_c = false;
    ros::Publisher coils_pub;
    ros::Subscriber read_inputs_subscriber;
    ethernet_remote_io_module::WriteCoilsList light;
    
    
    public:
    LightControl_cpp(ros::NodeHandle *nh) {
        
        coils_pub = nh->advertise<ethernet_remote_io_module::WriteCoilsList>("write_coils", 10);    
        read_inputs_subscriber = nh->subscribe("read_inputs", 10, &LightControl_cpp::read_digital_callback);

    }
    void read_digital_callback(const ethernet_remote_io_module::ReadDigitalInputs& msg) {
        ethernet_remote_io_module::ReadDigitalInputs data;
        data = msg;
        
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
int main (int argc, char **argv)
{
    ros::init(argc, argv, "Light_Control_cpp");
    ros::NodeHandle nh;
    LightControl_cpp nc = LightControl_cpp(&nh);
    ros::spin();
}