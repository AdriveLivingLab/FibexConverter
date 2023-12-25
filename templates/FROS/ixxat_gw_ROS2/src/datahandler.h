#ifndef IXXAT_ROS_DRIVER_DATAHANDLER_H_
#define IXXAT_ROS_DRIVER_DATAHANDLER_H_
#include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp>
#include "../include/ixxat_pkt.h"
#include "../include/pdudefs.h"
#include "../include/msgdefs.h"


namespace ixxat_gw_{{channel_name}}
{
    using boost::asio::ip::tcp;
    
    class DataHandler : public rclcpp::Node
    {
    typedef  void (ixxat_gw_{{channel_name}}::DataHandler::*IxxatMemFn)(unsigned char *data);
    public:
        DataHandler(boost::asio::io_service &io_service);
        ~DataHandler();
        
        void SetRosNodeAndAdvertise();
        void EstablishConnection();
        void ReceiveGatewayData();
        bool checkBit(unsigned char * framepointer, unsigned int bitToCheck);
        
        {% for frame in framedefs %}
        void {{frame["name"]}}(unsigned char *data);
        {% endfor %}
        
        void blank(unsigned char *data);

    private:
        std::string port_;
        std::string ip_;
        tcp::socket socket_;

        APP_IXX_PP *pApp_;

        rclcpp::Time rostimestamp_;
        rclcpp::Clock ros_clock_;

        std::string send_init_ = "c init 500\n\r";
        std::string send_start_ = "c start\n\r";
        std::string send_stop_ = "c stop\n\r";
    
        
        {% for pdu in list_of_pdus %}
        void Publish_{{pdu}}(unsigned char *raw_data_pointer);
        {% endfor %}
        
        {% for pdu in list_of_publishers %}
        rclcpp::Publisher<flexray_msgs::msg::{{pdu}}>::SharedPtr publisher_{{pdu}}_;
        {% endfor %}

    
        // fx:NUMBER-OF-CYCLES and flexray:STATIC-SLOT in the fibex.xml define this matrix
        // Example NUMBER-OF-CYCLES=64, STATIC-SLOT=29
        // first row is [0][], so cycle 0, which is possible
        // first column is [][0], so slot 0, that's not possible
        // FRAME_17_1_8: Slot=17, BaseCycle=1, Repition=8       
        // Size of fnktpntr is [NUMBER-OF-CYCLES][STATIC-SLOT+1] due to the column 0 always being empty
        
        IxxatMemFn fnktpntr_[{{fnktpntr_array|count}}][{{fnktpntr_array[0]|count}}] = {

        {% set ns = namespace(first_in_line=true) %}
        {% for row in fnktpntr_array %}
        { {% for elem in row %}{% if ns.first_in_line == true %}{{elem}}{% set ns.first_in_line = false %}{% else %},{{elem}}{% endif %}{% endfor %}{%- set ns.first_in_line = true -%} }, 
        {% endfor %}
        };
    };
    
} // namespace ixxat_gw_{{channel_name}}
#endif