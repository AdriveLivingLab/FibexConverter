#ifndef IXXAT_ROS_DRIVER_DATAHANDLER_CPP_
#define IXXAT_ROS_DRIVER_DATAHANDLER_CPP_

#include "datahandler.h"

#include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp>
#include <stdio.h>      /* printf, fopen */
#include <stdlib.h>     /* exit, EXIT_FAILURE */

namespace ixxat_gw_{{channel_name}}
{

    DataHandler::DataHandler(boost::asio::io_service &io_service)
        : rclcpp::Node("ixxat_gw_mlbevo_flexray"),
        socket_(io_service)
    {
        // Access the parameter        
        // this->declare_parameter("port_num_gw", rclcpp::ParameterType::PARAMETER_STRING);
        // std::string port_ = this->get_parameter("port_num_gw").as_string();
        port_ = "19227";

        // this->declare_parameter("ip_address_gw", rclcpp::ParameterType::PARAMETER_STRING);
        // std::string ip_ = this->get_parameter("ip_address_gw").as_string();
        ip_ = "169.254.200.123";

        /***************************************
         * Allocation for message decoding
         ***************************************/

        pApp_ = (APP_IXX_PP *)calloc(sizeof(APP_IXX_PP), 1);
        if (pApp_ == NULL)
        {
            perror("APP Buffer allocation");
            exit (EXIT_FAILURE);
        }
        // Socket packets buffer
        pApp_->pSBuf = (char *)malloc(512 * 3);
        if (pApp_->pSBuf == NULL)
        {
            perror("Buffer allocation");
            exit (EXIT_FAILURE);
        }

    }

    DataHandler::~DataHandler()
    {
        boost::system::error_code error_stop;
        boost::asio::write(socket_, boost::asio::buffer(send_stop_), error_stop);
        socket_.close();
    }

    bool DataHandler::checkBit(unsigned char *framepointer, unsigned int bitToCheck) {
        //Calculate how many full bytes we need to move the pointer
        int byteOffset = std::floor(bitToCheck / 8);
        //Calculate how many bits we need to shift to get to the requested bit
        int bitOffset = bitToCheck % 8;

        //Move the pointer and shift to the requested bit and check if it's 1
        return (*(framepointer+byteOffset) >> bitOffset) & 0x1;
    }

    void DataHandler::EstablishConnection()
    {
        RCLCPP_INFO(this->get_logger(), "Preparing Connection definition");
        const unsigned short port_gw = static_cast<unsigned short>(std::strtoul(port_.c_str(), NULL, 0));
        const boost::asio::ip::address address_gw = boost::asio::ip::address::from_string(ip_);
        socket_.connect(tcp::endpoint(address_gw, port_gw));

        RCLCPP_INFO(this->get_logger(), "Trying to initiate connection");

        boost::system::error_code error_init;
        boost::asio::write(socket_, boost::asio::buffer(send_init_), error_init);
        //(!error_init) ? ROS_INFO("Client sent init command!") : ROS_INFO("Init sent failed: ".append(error_init.message()));

        boost::system::error_code error_start;
        boost::asio::write(socket_, boost::asio::buffer(send_start_), error_start);
        //(!error_start) ? ROS_INFO("Client sent start command!") : ROS_INFO("Start sent failed: ".append(error_start.message()));

        /* Get the first packets and throw them away, as the first TCP messages received contain help text*/
        boost::asio::streambuf receive_buffer;
        boost::system::error_code error_receive;
        for (int i = 0; i <= 4; i++)
        {
            boost::asio::read(socket_, receive_buffer, boost::asio::transfer_at_least(1), error_receive);
        }
        RCLCPP_INFO(this->get_logger(), "Connection established");

    }

    void DataHandler::ReceiveGatewayData()
    {
        //ROS_INFO("Reading packets");
        boost::asio::streambuf receive_buffer;
        boost::system::error_code error_receive;
        // Read packet
        pApp_->iSLen = boost::asio::read(socket_, receive_buffer, boost::asio::transfer_at_least(1), error_receive);

        if (error_receive && error_receive != boost::asio::error::eof)
        {
            RCLCPP_INFO(this->get_logger(), "receive failed: %s", error_receive.message());
        }
        else
        {
            // Use system timestamp to stamp the incomping packet
            rostimestamp_ = ros_clock_.now();

            const char *data = boost::asio::buffer_cast<const char *>(receive_buffer.data());
            // Append packet to buffer
            memcpy(pApp_->pSBuf + pApp_->iRByt, data, pApp_->iSLen);
            // start of analysis from index 0
            pApp_->iSIdx = 0;
            pApp_->iSLen += pApp_->iRByt;

            // loop until we parsed the received packet
            while (pApp_->iSIdx < (pApp_->iSLen))
            {
                // extract and split into frames, a frame ranges from 4220 to 0d0a as per the Ixxat definition of the GenEthernet protocol
                int retVal = exIxxPkt(pApp_);

                if (retVal == 0)
                {
                    // A complete frame has been found in the buffer
                    // call the right function for this frame
                    //only predefined frames are allowed to not exceed the size of fnktpntr
                    
                    if (pApp_->pPkt->iCycleNmbr <= {{fnktpntr_array|count}} && pApp_->pPkt->iSlotNmbr <= {{fnktpntr_array[0]|count}})
                    {
                    (this->*fnktpntr_[pApp_->pPkt->iCycleNmbr][pApp_->pPkt->iSlotNmbr])(pApp_->pPkt->ab_data);
                    }

                    // the frame is parsed. the next frame starts where the current frame ended, so move the search index by the length of the previously parsed frame
                    pApp_->iSIdx = pApp_->iNIdx;

                    free(pApp_->pPkt); //Free the memory for the created object
                }
                else
                {
                    // there is no complete frame left, save the remains in the buffer and receive the next TCP packet
                    memcpy(pApp_->pSBuf, (pApp_->pSBuf + pApp_->iNIdx), pApp_->iRByt);
                    break;
                }
            }
        }
    }

    void DataHandler::blank(unsigned char *data)
    {
    }

    void DataHandler::SetRosNodeAndAdvertise()
    {
        RCLCPP_INFO(this->get_logger(), "Starting publishers");
        
        {% for pdu in list_of_publishers %}
        publisher_{{pdu}}_ = this->create_publisher<flexray_msgs::msg::{{pdu}}>("{{pdu}}", 1);
        {% endfor %}        

        RCLCPP_INFO(this->get_logger(), "Started publishers");
    }

    {% for frame in framedefs %}
    void DataHandler::{{frame["name"]}}(unsigned char *data)
    {
        {% for pdu in frame["pdus"] %}
        {% if pdu["update-bit"] %}
        if(checkBit(data, {{pdu["update-bit"]}})) {
            Publish_{{pdu["name"]}}(data + {{pdu["offset"]}});
        }
        {% else %}
            Publish_{{pdu["name"]}}(data + {{pdu["offset"]}});
        {% endif %}
        {% endfor %}
    }
    
    {% endfor %}

    {% for _, pdu in pdudefs.items() %}
    {% if pdu["signalinstances"] %}
    void DataHandler::Publish_{{pdu["name"]}}(unsigned char *raw_data_pointer)
    {
        flexray_msgs::msg::{{pdu["name"]}} phys_data;
        ixxat_gw_{{channel_name}}::{{pdu["name"]}}_raw *raw_data = (ixxat_gw_{{channel_name}}::{{pdu["name"]}}_raw *)raw_data_pointer;
        
        phys_data.header.stamp = rostimestamp_;
        
        {% for _ , signalinstance in pdu["signalinstances"].items() %}
        {% if signalinstance.__signal__.__compu_scale__%}
        phys_data.{{signalinstance.__signal__.__name__.lower()}} = raw_data->{{signalinstance.__signal__.__name__.lower()}}*{{signalinstance.__signal__.__compu_scale__[1]}}+{{signalinstance.__signal__.__compu_scale__[0]}};
        {% else %}
        phys_data.{{signalinstance.__signal__.__name__.lower()}} = raw_data->{{signalinstance.__signal__.__name__.lower()}};
        {% endif %}
        {% endfor %}

        publisher_{{pdu["name"]}}_->publish(phys_data);
    }
    {% endif %}
    {% endfor %}

    {% for _, muxpdu in muxpdudefs.items() %}
    void DataHandler::Publish_{{muxpdu["name"]}}(unsigned char *raw_data_pointer)
    {
        {% if muxpdu["staticpdu"]["signalinstances"]%}
        flexray_msgs::msg::{{muxpdu["staticpdu"]["name"]}} static_phys_data;
        ixxat_gw_{{channel_name}}::{{muxpdu["staticpdu"]["name"]}}_raw *static_raw_data = (ixxat_gw_{{channel_name}}::{{muxpdu["staticpdu"]["name"]}}_raw *)raw_data_pointer;
        static_phys_data.header.stamp = rostimestamp_;
        {% for _ , signalinstance in muxpdu["staticpdu"]["signalinstances"].items() %}
        {% if signalinstance.__signal__.__compu_scale__%}
        static_phys_data.{{signalinstance.__signal__.__name__.lower()}} = static_raw_data->{{signalinstance.__signal__.__name__.lower()}}*{{signalinstance.__signal__.__compu_scale__[1]}}+{{signalinstance.__signal__.__compu_scale__[0]}};
        {% else %}
        static_phys_data.{{signalinstance.__signal__.__name__.lower()}} = static_raw_data->{{signalinstance.__signal__.__name__.lower()}};
        {% endif %}
        {% endfor %}
        publisher_{{muxpdu["staticpdu"]["name"]}}_->publish(static_phys_data);
        {% else %}
        //Static PDU part is empty
        {% endif %}

        ixxat_gw_{{channel_name}}::{{muxpdu["name"]}}demux_raw *mux_raw_data = (ixxat_gw_{{channel_name}}::{{muxpdu["name"]}}demux_raw *)raw_data_pointer;
        switch (mux_raw_data->{{muxpdu["switchname"].lower()}}){
            {% for _, pdu in muxpdu["switchedpdus"].items() %}
            case {{pdu["switchcode"]}}:
                {
                flexray_msgs::msg::{{pdu["name"]}} switch_phys_data;
                ixxat_gw_{{channel_name}}::{{pdu["name"]}}_raw *switch_raw_data = (ixxat_gw_{{channel_name}}::{{pdu["name"]}}_raw *)raw_data_pointer;
                switch_phys_data.header.stamp = rostimestamp_;
                {% for _ , signalinstance in pdu["signalinstances"].items() %}
                {% if signalinstance.__signal__.__compu_scale__%}
                switch_phys_data.{{signalinstance.__signal__.__name__.lower()}} = switch_raw_data->{{signalinstance.__signal__.__name__.lower()}}*{{signalinstance.__signal__.__compu_scale__[1]}}+{{signalinstance.__signal__.__compu_scale__[0]}};
                {% else %}
                switch_phys_data.{{signalinstance.__signal__.__name__.lower()}} = switch_raw_data->{{signalinstance.__signal__.__name__.lower()}};
                {% endif %}
                {% endfor %}
                publisher_{{pdu["name"]}}_->publish(switch_phys_data);
                break;
                }
            {% endfor %}
        }
    }

    {% endfor %}



} // namespace ixxat_gw_{{channel_name}}
#endif