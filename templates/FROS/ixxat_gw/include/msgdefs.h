// typedef struct for pdu structure
#ifndef IXXAT_ROS_DRIVER_MSGDEFS_H_
#define IXXAT_ROS_DRIVER_MSGDEFS_H_

{% for _, pdu in pdudefs.items() %}
#include "flexray_msgs/{{pdu["name"]}}.h"
{% endfor %}
#endif