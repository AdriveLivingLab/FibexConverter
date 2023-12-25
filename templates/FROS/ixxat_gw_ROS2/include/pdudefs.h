// typedef struct for pdu structure
#ifndef IXXAT_ROS_DRIVER_PDUDEFS_H_
#define IXXAT_ROS_DRIVER_PDUDEFS_H_

#include <stdint.h>

namespace ixxat_gw_{{channel_name}}
{
{% for _, pdu in pdudefs.items() %}
    typedef struct __attribute__((packed, aligned(1))) {{pdu["name"]}}_raw
    {
    {% for sig in pdu["signal_list"] %}
        {{sig["type"]}} {{sig["name"].lower()}} : {{sig["length"]}};
    {% endfor %}
    } {{pdu["name"]}}_raw;

{% endfor %}

} // namespace ixxat_gw_{{channel_name}}
#endif