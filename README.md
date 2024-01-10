# [Introduction](#introduction)

**FRos** is a framework to integrate a [Fieldbus](https://en.wikipedia.org/wiki/Fieldbus) into the [Robot Operating System (ROS)](https://www.ros.org/).
This project aims to provide the necessary source code in the form of a so called node from a given .xml file for the FlexRay bus in the so called wildcard mapping functionality of the Ixxat FRC-EP170 or FRC-EP190 device. Please read more on the details in our main repo [here](https://github.com/AdriveLivingLab/FRos).

In case you have a *.dbc file, e.g. from CAN or from one of the mapping approaches inside the ACT tool, please also look into the main repo and use our [c-coderdbc fork](https://github.com/AdriveLivingLab/c-coderdbc).

# [FibexConverter](#fibexconverter)

This project is a fork from [FibexConverter](https://github.com/LarsVoelker/FibexConverter) by [LarsVoelker](https://github.com/LarsVoelker), who did an amazing and unique job providing a framework for processing the FlexRay *.xml description file.
What you find here is an extension of said processing into nodes for ROS1 noetic and ROS2 humble. The idea here is to observe all frames on the FlexRay bus (wildcard mapping) and extract their respective PDU's, each of which are assigned to their own topic and published to ROS. This then further opens up the possibilities to process, visualize and record the FlexRay communication. See further details on how we use that in our paper linked in the main repo.

# [Features](#features)

:ballot_box_with_check: ROS1 noetic and ROS2 humble supported

:ballot_box_with_check: FlexRay supported

:ballot_box_with_check: Channel A and Channel B supported

:ballot_box_with_check: Multiplex PDU's supported

:ballot_box_with_check: PDU's updating in different frames supported

:ballot_box_with_check: Optimized for computer resources with C++

:ballot_box_with_check: Only PDU updates, signaled by the update bit in the frame, are published 


Future features: Leveraging the device timestamps of the Ixxat device to even further improve the PDU timestamps. For current results, see the paper linked on your main repo, once it's published.

# [Usage](#usage)

## Convert a FIBEX file to ROS nodes:
Call the script like this:

```
python3 configuration_to_fros.py FIBEX your-file.xml
```

Now a folder named **your-file** will be created, where you find the subfolders **ROS1** and **ROS2**. Inside in each of these two folders a folder for the ROS message package is created and the packages for decoding the FlexRay communication is created, which we will now call fROS node. Depending on the number of FlexRay channels, one or two folders for the fROS nodes will be created here.

## Insert the IP address of your device

To adapt to your specific settings, e.g. IP settings of the gateway device, you will have to configure the nodes to use the correct IP and port.

**ROS1 noetic**

For the fROS nodes you will find the launch file `include/ixxat_gw.launch`. Inside there, edit the `<arg name="port_num_gw" default="19227"` and `<arg name="ip_address_gw" default="169.254.200.123"` to fit to your settings.

**ROS2 humble**

Edit the file `src/datahandler.cpp` in line 21 `port_ = "19227";` and in line 25 `ip_ = "169.254.200.123";` to fit to your settings.

# [Building](#building)

The message packages and the fROS nodes are fully supported by the standard ROS build tools **catkin_make** for ROS1 and **colcon build** for ROS2. Note: Remember to source after building the packages.

# [Launching](#launching)

**ROS1 noetic**

We provide launch files for ROS1. After sourcing the workspace, e.g with `source devel/setup.bash`, you can launch the node with `roslaunch ixxat_gw_{channel_name} ixxat_gw.launch`. The term `{channel_name}` depens on the provided .xml file, pressing TAB after typing `ixxat_gw_`, the full name will be autocompleted for you.

**ROS2 humble**

After sourcing the workspace, e.g. with `source install/setup.bash`, you can launch the node with `ros2 run ixxat_gw_{channel_name} ixxat_gw_{channel_name}`. The term `{channel_name}` depens on the provided .xml file, pressing TAB after typing `ixxat_gw_`, the full name will be autocompleted for you.

# [License](#license)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

# [Contact](#contact)

<div style="width: 50%; float:left" align="center">
  <a href="https://www.instagram.com/fza_hskempten/" style="text-decoration:none;"> &nbsp; <img src = "docs/instagram_ifm.svg" alt="instagram IFM" width=35px/></li> 
  <a href="https://www.youtube.com/@ifm8936" style="text-decoration:none;"> &nbsp;<img src = "docs/youtube_ifm.svg" alt="youtube IFM" width=35px/>
  <a href="https://de.linkedin.com/company/institute-for-driver-assistace-and-connected-mobility" style="text-decoration:none;"> &nbsp; <img src = "docs/linkedin_ifm.svg" alt="linkedin IFM" width=35px/>
  <a href="mailto:livinglab.info@hs-kempten.de" style="text-decoration:none;"> &nbsp; <img src = "docs/mail_ifm.svg" alt="email IFM" width=35px/>
  <br/>
  <a href="#" style="text-decoration:none;"> <img src = "docs/Kombi_Logo_farbig_freigestellt.png" alt="website FZA" width=200px/>
</div>

<div style="width: 50%; float:right" align="center">
  <a href="https://www.instagram.com/hmsnetworks/" style="text-decoration:none;"> &nbsp; <img src = "docs/instagram_hms.svg" alt="instagram HMS" width=35px/></li> 
  <a href="https://www.youtube.com/c/IxxatbyHMSNetworks" style="text-decoration:none;"> &nbsp;<img src = "docs/youtube_hms.svg" alt="youtube HMS" width=35px/>
  <a href="https://www.linkedin.com/company/hmsnetworks" style="text-decoration:none;"> &nbsp; <img src = "docs/linkedin_hms.svg" alt="linkedin HMS" width=35px/>
  <a href="mailto:info@hms-networks.de" style="text-decoration:none;"> &nbsp; <img src = "docs/mail_hms.svg" alt="email HMS" width=35px/>
  <br/>
  </br>
  <a href="#" style="text-decoration:none;"> <img src = "docs/hms_logo.png" alt="website HMS" width=100px/>
</div> 
