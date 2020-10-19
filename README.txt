----------------------------------------------------------------------
GAZEBO HLA
----------------------------------------------------------------------
HLA example plugin to allow Gazebo interacting with other flight simulators.

For now, the plug-in is only compliant with the old standard HLA 1.3.

It has been mainly tested with CERTI HLA/RTI Middleware:
https://savannah.nongnu.org/projects/certi

For any question, please contact:
- jean-baptiste.chaudron@isae.fr

----------------------------------------------------------------------
INSTALLATION
----------------------------------------------------------------------

1) Install CERTI:
prerequise: Check if libxml2-dev, flex, bison package are installed
On Ubuntu: 
>> sudo apt install libxml2-dev flex bison

install process:
>> cd ${WHERE_YOU_WANNA_INSTALL}
>> git clone https://git.savannah.nongnu.org/git/certi.git
>> mkdir build
>> mkdir install
>> cd build
>> cmake -DCMAKE_INSTALL_PREFIX=../install ../certi/
>> make -j4 install

2) Install Gazebo:
http://gazebosim.org/
See gazebo plugins:
http://gazebosim.org/tutorials?tut=ros_gzplugins

Then this is a pretty common CMake project as well.

>> source ${WHERE_YOU_HAVE_INSTALLED_CERTI}/install/share/scripts/myCERTI_env.sh
>> git clone 
>> mkdir build
>> mkdir install
>> cd build
>> cmake -DCMAKE_INSTALL_PREFIX=../install ../gazebo_hla
>> make -j4 install
