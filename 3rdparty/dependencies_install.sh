#!/bin/sh


#------------------------------------
if  [  "$1" = '-y' ] || [  "$1"  =  "-Y" ] ; then
all_yes="Y"
fi
#------------------------------------

#------------------------------------

if [  "${all_yes}"  != Y ];  then
read -p "INSTALL eigen3? Y OR N :" var
fi

if [ "$var" = 'Y' ]  ||  [ "$var" = 'y'  ]    ||  [  "$all_yes" = "Y" ]; then
sudo apt-get install libeigen3-dev  -y
fi
#------------------------------------
#------------------------------------

if [  "${all_yes}"  != Y ];  then
read -p "INSTALL boost? Y OR N :" var
fi

if [ "$var" = 'Y' ]  ||  [ "$var" = 'y'  ]    ||  [  "$all_yes" = "Y" ]; then
sudo apt-get install libboost-all-dev -y
fi         
#------------------------------------
#------------------------------------
        
if [  "${all_yes}"  != Y ];  then
read -p "INSTALL urdfdom? Y OR N :" var
fi

if [ "$var" = 'Y' ]  ||  [ "$var" = 'y'  ]    ||  [  "$all_yes" = "Y" ]; then
sudo apt-get install liburdfdom-dev  -y
fi        
#------------------------------------
#------------------------------------


if [  "${all_yes}"  != Y ];  then
read -p "INSTALL tinyxml2? Y OR N :" var
fi

if [ "$var" = 'Y' ]  ||  [ "$var" = 'y'  ]    ||  [  "$all_yes" = "Y" ]; then
sudo apt-get install libtinyxml2-dev   -y
fi         
#------------------------------------
#------------------------------------


if [  "${all_yes}"  != Y ];  then
read -p "INSTALL tinyxml? Y OR N :" var
fi

if [ "$var" = 'Y' ]  ||  [ "$var" = 'y'  ]    ||  [  "$all_yes" = "Y" ]; then
sudo apt-get install libtinyxml-dev      -y
fi         
#------------------------------------
#------------------------------------

if [  "${all_yes}"  != Y ];  then
read -p "INSTALL console-bridge? Y OR N :" var
fi

if [ "$var" = 'Y' ]  ||  [ "$var" = 'y'  ]    ||  [  "$all_yes" = "Y" ]; then
sudo apt-get install libconsole-bridge-dev -y
fi         
#------------------------------------












