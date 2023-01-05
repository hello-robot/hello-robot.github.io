#!/bin/bash
set -o pipefail

do_factory_install='false'
if getopts ":f:" opt && [[ $opt == : ]]; then
    do_factory_install='true'
fi

source /etc/os-release
factory_osdir="$VERSION_ID"
if [[ ! $factory_osdir =~ ^(18.04|20.04)$ ]]; then
    echo "Could not identify OS. Please contact Hello Robot Support."
    exit 1
fi


echo "#############################################"
echo "STARTING NEW ROBOT INSTALL"
echo "#############################################"

timestamp='stretch_install_'`date '+%Y%m%d%H%M'`;
logdir="$HOME/stretch_user/log/$timestamp"
logfile_initial="$logdir/stretch_initial_configuration.txt"
logfile_system="$logdir/stretch_system_install.txt"
logfile_user="$logdir/stretch_user_install.txt"
logfile_dev_tools="$logdir/stretch_dev_tools_install.txt"
logzip="$logdir/stretch_robot_install_logs.zip"
mkdir -p $logdir

function echo_failure_help {
    zip -r $logzip $logdir/ > /dev/null
    echo ""
    echo "#############################################"
    echo "FAILURE. INSTALLATION DID NOT COMPLETE."
    echo "Look at the troubleshooting guide for solutions to common issues: https://docs.hello-robot.com/0.2/stretch-install/docs/robot_install#troubleshooting"
    echo "or contact Hello Robot support and include $logzip"
    echo "#############################################"
    echo ""
    exit 1
}

cd $HOME/stretch_install/factory/$factory_osdir
./stretch_initial_setup.sh $do_factory_install |& tee $logfile_initial
if [ $? -ne 0 ]; then
    echo_failure_help
fi

echo ""
cd $HOME/stretch_install/factory/$factory_osdir
./stretch_install_system.sh -l $logdir |& tee $logfile_system
if [ $? -ne 0 ]; then
    echo_failure_help
fi

echo ""
cd $HOME/stretch_install
./stretch_new_user_install.sh -l $logdir |& tee $logfile_user
if [ $? -ne 0 ]; then
    echo_failure_help
fi

if $do_factory_install; then
    echo ""
    cd $HOME/stretch_install/factory/$factory_osdir
    ./stretch_install_dev_tools.sh -l $logdir |& tee $logfile_dev_tools
    if [ $? -ne 0 ]; then
        echo_failure_help
    fi
fi

zip -r $logzip $logdir/ > /dev/null

echo ""
echo "#############################################"
echo "DONE! COMPLETE THESE POST INSTALL STEPS:"
echo " 1. Perform a FULL reboot by power cycling the robot"
echo " 2. Run 'RE1_migrate_params.py' in the command line"
echo " 3. Run 'RE1_migrate_contacts.py' in the command line"
echo " 4. Run 'REx_firmware_updater.py --install' in the command line"
echo " 5. If using the Stretch Dex Wrist gripper:"
echo "    a. Run 'stretch_new_dex_wrist_install.sh' in the command line"
echo "    b. Run 'REx_calibrate_guarded_contact.py' in the command line"
echo "#############################################"
echo ""
