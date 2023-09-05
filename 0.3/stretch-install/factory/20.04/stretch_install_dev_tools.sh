#!/bin/bash
set -e

REDIRECT_LOGDIR="$HOME/stretch_user/log"
if getopts ":l:" opt && [[ $opt == "l" && -d $OPTARG ]]; then
    REDIRECT_LOGDIR=$OPTARG
fi
REDIRECT_LOGFILE="$REDIRECT_LOGDIR/stretch_install_dev_tools.`date '+%Y%m%d%H%M'`_redirected.txt"

echo "#############################################"
echo "INSTALLATION OF DEV TOOLS FOR HELLO ROBOT INTERNAL PRODUCTION"
echo "#############################################"

echo "Install Typora"
wget -qO - https://typora.io/linux/public-key.asc | sudo apt-key add - &>> $REDIRECT_LOGFILE
sudo add-apt-repository 'deb https://typora.io/linux ./' &>> $REDIRECT_LOGFILE
sudo apt-get update >> $REDIRECT_LOGFILE
sudo apt-get install --yes typora >> $REDIRECT_LOGFILE

echo "Install tools for system QC and bringup"
pip3 install -q --no-warn-script-location twine &>> $REDIRECT_LOGFILE
pip3 install -q --no-warn-script-location gspread &>> $REDIRECT_LOGFILE
pip3 install -q --no-warn-script-location gspread-formatting &>> $REDIRECT_LOGFILE
pip3 install -q --no-warn-script-location oauth2client rsa==3.4 &>> $REDIRECT_LOGFILE
pip3 install -q --no-warn-script-location mkdocs mkdocs-material mkdocstrings==0.17.0 pytkdocs[numpy-style] jinja2==3.0.3 &>> $REDIRECT_LOGFILE

echo "Cloning repos"
cd ~/repos/
git clone https://github.com/hello-robot/stretch_install.git >> $REDIRECT_LOGFILE
git clone https://github.com/hello-robot/stretch_factory.git >> $REDIRECT_LOGFILE
git clone https://github.com/hello-robot/stretch_body.git >> $REDIRECT_LOGFILE
git clone https://github.com/hello-robot/stretch_firmware.git >> $REDIRECT_LOGFILE
git clone https://github.com/hello-robot/stretch_fleet.git >> $REDIRECT_LOGFILE
git clone https://github.com/hello-robot/stretch_production_tools.git >> $REDIRECT_LOGFILE
git clone https://github.com/hello-robot/stretch_production_data.git >> $REDIRECT_LOGFILE
git clone https://github.com/hello-robot/hello-robot.github.io >> $REDIRECT_LOGFILE
git clone https://github.com/hello-robot/stretch_docs >> $REDIRECT_LOGFILE

echo "Install stretch_production_tools"
cd stretch_production_tools/python
pip3 install -e .

# update .bashrc to add body code directory to the Python path

#echo "export PATH=\${PATH}:~/repos/stretch_body/python/bin" >> ~/.bashrc
#echo "export PATH=\${PATH}:~/repos/stretch_factory/python/tools" >> ~/.bashrc

#echo "export PYTHONPATH=\${PYTHONPATH}:~/repos/stretch_factory/python/tools" >> ~/.bashrc
#echo "export PYTHONPATH=\${PYTHONPATH}:~/repos/stretch_body/python" >> ~/.bashrc

#echo "Source new bashrc now...Done."
#echo ""
