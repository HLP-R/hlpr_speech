# Speech Installation Instructions

Currently, hlp-r speech works with predefined commands. It uses the keyphrase spotting feature of pocketsphinx. If you want more generic speech recognition functionality, see http://wiki.ros.org/pocketsphinx.

##Installation
1. Install missing packages:
  - `sudo apt-get install python-espeak`
  - `sudo apt-get install python-pyaudio`

2. Install sphinxbase
  1. Download the sphinxbase package from [here](https://sourceforge.net/projects/cmusphinx/files/sphinxbase/5prealpha/)
  2. Set up sphinxbase by following the tutorial [here](https://github.com/cmusphinx/sphinxbase).
    - You may also choose to clone pocketsphinx and sphinxbase from the github links above instead of downloading them first. 

3. Install pocketsphinx
  1. Download the pocketsphinx package from [here](https://sourceforge.net/projects/cmusphinx/files/pocketsphinx/5prealpha/)
  2. Set up pocketsphinx by following the tutorial [here](https://github.com/cmusphinx/pocketsphinx).

4. Set up the speech package
  1. Add the following line to your ~/.bashrc file:
    - `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib`
  2. "Install" the package by adding a link to the speech package from your workspace.
    - e.g. `ln -s ~/PATH/TO/SIMLAB_REPO/hello_world/speech/ ~/YOUR_WORKSPACE_NAME/src/`
  3. Run `catkin_make` from your workspace folder.

##Creating a dictionary for the speech commands.
  1. Go to the data folder

  2. Add the desired commands to the kps.txt file
  
  3. Go to [this link](http://www.speech.cs.cmu.edu/tools/lmtool-new.html) and generate the .dic and .lm file. Replace the previous files with the new ones and rename them kps.dic and kps.lm respectively.

##Running speech recognition
1. To run the publisher node: `rosrun speech speech_recognizer`

2. To run the listener node: `rosrun speech speech_listener`. This listener is an example of how to utilize the recognizer

3. To run the speech commands interface (GUI): `rosrun speech speech_gui`. This is to fake the speech commands.




