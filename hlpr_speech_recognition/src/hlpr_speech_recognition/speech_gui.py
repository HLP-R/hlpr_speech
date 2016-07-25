import rospy
import sys
import rospkg
from std_msgs.msg import String
from PyQt4 import QtGui, QtCore

class SpeechGui(QtGui.QWidget):
  def __init__(self):
      QtGui.QWidget.__init__(self)
 
      newFont = QtGui.QFont("Times", 24, QtGui.QFont.Bold)

      # Add a main layout
      mainLayout = QtGui.QVBoxLayout(self)

      # Add buttons with the commands
      grid = QtGui.QGridLayout()
      grid.setSpacing(20)

      # get an instance of RosPack with the default search paths
      rospack = rospkg.RosPack()
      kps_path = rospack.get_path('hlpr_speech_recognition') + '/data/kps.txt'

      # Read commands from the kps.txt file and populate the list
      with open(kps_path) as f:
          commands = f.read().splitlines()
         
      positions = [(i,j) for i in range(len(commands)) for j in range(3)]
        
      for position, name in zip(positions, commands):
          button = QtGui.QPushButton(name)
          button.setObjectName('%s' % name)
          button.setFont(newFont)
          button.setStyleSheet("background-color: #ccffe6")
          button.clicked.connect(self.handleButton)
          grid.addWidget(button, *position)

      mainLayout.addLayout(grid)
      mainLayout.addStretch()
      
      # Show the GUI 
      self.adjustSize()
      self.setWindowTitle("Speech Commands Interface")
      self.show()
      self.raise_()

      # Create the publisher to publish the commands to
      self.pub = rospy.Publisher('speech_commands', String)
      rospy.init_node("speech_gui")
  
  # Button handler after its clicked
  def handleButton(self):
      clicked_button = self.sender()

      # Publish everytime a command is selected from the combo box
      print (str(clicked_button.objectName()))
      self.pub.publish(str(clicked_button.objectName()))

def gui_start():
    app = QtGui.QApplication(sys.argv)
    sg = SpeechGui()
    sys.exit(app.exec_())



