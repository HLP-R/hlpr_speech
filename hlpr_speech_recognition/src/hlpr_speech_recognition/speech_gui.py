import rospy
import sys
import rospkg
import yaml
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
      #kps_path = rospack.get_path('hlpr_speech_recognition') + '/data/kps.txt'

      #mapping from keywords to commands
  
      # Get the yaml file param, or use the default one
      self.DEFAULT_YAML_FILE = "kps.yaml"

      if rospy.has_param("/speech_gui/yaml_file"):
        self._yamlKeywords = rospy.get_param("/speech_gui/yaml_file")
      else:
        self._yamlKeywords = "kps.yaml"
      
      kps_path = rospack.get_path('hlpr_speech_recognition') + '/data/' + self._yamlKeywords

      self.commands = []
      for data in yaml.load_all(file(kps_path,'r')):
          for key, value in data.iteritems():
            if key == "speech":
              for i in value:
                self.commands.append(i)
 
      positions = [(i,j) for i in range(len(self.commands)) for j in range(3)]
           
      for position, name in zip(positions, self.commands):
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
      self.pub = rospy.Publisher('hlpr_speech_commands', String)
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



