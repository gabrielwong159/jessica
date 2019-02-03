from flask import Flask, render_template
import rospy
import moveit_commander
import moveit_msgs
import geometry_msgs
from niryo_one_python_api.niryo_one_api import *

from main import *
import running
running.bool = False

rospy.init_node('move_group_python', anonymous=True, disable_signals=True)
group_name='arm'
group = moveit_commander.MoveGroupCommander(group_name)
NIRYO = NiryoOne()

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/home')
def home():
    NIRYO.move_joints([0,0,0,0,0,0])
    return "Home"

@app.route('/main')
def main():
    running.bool = True
    arm_script()
    return "Main"

@app.route('/stopmain')
def stopmain():
    running.bool = False
    return "Main Stopped"

@app.route('/crouchingtiger')
def crouchingtiger():
    NIRYO.move_joints([0, 0.428, -1.237, 0.012, 0.814, -0.046])
    return "Crouchings"

@app.route('/calibrate')
def calibrate():
    NIRYO.calibrate_manual()
    return "Calibrated"

@app.route('/learning')
def learning():
    NIRYO.activate_learning_mode(1)
    return "Learning Mode"

if __name__=='__main__':
    app.run(host='0.0.0.0',port=1234,debug=True)
