from __future__ import print_function

from threading import Thread
from flask import Flask, redirect
from flask_cors import CORS

import rospy
import moveit_commander
import moveit_msgs
import geometry_msgs
from niryo_one_python_api.niryo_one_api import *

from main import arm_script
import running
running.bool = False

rospy.init_node('move_group_python', anonymous=True, disable_signals=True)
group_name = 'arm'
group = moveit_commander.MoveGroupCommander(group_name)
NIRYO = NiryoOne()
CROUCHINGTIGER = [0, 0.428, -1.237, 0.012, 0.814, -0.046]

app = Flask(__name__)
CORS(app)

@app.route('/')
def index():
    return redirect('http://10.12.175.231:8000')


@app.route('/home')
def home():
    NIRYO.move_joints([0]*6)
    return "Home"


@app.route('/main')
def main():
    running.bool = True
    Thread(target=arm_script).start()
    return "Main"


@app.route('/stopmain')
def stopmain():
    running.bool = False
    return "Main Stopped"


@app.route('/crouchingtiger')
def crouchingtiger():
    NIRYO.move_joints(CROUCHINGTIGER)
    return "Crouching"


@app.route('/calibrate')
def calibrate():
    NIRYO.calibrate_manual()
    NIRYO.move_joints(CROUCHINGTIGER)
    NIRYO.move_joints([0]*6)
    return "Calibrated"


@app.route('/learning')
def learning():
    NIRYO.activate_learning_mode(1)
    return "Learning Mode"


@app.route('/em/<int:status>')
def em(status):
    NIRYO.digital_write(SW_1, status)
    return 'EM ' + str(status)


if __name__=='__main__':
    PORT = 1234
    print('Serving Flask server on port', PORT)
    app.run(host='0.0.0.0', port=PORT, debug=True, use_reloader=False)

