
import rospy
from nav_scripts.movebase_driver import BaseBumperChecker, RobotImpls, RobotImpl as BaseImpl
from gazebo_msgs.msg import ContactsState




class BumperChecker(BaseBumperChecker):
    def __init__(self):
        super().__init__(name="quadrotor_bumper")
        self.sub = rospy.Subscriber("hummingbird/bumper", ContactsState, self.bumperCB, queue_size=5)

    def bumperCB(self,data):
        if len(data.states) > 0:
            self.collided = True


class RobotImpl(BaseImpl):
    name = "quadrotor"

    def get_terminal_conditions(self):
        return [BumperChecker()]

RobotImpls.register(RobotImpl)

