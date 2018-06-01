from hpp.ros_interface.planning_request_adapter import PlanningRequestAdapter as Parent
from sot_hpp_msgs.srv import SetString

class PlanningRequestAdapter (Parent):
    servicesDict = {
            "motion_planning": {
                "manipulation" : {
                    'set_robot_prefix': [ SetString, "set_robot_prefix" ],
                    },
                },
            }

    def __init__ (self, topicStateFeedback):
        super(PlanningRequestAdapter, self).__init__(topicStateFeedback)

    def set_robot_prefix (self, req):
        self.robot_name = req.value
        return SetStringResponse(True)

    def _stateEstimation (self, hpp, manip, qsensor, dev):
        #TODO find the closest state ? (the one with lower error)
        selectedProblem = hpp.problem.getSelected("problem")
        stateId = rospy.get_param("estimation/state/"+selectedProblem)
        return stateId

    def _estimation (self, hpp, qsensor, dev):
        """
        Generate a configuration that make 'sense':
        - no collisions (between objects, robots and world)
        - the current constraints are satisfied
        """
        manip = self._manip ()
        stateId = self._stateEstimation (hpp, manip, qsensor, dev)

        _setGaussianShooter (hpp, qsensor, stddev)
        qsemantic = qsensor[:]
        while True:
            valid, qsemantic, err = manip.problem.applyConstraints (stateid, qsemantic)
            if valid:
                valid, msg = hpp.robot.isConfigValid (qsemantic)
                if valid: break
            qsemantic = hpp.robot.shootRandomConfig()

        return qsemantic

    def _set_init_pose (self, msg):
        self.q_init = self.get_object_root_joints()
        # TODO: WHEN NEEDED Get the joint states of the objects.

    def get_object_root_joints(self):
        hpp = self._hpp()
        # Get the name of the objects required by HPP
        root_joints = [el for el in hpp.robot.getAllJointNames() if "root_joint" in el]
        root_joints = [el.split("/")[0] for el in root_joints if not self.robot_name in el]

        for obj in root_joints:
            if self.tfListener.frameExists(obj):
                p, q = self.tfListener.lookupTransform(self.world_frame, obj, rospy.Time(0))
                hpp.robot.setJointConfig(obj + "/root_joint", p + q)
            else:
                print obj + " is not published on the TF tree but is needed to plan the trajectory of the robot."

        return hpp.robot.getCurrentConfig()
