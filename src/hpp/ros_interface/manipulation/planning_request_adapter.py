from hpp.ros_interface.planning_request_adapter import PlanningRequestAdapter as Parent

class PlanningRequestAdapter (Parent):
    subscribersDict = {
            "motion_planning": {
                "set_goal" : [PlanningGoal, "set_goal" ],
                "request" : [Empty, "request" ],
                "manipulation" : {
                    'init_position_mode': [ String, "init_position_mode" ],
                    'set_init_pose': [ PlanningGoal, "set_init_pose" ],
                    },
                },
            }

    def __init__ (self, topicStateFeedback):
        super(PlanningRequestAdapter, self).__init__(topicStateFeedback)

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
