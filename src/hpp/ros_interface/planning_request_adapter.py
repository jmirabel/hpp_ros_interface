#!/usr/bin/env python
import rospy, hpp.corbaserver
from tf import TransformListener
from .client import HppClient
from .trajectory_publisher import JointPathCommandPublisher
from dynamic_graph_bridge_msgs.msg import Vector
from sot_hpp_msgs.msg import ProblemSolved, PlanningGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Empty, Bool
from math import cos, sin
from threading import Lock
from omniORB import CORBA
import traceback

def _fillVector(input, segments):
    """ Returns a vector that contains the segments extracted from input """
    output = []
    for s in segments:
        output.extend (input[s[0]:s[0]+s[1]])
    return output

def init_node ():
    rospy.init_node('planning_request_adapter')

def _setGaussianShooter (hpp, q, dev):
    hpp.robot.setCurrentConfig (q)
    hpp.problem.setParameter ("ConfigurationShooter/Gaussian/standardDeviation",
            CORBA.Any(CORBA.TC_double, dev))
    hpp.problem.selectConfigurationShooter ("Gaussian")

class PlanningRequestAdapter(HppClient):
    subscribersDict = {
            "motion_planning": {
                "set_goal" : [PlanningGoal, "set_goal" ],
                "estimate" : [Empty, "estimate" ],
                "request" : [Empty, "request" ],
                "param" : {
                    'init_position_mode': [ String, "init_position_mode" ],
                    'set_init_pose': [ PlanningGoal, "set_init_pose" ],
                    },
                },
            }
    publishersDict = {
            "motion_planning": {
                "problem_solved" : [ ProblemSolved, 1],
                "estimation"          : [ Vector, 1],
                "semantic_estimation" : [ Vector, 1],
                },
            }
    modes = [ "current", "estimated", "user_defined" ]

    def __init__ (self, topicStateFeedback):
        super(PlanningRequestAdapter, self).__init__ ()
        self.subscribers = self._createTopics ("", self.subscribersDict, True)
        self.publishers = self._createTopics ("", self.publishersDict, False)
        self.topicStateFeedback = topicStateFeedback
        self.setHppUrl()
        self.q_init = None
        self.init_mode = "user_defined"
        self.get_current_state = None
        self.tfListener = TransformListener()
        self.mutexSolve = Lock()
        self.world_frame = "world"
        self.robot_name = ""
        self.robot_base_frame = None

    def _hpp (self, reconnect = True):
        hpp = super(PlanningRequestAdapter, self)._hpp(reconnect)
        rjn = hpp.robot.getAllJointNames()[1]
        try:
          self.robot_name = rjn[:rjn.index('/')+1]
        except ValueError:
          self.robot_name = ""
        self.robot_base_frame = hpp.robot.getLinkNames(self.robot_name + "root_joint")[0]
        rootJointType = hpp.robot.getJointType(self.robot_name + "root_joint").lower()
        if rootJointType == "anchor":
            self.setRootJointConfig = lambda x : None
        elif rootJointType == "jointmodelfreeflyer":
            self.setRootJointConfig = lambda x : hpp.robot.setJointConfig(self.robot_name + "root_joint", x)
        elif rootJointType == "jointmodelplanar":
            self.setRootJointConfig = lambda x : hpp.robot.setJointConfig(self.robot_name + "root_joint", x[0:2] + [x[6]**2 - x[5]**2, 2 * x[5] * x[6]] )
        else:
            self.setRootJointConfig = lambda x : (_ for _ in ()).throw(Exception("Root joint type is not understood. It must be one of (anchor, freeflyer, anchor) and not " + str(rootJointType)))
        return hpp

    def _JointStateToConfig(self, placement, js_msg):
        hpp = self._hpp()
        if self.q_init is not None:
            hpp.robot.setCurrentConfig(self.q_init)
        self.setRootJointConfig(placement)
        for jn, q in zip(js_msg.name, js_msg.position):
            size = hpp.robot.getJointConfigSize(self.robot_name + jn)
            if size == 2:
                hpp.robot.setJointConfig(self.robot_name + jn, [cos(q), sin(q)])
            else:
                hpp.robot.setJointConfig(self.robot_name + jn, [q])
        return hpp.robot.getCurrentConfig()

    def _estimate (self, hpp, qsensor, stddev, transition = False):
        """
        Generate a configuration that make 'sense':
        - no collisions (between objects, robots and world)
        - the current constraints are satisfied
        """
        _setGaussianShooter (hpp, qsensor, stddev)
        qsemantic = qsensor[:]
        while True:
            valid, qsemantic, err = hpp.problem.applyConstraints (qsemantic)
            if valid:
                valid, msg = hpp.robot.isConfigValid (qsemantic)
                if valid: break
            qsemantic = hpp.robot.shootRandomConfig()

        return qsemantic

    def set_goal (self, msg):
        hpp = self._hpp()
        q_goal = self._JointStateToConfig(msg.base_placement, msg.joint_state)
        hpp.problem.resetGoalConfigs()
        hpp.problem.addGoalConfig(q_goal)

    def request (self, msg):
        self.mutexSolve.acquire()
        try:
            if self.init_mode == "current":
                self.set_init_pose (PlanningGoal(self.last_placement, self.last_joint_state))
            elif self.init_mode == "estimated":
                self.q_init = self.estimated_config
            hpp = self._hpp()
            rospy.loginfo("init done")
            rospy.loginfo(str(self.q_init))
            hpp.problem.setInitialConfig(self.q_init)
            rospy.loginfo("configured")
            t = hpp.problem.solve()
            rospy.loginfo("solved")
            pid = hpp.problem.numberPaths() - 1
            time = t[0] * 3600 + t[1] * 60 + t[2] + t[3] * 1e-3
            # print "Solved in", t, ", path id", pid
            rospy.loginfo("Path ({}) to reach target found in {} seconds".format(pid, t))
            rospy.sleep(0.1)
            self.publishers["motion_planning"]["problem_solved"].publish (ProblemSolved(True, "success", pid))
            if rospy.get_param("/hpp/publish_path", True):
                topic = rospy.get_param("/hpp/topic_robot_controller", "joint_path_command")
                rospy.loginfo("Publish path to " + str(topic))
                jpc = JointPathCommandPublisher(topic = topic)
                jpc.publish(pid)
        except Exception as e:
            rospy.loginfo (str(e))
            rospy.loginfo (traceback.format_exc())
            rospy.sleep(0.1)
            self.publishers["motion_planning"]["problem_solved"].publish (ProblemSolved(False, str(e), -1))
        finally:
            self.mutexSolve.release()

    def estimate (self, msg):
        hpp = self._hpp()
        self.mutexSolve.acquire()
        try:
            if "estimation" not in hpp.problem.getAvailable("problem"):
                raise Error("No 'estimation' problem in HPP server.")
            stddev = rospy.get_param ("estimation/std_dev")
            qsensor = self._JointStateToConfig (self.last_placement, self.last_joint_state)
            hpp.problem.selectProblem("estimation")
            qsemantic = self._estimation (hpp, qsensor, stddev)
            self.publishers["motion_planning"]["semantic_estimation"].publish (Vector(qsemantic))

            hpp.problem.selectProblem("default")
            self.estimated_config = self._estimation (hpp, qsemantic, stddev, transition=True)
            self.publishers["motion_planning"]["estimation"].publish (Vector(self.estimated_config))

            hpp.problem.selectProblem("estimation")
            hpp.problem.setInitialConfig (qsemantic)
            hpp.problem.resetGoalConfigs ()
            hpp.problem.addGoalConfig(self.estimated_config)
            t = hpp.problem.solve()
            pid = hpp.problem.numberPaths() - 1
            time = t[0] * 3600 + t[1] * 60 + t[2] + t[3] * 1e-3
            rospy.loginfo("Path ({}) to reach target found in {} seconds".format(pid, t))
            self.publishers["motion_planning"]["problem_solved"].publish (ProblemSolved(True, "success", pid))
        except Exception as e:
            rospy.loginfo (str(e))
            rospy.loginfo (traceback.format_exc())
            rospy.sleep(0.1)
            self.publishers["motion_planning"]["problem_solved"].publish (ProblemSolved(False, str(e), -1))
        finally:
            hpp.problem.selectProblem("default")
            self.mutexSolve.release()

    def init_position_mode(self, msg):
        if msg.data in self.modes:
            if msg.data == self.init_mode: return
            self.init_mode = msg.data
            rospy.loginfo("Initial position mode: %s" % msg.data)
            if msg.data == "current" or msg.data == "estimated":
                self.get_current_state = rospy.Subscriber (self.topicStateFeedback, JointState, self.get_joint_state)
            else:
                self.get_current_state = None

    def get_joint_state (self, msg):
        self.last_joint_state = msg
        try:
            base = "base_link"
            p, q = self.tfListener.lookupTransform(self.world_frame, base, rospy.Time(0))
            self.last_placement = p + q
        except Exception as e:
            print "non"
            rospy.loginfo( str(e) )
            pass

    def set_init_pose(self, msg):
        self.q_init = self._JointStateToConfig(msg.base_placement, msg.joint_state)
        self._set_init_pose (msg)

    def _set_init_pose (self, msg):
        """ To allow reimplementation """
        pass
