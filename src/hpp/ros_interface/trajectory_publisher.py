#!/usr/bin/env python
import rospy, hpp.corbaserver
import numpy as np
from .client import HppClient
from sot_hpp_msgs.msg import *
from sot_hpp_msgs.srv import *
import ros_tools
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import Queue
from collections import deque
from dynamic_graph_bridge_msgs.msg import Vector
from geometry_msgs.msg import Vector3, Quaternion, Transform
from std_msgs.msg import UInt32, Empty
import std_srvs.srv

def _fillVector(input, segments):
    """ Returns a vector that contains the segments extracted from input """
    output = []
    for s in segments:
        output.extend (input[s[0]:s[0]+s[1]])
    return output

def listToVector3(l):
    return Vector3 (l[0], l[1], l[2])
def listToQuaternion(l):
    return Quaternion (l[0], l[1], l[2], l[3])
def listToTransform(l):
    return Transform(listToVector3(l[0:3]), listToQuaternion(l[3:7]))

def init_node ():
    rospy.init_node('joint_path_command_publisher')


class JointPathCommandPublisher:
    def __init__ (self, topic = 'joint_path_command', hasVelocity = False, client = hpp.corbaserver.Client()):
        self.hpp = client
        self.pub = rospy.Publisher(topic, JointTrajectory, queue_size=10)
        self.solved = None
        self.msg = JointTrajectory()
        self.msg.header.frame_id = ""
        self.hasVelocity = hasVelocity
        self.setJointNames(self.hpp.robot.getJointNames())

    def setJointNames (self, jointNames):
        self.msg.joint_names = jointNames
        self.configSegments = list()
        self.velocitySegments = list()
        rkCfg = rkVel = 0
        for jn in jointNames:
            szCfg = self.hpp.robot.getJointConfigSize (jn)
            szVel = self.hpp.robot.getJointNumberDof (jn)
            self.configSegments   += [[rkCfg, szCfg ]]
            self.velocitySegments += [[rkVel, szVel ]]
            rkCfg += szCfg
            rkVel += szVel

    def _makeTrajectoryPoint(self, pathId, t, pointTime, pointId):
        q = self.hpp.problem.configAtParam(pathId, t)
        self.msg.points[pointId].positions = _fillVector (q, self.configSegments)
        if self.hasVelocity:
            v = self.hpp.problem.derivativeAtParam (pathId, 1, t)
            self.msg.points[pointId].velocities = _fillVector (v, self.velocitySegments)
        self.msg.points[pointId].time_from_start = rospy.Duration(pointTime)

    def publish(self, pathId, dt = 0.05, scale = 1):
        """
        dt: time between samples of the path in HPP
        scale: ratio (time between ROS points) / dt
        """
        self.msg.points = []
        pathLength = self.hpp.problem.pathLength(pathId)
        t = 0.
        last = False
        while True:
            self.msg.points.append(JointTrajectoryPoint())
            self._makeTrajectoryPoint (pathId, t, scale * t, len(self.msg.points)-1)
            if last: break
            t += dt
            if t > pathLength:
                t = pathLength
                last = True
        self.msg.header.stamp = rospy.Time.now()
        self.pub.publish(self.msg)
        self.msg.header.seq += 1

    def solve(self, goal, wait = True):
        if wait:
            self.solved = None
        elif self.solved is None:
            self.solved = rospy.Subscriber('/motion_planning/problem_solved', ProblemSolved, self._solved)
        solve = rospy.Publisher('/motion_planning/request', JointState, latch=True, queue_size=1)
        js = JointState(name = self.msg.joint_names, position = goal)
        solve.publish(js)
        if wait:
            msg = rospy.wait_for_message ('/motion_planning/problem_solved', ProblemSolved)
            if msg.success:
                self.publish(msg.path_id)
            else:
                print "Failed:", msg.msg

    def _solved(self, msg):
        if msg.success:
            print "Solved", msg.path_id
        else:
            print "Failed:", msg.msg

class HppOutputQueue(HppClient):
    subscribersDict = {
            "hpp": {
                "target": {
                    "read_path": [ UInt32, "read" ],
                    "read_subpath": [ ReadSubPath, "readSub" ],
                    "publish": [ Empty, "publish" ]
                    },
                },
            }
    publishersDist = {
            "read_path_done": [ UInt32, 1 ],
            "publish_done": [ Empty, 1 ]
            }
    servicesDict = {
            "hpp": {
                "target": {
                    "set_joint_names": [ SetJointNames, "setJointNames", ],
                    "reset_topics": [ std_srvs.srv.Empty, "resetTopics", ],
                    "add_center_of_mass": [ SetString, "addCenterOfMass", ],
                    "add_operational_frame": [ SetString, "addOperationalFrame", ],
                    "add_center_of_mass_velocity": [ SetString, "addCenterOfMassVelocity", ],
                    "add_operational_frame_velocity": [ SetString, "addOperationalFrameVelocity", ],

                    "publish_first": [ std_srvs.srv.Empty, "publishFirst", ],
                    }
                }
            }

    class Topic (object):
        def __init__ (self, reader, topicPub, MsgType, data = None):
            self.reader = reader
            self.pub = rospy.Publisher("/hpp/target/" + topicPub, MsgType, latch=False, queue_size=1000)
            self.MsgType = MsgType
            self.data = data

        def init (self, hpp):
            pass

        def read (self, hpp):
            return self.reader(hpp, self.data)

        def publish (self, msg):
            self.pub.publish(msg)
    class ConstantTopic (object):
        def __init__ (self, value, topicPub, MsgType):
            self.pub = rospy.Publisher("/hpp/target/" + topicPub, MsgType, latch=False, queue_size=1000)
            self.msg = value

        def init (self, hpp):
            pass

        def read (self, hpp):
            return None

        def publish (self, msg):
            assert msg==None
            self.pub.publish(self.msg)
    class SentToViewer (object):
        def __init__ (self, parent):
            self.parent = parent

        def read (self, hpp, uv):
            if uv:
                pos = list()
                for j, prefix, o in self.parent.viewer.robotBodies:
                    pos.append ( (prefix + o, hpp.robot.getLinkPosition (o) ) )
                return tuple(pos)
            else:
                return ()

        def publish (self, msg):
            if not isinstance(msg, (tuple, list)) or len(msg) == 0: return
            for name, pos in msg:
                self.parent.viewer.client.gui.applyConfiguration(name, pos)
            self.parent.viewer.client.gui.refresh()

    def __init__ (self):
        super(HppOutputQueue, self).__init__ (withViewer = False)

        self.frequency = 1. / rospy.get_param ("/sot_controller/dt") # Hz
        self.viewerFreq = 25 # Hz
        self.queue_size = 10 * self.frequency
        self.queue = Queue.Queue (self.queue_size)
        self.queueViewer = deque ()

        self.setJointNames (SetJointNamesRequest(self._hpp().robot.getJointNames()))

        self.subscribers = ros_tools.createTopics (self, "", self.subscribersDict, True)
        self.services = ros_tools.createServices (self, "", self.servicesDict, True)
        self.pubs = ros_tools.createTopics(self, "/hpp/target", self.publishersDist, subscribe = False)
        self.reading = False
        self.firstMsgs = None

        self.resetTopics ()

    def resetTopics (self, msg = None):
        self.topicViewer = self.SentToViewer (self)
        self.topics = [
                self.Topic (self._readConfigAtParam  , "position", Vector),
                self.Topic (self._readVelocityAtParam, "velocity", Vector),
                ]
        hpp = self._hpp()
        self.topics[0].init(hpp)
        self.topics[1].init(hpp)
        rospy.loginfo("Reset topics")
        if msg is not None:
            return std_srvs.srv.EmptyResponse()

    def addCenterOfMass (self, req):
        # TODO check that com exists
        comName = req.value
        n = "com"
        if comName != "":
            n += "/" + comName
        self.topics.append (
                self.Topic (self._readCenterOfMass, n, Vector3, data = comName),
                )
        self.topics[-1].init(self._hpp())
        rospy.loginfo("Add topic " + n)
        return SetStringResponse(True)

    def addCenterOfMassVelocity (self, req):
        # TODO check that com exists
        comName = req.value
        n = "velocity/com"
        if comName != "":
            n += "/" + comName
        self.topics.append (
                self.Topic (self._readCenterOfMassVelocity, n, Vector3, data = comName),
                )
        self.topics[-1].init(self._hpp())
        rospy.loginfo("Add topic " + n)
        return SetStringResponse(True)

    def _getFrameType (self, n):
        _hpp = self._hpp()
        try:
            _hpp.robot.getJointPosition (n)
            return "joint"
        # except hpp.Error:
        except:
            pass
        try:
            _hpp.robot.getLinkPosition (n)
            return "link"
        # except hpp.Error:
        except:
            pass
        try:
            _hpp.obstacle.getObstaclePosition (n)
            return "obstacle"
        # except hpp.Error:
        except:
            pass
        raise ValueError ("Unknown operational frame type of " + n)

    def addOperationalFrame (self, req):
        # TODO check that frame exists
        n = "op_frame/" + req.value
        try:
            frameType = self._getFrameType (req.value)
        except ValueError as e:
            rospy.logerr("Could not add operational frame: " + str(e))
            return SetStringResponse(False)
        if frameType == "joint":
            self.topics.append (self.Topic (self._readJointPosition, n, Transform, data = req.value))
        elif frameType == "link":
            self.topics.append (self.Topic (self._readLinkPosition, n, Transform, data = req.value))
        elif frameType == "obstacle":
            # TODO There should be a way for the node who requests this
            # to know the value is constant.
            _hpp = self._hpp()
            pos = _hpp.obstacle.getObstaclePosition (req.value)
            self.topics.append (self.ConstantTopic (listToTransform(pos), n, Transform))
        self.topics[-1].init(self._hpp())
        rospy.loginfo("Add topic " + n + " " + frameType)
        return SetStringResponse(True)

    def addOperationalFrameVelocity (self, req):
        # TODO check that frame exists
        n = "velocity/op_frame/" + req.value
        try:
            frameType = self._getFrameType (req.value)
        except ValueError as e:
            rospy.logerr("Could not add operational frame: " + str(e))
            return SetStringResponse(False)
        if frameType == "joint":
            self.topics.append (self.Topic (self._readJointVelocity, n, Vector, data = req.value))
        elif frameType == "link":
            self.topics.append (self.Topic (self._readLinkVelocity, n, Vector, data = req.value))
        elif frameType == "obstacle":
            # TODO There should be a way for the node who requests this
            # to know the value is constant.
            self.topics.append (self.ConstantTopic ([0,0,0,0,0,0], n, Vector))
        self.topics[-1].init(self._hpp())
        rospy.loginfo("Add topic " + n + " " + frameType)
        return SetStringResponse(True)

    def setJointNames (self, req):
        try:
            hpp = self._hpp()
            jns = hpp.robot.getJointNames() + [None]
            # list of segments in [config, velocity]
            joint_selection = [ [], [] ]
            # rank in [config, velocity]
            rks = [0, 0]
            segments = None
            for jn in jns:
                szs = [hpp.robot.getJointConfigSize(jn), hpp.robot.getJointNumberDof(jn)] if jn is not None else [0,0]
                if jn in req.names:
                    if segments is None:
                        segments = [ [rks[0], rks[0] + szs[0]], [rks[1], rks[1] + szs[1]] ]
                    else:
                        for i in range(2): segments[i][1] += szs[i]
                else:
                    if segments is not None: # insert previous segments
                        joint_selection[0].append(segments[0])
                        joint_selection[1].append(segments[1])
                        segments = None
                for i in range(2): rks[i] += szs[i]
            self.jointNames = req.names
            self.joint_selection = joint_selection
        except:
            return SetJointNamesResponse(False)
        rospy.loginfo("Joint names set to " + str(self.jointNames))
        self.rootJointName = None
        self.rootJointSizes = 0
        for n in self.jointNames:
            if n.endswith("root_joint"):
                self.rootJointName = n
                self.rootJointSizes = ( hpp.robot.getJointConfigSize(n), hpp.robot.getJointNumberDof(n) )
                break
        return SetJointNamesResponse(True)

    def _readConfigAtParam (self, client, data):
        qin = client.robot.getCurrentConfig()
        qout = list()
        for segment in self.joint_selection[0]:
            qout.extend(qin[segment[0]:segment[1]])
        if self.rootJointName is not None:
            rootpos = client.robot.getJointPosition(self.rootJointName)
            from hpp import Quaternion
            q = Quaternion(rootpos[3:7])
            # TODO although it is weird, the root joint may not be at
            # position 0
            qout[0:self.rootJointSizes[0]] = rootpos[0:3] + q.toRPY().tolist()
        return Vector(qout)

    def _readVelocityAtParam (self, client, data):
        vin = client.robot.getCurrentVelocity()
        vout = list()
        for segment in self.joint_selection[1]:
            vout.extend(vin[segment[0]:segment[1]])
        if self.rootJointName is not None:
            rootvel = client.robot.getJointVelocity(self.rootJointName)
            # TODO although it is weird, the root joint may not be at
            # position 0
            vout[0:self.rootJointSizes[1]] = rootvel
        return Vector(vout)

    def _readCenterOfMass (self, client, data):
        if data == "":
            v = client.robot.getCenterOfMass()
        else:
            v = client.robot.getPartialCom(data)
        return listToVector3(v)

    def _readCenterOfMassVelocity (self, client, data):
        if data == "":
            v = client.robot.getCenterOfMassVelocity()
        else:
            v = client.robot.getVelocityPartialCom(data)
        return listToVector3(v)

    def _readJointPosition (self, client, data):
        t = client.robot.getJointPosition(data)
        return listToTransform(t)

    def _readJointVelocity (self, client, data):
        t = client.robot.getJointVelocityInLocalFrame(data)
        return Vector(t)

    def _readLinkPosition (self, client, data):
        t = client.robot.getLinkPosition(data)
        return listToTransform(t)

    def _readLinkVelocity (self, client, data):
        rospy.logerr ("Link velocity cannot be obtained from HPP")
        return Vector()

    def readAt (self, pathId, time, uv = False, timeShift = 0):
        hpp = self._hpp()
        hpp.robot.setCurrentConfig( hpp.problem.configAtParam (pathId, time))
        hpp.robot.setCurrentVelocity( hpp.problem.derivativeAtParam (pathId, 1, time))
        if uv:
            self.queueViewer.append ((time - timeShift, self.topicViewer.read (hpp, uv)))
        msgs = []
        for topic in self.topics:
            msgs.append (topic.read(hpp))
        self.queue.put (msgs, True)
        return msgs

    def publishNext (self):
        msgs = self.queue.get(True)
        for topic, msg in zip(self.topics, msgs):
            topic.publish (msg)
        self.queue.task_done()

    def publishViewerAtTime (self, time):
        if hasattr(self, "viewer"):
            while len(self.queueViewer) > 0:
                # There is no message in queueViewer
                t, msg = self.queueViewer[0]
                if t < time - 1. / self.frequency:
                    self.topicViewer.publish (msg)
                    self.queueViewer.popleft()
                else:
                    break

    def _read (self, pathId, start, L):
        from math import ceil, floor
        N = int(ceil(abs(L) * self.frequency))
        rospy.loginfo("Start reading path {} (t in [ {}, {} ]) into {} points".format(pathId, start, start + L, N+1))
        self.reading = True
        self.queue = Queue.Queue (self.queue_size)
        times = (-1 if L < 0 else 1 ) *np.array(range(N+1), dtype=float) / self.frequency
        times[-1] = L
        times += start
        Nv = int(ceil(float(self.frequency) / float(self.viewerFreq)))
        updateViewer = [ False ] * (N+1)
        if hasattr(self, "viewer"):
            for i in range(0,len(updateViewer), Nv): updateViewer[i] = True
            updateViewer[-1] = True
        self.firstMsgs = None
        for t, uv in zip(times, updateViewer):
            msgs = self.readAt(pathId, t, uv, timeShift = start)
            if self.firstMsgs is None: self.firstMsgs = msgs
        self.pubs["read_path_done"].publish(UInt32(pathId))
        rospy.loginfo("Finish reading path {}".format(pathId))
        self.reading = False

    def read (self, msg):
        pathId = msg.data
        hpp = self._hpp()
        L = hpp.problem.pathLength(pathId)
        self._read (pathId, 0, L)

    def readSub (self, msg):
        self._read (msg.id, msg.start, msg.length)

    def publishFirst(self, empty):
        if self.firstMsgs is not None:
            for topic, msg in zip(self.topics, self.firstMsgs):
                topic.publish (msg)
            self.firstMsgs = None
        else:
            rospy.logerr("Could not print first message")
        return std_srvs.srv.EmptyResponse()

    def publish(self, empty):
        import time
        rospy.loginfo("Start publishing queue (size is {})".format(self.queue.qsize()))
        # The queue in SOT should have about 100ms of points
        n = 0
        advance = 1.5 * self.frequency / 10. # Begin with 150ms of points
        start = time.time()
        # highrate = rospy.Rate (5 * self.frequency)
        rate = rospy.Rate (10) # Send 100ms every 100ms
        while not self.queue.empty() or self.reading:
            dt = time.time() - start
            nstar = advance + dt * self.frequency
            while n < nstar and not self.queue.empty():
                self.publishNext()
                n += 1
                # highrate.sleep()
            self.publishViewerAtTime(dt)
            rate.sleep()
        rate = rospy.Rate(self.viewerFreq)
        while len(self.queueViewer) > 0:
            dt = time.time() - start
            self.publishViewerAtTime(dt)
            rate.sleep()
        self.pubs["publish_done"].publish(Empty())
        rospy.loginfo("Finish publishing queue ({})".format(n))
