#!/usr/bin/env python
import rospy
import copy
from duckietown_msgs.msg import FSMState, BoolStamped
from duckietown_msgs.srv import SetFSMState, SetFSMStateRequest, SetFSMStateResponse
import Adafruit_PCA9685

class FSMNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(60)
        # Build transition dictionray
        self.states_dict = rospy.get_param("~states",{})        
        # Validate state and global transitions
        if not self._validateStates(self.states_dict):
            rospy.signal_shutdown("[%s] Incoherent definition." %self.node_name)
            return          

        # Load global transitions
        self.global_transitions_dict = rospy.get_param("~global_transitions", {})
        if not self._validateGlobalTransitions(self.global_transitions_dict,self.states_dict.keys()):
            rospy.signal_shutdown("[%s] Incoherent definition." %self.node_name)
            return          

        # Setup initial state
        self.state_msg = FSMState()
        self.state_msg.state = rospy.get_param("~initial_state","")
        self.state_msg.header.stamp = rospy.Time.now()
        # Setup publisher and publish initial state
        self.pub_state = rospy.Publisher("~mode",FSMState,queue_size=1,latch=True)
    
        # Provide service
        self.srv_state = rospy.Service("~set_state",SetFSMState,self.cbSrvSetState)

        # Construct publishers
        self.pub_dict = dict()
        nodes = rospy.get_param("~nodes")

        self.active_nodes = None
        for node_name, topic_name in nodes.items():
            self.pub_dict[node_name] = rospy.Publisher(topic_name, BoolStamped, queue_size=1, latch=True)

        # print self.pub_dict
        # Process events definition
        param_events_dict = rospy.get_param("~events",{})
        # Validate events definition
        if not self._validateEvents(param_events_dict):
            rospy.signal_shutdown("[%s] Invalid event definition." %self.node_name)
            return          


        self.sub_list = list()
        self.event_trigger_dict = dict()
        for event_name, event_dict in param_events_dict.items():
            topic_name = event_dict["topic"]
            msg_type = event_dict["msg_type"]
            self.event_trigger_dict[event_name] = event_dict["trigger"]
        # TODO so far I can't figure out how to put msg_type instead of BoolStamped.
        # importlib might help. But it might get too complicated since different type 
            self.sub_list.append(rospy.Subscriber("%s"%(topic_name), BoolStamped, self.cbEvent, callback_args=event_name))

        rospy.loginfo("[%s] Initialized." %self.node_name)
        # Publish initial state
        self.publish()

 

    def _validateGlobalTransitions(self,global_transitions, valid_states):
        pass_flag = True
        for event_name, state_name in global_transitions.items():
            if state_name not in valid_states:
                rospy.logerr("[%s] State %s is not valid. (From global_transitions of %s)" %(self.node_name,state_name, event_name)) 
                pass_flag = False
        return pass_flag

    def _validateEvents(self,events_dict):
        pass_flag = True
        for event_name, event_dict in events_dict.items():
            print 
            if "topic" not in event_dict:
                rospy.logerr("[%s] Event %s missing topic definition." %(self.node_name,event_name))
                pass_flag = False
            if "msg_type" not in event_dict:
                rospy.logerr("[%s] Event %s missing msg_type definition." %(self.node_name,event_name))
                pass_flag = False
            if "trigger" not in event_dict:
                rospy.logerr("[%s] Event %s missing trigger definition." %(self.node_name,event_name))
                pass_flag = False
        return pass_flag

    def _validateStates(self,states_dict):
        pass_flag = True
        valid_states = states_dict.keys()
        for state, state_dict in states_dict.items():        
            # Validate the existence of all reachable states
            transitions_dict = state_dict.get("transitions")
            if transitions_dict is None:
                continue
            else:
                for transition, next_state in transitions_dict.items():
                    if next_state not in valid_states:
                        rospy.logerr("[%s] %s not a valide state. (From %s with event %s)" %(self.node_name,next_state,state,transition))
                        pass_flag = False
        return pass_flag 

    def _getNextState(self, state_name, event_name):
        if not self.isValidState(state_name):
            rospy.logwarn("[%s] %s not defined. Treat as terminal. "%(self.node_name,state_name))
            return None

        # state transitions overwrites global transition
        state_dict = self.states_dict.get(state_name)
        if "transitions" in state_dict:
            next_state = state_dict["transitions"].get(event_name)
        else:
            next_state = None

        # state transitions overwrites global transitions
        if next_state is None:
            # No state transition defined, look up global transition
            next_state = self.global_transitions_dict.get(event_name) #None when no global transitions
        return next_state
                
    def _getActiveNodesOfState(self,state_name):
        state_dict = self.states_dict[state_name]
        active_nodes = state_dict.get("active_nodes")
        if active_nodes is None:
            rospy.logwarn("[%s] No active nodes defined for %s. Deactive all nodes."%(self.node_name,state_name))
            active_nodes = []
        return active_nodes

    def publish(self):
        self.publishBools()
        self.publishState()

    def isValidState(self,state):
        return state in self.states_dict.keys()

    def cbSrvSetState(self,req):
        if self.isValidState(req.state):
            self.state_msg.header.stamp = rospy.Time.now()
            self.state_msg.state = req.state
            self.publish()
        else:
            rospy.logwarn("[%s] %s is not a valid state." %(self.node_name,req.state))
        return SetFSMStateResponse()

    def publishState(self):
        self.pub_state.publish(self.state_msg)
        rospy.loginfo("[%s] FSMState: %s" %(self.node_name, self.state_msg.state))

    def publishBools(self):
        active_nodes = self._getActiveNodesOfState(self.state_msg.state)
        for node_name, node_pub in self.pub_dict.items():
            msg = BoolStamped()
            msg.header.stamp = self.state_msg.header.stamp
            msg.data = bool(node_name in active_nodes)
            node_state = "ON" if msg.data else "OFF"
            rospy.loginfo("[%s] Node %s is %s in %s" %(self.node_name, node_name, node_state, self.state_msg.state))
            if self.active_nodes is not None:
                if (node_name in active_nodes) == (node_name in self.active_nodes):
                    continue
            # else:
            #     rospy.logwarn("[%s] self.active_nodes is None!" %(self.node_name))
                # continue
            node_pub.publish(msg)
            # rospy.loginfo("[%s] node %s msg %s" %(self.node_name, node_name, msg))
            # rospy.loginfo("[%s] Node %s set to %s." %(self.node_name, node_name, node_state))
        self.active_nodes = copy.deepcopy(active_nodes)

    def ledlight(self):
        if (self.state_msg.state == "JOYSTICK_CONTROL"):
            self.pwm.set_pwm(0,0,4095)
            self.pwm.set_pwm(1,0,0)
            self.pwm.set_pwm(2,0,0)
        elif (self.state_msg.state == "LANE_FOLLOWING"):
            self.pwm.set_pwm(0,0,0)
            self.pwm.set_pwm(1,0,4095)
            self.pwm.set_pwm(2,0,0)
        elif (self.state_msg.state == "LANESTOP"):
            self.pwm.set_pwm(0,0,0)
            self.pwm.set_pwm(1,0,0)
            self.pwm.set_pwm(2,0,4095)
        elif (self.state_msg.state == "JOYSTOP"):
            self.pwm.set_pwm(0,0,0)
            self.pwm.set_pwm(1,0,0)
            self.pwm.set_pwm(2,0,4095)
        else:
            self.pwm.set_pwm(0,0,0)
            self.pwm.set_pwm(1,0,0)
            self.pwm.set_pwm(2,0,0)

    def cbEvent(self,msg,event_name):
        if (msg.data == self.event_trigger_dict[event_name]):
            # Update timestamp
            rospy.loginfo("[%s] Event: %s" %(self.node_name,event_name))
            self.state_msg.header.stamp = msg.header.stamp
            next_state = self._getNextState(self.state_msg.state,event_name)
            if next_state is not None:
                # Has a defined transition
                self.state_msg.state = next_state
                self.publish()
        self.ledlight()

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('fsm_node', anonymous=False)

    # Create the NodeName object
    node = FSMNode()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()