import smach
import smach_ros

class LeaveGarage(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        # Code for opening the garage door and driving out of the garage
        # ...
        return 'succeeded'

class NavigateWaypoints(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        # Code for navigating to the waypoints
        # ...
        return 'succeeded'

class ReturnToGarage(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        # Code for driving back to the garage and closing the garage door
        # ...
        return 'succeeded'

def main():
    sm = smach.StateMachine(outcomes=['succeeded'])

    with sm:
        smach.StateMachine.add('LEAVE_GARAGE', LeaveGarage(),
                               transitions={'succeeded':'NAVIGATE_WAYPOINTS'})
        smach.StateMachine.add('NAVIGATE_WAYPOINTS', NavigateWaypoints(),
                               transitions={'succeeded':'RETURN_TO_GARAGE'})
        smach.StateMachine.add('RETURN_TO_GARAGE', ReturnToGarage(),
                               transitions={'succeeded':'succeeded'})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()