#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
import datetime
import json

running = False
schedules = []


def handle_start(req):
    global running
    running = True
    rospy.loginfo('Core: start requested')
    return TriggerResponse(success=True, message='started')


def handle_stop(req):
    global running
    running = False
    rospy.loginfo('Core: stop requested')
    return TriggerResponse(success=True, message='stopped')


def load_schedules_from_param():
    """Load schedules from ROS param `/core_manager/schedules`.
    Expected format: a list of dicts with 'start' and optional 'stop' fields, times as 'HH:MM' 24-hour strings.
    Example:
      [{'start':'09:00','stop':'09:05'},{'start':'18:00'}]
    """
    global schedules
    schedules = []
    try:
        param = rospy.get_param('/core_manager/schedules', [])
        for item in param:
            s = {'start': item.get('start'), 'stop': item.get('stop'), 'last_start': None, 'last_stop': None}
            schedules.append(s)
        rospy.loginfo('Loaded %d schedules from param' % len(schedules))
    except Exception as e:
        rospy.logwarn('Failed to load schedules param: %s' % e)


def schedule_update_cb(msg):
    """Update schedules at runtime by publishing JSON to /core/schedule_update.
    Supported commands (JSON string):
      {"op":"add","start":"HH:MM","stop":"HH:MM"}
      {"op":"clear"}
      {"op":"set","schedules":[{"start":"HH:MM","stop":"HH:MM"}, ...]}
    """
    global schedules
    try:
        j = json.loads(msg.data)
        op = j.get('op')
        if op == 'add':
            schedules.append({'start': j.get('start'), 'stop': j.get('stop'), 'last_start': None, 'last_stop': None})
            rospy.loginfo('Added schedule %s' % str(j))
        elif op == 'clear':
            schedules = []
            rospy.loginfo('Cleared schedules')
        elif op == 'set':
            schedules = []
            for item in j.get('schedules', []):
                schedules.append({'start': item.get('start'), 'stop': item.get('stop'), 'last_start': None, 'last_stop': None})
            rospy.loginfo('Set schedules to %s' % str(j.get('schedules', [])))
        else:
            rospy.logwarn('Unknown schedule op: %s' % str(op))
    except Exception as e:
        rospy.logerr('Invalid schedule update message: %s (%s)' % (msg.data, e))


def time_matches(time_str, now):
    """Return True if now (datetime.time) matches time_str 'HH:MM'."""
    try:
        hh, mm = time_str.split(':')
        return now.hour == int(hh) and now.minute == int(mm)
    except Exception:
        return False


def main():
    global running, schedules
    rospy.init_node('core_manager')
    start_srv = rospy.Service('start_system', Trigger, handle_start)
    stop_srv = rospy.Service('stop_system', Trigger, handle_stop)
    cmd_pub = rospy.Publisher('/swarm_cmd', String, queue_size=10)
    rospy.Subscriber('/core/schedule_update', String, schedule_update_cb)

    # Initial load from param
    load_schedules_from_param()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        # publish regular clean command if running
        if running:
            cmd_pub.publish('clean')

        # schedule handling: check schedules
        now = datetime.datetime.now().time()
        for s in schedules:
            # check start
            if s.get('start') and time_matches(s['start'], now):
                # avoid repeating within the same minute
                if s.get('last_start') != datetime.date.today():
                    rospy.loginfo('Schedule start triggered: %s' % s['start'])
                    cmd_pub.publish('start')
                    running = True
                    s['last_start'] = datetime.date.today()
            # check stop
            if s.get('stop') and time_matches(s['stop'], now):
                if s.get('last_stop') != datetime.date.today():
                    rospy.loginfo('Schedule stop triggered: %s' % s['stop'])
                    cmd_pub.publish('stop')
                    running = False
                    s['last_stop'] = datetime.date.today()

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
