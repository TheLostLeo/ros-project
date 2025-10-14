#!/usr/bin/env python3
"""
CleanTask Test Client
---------------------
Simple ROS action client that sends a CleanTask goal to the action server,
prints feedback in real time, and reports the final result. Useful for quick
end-to-end testing of the multi-bot cleaning action.

Params (private ~ namespace):
- action_name (str): Action name to connect to. Default: 'clean_task'
- bot_id (int): Target bot id. Default: 1
- grid_rows (int): Grid rows for zig-zag planner. Default: 3
- grid_cols (int): Grid cols for zig-zag planner. Default: 3
- pattern (str): Pattern hint (currently not used by server). Default: 'zigzag'
- start_delay (float): Seconds to wait before sending goal. Default: 0.5
- cancel_after (float): If > 0, cancel the goal after N seconds. Default: -1 (disabled)
- timeout (float): If > 0, wait for result up to N seconds. Default: 0 (wait forever)
"""

import threading
import time
import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from swarm_manage.msg import CleanTaskAction, CleanTaskGoal


def main():
    rospy.init_node('clean_task_test_client')

    action_name = rospy.get_param('~action_name', 'clean_task')
    bot_id = int(rospy.get_param('~bot_id', 1))
    grid_rows = int(rospy.get_param('~grid_rows', 3))
    grid_cols = int(rospy.get_param('~grid_cols', 3))
    pattern = rospy.get_param('~pattern', 'zigzag')
    start_delay = float(rospy.get_param('~start_delay', 0.5))
    cancel_after = float(rospy.get_param('~cancel_after', -1.0))
    timeout = float(rospy.get_param('~timeout', 0.0))

    rospy.loginfo(f"Connecting to action server '{action_name}'...")
    client = actionlib.SimpleActionClient(action_name, CleanTaskAction)
    client.wait_for_server()
    rospy.loginfo("Connected.")

    # Build goal
    goal = CleanTaskGoal()
    goal.bot_id = bot_id
    goal.grid_rows = grid_rows
    goal.grid_cols = grid_cols
    goal.pattern = pattern

    # Optional delay before sending
    if start_delay > 0:
        rospy.loginfo(f"Waiting {start_delay:.2f}s before sending goal...")
        rospy.sleep(start_delay)

    # Feedback callback
    def fb_cb(fb):
        rospy.loginfo(
            f"[feedback] bot={fb.bot_id} idx={fb.current_index}/{fb.total} zone={fb.current_zone} phase={fb.phase}"
        )

    # Done callback
    def done_cb(state, result):
        state_str = GoalStatus.to_string(state) if hasattr(GoalStatus, 'to_string') else str(state)
        rospy.loginfo(f"[done] state={state_str} result={getattr(result, 'success', False)} msg={getattr(result, 'message', '')} zones={getattr(result, 'zones_cleaned', -1)}")

    # Active callback
    def active_cb():
        rospy.loginfo("Goal is now active")

    rospy.loginfo(f"Sending goal: bot_id={bot_id}, grid={grid_rows}x{grid_cols}, pattern={pattern}")
    client.send_goal(goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=fb_cb)

    # Optional auto-cancel after N seconds
    cancel_timer = None
    if cancel_after and cancel_after > 0:
        def cancel_later():
            rospy.sleep(cancel_after)
            if not rospy.is_shutdown():
                rospy.logwarn(f"Auto-canceling goal after {cancel_after}s")
                client.cancel_goal()
        cancel_timer = threading.Thread(target=cancel_later, daemon=True)
        cancel_timer.start()

    # Wait for result (optionally with timeout)
    if timeout and timeout > 0:
        finished = client.wait_for_result(rospy.Duration(timeout))
        if not finished:
            rospy.logwarn(f"Result not received within {timeout}s; canceling goal and shutting down")
            client.cancel_goal()
            client.wait_for_result(rospy.Duration(2.0))
    else:
        client.wait_for_result()

    state = client.get_state()
    result = client.get_result()
    state_str = GoalStatus.to_string(state) if hasattr(GoalStatus, 'to_string') else str(state)
    rospy.loginfo(f"Final state={state_str} success={getattr(result, 'success', False)} message='{getattr(result, 'message', '')}' zones={getattr(result, 'zones_cleaned', -1)}")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
