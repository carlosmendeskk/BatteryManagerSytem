#!/usr/bin/env python

import threading
import rospy
from robots import srv

MAX_NR_ROBOTS = 10

BATTERY_SWITCH_LEVEL = 10.0


class Manager:
    """ Manager class """
    def __init__(self):
        rospy.init_node('manager_node')
        # To avoid racing condition for the list and vehicle execution fifo
        # a mutex is required
        self._lock = threading.Lock()
        # List containing the robot ids
        self._node_list = {}
        # Fifo containing execution order
        self._running_robot = None
        self._next_in_line = []
        self._last_print_stamp = rospy.get_time()
        #
        rospy.Service('manage_operation', srv.Register, self.__handshake)

    def run(self):
        """ Main function to manage robot operation """
        rate = rospy.Rate(1)  # 1hz
        while rospy.is_shutdown() is False:
            if len(self._node_list) > 0:
                self.__update()
            rate.sleep()

    def __handshake(self, req):
        """ Callback when handshake between vehicle and Manager """
        success = False
        # Check if vehicle is in the list
        if req.command is srv.RegisterRequest.CONNECT:
            success = self.__register(req._connection_header['callerid'])
        elif req.command is srv.RegisterRequest.DISCONNECT:
            success = self.__unregister(req._connection_header['callerid'])
        return success

    def __register(self, caller_id):
        """ Register user in the list """
        success = False
        with self._lock:
            if caller_id in self._node_list:
                rospy.logerr("Vehicle already registered!")
            elif len(self._node_list) < MAX_NR_ROBOTS:
                # check if there service is available for the robot
                rospy.wait_for_service(caller_id + "_control_sv")
                self._node_list[caller_id] = rospy.ServiceProxy(caller_id + "_control_sv",
                                                                srv.Control,
                                                                persistent=True)
                rospy.loginfo("Registered: " + caller_id)
                success = True
            else:
                rospy.logerr("Number of vehicle max reached!")
        return success

    def __unregister(self, caller_id):
        """ Register user in the list """
        success = False
        with self._lock:
            if caller_id not in self._node_list:
                rospy.logerr("Robot already registered.")
            else:
                rospy.loginfo("Robot with id: " + caller_id + " unregistered")
                if caller_id in self._next_in_line:
                    self._next_in_line.remove(caller_id)
                del self._node_list[caller_id]
        return success

    def __print(self, robot_id, robot_state, sec):
        """ This function prints a message with sec frequency """
        current_time = rospy.get_time()
        if (current_time - self._last_print_stamp) >= sec:
            printable_mode = "NA"

            if robot_state.state is srv.ControlResponse.OPERATING:
                printable_mode = "operating"
            elif robot_state.state is srv.ControlResponse.IDLE_CHARGING:
                printable_mode = "charging"

            rospy.loginfo(robot_id + ": " + printable_mode + " current " + str(robot_state.battery_lvl) + " %")
            self._last_print_stamp = current_time

    def __update(self):
        """ Here the goal is to request the vehicle status. """
        with self._lock:
            is_robot_operating = False
            for key in self._node_list.keys():
                # Request robot state
                robot_state = self.__request_state(key)
                self.__print(robot_id=key, robot_state=robot_state, sec=0)
                # Check if action is needed
                if robot_state.state is srv.ControlResponse.OPERATING:
                    if (robot_state.battery_lvl <= BATTERY_SWITCH_LEVEL) \
                            and robot_state.state is not srv.ControlResponse.IDLE_CHARGING:
                        self.__swap_robots(key)
                    else:
                        is_robot_operating = True

                elif (robot_state.state is srv.ControlResponse.IDLE_CHARGING) \
                        and (100.0 - robot_state.battery_lvl) <= 0.001:
                    if (key in self._next_in_line) is False:
                        self._next_in_line.append(key)
            if not is_robot_operating:
                self.__start_next_in_line()

    def __start_next_in_line(self):
        """ Sets the next in line robot to operating """
        if len(self._next_in_line) > 0:
            success = self.__set_state(self._next_in_line[0],
                                       srv.ControlRequest.REQUEST_OPERATION,
                                       srv.ControlResponse.OPERATING)
            if success is True:
                self._next_in_line.pop(0)

    def __swap_robots(self, key):
        """ Stop robot operation and replace if possible with a fully charged robot """
        rospy.loginfo("Swapping operating - charging")

        success = self.__set_state(key,
                                   srv.ControlRequest.REQUEST_CHARGING,
                                   srv.ControlResponse.IDLE_CHARGING)

        if (success is True) and (len(self._next_in_line) != 0):
            success = self.__set_state(self._next_in_line[0],
                                       srv.ControlRequest.REQUEST_OPERATION,
                                       srv.ControlResponse.OPERATING)
            if success is True:
                self._next_in_line.pop(0)
        return success

    def __set_state(self, key, request_state, expected_state):
        """ Calls the control service with set_state method id """
        request = srv.ControlRequest()
        request.method_id = srv.ControlRequest.SET_STATE
        request.requested_mode = request_state
        response = self._node_list[key](request)
        return response.state == expected_state

    def __request_state(self, key):
        tmp = srv.ControlRequest()
        tmp.method_id = srv.ControlRequest.GET_STATE
        tmp.requested_mode = srv.ControlRequest.NO_REQUEST
        return self._node_list[key](tmp)


if __name__ == '__main__':
    #
    m = Manager()
    try:
        m.run()
    except Exception as e:
        rospy.logerr(e.args[0])
