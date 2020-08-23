#!/usr/bin/env python

import rospy
import rosservice
from auxiliar_types.battery import Battery, BatteryState
from robots import srv


# Default values
# Max discharge rate ( max 10 sec to reach dock with 10 percent)
MAX_DISCHARGE_RATE = 58.82
# Min discharge rate ( min 1 % per min)
MIN_DISCHARGE_RATE = 1.0
# 100 % PER MIN
MAX_CHARGE_RATE = 100.0
# 1 % PER MIN
MIN_CHARGE_RATE = 1.0


def _get_params(param_name, min_value, max_value, default_value):
    """ Retrieve and validate params """
    if rospy.has_param(param_name):
        tmp = rospy.get_param(param_name)
        if (tmp < min_value) or (tmp > max_value):
            rospy.logwarn("Defined value for " + param_name + " out of bounds! Using " + default_value)
            value = default_value
        else:
            value = tmp
    else:
        value = default_value
    return value


class Robot:
    """ Robot behaviour class """
    def __init__(self):
        # Init node and retrieve
        rospy.init_node('robot_node', anonymous=True)
        self._control_sv = rospy.Service(rospy.get_name() + "_control_sv", srv.Control, self.__control_api)
        # variables
        battery_lvl = _get_params("~initial_battery_level",
                                  min_value=0.0,
                                  max_value=100.0,
                                  default_value=100.0)
        discharge_rate = _get_params("~discharge_rate",
                                     min_value=MIN_DISCHARGE_RATE,
                                     max_value=MAX_DISCHARGE_RATE,
                                     default_value=MAX_DISCHARGE_RATE)
        charge_rate = _get_params("~charge_rate",
                                  min_value=MIN_CHARGE_RATE,
                                  max_value=MAX_CHARGE_RATE,
                                  default_value=MAX_CHARGE_RATE)

        self._battery = Battery(battery_lvl=battery_lvl,
                                charge_rate=charge_rate,
                                discharge_rate=discharge_rate)
        #
        self._connection_service = None
        self._state = srv.ControlResponse.IDLE_CHARGING

    def init(self):
        """ Initialize object """
        rospy.wait_for_service("manage_operation", timeout=rospy.Duration(1))
        rospy.loginfo("Found manager service!")
        rospy.on_shutdown(self.stop)
        self._connection_service = rospy.ServiceProxy('manage_operation', srv.Register, persistent=True)

        command = srv.RegisterRequest.CONNECT

        if self._connection_service(command) is False:
            raise Exception("Handshake failed!")
        else:
            rospy.loginfo("Connection successful!")

    def __is_service_alive(self):
        """ This function checks if the required manager service is still alive """
        if self._connection_service.resolved_name in rosservice.get_service_list():
            return True
        else:
            return False

    def run(self):
        """ Robot main function. Currently no action is needed apart from service integrity check."""
        while (rospy.is_shutdown() is False) and \
                (self.__is_service_alive() is True):
            rospy.sleep(1)

    def stop(self):
        """ Stops robot. Requests disconnect if the service is active. """
        if self.__is_service_alive():
            command = srv.RegisterRequest.DISCONNECT
            self._connection_service(command)
        else:
            rospy.logerr("Service " + self._connection_service.resolved_name + " is no longer available. "
                                                                               "Shutting down!")

    def __get_state(self):
        """ Retrieves state of the robot. """
        response = srv.ControlResponse()
        response.state = self._state
        response.battery_lvl = self._battery.get_level()
        return response

    def __update_state(self, requested_transition):
        """ Update robot state """
        response = srv.ControlResponse()
        if requested_transition is srv.ControlRequest.REQUEST_OPERATION:
            if self._state is srv.ControlResponse.IDLE_CHARGING:
                self._state = srv.ControlResponse.OPERATING
                self._battery.change_state(BatteryState.DISCHARGING)
            else:
                rospy.logwarn("Transition not possible!")
        elif requested_transition is srv.ControlRequest.REQUEST_CHARGING:
            if self._state is srv.ControlResponse.OPERATING:
                self._state = srv.ControlResponse.RETURNING_TO_STATION
                rospy.loginfo("Simulating return to station during 5 sec")
                rospy.sleep(5)
                self._state = srv.ControlResponse.IDLE_CHARGING
                self._battery.change_state(BatteryState.CHARGING)
            else:
                rospy.logwarn("Transition not possible!")
        else:
            rospy.logwarn("Request not handled!")
        response.state = self._state
        response.battery_lvl = self._battery.get_level()
        return response

    def __control_api(self, request):
        """ """
        if request.method_id is srv.ControlRequest.GET_STATE:
            return self.__get_state()
        elif request.method_id is srv.ControlRequest.SET_STATE:
            return self.__update_state(request.requested_mode)
        else:
            raise Exception("Method ID not supported")


if __name__ == '__main__':
    # Init Manager
    r = Robot()
    # Initialize Specific robot routines
    try:
        r.init()
        r.run()
    except Exception as e:
        rospy.logerr(e.args[0])
