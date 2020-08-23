#!/usr/bin/env python

import rospy


class BatteryState(enumerate):
    CHARGING = 0
    DISCHARGING = 1


class Battery:
    """ Class representing battery level """
    def __init__(self, battery_lvl, charge_rate, discharge_rate):
        self._battery_lvl = battery_lvl
        self._charge_rate = charge_rate / 60.0
        self._discharge_rate = discharge_rate / 60.0
        self._last_time_stamp = rospy.get_time()
        # By default the state is charging
        self._battery_state = BatteryState.CHARGING
        print("This", self._battery_lvl, self._charge_rate, self._discharge_rate)

    def __update_battery_level(self):
        """ Sets the battery value """
        value_battery = self.__calculate_battery_level()
        if value_battery > 100.0:
            self._battery_lvl = float(100)
        elif value_battery < 0.0:
            self._battery_lvl = 0
        else:
            self._battery_lvl = value_battery

    def __calculate_battery_level(self):
        """ Calculate battery level based on the charge and discharge rate """
        # Retrieve time
        current_time = rospy.get_time()
        elapsed_time = current_time - self._last_time_stamp
        # update time stamp
        self._last_time_stamp = current_time
        value_battery = self._battery_lvl
        if self._battery_state is BatteryState.CHARGING:
            value_battery += self._charge_rate * elapsed_time
        else:
            value_battery -= self._discharge_rate * elapsed_time
        return value_battery

    def change_state(self, state):
        """ Change the state of the battery """
        # update battery level
        self.__update_battery_level()
        self._battery_state = state

    def get_level(self):
        """ Update and retrieve the level of the battery """
        self.__update_battery_level()
        return self._battery_lvl
