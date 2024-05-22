# Control of INO hotend
#
# Copyright (C) 2023  Johannes Zischg <johannes.zischg@plasmics.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, threading

KELVIN_TO_CELSIUS = -273.15
MAX_HEAT_TIME = 5.0
AMBIENT_TEMP = 25.0
PID_PARAM_BASE = 255.0
INO_REPORT_TIME = 0.3
INO_MIN_REPORT_TIME = 0.1


class PLA_INO_Heater:
    """Custom heater class for the INO heater"""

    def __init__(self, config, sensor):
        """Initialization of the INO Heater class

        :param config: conifg object holding the information needed for initialization
        :type config: ?
        :param sensor: already defined sensor object (INO Sensor) that should be assigned to the heater
        :type sensor: ?
        """
        self.config = config
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        # Setup sensor
        self.sensor = sensor
        self.min_temp = config.getfloat("min_temp", minval=KELVIN_TO_CELSIUS)
        self.max_temp = config.getfloat("max_temp", above=self.min_temp)
        self.pid_Kp = config.getfloat(
            "pid_Kp", 13.41, minval=0.0, maxval=40
        )  # ('var_name', standardwert, minval=0.0)
        self.pid_Ki = config.getfloat("pid_Ki", 30.91, minval=0.0, maxval=80)
        self.pid_Kd = config.getfloat("pid_Kd", 1.46, minval=0.0, maxval=10)

        self.sensor.setup_minmax(self.min_temp, self.max_temp)
        self.sensor.setup_callback(self.temperature_callback)
        self.pwm_delay = config.getfloat(
            "PLA_INO_report_time", INO_REPORT_TIME, minval=INO_MIN_REPORT_TIME
        )
        # Setup temperature checks
        self.min_extrude_temp = config.getfloat(
            "min_extrude_temp", 170.0, minval=self.min_temp, maxval=self.max_temp
        )
        is_fileoutput = self.printer.get_start_args().get("debugoutput") is not None
        self.can_extrude = self.min_extrude_temp <= 0.0 or is_fileoutput
        self.max_power = config.getfloat("max_power", 1.0, above=0.0, maxval=1.0)
        self.smooth_time = config.getfloat("smooth_time", 1.0, above=0.0)
        self.inv_smooth_time = 1.0 / self.smooth_time
        self.lock = threading.Lock()
        self.last_temp = self.smoothed_temp = self.target_temp = 0.0
        self.last_temp_time = 0.0
        # pwm caching
        self.next_pwm_time = 0.0
        self.last_pwm_value = 0.0
        # Setup control algorithm sub-class
        algos = {"watermark": ControlBangBang, "pid": ControlPID}
        algo = config.getchoice("control", algos)
        self.control = algo(self, config)
        # # Load additional modules
        self.printer.load_object(config, "verify_heater %s" % (self.name,))
        self.printer.load_object(config, "pid_calibrate")
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "SET_HEATER_TEMPERATURE",
            "HEATER",
            self.name,
            self.cmd_SET_HEATER_TEMPERATURE,
            desc=self.cmd_SET_HEATER_TEMPERATURE_help,
        )

    def set_pwm(self, read_time, value):
        if self.target_temp <= 0.0:
            value = 0.0
        if (read_time < self.next_pwm_time or not self.last_pwm_value) and abs(
            value - self.last_pwm_value
        ) < 0.05:
            # No significant change in value - can suppress update
            return
        pwm_time = read_time + self.pwm_delay
        self.next_pwm_time = pwm_time + 0.75 * MAX_HEAT_TIME
        self.last_pwm_value = value

    def temperature_callback(self, read_time, temp):
        with self.lock:
            time_diff = read_time - self.last_temp_time
            self.last_temp = temp
            self.last_temp_time = read_time
            self.control.temperature_update(read_time, temp, self.target_temp)
            temp_diff = temp - self.smoothed_temp
            adj_time = min(time_diff * self.inv_smooth_time, 1.0)
            self.smoothed_temp += temp_diff * adj_time
            self.can_extrude = self.smoothed_temp >= self.min_extrude_temp

    # External commands
    def get_pwm_delay(self):
        return self.pwm_delay

    def get_max_power(self):
        return self.max_power

    def get_smooth_time(self):
        return self.smooth_time

    def set_temp(self, degrees):
        """Function used to set the temperature.
        In contrast to the normal heater, this command will write to the serial connection
        opened by the respective INO Sensor.

        :param degrees: the target temperature
        :raises self.printer.command_error: raises error if target temperature is out of bounds
        """
        if degrees and (degrees < self.min_temp or degrees > self.max_temp):
            raise self.printer.command_error(
                "Requested temperature (%.1f) out of range (%.1f:%.1f)"
                % (degrees, self.min_temp, self.max_temp)
            )
        with self.lock:
            self.target_temp = degrees
            logging.info(f"J: set_temp -> to {self.target_temp} degrees")
            self.sensor.target_temp = int(self.target_temp)
            self.sensor.send_temp()

    def get_temp(self, eventtime):
        with self.lock:
            return self.smoothed_temp, self.target_temp

    def check_busy(self, eventtime):
        with self.lock:
            return self.control.check_busy(
                eventtime, self.smoothed_temp, self.target_temp
            )

    def set_control(self, control):
        with self.lock:
            old_control = self.control
            self.control = control
            self.target_temp = 0.0
        return old_control

    def alter_target(self, target_temp):
        if target_temp:
            target_temp = max(self.min_temp, min(self.max_temp, target_temp))
        self.target_temp = target_temp

    def stats(self, eventtime):
        with self.lock:
            target_temp = self.target_temp
            last_temp = self.last_temp
            last_pwm_value = self.last_pwm_value
        is_active = target_temp or last_temp > 50.0
        return is_active, "%s: target=%.0f temp=%.1f pwm=%.3f" % (
            self.name,
            target_temp,
            last_temp,
            last_pwm_value,
        )

    def get_status(self, eventtime):
        with self.lock:
            target_temp = self.target_temp
            smoothed_temp = self.smoothed_temp
            last_pwm_value = self.last_pwm_value
        return {
            "temperature": round(smoothed_temp, 2),
            "target": target_temp,
            "power": last_pwm_value,
        }

    cmd_SET_HEATER_TEMPERATURE_help = "Sets a heater temperature"

    def cmd_SET_HEATER_TEMPERATURE(self, gcmd):
        temp = gcmd.get_float("TARGET", 0.0)
        logging.info(f"J: SET_HEATER_TEMPERATURE fired with temp {temp} ")
        pheaters = self.printer.lookup_object("heaters")
        pheaters.set_temperature(self, temp)


######################################################################
# Bang-bang control algo
######################################################################


class ControlBangBang:
    def __init__(self, heater, config):
        self.heater = heater
        self.heater_max_power = heater.get_max_power()
        self.max_delta = config.getfloat("max_delta", 2.0, above=0.0)
        self.heating = False

    def temperature_update(self, read_time, temp, target_temp):
        if self.heating and temp >= target_temp + self.max_delta:
            self.heating = False
        elif not self.heating and temp <= target_temp - self.max_delta:
            self.heating = True
        if self.heating:
            self.heater.set_pwm(read_time, self.heater_max_power)
        else:
            self.heater.set_pwm(read_time, 0.0)

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        return smoothed_temp < target_temp - self.max_delta


######################################################################
# Proportional Integral Derivative (PID) control algo
######################################################################

PID_SETTLE_DELTA = 1.0
PID_SETTLE_SLOPE = 0.1


class ControlPID:
    def __init__(self, heater, config):
        self.heater = heater
        self.heater_max_power = heater.get_max_power()
        self.Kp = config.getfloat("pid_Kp") / PID_PARAM_BASE
        self.Ki = config.getfloat("pid_Ki") / PID_PARAM_BASE
        self.Kd = config.getfloat("pid_Kd") / PID_PARAM_BASE
        self.min_deriv_time = heater.get_smooth_time()
        self.temp_integ_max = 0.0
        if self.Ki:
            self.temp_integ_max = self.heater_max_power / self.Ki
        self.prev_temp = AMBIENT_TEMP
        self.prev_temp_time = 0.0
        self.prev_temp_deriv = 0.0
        self.prev_temp_integ = 0.0

    def temperature_update(self, read_time, temp, target_temp):
        time_diff = read_time - self.prev_temp_time
        # Calculate change of temperature
        temp_diff = temp - self.prev_temp
        if time_diff >= self.min_deriv_time:
            temp_deriv = temp_diff / time_diff
        else:
            temp_deriv = (
                self.prev_temp_deriv * (self.min_deriv_time - time_diff) + temp_diff
            ) / self.min_deriv_time
        # Calculate accumulated temperature "error"
        temp_err = target_temp - temp
        temp_integ = self.prev_temp_integ + temp_err * time_diff
        temp_integ = max(0.0, min(self.temp_integ_max, temp_integ))
        # Calculate output
        co = self.Kp * temp_err + self.Ki * temp_integ - self.Kd * temp_deriv
        # logging.debug("pid: %f@%.3f -> diff=%f deriv=%f err=%f integ=%f co=%d",
        #    temp, read_time, temp_diff, temp_deriv, temp_err, temp_integ, co)
        bounded_co = max(0.0, min(self.heater_max_power, co))
        self.heater.set_pwm(read_time, bounded_co)
        # Store state for next measurement
        self.prev_temp = temp
        self.prev_temp_time = read_time
        self.prev_temp_deriv = temp_deriv
        if co == bounded_co:
            self.prev_temp_integ = temp_integ

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        temp_diff = target_temp - smoothed_temp
        return (
            abs(temp_diff) > PID_SETTLE_DELTA
            or abs(self.prev_temp_deriv) > PID_SETTLE_SLOPE
        )
