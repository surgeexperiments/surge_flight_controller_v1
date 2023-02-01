/**
 * @author SurgeExperiments
 */

#include "running_mode.h"
#include "pid.h"

void set_running_mode(flyMode_et *mode, rxChannels_st rx_pulse)
{
    /* For starting the motors: throttle low and yaw left (step 1).
     * Also we do not want the quad to do PID adjustments when the throttle is not activated
     */
    if ((rx_pulse.three < 1108) && (rx_pulse.four < 1084))
    {
        *mode = MOTORS_STANDBY_MODE;
        return;
    }

    /* From standBy to flymode: When yaw stick is back in the center position start the motors (step 2). */
    else if ((*mode == MOTORS_STANDBY_MODE) && (rx_pulse.three < 1105) && (rx_pulse.four > 1450))
    {
        *mode = MOTORS_FLY_MODE;

        /* Reset the pid controllers to avoid old PID values making the quad unstable */
        pid_reset();
    }

    /* From flyMode to engineStop: Stopping the motors: throttle low and yaw right. */
    if ((*mode == MOTORS_FLY_MODE) && (rx_pulse.three < 1160) && (rx_pulse.four > 1850))
        *mode = MOTORS_OFF;
}