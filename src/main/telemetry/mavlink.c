/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * telemetry_mavlink.c
 *
 * Author: Konstantin Sharlaimov
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_TELEMETRY) && defined(USE_TELEMETRY_MAVLINK)

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"

#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/navigation.h"
#include "flight/altitude.h"

#include "io/serial.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/motors.h"
#include "io/beeper.h"

#include "rx/rx.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/battery.h"

#include "telemetry/telemetry.h"
#include "telemetry/mavlink.h"

#include "build/debug.h"

// mavlink library uses unnames unions that's causes GCC to complain if -Wpedantic is used
// until this is resolved in mavlink library - ignore -Wpedantic for mavlink code
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "common/mavlink.h"
#pragma GCC diagnostic pop

#define TELEMETRY_MAVLINK_INITIAL_PORT_MODE MODE_TX
#define TELEMETRY_MAVLINK_MAXRATE 50
#define TELEMETRY_MAVLINK_DELAY ((1000 * 1000) / TELEMETRY_MAVLINK_MAXRATE)

extern uint16_t rssi; // FIXME dependency on mw.c

static serialPort_t *mavlinkPort = NULL;
static serialPortConfig_t *portConfig;

static bool mavlinkTelemetryEnabled =  false;
static portSharing_e mavlinkPortSharing;
static uint32_t lastJeVoistime = 0;
static uint32_t lastJeVoischeck = 0;
uint8_t my_mode = 0;
uint8_t JeVois_alive = 0;
float uart_altitude;
float uart_roll;
float uart_pitch;
float uart_yaw;

float my_param[8] = {0,0,0,0,0,0,0,0};

char param_name[6];

uint8_t parameterchanged = 0;

static void mavlinkReceive(uint16_t c, void *data)
{
    UNUSED(data);
    mavlink_message_t msg;
    mavlink_status_t status;
    if(mavlink_parse_char(MAVLINK_COMM_0, (uint8_t)c,&msg,&status))
    {

        switch(msg.msgid)
        {
            case 105: // highres_imu for test of communication
            {
                mavlink_highres_imu_t hr;
                mavlink_msg_highres_imu_decode(&msg,&hr);
                uart_altitude = hr.abs_pressure;
                DEBUG_SET(DEBUG_UART,1,hr.time_usec);	
                DEBUG_SET(DEBUG_UART,3,100*uart_altitude);
                break;
            }
            case 81: //setpoint command
            {
                lastJeVoistime = micros();
                mavlink_manual_setpoint_t command;
                mavlink_msg_manual_setpoint_decode(&msg,&command);
                uart_altitude = -command.thrust * 100;
                uart_roll = command.roll;   //maybe need normalization but this should be done in the JeVois
                uart_pitch = -command.pitch;
                uart_yaw = command.yaw;
                DEBUG_SET(DEBUG_UART,1,command.time_boot_ms);	
                DEBUG_SET(DEBUG_UART,3,uart_altitude);
                DEBUG_SET(DEBUG_COMMAND,0,uart_altitude);
                DEBUG_SET(DEBUG_COMMAND,1,uart_roll / 3.14 * 180);
                DEBUG_SET(DEBUG_COMMAND,2,uart_pitch / 3.14 * 180);
                DEBUG_SET(DEBUG_COMMAND,3,uart_yaw / 3.14 * 180);
                break;
            }
            case 21:
            {
                DEBUG_SET(DEBUG_REQUEST,0,100);
                break;
            }
            case 102:
            {
                mavlink_vision_position_estimate_t vision;
                mavlink_msg_vision_position_estimate_decode(&msg,&vision);
                DEBUG_SET(DEBUG_PHIL,0,vision.x);
                DEBUG_SET(DEBUG_PHIL,1,vision.y);
            }
        }
    }
}

/* MAVLink datastream rates in Hz */
static const uint8_t mavRates[] = {
    [MAV_DATA_STREAM_EXTENDED_STATUS] = 1, //1Hz
    [MAV_DATA_STREAM_RC_CHANNELS] = 5, //5Hz
    [MAV_DATA_STREAM_POSITION] = 2, //2Hz
    [MAV_DATA_STREAM_EXTRA1] = 15, //15Hz
    [MAV_DATA_STREAM_EXTRA2] = 15 //15Hz
};

#define MAXSTREAMS (sizeof(mavRates) / sizeof(mavRates[0]))

static uint8_t mavTicks[MAXSTREAMS];
static mavlink_message_t mavMsg;
static uint8_t mavBuffer[MAVLINK_MAX_PACKET_LEN];
static uint32_t lastMavlinkMessage = 0;

static int mavlinkStreamTrigger(enum MAV_DATA_STREAM streamNum)
{
    uint8_t rate = (uint8_t) mavRates[streamNum];
    if (rate == 0) {
        return 0;
    }

    if (mavTicks[streamNum] == 0) {
        // we're triggering now, setup the next trigger point
        if (rate > TELEMETRY_MAVLINK_MAXRATE) {
            rate = TELEMETRY_MAVLINK_MAXRATE;
        }

        mavTicks[streamNum] = (TELEMETRY_MAVLINK_MAXRATE / rate);
        return 1;
    }

    // count down at TASK_RATE_HZ
    mavTicks[streamNum]--;
    return 0;
}


static void mavlinkSerialWrite(uint8_t * buf, uint16_t length)
{
    for (int i = 0; i < length; i++)
        serialWrite(mavlinkPort, buf[i]);
}

void freeMAVLinkTelemetryPort(void)
{
    closeSerialPort(mavlinkPort);
    mavlinkPort = NULL;
    mavlinkTelemetryEnabled = false;
}

void initMAVLinkTelemetry(void)
{
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_MAVLINK);
    mavlinkPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_MAVLINK);
}

void configureMAVLinkTelemetryPort(void)
{
    if (!portConfig) {
        return;
    }

    baudRate_e baudRateIndex = portConfig->telemetry_baudrateIndex;
    if (baudRateIndex == BAUD_AUTO) {
        // default rate for minimOSD
        baudRateIndex = BAUD_57600;
    }

    mavlinkPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_MAVLINK, mavlinkReceive, NULL, baudRates[baudRateIndex], MODE_RXTX, telemetryConfig()->telemetry_inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED);

    if (!mavlinkPort) {
        return;
    }

    mavlinkTelemetryEnabled = true;
}

void checkMAVLinkTelemetryState(void)
{
    if (portConfig && telemetryCheckRxPortShared(portConfig)) {
        if (!mavlinkTelemetryEnabled && telemetrySharedPort != NULL) {
            mavlinkPort = telemetrySharedPort;
            mavlinkTelemetryEnabled = true;
        }
    } else {
        bool newTelemetryEnabledValue = telemetryDetermineEnabledState(mavlinkPortSharing);

        if (newTelemetryEnabledValue == mavlinkTelemetryEnabled) {
            return;
        }

        if (newTelemetryEnabledValue)
            configureMAVLinkTelemetryPort();
        else
            freeMAVLinkTelemetryPort();
    }
}

void mavlinkSendSystemStatus(void)
{
    uint16_t msgLength;

    uint32_t onboardControlAndSensors = 35843;

    /*
    onboard_control_sensors_present Bitmask
    fedcba9876543210
    1000110000000011    For all   = 35843
    0001000000000100    With Mag  = 4100
    0010000000001000    With Baro = 8200
    0100000000100000    With GPS  = 16416
    0000001111111111
    */

    if (sensors(SENSOR_MAG))  onboardControlAndSensors |=  4100;
    if (sensors(SENSOR_BARO)) onboardControlAndSensors |=  8200;
    if (sensors(SENSOR_GPS))  onboardControlAndSensors |= 16416;

    uint16_t batteryVoltage = 0;
    int16_t batteryAmperage = -1;
    int8_t batteryRemaining = 100;

    if (getBatteryState() < BATTERY_NOT_PRESENT) {
        batteryVoltage = isBatteryVoltageConfigured() ? getBatteryVoltage() * 100 : batteryVoltage;
        batteryAmperage = isAmperageConfigured() ? getAmperage() : batteryAmperage;
        batteryRemaining = isBatteryVoltageConfigured() ? calculateBatteryPercentageRemaining() : batteryRemaining;
    }

    mavlink_msg_sys_status_pack(0, 200, &mavMsg,
        // onboard_control_sensors_present Bitmask showing which onboard controllers and sensors are present.
        //Value of 0: not present. Value of 1: present. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure,
        // 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position,
        // 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization,
        // 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
        onboardControlAndSensors,
        // onboard_control_sensors_enabled Bitmask showing which onboard controllers and sensors are enabled
        onboardControlAndSensors,
        // onboard_control_sensors_health Bitmask showing which onboard controllers and sensors are operational or have an error.
        onboardControlAndSensors & 1023,
        // load Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
        0,
        // voltage_battery Battery voltage, in millivolts (1 = 1 millivolt)
        batteryVoltage,
        // current_battery Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
        batteryAmperage,
        // battery_remaining Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
        batteryRemaining,
        // drop_rate_comm Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
        0,
        // errors_comm Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
        0,
        // errors_count1 Autopilot-specific errors
        0,
        // errors_count2 Autopilot-specific errors
        0,
        // errors_count3 Autopilot-specific errors
        0,
        // errors_count4 Autopilot-specific errors
        0);
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}

void mavlinkSendRCChannelsAndRSSI(void)
{
    uint16_t msgLength;
    mavlink_msg_rc_channels_raw_pack(0, 200, &mavMsg,
        // time_boot_ms Timestamp (milliseconds since system boot)
        millis(),
        // port Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
        0,
        // chan1_raw RC channel 1 value, in microseconds
        (rxRuntimeConfig.channelCount >= 1) ? rcData[0] : 0,
        // chan2_raw RC channel 2 value, in microseconds
        (rxRuntimeConfig.channelCount >= 2) ? rcData[1] : 0,
        // chan3_raw RC channel 3 value, in microseconds
        (rxRuntimeConfig.channelCount >= 3) ? rcData[2] : 0,
        // chan4_raw RC channel 4 value, in microseconds
        (rxRuntimeConfig.channelCount >= 4) ? rcData[3] : 0,
        // chan5_raw RC channel 5 value, in microseconds
        (rxRuntimeConfig.channelCount >= 5) ? rcData[4] : 0,
        // chan6_raw RC channel 6 value, in microseconds
        (rxRuntimeConfig.channelCount >= 6) ? rcData[5] : 0,
        // chan7_raw RC channel 7 value, in microseconds
        (rxRuntimeConfig.channelCount >= 7) ? rcData[6] : 0,
        // chan8_raw RC channel 8 value, in microseconds
        (rxRuntimeConfig.channelCount >= 8) ? rcData[7] : 0,
        // rssi Receive signal strength indicator, 0: 0%, 255: 100%
        scaleRange(getRssi(), 0, 1023, 0, 255));
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}

void mavlinkSendHeartbeat(void)
{
    uint16_t msgLength;
    mavlink_msg_heartbeat_pack(0, 200, &mavMsg,
        // time_boot_ms Timestamp (milliseconds since system boot)
        // type
        0,
        // autopilot
        0,
        //base_mode
        0,
        //custon_mode
        0,
        // system_status
        0);
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}

#if defined(USE_GPS)
void mavlinkSendPosition(void)
{
    uint16_t msgLength;
    uint8_t gpsFixType = 0;

    if (!sensors(SENSOR_GPS))
        return;

    if (!STATE(GPS_FIX)) {
        gpsFixType = 1;
    }
    else {
        if (gpsSol.numSat < 5) {
            gpsFixType = 2;
        }
        else {
            gpsFixType = 3;
        }
    }

    mavlink_msg_gps_raw_int_pack(0, 200, &mavMsg,
        // time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        micros(),
        // fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
        gpsFixType,
        // lat Latitude in 1E7 degrees
        gpsSol.llh.lat,
        // lon Longitude in 1E7 degrees
        gpsSol.llh.lon,
        // alt Altitude in 1E3 meters (millimeters) above MSL
        gpsSol.llh.alt * 1000,
        // eph GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
        65535,
        // epv GPS VDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
        65535,
        // vel GPS ground speed (m/s * 100). If unknown, set to: 65535
        gpsSol.groundSpeed,
        // cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
        gpsSol.groundCourse * 10,
        // satellites_visible Number of satellites visible. If unknown, set to 255
        gpsSol.numSat);
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);

    // Global position
    mavlink_msg_global_position_int_pack(0, 200, &mavMsg,
        // time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        micros(),
        // lat Latitude in 1E7 degrees
        gpsSol.llh.lat,
        // lon Longitude in 1E7 degrees
        gpsSol.llh.lon,
        // alt Altitude in 1E3 meters (millimeters) above MSL
        gpsSol.llh.alt * 1000,
        // relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
#if defined(USE_BARO) || defined(USE_RANGEFINDER)
        (sensors(SENSOR_RANGEFINDER) || sensors(SENSOR_BARO)) ? getEstimatedAltitude() * 10 : gpsSol.llh.alt * 1000,
#else
        gpsSol.llh.alt * 1000,
#endif
        // Ground X Speed (Latitude), expressed as m/s * 100
        0,
        // Ground Y Speed (Longitude), expressed as m/s * 100
        0,
        // Ground Z Speed (Altitude), expressed as m/s * 100
        0,
        // heading Current heading in degrees, in compass units (0..360, 0=north)
        DECIDEGREES_TO_DEGREES(attitude.values.yaw)
    );
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);

    mavlink_msg_gps_global_origin_pack(0, 200, &mavMsg,
        // latitude Latitude (WGS84), expressed as * 1E7
        GPS_home[LAT],
        // longitude Longitude (WGS84), expressed as * 1E7
        GPS_home[LON],
        // altitude Altitude(WGS84), expressed as * 1000
        0);
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}
#endif

// send MAV states via MAVlink
void mavlinkSendMAVStates(void)
{
    uint16_t msgLength;
    mavlink_msg_highres_imu_pack(0, 200, &mavMsg,
    // time_boot_ms Timestamp (milliseconds since system boot)
    (uint64_t)microsISR(),
    // xacc
    accmx / 2048 * 9.80665,
    // yacc
    -accmy / 2048 * 9.80665,
    // zacc
    -accmz / 2048 * 9.80665,
    // rollspeed Roll angular speed (rad/s)
    gyrox,
    // pitchspeed Pitch angular speed (rad/s)
    gyroy,
    // yawspeed Yaw angular speed (rad/s)
    gyroz,
    // roll Roll angle (rad)
    DECIDEGREES_TO_RADIANS(attitude.values.roll),
    // pitch Pitch angle (rad)
    DECIDEGREES_TO_RADIANS(-attitude.values.pitch),
    // yaw Yaw angle (rad)
    DECIDEGREES_TO_RADIANS(attitude.values.yaw),
    // altitude (cm)
    (float)-my_altitude/100,   
    // extract 1 time
    0,
    // extract 2
    0,
    // extract 3
    0,
    // extract 4
    0);
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
    DEBUG_SET(DEBUG_UART,2,my_altitude);
    DEBUG_SET(DEBUG_MSG,0,accmx);
    DEBUG_SET(DEBUG_MSG,1,accmy);
    DEBUG_SET(DEBUG_MSG,2,accmz);
    DEBUG_SET(DEBUG_MSG2,0,gyrox);
    DEBUG_SET(DEBUG_MSG2,1,gyroy);
    DEBUG_SET(DEBUG_MSG2,2,gyroz);
    DEBUG_SET(DEBUG_ATTITUDE,0,attitude.values.roll);
    DEBUG_SET(DEBUG_ATTITUDE,1,attitude.values.pitch);
    DEBUG_SET(DEBUG_ATTITUDE,2,attitude.values.yaw);
}

void mavlinkSendAttitude(void)
{
    uint16_t msgLength;
    mavlink_msg_attitude_pack(0, 200, &mavMsg,
        // time_boot_ms Timestamp (milliseconds since system boot)
        millis(),
        // roll Roll angle (rad)
        DECIDEGREES_TO_RADIANS(attitude.values.roll),
        // pitch Pitch angle (rad)
        DECIDEGREES_TO_RADIANS(-attitude.values.pitch),
        // yaw Yaw angle (rad)
        DECIDEGREES_TO_RADIANS(attitude.values.yaw),
        // rollspeed Roll angular speed (rad/s)
        0,
        // pitchspeed Pitch angular speed (rad/s)
        0,
        // yawspeed Yaw angular speed (rad/s)
        0);
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}

// send MAV states via MAVlink
void mavlinkSendMAVMode(void)
{
    // check if Jevois still alive
    lastJeVoischeck = micros();
    if(lastJeVoischeck - lastJeVoistime < 5000000)
    {
        JeVois_alive = 1;
    }
    else
    {
        JeVois_alive = 0;
        beeper(BEEPER_RX_LOST);
    }
    if(FLIGHT_MODE(RANGEFINDER_MODE))
    {
        my_mode = 1; // autonomous mode
    }
    else
    {
        my_mode = 2;
    }
    uint16_t msgLength;
    mavlink_msg_set_mode_pack(0, 200, &mavMsg,
    // time_boot_ms Timestamp (milliseconds since system boot)
    0,
    my_mode,
    JeVois_alive);
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}

void mavlinkSendMAVParam(void)
{
    uint16_t msgLength;
    char name[16] = {'P','A','R','A','M'};
    mavlink_msg_param_request_read_pack(0, 200, &mavMsg,
    1,
    1,
    name,
    0);
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
    mavlink_msg_param_value_pack(0, 200, &mavMsg,
    // time_boot_ms Timestamp (milliseconds since system boot)
    name,
    0,
    0,
    1,
    0);
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}

void mavlinkSendHUDAndHeartbeat(void)
{
    uint16_t msgLength;
    float mavAltitude = 0;
    float mavGroundSpeed = 0;
    float mavAirSpeed = 0;
    float mavClimbRate = 0;

#if defined(USE_GPS)
    // use ground speed if source available
    if (sensors(SENSOR_GPS)) {
        mavGroundSpeed = gpsSol.groundSpeed / 100.0f;
    }
#endif

    // select best source for altitude
#if defined(USE_BARO) || defined(USE_RANGEFINDER)
    if (sensors(SENSOR_RANGEFINDER) || sensors(SENSOR_BARO)) {
        // Baro or sonar generally is a better estimate of altitude than GPS MSL altitude
        mavAltitude = getEstimatedAltitude() / 100.0;
    }
#if defined(USE_GPS)
    else if (sensors(SENSOR_GPS)) {
        // No sonar or baro, just display altitude above MLS
        mavAltitude = gpsSol.llh.alt;
    }
#endif
#elif defined(USE_GPS)
    if (sensors(SENSOR_GPS)) {
        // No sonar or baro, just display altitude above MLS
        mavAltitude = gpsSol.llh.alt;
    }
#endif

    mavlink_msg_vfr_hud_pack(0, 200, &mavMsg,
        // airspeed Current airspeed in m/s
        mavAirSpeed,
        // groundspeed Current ground speed in m/s
        mavGroundSpeed,
        // heading Current heading in degrees, in compass units (0..360, 0=north)
        DECIDEGREES_TO_DEGREES(attitude.values.yaw),
        // throttle Current throttle setting in integer percent, 0 to 100
        scaleRange(constrain(rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX), PWM_RANGE_MIN, PWM_RANGE_MAX, 0, 100),
        // alt Current altitude (MSL), in meters, if we have sonar or baro use them, otherwise use GPS (less accurate)
        mavAltitude,
        // climb Current climb rate in meters/second
        mavClimbRate);
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);


    uint8_t mavModes = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    if (ARMING_FLAG(ARMED))
        mavModes |= MAV_MODE_FLAG_SAFETY_ARMED;

    uint8_t mavSystemType;
    switch (mixerConfig()->mixerMode)
    {
        case MIXER_TRI:
            mavSystemType = MAV_TYPE_TRICOPTER;
            break;
        case MIXER_QUADP:
        case MIXER_QUADX:
        case MIXER_Y4:
        case MIXER_VTAIL4:
            mavSystemType = MAV_TYPE_QUADROTOR;
            break;
        case MIXER_Y6:
        case MIXER_HEX6:
        case MIXER_HEX6X:
            mavSystemType = MAV_TYPE_HEXAROTOR;
            break;
        case MIXER_OCTOX8:
        case MIXER_OCTOFLATP:
        case MIXER_OCTOFLATX:
            mavSystemType = MAV_TYPE_OCTOROTOR;
            break;
        case MIXER_FLYING_WING:
        case MIXER_AIRPLANE:
        case MIXER_CUSTOM_AIRPLANE:
            mavSystemType = MAV_TYPE_FIXED_WING;
            break;
        case MIXER_HELI_120_CCPM:
        case MIXER_HELI_90_DEG:
            mavSystemType = MAV_TYPE_HELICOPTER;
            break;
        default:
            mavSystemType = MAV_TYPE_GENERIC;
            break;
    }

    // Custom mode for compatibility with APM OSDs
    uint8_t mavCustomMode = 1;  // Acro by default

    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
        mavCustomMode = 0;      //Stabilize
        mavModes |= MAV_MODE_FLAG_STABILIZE_ENABLED;
    }
    if (FLIGHT_MODE(BARO_MODE) || FLIGHT_MODE(RANGEFINDER_MODE))
        mavCustomMode = 2;      //Alt Hold
    if (FLIGHT_MODE(GPS_HOME_MODE))
        mavCustomMode = 6;      //Return to Launch
    if (FLIGHT_MODE(GPS_HOLD_MODE))
        mavCustomMode = 16;     //Position Hold (Earlier called Hybrid)

    uint8_t mavSystemState = 0;
    if (ARMING_FLAG(ARMED)) {
        if (failsafeIsActive()) {
            mavSystemState = MAV_STATE_CRITICAL;
        }
        else {
            mavSystemState = MAV_STATE_ACTIVE;
        }
    }
    else {
        mavSystemState = MAV_STATE_STANDBY;
    }

    mavlink_msg_heartbeat_pack(0, 200, &mavMsg,
        // type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
        mavSystemType,
        // autopilot Autopilot type / class. defined in MAV_AUTOPILOT ENUM
        MAV_AUTOPILOT_GENERIC,
        // base_mode System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
        mavModes,
        // custom_mode A bitfield for use for autopilot-specific flags.
        mavCustomMode,
        // system_status System status flag, see MAV_STATE ENUM
        mavSystemState);
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}

void processMAVLinkTelemetry(void)
{
    DEBUG_SET(DEBUG_UART,0,100);
    // is executed @ TELEMETRY_MAVLINK_MAXRATE rate
    if (mavlinkStreamTrigger(MAV_DATA_STREAM_EXTENDED_STATUS)) {
        mavlinkSendSystemStatus();
        mavlinkSendMAVParam();
    }

    if (mavlinkStreamTrigger(MAV_DATA_STREAM_RC_CHANNELS)) {
        mavlinkSendHeartbeat();
    }
    if (mavlinkStreamTrigger(MAV_DATA_STREAM_EXTRA1)) {
        mavlinkSendMAVStates();
        mavlinkSendAttitude();
        mavlinkSendMAVMode();
    }
    // if (mavlinkStreamTrigger(MAV_DATA_STREAM_EXTRA2)) {
        
    // }
}

void handleMAVLinkTelemetry(void)
{
    if (!mavlinkTelemetryEnabled) {
        return;
    }

    if (!mavlinkPort) {
        return;
    }

    uint32_t now = micros();
    if ((now - lastMavlinkMessage) >= TELEMETRY_MAVLINK_DELAY) {
        processMAVLinkTelemetry();
        lastMavlinkMessage = now;
    }
}

#endif
