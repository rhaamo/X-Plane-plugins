"""
PI_NMEA_Serial.py version 0.1
copyright (c) 2009 Timor at cyhex; released under GPL 2.0 or later
copyright (c) 2021 Dashie; released under GPL 2.0 or later

PI_NMEA_Serial is a plugin for X-Plane. Based on Timor XTCPgps, It sends these NMEA sentences
over a Serial connection to mapping hardware: GPRMC, GPGGA.

Yes X-Plane has builtin NMEA Serial output, but it doesn't work, and my bug reports gets ignored.

This requires https://xppython3.readthedocs.io/en/latest/index.html

Put the script into Resources/plugins/PythonPlugins/

Requirements are:
- xppython3
- pyserial (install through pip)
"""

from XPLMProcessing import *  # noqa F403
from XPLMDataAccess import *  # noqa F403
from XPLMUtilities import *  # noqa F403

# import math
from datetime import date
import os
import serial

OutputFile = open(
    os.path.join(
        XPLMGetSystemPath(), "Resources", "plugins", "PythonPlugins", "XTCPgps.txt"
    ),
    "w",
)

"""
To test:
$GPBOD - Bearing, origin to destination http://aprs.gids.nl/nmea/#bod
$GPR00 - List of waypoints in currently active route http://aprs.gids.nl/nmea/#r00
$GPWPL - Waypoint location http://aprs.gids.nl/nmea/#wpl
$GPRTE - Routes http://aprs.gids.nl/nmea/#rte

Maybe a $GPRMB Recommended minimum navigation info for current WP

Sending waypoints:
GPR00 send the list
then one GPWPL per waypoint
"""

"""
http://www.cedricaoun.net/eie/trames%20NMEA183.pdf
http://aprs.gids.nl/nmea/
https://docs.novatel.com/OEM7/Content/Logs/GPGSA.htm
"""

"""
http://eventidier.com/argus/compat.htm
Argus 7000/CE Setup:
Powers on with ENR + AUX until self-test
Quickly press AUX 3 times, then ARR
Go to LRN TYPE SELECTION page
Choose: NMEA 0183
Then PAGE (DEP)
By default the BAUD RATE is 4800
Choose 19200 8 NONE 1
Then AUX to exit

Supported NMEA-0183 sentences (*** denotes already supported sentences by the plugin):
    APA: AUTO-PILOT 'A' (VALID, CROSS-TRACK DEVIATION, BEARING)
    GLL: LATITUDE, LONGITUDE
    VTG: TRACK, GROUND SPEED
    BOD: DESIRED TRACK
    BWC: BEARING, DISTANCE (GT. CIRCLE)
    HVD: MAGVAR (DERIVED)
    APB: AUTO-PILOT 'B' (VALID,CROSS-TRACK DEVIATION,BEARING,WAYPOINT ID,DISTANCE)
    RMB: GENERIC NAV INFO (VALID,CROSS_TRACK DEV.,WPT ID,DISTANCE,BEARING)
    ***RMC: GPS AND TRANSIT INFO (UTC TIME,VALID,LAT,LON,GROUND SPEED,TRACK,MAGVAR)
    R00: ROUTE DEFINITION
    RTE: ROUTE DEFINITION
    WPL: WAYPOINT LOCATION
    MAP: KING MARINE - AUTO-PILOT 'B'
    MLC: KING MARINE - LAT/LON
    ***GGA: GPS FIX RECORD
"""


def cksum(sentence):
    """calculates checksum for NMEA sentences"""
    i = 0
    cksum = 0
    senlen = len(sentence)
    while i < senlen:
        cksum = cksum ^ ord(sentence[i:i + 1])
        i = i + 1
    cksum = hex(cksum)[2:]
    return cksum


class SocketPlugin(object):
    SERPORT = "COM12"
    s = None

    def __init__(self):
        self.connect()

    def connect(self):
        self.s = serial.Serial(self.SERPORT, 19200)

    def write(self, data):
        self.s.write(data.encode())


class PythonInterface:
    def XPluginStart(self):
        self.Name = "NMEA_Serial"
        self.Sig = "Dashie.Python.NMEA_Serial"
        self.Desc = "A plugin to send NMEA sentences to mapping hardware over Serial."

        self.DEBUG = False

        # For possible debugging use:
        # Open a file to write to, located in the same directory as this plugin.
        self.OutputFile = OutputFile
        self.ser = SocketPlugin()

        # Locate data references for all communicated variables.

        # time and date
        self.drZulu_time = XPLMFindDataRef("sim/time/zulu_time_sec")
        self.drDate = XPLMFindDataRef("sim/time/local_date_days")
        # probably ok to set fixed date from system, not x-plane
        self.n_date = date.today().strftime("%d%m%y")

        # ground speed
        self.drVgnd_kts = XPLMFindDataRef("sim/flightmodel/position/groundspeed")

        # magnetic heading and variation
        self.drHding_mag = XPLMFindDataRef("sim/flightmodel/position/magpsi")
        self.drMag_var = XPLMFindDataRef("sim/flightmodel/position/magnetic_variation")

        # latitude, longitude, and altitude
        self.drLat_deg = XPLMFindDataRef("sim/flightmodel/position/latitude")
        self.drLon_deg = XPLMFindDataRef("sim/flightmodel/position/longitude")
        self.drAlt_ind = XPLMFindDataRef("sim/flightmodel/position/elevation")

        # indicated airspeed
        self.drIAS_ind = XPLMFindDataRef("sim/flightmodel/position/indicated_airspeed")

        # wind vector currently acting on the plane in KTS
        self.drWind_dir = XPLMFindDataRef(
            "sim/cockpit2/gauges/indicators/wind_heading_deg_mag"
        )
        self.drWind_speed = XPLMFindDataRef(
            "sim/cockpit2/gauges/indicators/wind_speed_kts"
        )

        # barometric pressure
        self.drBaro_alt = XPLMFindDataRef("sim/flightmodel/misc/h_ind")
        self.drVario_fpm = XPLMFindDataRef(
            "sim/cockpit2/gauges/indicators/total_energy_fpm"
        )

        # Register our callback for twice per 1-second.  Positive intervals
        # are in seconds, negative are the negative of sim frames.  Zero
        # registers but does not schedule a callback for time.
        self.FlightLoopCB = self.FlightLoopCallback
        XPLMRegisterFlightLoopCallback(self, self.FlightLoopCB, 0.5, 0)
        return self.Name, self.Sig, self.Desc

    def XPluginStop(self):
        # Unregister the callback.
        XPLMUnregisterFlightLoopCallback(self, self.FlightLoopCB, 0)
        self.OutputFile.close()

    def XPluginEnable(self):
        return 1

    def XPluginDisable(self):
        self.ser.close()

    def XPluginReceiveMessage(self, inFromWho, inMessage, inParam):
        pass

    def FlightLoopCallback(self, elapsedMe, elapsedSim, counter, refcon):
        # Get current values for communicated variables.
        self.Zulu_time = XPLMGetDataf(self.drZulu_time)  # sec since midnight
        self.Lat_deg = XPLMGetDatad(self.drLat_deg)
        self.Lon_deg = XPLMGetDatad(self.drLon_deg)
        self.Vgnd_kts = XPLMGetDataf(self.drVgnd_kts) * 1.943  # m/sec -> kts
        self.Hding_mag = XPLMGetDataf(self.drHding_mag)
        self.Mag_var = XPLMGetDataf(self.drMag_var)
        self.Alt_ind = XPLMGetDatad(self.drAlt_ind)

        # get values for LXWP0
        self.IAS = XPLMGetDataf(self.drIAS_ind) * 1.852  # kias -> kph
        self.Baro_Alt = XPLMGetDataf(self.drBaro_alt) * 0.3048  # feet-> meter
        self.Vario = XPLMGetDataf(self.drVario_fpm) * 0.00508  # fpm -> m/s
        self.Wind_Dir = XPLMGetDataf(self.drWind_dir)
        self.Wind_Speed = XPLMGetDataf(self.drWind_speed) * 1.852  # kias -> kph

        # put in nmea format (matches x-plane | equipment | nmea serial feed)
        # time ssss.ssss since midnight --> hhmmss
        hh = int(self.Zulu_time / 3600)
        mm = int(self.Zulu_time / 60) - 60 * hh
        ss = int(round(self.Zulu_time, 0)) - 3600 * hh - 60 * mm
        n_time = str(hh).zfill(2) + str(mm).zfill(2) + str(ss).zfill(2) + ".00"

        # lat and lon +/- ddd.dddd --> dddmm.mmm,h
        h = "N"
        if self.Lat_deg < 0:
            h = "S"
        self.Lat_deg = abs(self.Lat_deg)
        ddd = int(self.Lat_deg)
        mmm = 60 * (self.Lat_deg - ddd)
        n_lat = str(ddd).zfill(2) + ("%.4f" % mmm).zfill(7) + "," + h

        h = "E"
        if self.Lon_deg < 0:
            h = "W"
        self.Lon_deg = abs(self.Lon_deg)
        ddd = int(self.Lon_deg)
        mmm = 60 * (self.Lon_deg - ddd)
        n_lon = str(ddd).zfill(3) + ("%.4f" % mmm).zfill(7) + "," + h

        # speed and heading may need some padding
        n_speed = ("%.1f" % self.Vgnd_kts).zfill(5)
        # heading mag --> true
        n_heading = "%.1f" % (self.Hding_mag - self.Mag_var)

        # date set once above

        # magnetic variation +/- dd.dddd --> ddd.d,h
        h = "W"
        if self.Mag_var < 0:
            h = "E"
        self.Mag_var = abs(self.Mag_var)
        ddd = "%.1f" % self.Mag_var
        n_magvar = str(ddd) + "," + h

        # altitude meters
        n_alt = "%.1f" % self.Alt_ind

        # construct the nmea gprmc sentence
        gprmc = (
            "GPRMC"
            + ","
            + n_time
            + ","
            + "A"
            + ","
            + n_lat
            + ","
            + n_lon
            + ","
            + n_speed
            + ","
            + n_heading
            + ","
            + self.n_date
            + ","
            + n_magvar
        )

        # append check sum and inital $
        cks = cksum(gprmc)
        gprmc = f"${gprmc}*{cks}\r\n"

        # construct the nmea gpgga sentence
        gpgga = (
            "GPGGA"
            + ","
            + n_time
            + ","
            + n_lat
            + ","
            + n_lon
            + ",1,04,0.0,"
            + n_alt
            + ",M,,,,"
        )

        # append check sum and inital $
        cks = cksum(gpgga)
        gpgga = f"${gpgga}*{cks}\r\n"

        # serial write at 4800 baud can take .3 sec, so put in own thread;
        # write_thread = threading.Thread(target=self.ser.write, args=(gprmc + gpgga,))
        # write_thread.start()
        self.ser.write(gprmc + gpgga)
        if self.DEBUG:
            OutputFile.write(gprmc + gpgga)
            OutputFile.flush()

        # Return s.s to indicate that we want to be called again in s.s seconds.
        return 0.1
