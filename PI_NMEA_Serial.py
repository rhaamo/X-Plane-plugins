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
from XPLMNavigation import *

# import math
from datetime import date
import os
import serial
import threading
import pynmea2
from math import floor, ceil

OutputFile = open(
    os.path.join(
        XPLMGetSystemPath(), "Resources", "plugins", "PythonPlugins", "XTCPgps.txt"
    ),
    "w",
)

"""
http://www.cedricaoun.net/eie/trames%20NMEA183.pdf
http://aprs.gids.nl/nmea/
https://docs.novatel.com/OEM7/Content/Logs/GPGSA.htm

max sentence length, including $ and CRLF is 82 bytes
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

Supported NMEA-0183 sentences.
- *** denotes already supported sentences by the plugin
- !!! denotes WIP/Broken sentences by the plugin
- --- denotes to be implemented next
    APA: AUTO-PILOT 'A' (VALID, CROSS-TRACK DEVIATION, BEARING)
    GLL: LATITUDE, LONGITUDE
    VTG: TRACK, GROUND SPEED
    BOD: DESIRED TRACK
    BWC: BEARING, DISTANCE (GT. CIRCLE)
    HVD: MAGVAR (DERIVED)
    APB: AUTO-PILOT 'B' (VALID,CROSS-TRACK DEVIATION,BEARING,WAYPOINT ID,DISTANCE)
    ***RMB: GENERIC NAV INFO (VALID,CROSS_TRACK DEV.,WPT ID,DISTANCE,BEARING)
    ***RMC: GPS AND TRANSIT INFO (UTC TIME,VALID,LAT,LON,GROUND SPEED,TRACK,MAGVAR)
    ***R00: ROUTE DEFINITION --- the argus don't seems to understand/use it correctly
    ***RTE: ROUTE DEFINITION --- the argus don't seems to understand/use it correctly
    ***WPL: WAYPOINT LOCATION --- the argus don't seems to understand/use it correctly
    MAP: KING MARINE - AUTO-PILOT 'B'
    MLC: KING MARINE - LAT/LON
    ***GGA: GPS FIX RECORD
"""

"""
TODO:
- APA & APB ?
"""

### From https://github.com/rossengeorgiev/aprs-python/blob/master/aprslib/util/__init__.py

def degrees_to_ddm(dd):
    degrees = int(floor(dd))
    minutes = (dd - degrees) * 60
    return (degrees, minutes)


def latitude_to_ddm(dd):
    direction = "S" if dd < 0 else "N"
    degrees, minutes = degrees_to_ddm(abs(dd))

    return "{0:02d}{1:05.4f}".format(
        degrees,
        minutes,
        ), direction

def longitude_to_ddm(dd):
    direction = "W" if dd < 0 else "E"
    degrees, minutes = degrees_to_ddm(abs(dd))

    return "{0:03d}{1:05.4f}".format(
        degrees,
        minutes,
        ), direction

"""
Examples
NMEA: 1929.045
Lat: -19.484083333333334 S
NMEA: 02410.506
Lon: 24.1751 E
"""

### End from aprslib


class SocketPlugin(object):
    SERPORT = "COM12"
    SIM = True
    s = None

    def __init__(self):
        if not self.SIM:
            self.connect()
        else:
            print("serial: connect()")

    def connect(self):
        self.s = serial.Serial(self.SERPORT, 19200)
    
    def disconnect(self):
        if not self.SIM:
            self.s.close()
        else:
            print("serial: disconnect()")

    def write(self, data):
        if not self.SIM:
            try:
                self.s.write(data.encode())
            except serial.serialutil.SerialTimeoutException:
                print("Serial port timeout")
            except serial.serialutil.SerialException as e:
                print(f"Serial port error: {e}")
        else:
            print(data)


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

        # Autopilot
        self.drAutopilotMode = XPLMFindDataRef("sim/cockpit/autopilot/autopilot_mode")  # The autopilot master mode (off=0, flight director=1, on=2)

        self.drGPSdistance = XPLMFindDataRef("sim/cockpit2/radios/indicators/gps_dme_distance_nm")
        self.drGPSbearing = XPLMFindDataRef("sim/cockpit2/radios/indicators/gps_bearing_deg_mag")
        self.drGPShdefDot = XPLMFindDataRef("sim/cockpit/radios/gps_hdef_dot")  # sim/cockpit2/radios/indicators/gps_hdef_dots_pilot
        self.drGPShdefDotNm = XPLMFindDataRef("sim/cockpit/radios/gps_hdef_nm_per_dot")

        # Register our callback for twice per 1-second.  Positive intervals
        # are in seconds, negative are the negative of sim frames.  Zero
        # registers but does not schedule a callback for time.
        self.FlightLoopCB = self.FlightLoopCallback
        XPLMRegisterFlightLoopCallback(self, self.FlightLoopCB, 0.4, 0)

        # FlightPlan update every 5s
        self.FlightPlanCB = self.FlightPlanCallback
        XPLMRegisterFlightLoopCallback(self, self.FlightPlanCB, 10, 0)

        # Nav infos update every 0.5s
        self.FlightNavCB = self.FlightNavCallback
        XPLMRegisterFlightLoopCallback(self, self.FlightNavCB, 0.5, 0)

        return self.Name, self.Sig, self.Desc

    def XPluginStop(self):
        # Unregister the callback.
        XPLMUnregisterFlightLoopCallback(self, self.FlightLoopCB, 0)
        XPLMUnregisterFlightLoopCallback(self, self.FlightPlanCB, 0)
        self.OutputFile.close()
        self.ser.disconnect()

    def XPluginEnable(self):
        return 1

    def XPluginDisable(self):
        self.ser.disconnect()

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
        n_lat = latitude_to_ddm(self.Lat_deg)
        n_lon = longitude_to_ddm(self.Lon_deg)

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
        gprmc = pynmea2.RMC("GP", "RMC", (n_time, "A", n_lat[0], n_lat[1], n_lon[0], n_lon[1], n_speed, n_heading, self.n_date, n_magvar)).render() + '\r\n'
        
        # construct the nmea gpgga sentence
        gpgga = pynmea2.GGA("GP", "GGA", (n_time, n_lat[0], n_lat[1], n_lon[0], n_lon[1], "1", "04", "0.0", n_alt, "M", "", "", "", "")).render() + '\r\n'

        # serial write at 4800 baud can take .3 sec, so put in own thread;
        write_thread = threading.Thread(target=self.ser.write, args=(gprmc + gpgga,))
        write_thread.start()
        self.ser.write(gprmc + gpgga)
        if self.DEBUG:
            OutputFile.write(gprmc + gpgga)
            OutputFile.flush()

        return 0.4 # in 0.4s

    def FlightPlanCallback(self, elapsedMe, elapsedSim, counter, refcon):
        entriesInFMC = XPLMCountFMSEntries()

        if entriesInFMC > 0:
            entries = []
            wpts_list = []
            for i in range(entriesInFMC):
                wpt = XPLMGetFMSEntryInfo(i)
                entries.append(wpt)
                if wpt.navAidID.startswith("("):
                    wpts_list.append(wpt.navAidID[1:-1])
                else:
                    wpts_list.append(wpt.navAidID)

            # GPRTE
            # doesn't seems that great to use with the ARGUS
            # the order seems to be reversed, and the current route not taken into account
            # but this sentence is parsed
            # rte_list = []
            # rte_idx = ceil(entriesInFMC / 5)
            # split_by = 5
            # for wpts in [wpts_list[i:i + split_by] for i in range(0, len(wpts_list), split_by)] :
            #     gprte = pynmea2.RTE("GP", "RTE", (str(ceil(entriesInFMC / 5)), str(rte_idx), "c", *wpts)).render()
            #     rte_list.append(gprte + '\r\n')
            #     rte_idx -= 1
            # # For some reasons, the Argus parses them in reverse order received
            # rte_list.reverse()
            # print(rte_list)

            # R00 and WPL
            # They are weirdly understood by the Argus
            # some coordinates are mangled in the waypoints list
            # and the waypoint edit shows glitchy coordinates, but that may be because the argus don't known thoses waypoints ID in its DB
            # GPR00
            gpr00 = pynmea2.R00("GP", "R00", wpts_list).render() + '\r\n'
            print(gpr00)

            # Followed by all the GPWPL
            gpwpl_list = []
            for wpt in entries:
                if wpt.navAidID.startswith("("):
                    wpt_name = wpt.navAidID[1:-1]
                else:
                    wpt_name = wpt.navAidID
                lat =latitude_to_ddm(wpt.lat)
                lon = longitude_to_ddm(wpt.lon)
                gpwpl = pynmea2.WPL("GP", "WPL", (lat[0],lat[1], lon[0], lon[1], wpt_name)).render() + '\r\n'
                gpwpl_list.append(gpwpl)
                print(gpwpl)

            write_thread = threading.Thread(target=self.ser.write, args=(gpr00 +"".join(gpwpl_list),))
            # write_thread = threading.Thread(target=self.ser.write, args=("".join(rte_list),))
            write_thread.start()         
            

        # In 5 seconds
        return 10


    def FlightNavCallback(self, elapsedMe, elapsedSim, counter, refcon):
        currentDestinationID = XPLMGetGPSDestination()
        currentDestination = XPLMGetNavAidInfo(currentDestinationID)
        currentDestinationFMSID = XPLMGetDestinationFMSEntry()
        currentDestinationFMS = XPLMGetFMSEntryInfo(currentDestinationFMSID)

        print(f"Got GPS Dest {currentDestination.navAidID} and FMS dest {currentDestinationFMS.navAidID}")

        if currentDestination.navAidID != currentDestinationFMS.navAidID:
            return 0.5
        
        fmsEntries = XPLMCountFMSEntries()
        if currentDestinationFMSID >= fmsEntries:
            # last waypoint, no more after that
            return 0.5
        
        nextWaypointFMS = XPLMGetFMSEntryInfo(currentDestinationFMSID + 1)
        nextWaypointLat = latitude_to_ddm(nextWaypointFMS.lat)
        nextWaypointLon = longitude_to_ddm(nextWaypointFMS.lon)

        if currentDestinationFMSID == 0:
            lastWaypoint = ""
        else:
            lastWaypointFMS = XPLMGetFMSEntryInfo(currentDestinationFMSID - 1)
            lastWaypoint = lastWaypointFMS.navAidID
        
        self.Lat_deg = XPLMGetDatad(self.drLat_deg)
        self.Lon_deg = XPLMGetDatad(self.drLon_deg)

        us_to_wpt = XPLMGetDataf(self.drGPSdistance)

        self.Vgnd_kts = XPLMGetDataf(self.drVgnd_kts) * 1.943  # m/sec -> kts

        bearingToWpt = XPLMGetDataf(self.drGPSbearing)

        gpsDot = XPLMGetDataf(self.drGPShdefDot)
        gpsDotNm = XPLMGetDataf(self.drGPShdefDotNm)
        deviation = abs(gpsDot * gpsDotNm)

        gprmb = pynmea2.RMB("GP", "RMB", (
            "A",
            str("%.2f" % deviation) if deviation < 9.99 else "9.99",
            "L" if gpsDot < 0.0 else "R" , # Steer to L or R
            str(lastWaypoint),
            str(nextWaypointFMS.navAidID),
            str(nextWaypointLat[0]),
            str(nextWaypointLat[1]),
            str(nextWaypointLon[0]),
            str(nextWaypointLon[1]),
            str(999.9 if us_to_wpt >= 999.9 else ("%.1f" % us_to_wpt)),
            str(("%.1f" % bearingToWpt)),
            str(("%.1f" % self.Vgnd_kts).zfill(5)),
            "A" if us_to_wpt < 1 else "V"  # Arrived if <1nm, or V if not arrived
        )).render() + '\r\n'

        write_thread = threading.Thread(target=self.ser.write, args=(gprmb,))
        write_thread.start()       


        return 0.5
