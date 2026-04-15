import joblib
import time
from pathlib import Path

from model.helpers import MetricPoint, TelemetrySnapshot, ThermalPoint
from model.shadows.shadowState import ShadowState

class ThermalState(ShadowState):
    def __init__(self):
        self.state = "Unknown"
        self.confidence = 0.0

        self.stateStartTime = time.time()
        self.minStateTime_s = 1.0

        self.minSwitchConf = 0.55
        self.unknownCount = 0
        self.unknownLimit = 3

        self.warmedUp = False
        self.startTemp = None

    def reset_trip(self):
        self.state = "Unknown"
        self.confidence = 0.0
        self.stateStartTime = time.time()
        self.unknownCount = 0

        self.startTemp = None
        self.warmedUp = False
    

    # TODO: work on the heatsoak issues
    def determine_start(self, coolantTemp, airIntakeTemp = None) -> str:
        # clear cold and warm => thresholds from "x"
        self.startTemp = coolantTemp
        if coolantTemp < 35: 
            return "Cold Start"
        
        if coolantTemp > 50:
            return "Warm Start"
        
        if airIntakeTemp is not None:
            delta = coolantTemp - airIntakeTemp
            if delta < 5: # close we assume cold
                return "Cold Start"
        return "Warm Start"

    def is_cooling(self, tempROC: float, ROCthresh: float, speed: float, load: float, throttle: float) -> bool:
        return (
            tempROC < -ROCthresh and # temp dropping
            speed is not None and speed * 3.6 > 40 and # moving for airflow
            load is not None and load < 40 and # low engine load
            throttle is not None and throttle < 40 # low driver demand
        )

    def estimate_state(self, coolantTemp: float, coolantThreshold: float, 
                       overheatThreshold: float, tempLowThreshold: float, intakeTemp: float, 
                       tempROC: float, tempROCThresh: float, speed: float, load: float, throttle: float) -> str:

        coolantThreshold = coolantThreshold or 85.0
        overheatThreshold = overheatThreshold or 110.0
        tempLowThreshold = tempLowThreshold or 75.0

        if not self.warmedUp:
            if self.state == "Unknown": # first run 
                return self.determine_start(coolantTemp, intakeTemp)
            if self.state in ["Cold Start", "Warm Start"]:
                if self.startTemp is not None:
                    if coolantTemp >= self.startTemp + 10: # some increase in temp since start, we assume warming up
                        return self.state + ": Warming"
            if coolantTemp >= coolantThreshold:
                self.warmedUp = True
                return "Normal Running"
            
            return self.state
        
        if coolantTemp >= overheatThreshold:
            return "Overheating"

        if coolantTemp >= (overheatThreshold - 5):
            return "Overheat Risk"

        if coolantTemp >= (overheatThreshold - 5):
            return "Overheat Risk"

        if coolantTemp > tempLowThreshold:
            if self.is_cooling(tempROC, tempROCThresh, speed, load, throttle):
                return "Cooling"

        return "Normal Running"


    def get_state(self, snapshot: TelemetrySnapshot) -> str:
        thermalMetr = snapshot.metrics.get("0x05")
        if not thermalMetr or thermalMetr.current == 0.0:
            return self.state

        coolantThreshold = None
        overheatThreshold = None
        tempLowThreshold = None

        coolantMax = thermalMetr.allTripMax if thermalMetr.allTripMax is not None else 100.0
        coolantROCThresh = thermalMetr.allTripAverageROC * 1.5 if thermalMetr.allTripAverageROC is not None else 0.05

        if isinstance(thermalMetr, ThermalPoint):
            coolantThreshold = thermalMetr.coolantThreshold
            overheatThreshold = thermalMetr.overheatThreshold
            tempLowThreshold = thermalMetr.tempLowThreshold

        iatMetric = snapshot.metrics.get("0x0F")
        intakeTemp = iatMetric.current if iatMetric else None

        tempROC = thermalMetr.wAvgROC if thermalMetr.wAvgROC else 0.0


        speedMetric = snapshot.metrics.get("0x0D")
        speed = speedMetric.current if speedMetric else None

        loadMetric = snapshot.metrics.get("0x04")
        load = loadMetric.current if loadMetric else None

        throttleMetric = snapshot.metrics.get("0x11")
        throttle = throttleMetric.current if throttleMetric else None


        newState = self.estimate_state(
            coolantTemp=thermalMetr.current,
            coolantThreshold=coolantThreshold,
            overheatThreshold=overheatThreshold,
            tempLowThreshold=tempLowThreshold,
            intakeTemp=intakeTemp,
            tempROC=tempROC,
            speed=speed,
            load=load,
            throttle=throttle,
            tempROCThresh=coolantROCThresh
        )

        if newState is None:
            return self.state
        self.state = newState
        return self.state