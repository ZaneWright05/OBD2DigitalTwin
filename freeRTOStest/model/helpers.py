from dataclasses import dataclass
from typing import Dict, Optional

@dataclass
class MetricPoint:
    current: float
    average: float
    wAvgROC: float
    min: float
    max: float
    outOfSequence: bool
    eventCount: int
    highestPriority: int | None
    allTripAverage: float

    allTripMin: float | None = None
    allTripMax: float | None = None
    allTripAverageROC: float | None = None
    tripCount: int = 0

@dataclass
class TelemetrySnapshot:
    trip_time_s: float
    trip_distance_km: float
    freshness: float
    current_gear: Optional[int]
    gear_ratio: Optional[float]
    metrics: Dict[str, MetricPoint]

@dataclass(frozen=True)
class pid:
    name: str
    byte_count: int
    unit: str
    func: callable
    period_ms: int

PIDS = {
    "0x0C": pid("RPM", 2, "rpm", lambda a,b: ((a * 256) + b )/ 4, 333),
    "0x0D": pid("Speed", 1, "kmh", lambda a,b: a, 333),

    "0x11": pid("Throttle", 1, "%", lambda a,b: (a * 100)/255, 500),
    "0x05": pid("Engine Coolant Temperature", 1, "C", lambda a,b : a - 40, 500),
    "0x04": pid("Engine Load", 1, "%", lambda a,b: (a * 100)/255, 500),

    "0x10": pid("Mass Air Flow", 2, "g/s", lambda a,b : ((256 * a) + b)/100,1000),
    "0x42": pid("Battery Voltage", 2, "V", lambda a,b: ((256* a) + b)/1000, 1000),
    
    "0x0F": pid("Intake Air Temperature", 1, "C", lambda a,b : a - 40,2000),
    "0x23": pid("Intake Manifold Pressure", 2, "kPa", lambda a,b : 10*((256*a) + b),2000),

    "0x1F": pid("Engine Runtime", 2, "s", lambda a, b: (256 * a) + b,4000)
    }

FUELCONSPID = "0xFF"
COMPUTEDPIDS = {
    "0xFF" : pid("Instantaneous Fuel Consumption", 2, "l/100km", lambda speed, speedSeq, maf, mafSeq: speed, max(PIDS["0x0D"].period_ms, PIDS["0x10"].period_ms, PIDS["0x04"].period_ms))
}

# ranges from https://caracaltech.com/articles/article/627981fe2fb15a8d9e50f99c
# lambda afr/afrStoch (14.5 diesel), afr vals from link above
def load_to_lambda(load):
    if load < 100/3: # low load
        lowRange = (6.9, 2.8)
        pos = load / (100/3)
        return lowRange[0] + pos * (lowRange[1] - lowRange[0])
    elif load < 100 * (2/3): # medium load
        medRange = (2.8, 1.7)
        pos = (load - 100/3) / (100 * (2/3) - 100/3)
        return medRange[0] + pos * (medRange[1] - medRange[0])
    else: # highload
        highRange = (1.7, 1.1)
        pos = (load - (100 * (2/3))) / (100 - (100 * (2/3)))
        return highRange[0] + pos * (highRange[1] - highRange[0])

def calcInstFuelCons(speed, speedSeq, maf, mafSeq, load, loadSeq):
    speedTime = speedSeq * PIDS["0x0D"].period_ms / 1000.0
    mafTime = mafSeq * PIDS["0x10"].period_ms / 1000.0
    loadTime = loadSeq * PIDS["0x04"].period_ms / 1000.0
    valid = abs(speedTime - mafTime) < 0.5 and abs(speedTime - loadTime) < 0.5 and abs(mafTime - loadTime) < 0.5
    if valid and speed != 0: # compare timestamps

        ff = (maf * 3600) / (load_to_lambda(load) *14.5 * 820)
        fCons = ff / speed
        return (fCons * 100) # convert to l/100km
    else:
        return None
