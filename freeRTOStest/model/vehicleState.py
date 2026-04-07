from time import sleep
from dataclasses import dataclass
from model.metricAnalyser import Metrics
from model.dataParser import Parser
from model.helpers import MetricPoint, TelemetrySnapshot

@dataclass
class VehicleStateModel:
    operational: str
    powertrain: str
    thermal: str
    efficiency: str
    health: str
    confidence: float
    reason: str

class VehicleState:
    def __init__(self):
        self.snapshot = None

        self.current = VehicleStateModel(
            operational='Unknown',
            powertrain='Unknown',
            thermal='Unknown',
            efficiency='Unknown',
            health='Unknown',
            confidence=0.0,
            reason=''
        )
        # self.speed = None
        # self.rpm = None
        # self.fuelCons = None
        # self.engineLoad = None
        # self.intakeTemp = None
        # self.coolantTemp = None
        # self.torque = None
        # self.gear = None


# need an anlyser so we can call get snapshot
# will create the operation state first 
# speed and rpm used here '
# cruisng, accelerating, decelerating, idling, off
    def operationalState(self):
        if self.snapshot is None:
            return 'Unknown'
        speedMetric = self.snapshot.metrics.get("0x0D")
        rpmMetric = self.snapshot.metrics.get("0x0C")

        if speedMetric is None or rpmMetric is None:
            return 'Unknown'

        speedVal = speedMetric.current
        rpmVal = rpmMetric.current
        accVal = speedMetric.wAvgROC

        if speedVal == 0 and rpmVal == 0:
            return 'Stopped'
        ## intend to add data based rpm range, d points with speed = 0, low thrttle and varying temp
        elif speedVal == 0 and rpmVal < 1000:
            return 'Idling'
        
        elif abs(accVal) < 0.5 and rpmVal >= 800 and rpmVal < 3000: ## want to improve 
            return 'Cruising'
        elif accVal > 0:
            return 'Accelerating'
        elif accVal < 0:
            return 'Decelerating'
        return 'Unknown'

    def update(self, snapshot: TelemetrySnapshot) -> VehicleStateModel:
        self.snapshot = snapshot
        self.current.operational = self.operationalState()
        self.current.freshness = snapshot.freshness
        self.current.reason = (
            f"speed={snapshot.metrics['0x0D'].current:.1f}, "
            f"rpm={snapshot.metrics['0x0C'].current:.0f}"
        )
        return self.current
