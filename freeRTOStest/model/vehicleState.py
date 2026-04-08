from time import sleep
from dataclasses import dataclass
from model.metricAnalyser import Metrics
from model.dataParser import Parser
from model.helpers import MetricPoint, TelemetrySnapshot
from model.shadows.operationalState import OperationalState
from model.shadows.powertrainState import PowerTrainState

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
        self.operationalState = OperationalState()
        self.powertrainState = PowerTrainState()

        self.current = VehicleStateModel(
            operational='Unknown',
            powertrain='Unknown',
            thermal='Unknown',
            efficiency='Unknown',
            health='Unknown',
            confidence=0.0,
            reason=''
        )
    
        
        self.lastChange_s = 0.0
        self.holdTime_s = 1.0

    def reset_state(self):
        self.current = VehicleStateModel(
            operational='Unknown',
            powertrain='Unknown',
            thermal='Unknown',
            efficiency='Unknown',
            health='Unknown',
            confidence=0.0,
            reason=''
        )
        self.operationalState.reset_trip()
        self.powertrainState.reset_trip()

    def update(self, snapshot: TelemetrySnapshot) -> VehicleStateModel:
        self.snapshot = snapshot
        if snapshot.metrics["0x11"].allTripMin is None:
            if snapshot.metrics["0x11"].min is not None:
                snapshot.metrics["0x11"].allTripMin = snapshot.metrics["0x11"].min
            else:
                snapshot.metrics["0x11"].allTripMin = 20 # no data 20% min throttle
        self.current.operational = self.operationalState.get_state(snapshot)
        self.current.powertrain = self.powertrainState.get_state(snapshot)
        self.current.confidence = 0.0 # placeholder
        # self.current.reason = (
        #     f"speed={snapshot.metrics['0x0D'].current:.1f}, "
        #     f"rpm={snapshot.metrics['0x0C'].current:.0f}"
        # )
        return self.current
