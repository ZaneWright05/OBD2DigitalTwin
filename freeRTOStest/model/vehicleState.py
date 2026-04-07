from time import sleep
from dataclasses import dataclass
from model.metricAnalyser import Metrics
from model.dataParser import Parser
from model.helpers import MetricPoint, TelemetrySnapshot
from model.operationalState import OperationalState
from model.powertrainState import PowerTrainState

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

    def getOpState(self):
        state = self.operationalState.get_state(
            rpm=self.snapshot.metrics['0x0C'],
            speed=self.snapshot.metrics['0x0D'],
            throttle=self.snapshot.metrics['0x11'],
            temp=self.snapshot.metrics['0x05']
        )
        return state

    def getPowerTrainState(self):
        state = self.powertrainState.get_state(
            gear=self.snapshot.current_gear,
            gearRatio=self.snapshot.gear_ratio,
            load=self.snapshot.metrics['0x04'],
            throttle=self.snapshot.metrics['0x11'],
            rpm=self.snapshot.metrics['0x0C']
        )
        return state

    def update(self, snapshot: TelemetrySnapshot) -> VehicleStateModel:
        self.snapshot = snapshot
        if snapshot.metrics["0x11"].allTripMin is None:
            if snapshot.metrics["0x11"].min is not None:
                snapshot.metrics["0x11"].allTripMin = snapshot.metrics["0x11"].min
            else:
                snapshot.metrics["0x11"].allTripMin = 20 # no data 20% min throttle
        self.current.operational = self.getOpState()
        self.current.powertrain = self.getPowerTrainState()
        self.current.confidence = 0.0 # placeholder
        # self.current.reason = (
        #     f"speed={snapshot.metrics['0x0D'].current:.1f}, "
        #     f"rpm={snapshot.metrics['0x0C'].current:.0f}"
        # )
        return self.current
