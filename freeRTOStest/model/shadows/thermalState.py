import joblib
import time
from pathlib import Path

from model.helpers import MetricPoint, TelemetrySnapshot
from model.shadows.shadowState import ShadowState

class OperationalState(ShadowState):
    def __init__(self):
        self.state = "Unknown"
        self.confidence = 0.0

        self.stateStartTime = time.time()
        self.minStateTime_s = 1.0

        self.minSwitchConf = 0.55
        self.unknownCount = 0
        self.unknownLimit = 3

    def reset_trip(self):
        self.state = "Unknown"
        self.confidence = 0.0
        self.stateStartTime = time.time()
        self.unknownCount = 0

    def get_state(self, snapshot: TelemetrySnapshot) -> str:
        return self.state