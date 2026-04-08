from abc import ABC, abstractmethod
from dataclasses import dataclass
from model.helpers import TelemetrySnapshot

class ShadowState(ABC):
    @abstractmethod
    def get_state(self, snapshot:TelemetrySnapshot) -> str:
        pass

    @abstractmethod
    def reset_trip(self) -> None:
        pass