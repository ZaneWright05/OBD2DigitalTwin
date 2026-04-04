from dataclasses import dataclass

@dataclass(frozen=True)
class pid:
    name: str
    byte_count: int
    unit: str
    func: callable
    period_ms: int
