class Analyser:
    def process_packet(self, pid: str, data0: int, data1: int, seq: int) -> None:
        pass

    def store_gear(self, gear: int) -> None:
        pass
    def save_historic_metrics(self) -> None:
        pass
    def get_snapshot(self) -> "TelemetrySnapshot":
        pass