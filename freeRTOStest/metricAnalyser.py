from helpers import pid
from dataclasses import dataclass
from collections import deque

@dataclass
class Metrics():
    average: float = 0.0
    window_Avg: float = 0.0
    max: float = float('-inf')
    min: float = float('inf')
    instROC: float = 0.0 # instantaneous rate of change
    minInstROC: float = float('inf')
    maxInstROC: float = float('-inf')
    wAvgROC: float = 0.0 # average rate of change over window
    minWAvgROC: float = float('inf')
    maxWAvgROC: float = float('-inf')
    current: float = 0.0

@dataclass
class HistoricMetrics(Metrics):
    tripCount: int = 0
    sumAverage: float = 0.0
    sumWAvgROC: float = 0.0
    sumInstROC: float = 0.0

@dataclass
class Event():
    timestamp: float
    type: str
    length: int # period * datapoints in thresh
    details: dict
    values: list[float] # raw values during event, for later analysis
    priority: int

class MetricAnalyser:
    def __init__(self, pid: pid, threshold, window_size, historicMetrics: HistoricMetrics = None):
        self.pid = pid
        self.data = []
        self.threshold = threshold
        self.historicMetrics = historicMetrics

        self.window_size = window_size
        self.sliding_window = deque(maxlen=window_size)
        self.window_sum = 0.0 # running sum for average calculation

        self.global_sum = 0.0


        self.metrics = Metrics()
        self.events = []

        self.above_threshold = False
        self.event_length = 0
        self.active_event = None

    def update_HistoricMetrics(self):
        if self.historicMetrics is None:
            self.historicMetrics = HistoricMetrics(
                average=self.metrics.average,
                window_Avg=self.metrics.window_Avg,
                max=self.metrics.max,
                min=self.metrics.min,
                instROC=self.metrics.instROC,
                minInstROC=self.metrics.minInstROC,
                maxInstROC=self.metrics.maxInstROC,
                wAvgROC=self.metrics.wAvgROC,
                minWAvgROC=self.metrics.minWAvgROC,
                maxWAvgROC=self.metrics.maxWAvgROC,
                tripCount=1,
                sumAverage=self.metrics.average,
                sumWAvgROC=self.metrics.wAvgROC,
                sumInstROC=self.metrics.instROC
            )
        else:
            self.historicMetrics.tripCount += 1

            self.historicMetrics.sumAverage += self.metrics.average # update running sum
            self.historicMetrics.sumWAvgROC += self.metrics.wAvgROC
            self.historicMetrics.sumInstROC += self.metrics.instROC

            # update historic metrics averages
            self.historicMetrics.average = self.historicMetrics.sumAverage / self.historicMetrics.tripCount
            self.historicMetrics.wAvgROC = self.historicMetrics.sumWAvgROC / self.historicMetrics.tripCount
            self.historicMetrics.instROC = self.historicMetrics.sumInstROC / self.historicMetrics.tripCount

            # update historic metrics max/min
            self.historicMetrics.max = max(self.historicMetrics.max, self.metrics.max)
            self.historicMetrics.min = min(self.historicMetrics.min, self.metrics.min)

            self.historicMetrics.maxInstROC = max(self.historicMetrics.maxInstROC, self.metrics.maxInstROC)
            self.historicMetrics.minInstROC = min(self.historicMetrics.minInstROC, self.metrics.minInstROC)

            self.historicMetrics.maxWAvgROC = max(self.historicMetrics.maxWAvgROC, self.metrics.maxWAvgROC)
            self.historicMetrics.minWAvgROC = min(self.historicMetrics.minWAvgROC, self.metrics.minWAvgROC)
        return self.historicMetrics


    def add_data_point(self, seq, value):
        self.data.append((seq, value))

        if(len(self.sliding_window) == self.window_size):
            _, oldest = self.sliding_window.popleft() # remove oldest value from window
            self.window_sum -= oldest
        
        self.window_sum += value
        self.global_sum += value

        self.sliding_window.append((seq, value))
        self.metrics.window_Avg = self.window_sum / len(self.sliding_window) if self.sliding_window else 0.0
        self.metrics.average = self.global_sum / seq if seq != 0 else 0.0 # seq is a proxy for num of data points

        self.metrics.min = min(self.metrics.min, value) if self.metrics.min != float('inf') else value
        self.metrics.max = max(self.metrics.max, value) if self.metrics.max != float('-inf') else value

        self.metrics.minInstROC = min(self.metrics.minInstROC, self.metrics.instROC) if self.metrics.minInstROC != float('inf') else self.metrics.instROC
        self.metrics.maxInstROC = max(self.metrics.maxInstROC, self.metrics.instROC) if self.metrics.maxInstROC != float('-inf') else self.metrics.instROC

        self.metrics.minWAvgROC = min(self.metrics.minWAvgROC, self.metrics.wAvgROC) if self.metrics.minWAvgROC != float('inf') else self.metrics.wAvgROC
        self.metrics.maxWAvgROC = max(self.metrics.maxWAvgROC, self.metrics.wAvgROC) if self.metrics.maxWAvgROC != float('-inf') else self.metrics.wAvgROC
        
        self.metrics.current = value

        if len(self.sliding_window) >= 2:
                ## calc roc between last 2 points
                (prev_seq, prev_val), (curr_seq, curr_val) = self.sliding_window[-2], self.sliding_window[-1]
                seq_diff = (curr_seq - prev_seq) * self.pid.period_ms # convert to ms
                self.metrics.instROC = (curr_val - prev_val) / seq_diff if seq_diff != 0 else 0.0

                # calc avg roc over window
                first_seq, first_val = self.sliding_window[0]
                total_seq_diff = (curr_seq - first_seq) * self.pid.period_ms
                self.metrics.wAvgROC = (curr_val - first_val) / total_seq_diff if total_seq_diff != 0 else 0.0

        else:
            self.metrics.instROC = 0.0
            self.metrics.wAvgROC = 0.0



        if value > self.threshold:
            if self.above_threshold:
                # self.event_length += 1
                self.active_event.length += 1
                self.active_event.values.append(value)
            else:
                self.above_threshold = True
                self.event_length = 1
                self.active_event = Event(
                    timestamp=seq,
                    type="above_threshold",
                    length=1,
                    details={"pid": self.pid, "threshold": self.threshold},
                    values=[value]
                )
        elif self.above_threshold:

            self.above_threshold = False
            # self.active_event.length = self.event_length * pid.period_ms
            self.events.append(self.active_event)

            self.active_event = None



    # def calc_average(self):
    #     if not self.data:
    #         return 0.0
    #     return sum(v for _, v in self.data) / len(self.data)