from helpers import pid
from dataclasses import dataclass
from collections import deque
import numpy as np
from scipy.signal import savgol_filter

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
    timestamp: float # start seq
    type: str
    length: int # * period for the acc len
    details: list[str] # string based info
    values: list[float] # raw values during event, for later analysis
    priority: int

eventTypes = ["above_threshold", "below_threshold", "rapid_increase", "rapid_decrease", ]
minTrips = 3

class MetricAnalyser:
    def __init__(self, pid: pid, conversionFactor: float = 1.0, highThreshold: float = None, lowThreshold: float = None, rocMin: float = 0.1, window_size: int = 5, historicMetrics: HistoricMetrics = None, eventsTracked: bool = True):
        self.pid = pid
        self.data = []
        self.highThreshold = highThreshold # default to None, instances where we look for above and below thresh events
        self.lowThreshold = lowThreshold
        self.rocMin = rocMin
        self.historicMetrics = historicMetrics

        self.window_size = window_size
        self.sliding_window = deque(maxlen=window_size)
        self.window_sum = 0.0 # running sum for average calculation

        self.global_sum = 0.0
        self.global_count = 0

        self.conversionFactor = conversionFactor

        self.metrics = Metrics()
        self.events = []

        self.active_events = {}
        self.eventsTracked = eventsTracked

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

    def all_trip_average(self):
        if self.historicMetrics and self.historicMetrics.tripCount > 0:
            if(self.metrics.average == 0): # if no current data, return historic average
                return self.historicMetrics.average
            else:
                return (self.metrics.average + ( self.historicMetrics.average *  self.historicMetrics.tripCount)) / (1 + self.historicMetrics.tripCount)
                # current ave * 1 + historic ave * trips / 1 + trip count - average over all trips
        elif (self.metrics.average != 0):
            return self.metrics.average
        return 0.0


    def add_data_point(self, seq, value):
        if value is None:
            self.metrics.current = 0.00 # mark to the UI but dont store for calcs, none is is no data not the value 0
            return

        self.data.append((seq, value))

        if(len(self.sliding_window) == self.window_size):
            _, oldest = self.sliding_window.popleft() # remove oldest value from window
            self.window_sum -= oldest
        
        self.window_sum += value
        self.global_sum += value
        self.global_count += 1

        self.sliding_window.append((seq, value/ self.conversionFactor))
        self.metrics.window_Avg = self.window_sum / len(self.sliding_window) if self.sliding_window else 0.0
        self.metrics.average = self.global_sum / self.global_count if self.global_count != 0 else 0.0 # seq is a proxy for num of data points

        self.metrics.min = min(self.metrics.min, value) if self.metrics.min != float('inf') else value
        self.metrics.max = max(self.metrics.max, value) if self.metrics.max != float('-inf') else value
        
        self.metrics.current = value

        if len(self.sliding_window) >= 2:
                ## calc roc between last 2 points
                (prev_seq, prev_val), (curr_seq, curr_val) = self.sliding_window[-2], self.sliding_window[-1]
                seq_diff = (curr_seq - prev_seq) * self.pid.period_ms # convert to ms
                # print(f"Calculating ROC: curr_val={curr_val}, prev_val={prev_val}, seq_diff={seq_diff}ms")
                self.metrics.instROC = (curr_val - prev_val) / seq_diff if seq_diff != 0 else 0.0

                # # calc avg roc over window
                # first_seq, first_val = self.sliding_window[0]
                # total_seq_diff = (curr_seq - first_seq) * self.pid.period_ms
                # self.metrics.wAvgROC = (curr_val - first_val) / total_seq_diff if total_seq_diff != 0 else 0.0
        else:
            self.metrics.instROC = 0.0
            self.metrics.wAvgROC = 0.0

        self.metrics.wAvgROC = self.applySGFilter()

        self.metrics.minInstROC = min(self.metrics.minInstROC, self.metrics.instROC) if self.metrics.minInstROC != float('inf') else self.metrics.instROC
        self.metrics.maxInstROC = max(self.metrics.maxInstROC, self.metrics.instROC) if self.metrics.maxInstROC != float('-inf') else self.metrics.instROC

        self.metrics.minWAvgROC = min(self.metrics.minWAvgROC, self.metrics.wAvgROC) if self.metrics.minWAvgROC != float('inf') else self.metrics.wAvgROC
        self.metrics.maxWAvgROC = max(self.metrics.maxWAvgROC, self.metrics.wAvgROC) if self.metrics.maxWAvgROC != float('-inf') else self.metrics.wAvgROC

        if not self.eventsTracked:
            return

        if self.highThreshold is not None:
            self.above_event(value, seq)
        if self.lowThreshold is not None:
            self.below_event(value, seq)
        
        self.rapid_increase(seq)
        self.rapid_decrease(seq)

    def applySGFilter(self):
    # print(f"Applying SG filter to window: {window}")
                    # self.timeWindow.append(time)
        if len(self.sliding_window) == self.sliding_window.maxlen:
            y = np.array([val for _, val in self.sliding_window], dtype=np.float64)
            delta = self.pid.period_ms / 1000.0

            dydt = savgol_filter(
                y,
                window_length=self.sliding_window.maxlen,
                polyorder=2,
                deriv=1,
                delta=delta,
                mode="interp" 
                )
            return float(dydt[-1])
        else:
            return 0.0

    def end_event(self, eventType, seq):
        if self.active_events.get(eventType):
                print(f"Ending {eventType} event")
                event = self.active_events.pop(eventType)
                self.events.append(event)
                print(f"Ended Event: {event.type} at {seq}, duration {event.length}, details: {event.details}, values: {event.values}, priority: {event.priority}")
        return

    def handle_event(self, seq, eventType, val, pri):
        if(self.active_events.get(eventType)):
            event = self.active_events[eventType]
            print(f"Event {eventType}, updating values")
            currPri = event.priority
            event.length += 1
            event.priority = pri
            event.values.append((seq, val))
            if pri > currPri:
                event.details.append(f"{seq}: {val}, new priority: {pri}")
            elif pri < currPri:
                event.details.append(f"{seq}: {val}, new priority: {pri}")
            self.active_events[eventType] = event
        else:
            print(f"New {eventType} event detected, creating event")
            self.active_events[eventType] = Event(
                    timestamp=seq,
                    type=eventType,
                    length=1,
                    details=[f"{seq}: {val}, priority {pri}"],
                    values=[(seq, val)],
                    priority=pri
            )

    def rapid_decrease(self, seq):
        roc = self.metrics.wAvgROC
        if roc >= 0 or abs(roc) < self.rocMin:
            self.end_event("rapid_decrease", seq)
            return
        if not self.historicMetrics or self.historicMetrics.tripCount <= minTrips:
            bound = self.metrics.minWAvgROC * 1.2 if self.metrics.minWAvgROC < 0 else float('-inf')
            hist_min = bound
        else:
            hist_min = self.historicMetrics.minWAvgROC if self.historicMetrics.minWAvgROC != float('inf') else -0.1
            bound = min(hist_min * 1.2, self.metrics.wAvgROC * 2)

        pri = None
        if roc < bound:
            pri = 2
        elif roc < hist_min:
            pri = 1
        elif roc < self.metrics.wAvgROC * 1.5:
            pri = 0
        else:
            self.end_event("rapid_decrease", seq)
            return
        
        self.handle_event(seq, "rapid_decrease", roc, pri)

    def rapid_increase(self, seq):
        roc = self.metrics.wAvgROC
        if roc <= 0 or abs(roc) < self.rocMin:
            self.end_event("rapid_increase", seq)
            return
        
        if not self.historicMetrics or self.historicMetrics.tripCount <= minTrips:
            bound = self.metrics.maxWAvgROC * 2 if self.metrics.maxWAvgROC > 0 else float('inf')
            hist_max = bound
        else:
            hist_max = self.historicMetrics.maxWAvgROC
            bound = max(hist_max * 1.2, self.metrics.wAvgROC * 2)

        pri = None
        if roc > bound:
            pri = 2
        elif roc > hist_max:
            pri = 1
        elif roc > self.metrics.wAvgROC * 1.5:
            pri = 0
        else:
            self.end_event("rapid_increase", seq)
            return
        
        self.handle_event(seq, "rapid_increase", roc, pri) 

    def above_event(self, val, seq):
        baseLine = self.highThreshold if self.highThreshold is not None else float('inf')
        if(self.historicMetrics.tripCount <= minTrips): # not enough historic data
            historicPeak = 0
            avg = 0
            dynamicThreshold = baseLine * 0.85
        else:
            historicPeak = self.historicMetrics.max if self.historicMetrics and self.historicMetrics.max > 0 else 0
            avg = self.historicMetrics.average if self.historicMetrics else 0
            dynamicThreshold = max(baseLine * 0.85, historicPeak * 0.85)

        pri = None
        if val >= baseLine * 0.9: # close to abs max
            pri = 2
        elif val >= dynamicThreshold: # close to worst val recorded
            pri = 1
        elif avg != 0 and val >= avg * 1.5: # above average
            pri = 0
        else:
            self.end_event("above_threshold", seq)
            return
        
        self.handle_event(seq, "above_threshold", val, pri)

    def below_event(self, val, seq):
        baseLine = self.lowThreshold if self.lowThreshold is not None else float('-inf')
        if not self.historicMetrics or self.historicMetrics.tripCount <= minTrips:
            historicMin = baseLine
            avg = 0
            dynamicThreshold = baseLine * 1.15
        else:
            historicMin = self.historicMetrics.min if self.historicMetrics.min != float('inf') else baseLine
            avg = self.historicMetrics.average if self.historicMetrics else baseLine
            dynamicThreshold = min(baseLine * 1.15, historicMin * 1.15)

        pri = None
        if val <= baseLine * 1.1: # close to abs min
            pri = 2
        elif val <= dynamicThreshold: # close to lowest val recorded
            pri = 1
        elif avg != 0 and val <= avg * 0.5: # below average
            pri = 0
        else:
            self.end_event("below_threshold", seq)
            return
        
        self.handle_event(seq, "below_threshold", val, pri)