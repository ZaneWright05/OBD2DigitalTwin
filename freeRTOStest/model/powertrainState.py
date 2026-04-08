# gear, ratio, engine load, throttle, rpm

from model.helpers import MetricPoint, TelemetrySnapshot
from time import time
from model.shadowState import ShadowState

class PowerTrainState(ShadowState):
    def __init__(self):
        self.state = "Unknown"
        self.confidence = 0.0
        self.stateStartTime = time()
        self.minStateTime_s = 1.0

        self.minSwitchConf = 0.55
        # number of consec unknowns
        self.unknownCount = 0
        self.unknownLimit = 3
    
    def reset_trip(self):
        self.state = "Unknown"
        self.confidence = 0.0
        self.stateStartTime = time()
        self.unknownCount = 0

    def gen_neutral_score(self, gear, rpm, minRpm) -> float:
        score = 0.0
        if (gear is None or gear == 0) and rpm > minRpm: # check enif is on
            score = 0.5
            if rpm < minRpm * 1.25: # low rpm in neutral
                score += 0.3
            if gear == 0: 
                score += 0.1
        
        return score

    def gen_driving_score(self, gear, load, throttle, throttleMin) -> float:
        score = 0.5 if gear else 0.3 # bump if in gear
        if load > 10 and load < 60: # 20-60%
            score += 0.3
        if throttle > throttleMin * 1.3: # above historical min throttle
            score += 0.1
        return score

    def gen_highLoad_score(self, gear, load, rpmROC) -> float:
        score = 0.0
        if load > 70:
            score = 0.8
        elif load > 50:
            score = 0.5

        if gear:
            if gear <= 3 and gear > 0:
                score += 0.1
            if gear > 3: # higher gear == lower load generally
                score += 0.05 

        if rpmROC > 150: ## peak in rpm == high load
            score += 0.1

        return min(score, 0.99)

    def gen_engineBraking_score(self, gear, throttle, throttleMin, load, rpmROC) -> float:
        score = 0.0
        if throttle <= throttleMin * 1.2:
            score += 0.5
        if load <= 0.2:
            score += 0.1
        if rpmROC < -0.25: # engine speed slowing
            score += 0.1
        if gear:
            score += 0.1
        return min(score, 0.99)

    def gen_gearShift_score(self, gear, throttle, throttleMin, rpmROC) -> float:
        score = 0.0
        if abs(rpmROC) > 250 and throttle <= throttleMin * 1.5: # rpm jump with low throttle
            score = 0.7
        if gear:
            score += 0.1
        return score


    def estimate_state(self, gear, load, throttle, minThrottle, rpm, rpmROC) -> tuple[str, float]:
        # histThrottleMin = throttle.allTripMin if throttle.allTripMin is not None else 20.0
        
        states = {"NeutralIdle": self.gen_neutral_score(gear, rpm, minThrottle), # engine on, no power delivery
          "Driving": self.gen_driving_score(gear, load, throttle, minThrottle), # normal drive
          "HighLoad": self.gen_highLoad_score(gear, load, rpmROC), # engine working hard (acc/uphill)
          "EngineBraking": self.gen_engineBraking_score(gear, throttle, minThrottle, load, rpmROC), # deceleration in gear with low throttle
          "GearShift": self.gen_gearShift_score(gear, throttle, minThrottle, rpmROC), # rpm drop with low throttle, not decelerating
        }

        maxState = max(states, key=states.get)
        maxScore = states[maxState]

        if maxScore < 0.35:
            return "Unknown", maxScore

        return maxState, maxScore

    def get_state(self, snapshot: TelemetrySnapshot) -> str:
        gear = snapshot.current_gear
        load = snapshot.metrics['0x10']
        throttle = snapshot.metrics['0x11']
        rpm = snapshot.metrics['0x0C']

        newState, newConf = self.estimate_state(gear, load.current, throttle.current, throttle.allTripMin, rpm.current, rpm.wAvgROC)
        now = time()
        
        if newState == self.state: # update confidence if same state
            self.confidence = newConf
            self.unknownCount = 0
            return self.state

        if newState == "Unknown" and self.state != "Unknown":
            self.unknownCount += 1

            if self.unknownCount < self.unknownLimit:
                return self.state
            
            self.state = "Unknown"
            self.confidence = newConf
            self.stateStartTime = now
            self.unknownCount = 0
            return self.state

        self.unknownCount = 0 # reset unknown count

        if self.state == "Unknown":
            if newConf >= self.minSwitchConf:
                self.state = newState
                self.confidence = newConf
                self.stateStartTime = now
            return self.state

        if (now - self.stateStartTime) >= self.minStateTime_s or newConf > self.confidence:
            self.state = newState
            self.confidence = newConf
            self.stateStartTime = now
        return self.state