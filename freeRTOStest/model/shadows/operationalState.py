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
        # number of consec unknowns
        self.unknownCount = 0
        self.unknownLimit = 3


        self.idleBin = None
        self.idleCount = 0
        self.load_bin("idle_bin.joblib")
        if self.idleBin is None:
            self.idleBin = {}
            for temp in range(30, 130, 10):
                self.idleBin[temp] = {"rpm_ave" : None, "count": 0}

    def reset_trip(self):
        self.save_bin("idle_bin.joblib")
        self.state = "Unknown"
        self.confidence = 0.0
        self.stateStartTime = time.time()
        self.unknownCount = 0
        self.idleCount = 0

    def load_bin(self, filepath):
        try:
            if not Path(filepath).exists():
                self.idleBin = None
                return
            self.idleBin = joblib.load(filepath)
        except Exception as e:
            print(f"Error loading bin from {filepath}: {e}")
            self.idleBin = None

    def save_bin(self, filepath):
        try:
            joblib.dump(self.idleBin, filepath)
        except Exception as e:
            print(f"Error saving bin to {filepath}: {e}")

    def to_temp_bin(self, temp: float) -> int:
        if temp is None:
            return -1
        if temp < 30:
            return 30
        elif temp > 120:
            return 120
        else:
            return int(temp // 10 * 10)

    def learn_idle_rpm(self, rpm: float, temp: float):
        key = self.to_temp_bin(temp)
        val = self.idleBin[key]

        if val["rpm_ave"] is None:
            val["rpm_ave"] = rpm
        else:
            if val["count"] < 100:
                val["rpm_ave"] = (val["rpm_ave"] * val["count"] + rpm) / (val["count"] + 1)
                val["count"] += 1

    def gen_idle_score(self, rpm: float, temp: float) -> float:
        if temp == 0.0:
            return 0.1
        key = self.to_temp_bin(temp)
        val = self.idleBin[key]
        ave = val["rpm_ave"]
        count = val["count"]

        if ave is None or count < 3: # not enough data to be confident
            if rpm > 500 and rpm < 1300: # typical idle, low confidence
                return 0.6
            return 0.1

        diff = abs(rpm - ave)
        if diff <= 100:
            return 0.95
        elif diff <= 200:
            return 0.75
        elif diff <= 400: # not good enough to store but could be idle
            return 0.4

        if rpm < 1100: # last ditch low confidence idle
            return 0.3
        return 0.1  

    def gen_cruise_score(self, acc: float, rpmROC: float, throttle: float, historicThrottleMin: float, speed: float) -> float:
        score = 0.55 

        if abs(acc) <= 0.35:
            score += 0.2
        elif abs(acc) <= 0.5:
            score += 0.1

        if abs(rpmROC) <= 50:
            score += 0.1
        elif abs(rpmROC) <= 100:
            score += 0.05

        if throttle < historicThrottleMin * 1.8: 
            score += 0.1
        elif throttle < historicThrottleMin * 2.5:
            score += 0.05

        if speed < 20:
            score -= 0.1

        return max(0.0, min(score, 0.99))

    def gen_accel_score(self, acc: float, rpmROC: float, throttle: float, histThrottleMin: float) -> float:
        score = 0.5

        if acc > 1.5:
            score += 0.3
        elif acc > 1.0:
            score += 0.15

        if rpmROC > 100:
            score += 0.1
        elif rpmROC > 50:
            score += 0.05

        # base is ~ 20 so we look for 30 or 40 %
        if throttle > histThrottleMin * 2.5:
            score += 0.1
        elif throttle > histThrottleMin * 1.5: ## close to idle
            score += 0.05

        return max(0.0, min(score, 0.99))

    def gen_decel_score(self, acc: float, rpmROC: float, throttle: float, histThrottleMin: float) -> float:
        score = 0.5

        if acc < -1.5:
            score += 0.3
        elif acc < -1.0:
            score += 0.15

        if rpmROC < -100:
            score += 0.1
        elif rpmROC < -50:
            score += 0.05

        if throttle < histThrottleMin * 1.5: ## close to idle (<30%)
            score += 0.1
        elif throttle < histThrottleMin * 2: ## moderate throttle (30-40%)
            score += 0.05

        return max(0.0, min(score, 0.99))

    def estimate_state(self, rpm, rpmROC, speed, throttle, histThrottleMin, temp, acc) -> tuple[str, float]:
        if speed <= 1 and rpm <= 400:
            return 'Stopped', 1.0
        
        scores = {
            "Idling": 0.0,
            "Cruising": 0.0,
            "Accelerating": 0.0,
            "Decelerating": 0.0
        }
        
        # idle = low speed, low throttle, low acc
        if histThrottleMin is None:
            histThrottleMin = 20.0 # if no historic data, use 20% * 30 as threshold

        if speed * 3.6 <= 3 and throttle <= histThrottleMin * 1.2 and abs(acc) <= 0.4:
            idleScore = self.gen_idle_score(rpm, temp)
            scores["Idling"] = idleScore
            if idleScore >= 0.6:
                self.learn_idle_rpm(rpm, temp) # learn from confident idle data
                self.idleCount += 1
                if self.idleCount % 10 == 0: # save every 10 idle data points
                    self.save_bin("idle_bin.joblib") # save after learning
        
        # not wanted ass decel or cruising override -> if unknown is occ a lot can adapt other thresholds
        # if throttle <= histThrottleMin * 1.2 and acc > -0.4 and acc < -0.05 and speed > 2:
        #     scores["Coasting"] = 0.7 # coasting, not fully calib

        if rpm > 800 and rpm < 3000 and speed * 3.6 > 5 and abs(acc) <= 2: # high and low rpm is not cruising
            scores["Cruising"] = self.gen_cruise_score(acc, rpmROC, throttle, histThrottleMin, speed)

        ## filter out very low speed
        if acc > 0.5 and speed * 3.6 > 5:
            scores["Accelerating"] = self.gen_accel_score(acc, rpmROC, throttle, histThrottleMin)
        
        if acc < -0.5 and speed * 3.6 > 5:
            scores["Decelerating"] = self.gen_decel_score(acc, rpmROC, throttle, histThrottleMin)

        maxState = max(scores, key=scores.get)
        maxScore = scores[maxState]

        if maxScore < 0.30:
            return "Unknown", maxScore
        
        return maxState, maxScore
    
    def get_state(self, snapshot: TelemetrySnapshot) -> str:
        rpm = snapshot.metrics['0x0C']
        speed = snapshot.metrics['0x0D']
        throttle = snapshot.metrics['0x11']
        temp = snapshot.metrics['0x05']

        rpmROC = rpm.wAvgROC if rpm.wAvgROC is not None else 0.0
        acc = speed.wAvgROC if speed.wAvgROC is not None else 0.0
        histThrottleMin = throttle.allTripMin if throttle.allTripMin is not None else 20.0

        newState, newConf = self.estimate_state(rpm.current, rpmROC, speed.current, throttle.current, histThrottleMin, temp.current, acc)
        now = time.time()
        
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