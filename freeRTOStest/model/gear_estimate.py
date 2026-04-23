# semi - supervised estimator, use speed & rpm with clustering to estimate
# with enough user input KNN can be used

from dataclasses import dataclass
from pathlib import Path
import json
import joblib
import numpy as np
from sklearn.neighbors import KNeighborsClassifier
from sklearn.preprocessing import StandardScaler

@dataclass
class GearConfig:
    n_gears: int = 6 # number of gears excl rev
    min_speed: float = 5.0 # above this we consider the data
    k_neighbors: int = 5 # need tuning based on user input
    k_min_total: int = 24
    k_min_per_gear: int = 2

class GearEstimator:
    def __init__(self,model_path="gear_model.joblib"):
        self.config = GearConfig()
        self.model_path = Path(model_path)

        self.knn = KNeighborsClassifier(n_neighbors=self.config.k_neighbors, weights="distance")

        self.scaler = StandardScaler()

        self.knn_ready = False

        self.ratios = [] 
        self.gears = []
        self.last_save = 0

        self._load_if_exists()

    def _load_if_exists(self):
        if not self.model_path.exists():
            # print("No existing model found, starting fresh.")
            return
        obj = joblib.load(self.model_path)
        self.knn = obj["knn"]
        self.ratios = obj["ratios"]
        self.gears = obj["gears"]
        self.knn_ready = obj["knn_ready"]
        self.scaler = obj["scaler"]

    def save(self):
        joblib.dump({
            "knn": self.knn,
            "ratios": self.ratios,
            "gears": self.gears,
            "knn_ready": self.knn_ready,
            "scaler": self.scaler
        }, self.model_path)

    def add_data_point(self, rpm, speed, gear, throttle):
        ratio = rpm / speed
        data_point = np.array([rpm, speed, ratio, throttle], dtype=np.float64)
        print(f"Adding data point: RPM={rpm}, Speed={speed}, Ratio={ratio:.2f}, Gear={gear}")
        self.ratios.append(data_point)
        self.gears.append(int(gear))

        # self.last_save += 1
        self.train()

        # if(self.last_save >= 5):
        self.save()
            # self.last_save = 0
    
    def train(self):
        if len(self.gears) < self.config.k_min_total:
            return
        
        counts = {g: self.gears.count(g) for g in set(self.gears)}
        if min(counts.values()) < self.config.k_min_per_gear and len(counts) == self.config.n_gears:
            return
        
        X = np.vstack(self.ratios)
        Y = np.array(self.gears, dtype=int)
        X_scaled = self.scaler.fit_transform(X)  # Scale features
        self.knn.fit(X_scaled, Y)
        self.knn_ready = True

    def predict(self, rpm, speed, throttle):
        if not self.knn_ready:
            return (None, None)
        ratio = rpm / speed
        data_point = np.array([[rpm, speed, ratio, throttle]], dtype=np.float64)
        data_point_scaled = self.scaler.transform(data_point)  # Scale features
        gear = int(self.knn.predict(data_point_scaled)[0])
        
        distances, _ = self.knn.kneighbors(data_point_scaled, n_neighbors=1)
        confidence = 1.0 / (1.0 + float(distances[0][0]))
        return gear, confidence