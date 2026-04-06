from numpy import percentile
import joblib
from pathlib import Path
# modal estimation using max maf at a given RPM
LOWRPMMAX = 2000
MEDRPMMAX = 4000

class EngineLoadEstimator:
    def __init__(self, maxRPM=7000, lowBinSize=250, medBinSize=250, highBinSize=500, path="engine_load_model.joblib"):
        self.path = Path(path)

        if maxRPM < 4000:
            raise ValueError("maxRPM must be at least 4000 to have a valid high RPM bin")
        self.bins = None
        self.load_bins()
        if self.bins is None:
            self.bins = {}
            for rpm in range(0,  LOWRPMMAX, lowBinSize):
                self.bins[(rpm, rpm + lowBinSize)] = None

            for rpm in range(LOWRPMMAX, MEDRPMMAX, medBinSize):
                self.bins[(rpm, rpm + medBinSize)] = None
            for rpm in range(MEDRPMMAX, maxRPM + highBinSize, highBinSize):
                self.bins[(rpm, rpm + highBinSize)] = None

    def get_bin(self, rpm):
        for (lower, upper) in self.bins:
            if lower <= rpm and rpm < upper:
                return (lower, upper)
        return None

    def save_bins(self):
        joblib.dump(self.bins, self.path)

    def load_bins(self):
        if not self.path.exists():
            print("No existing model found, starting fresh.")
            return
        self.bins = joblib.load(self.path)

    def update(self, rpm, maf, acc):
        # filter out acc spikes which cause maf spikes
        if maf is None or rpm is None or acc is None or abs(acc) > 0.2:
            return

        bin = self.get_bin(rpm)
        if bin is None:
            return
        
        
        current = self.bins[bin]
        if current is None:
            self.bins[bin] = maf
        else:
            print(f"Updating bin {bin} with maf {maf} at rpm {rpm}")
            self.bins[bin] = max(current, maf)

    def estimate_load(self, rpm, maf):
        bin = self.get_bin(rpm)
        if bin is None:
            return None
        
        # maxMaf = self.bins[bin]
        # maxMaf = percentile(mafValues, 95)
        return (maf / self.bins[bin]) * 100 if self.bins[bin] is not None else None