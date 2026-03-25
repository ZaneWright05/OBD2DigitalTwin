import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

speed_by_seq = {}
rpm_by_seq = {}

with open("data_log_4.csv", "r", newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
        pid = row["PID"]
        data0 = int(row["Data0"], 16)
        data1 = int(row["Data1"], 16)
        seq = int(row["Seq"])

        if pid == "0x0D":
            speed_by_seq[seq] = float(data0)  # km/h
        elif pid == "0x0C":
            rpm_by_seq[seq] = ((data0 << 8) + data1) / 4.0

matching_seq = sorted(set(speed_by_seq) & set(rpm_by_seq))

speeds = []
rpms = []
ratios = []
for seq in matching_seq:
    s = speed_by_seq[seq]
    r = rpm_by_seq[seq]
    if s <= 5:
        continue
    speeds.append(s)
    rpms.append(r)
    ratios.append(r / s)

speeds = np.array(speeds, dtype=float)
rpms = np.array(rpms, dtype=float)
ratios = np.array(ratios, dtype=float)

# if len(ratios) == 0:
#     raise RuntimeError("No matching RPM/speed points above 5 km/h.")

# # Split ratio into 6 bands (approx gear clusters)
# n_clusters = 6
# edges = np.quantile(ratios, np.linspace(0, 1, n_clusters + 1))
# # Make edges strictly increasing if data has duplicates
# for i in range(1, len(edges)):
#     if edges[i] <= edges[i - 1]:
#         edges[i] = edges[i - 1] + 1e-6

# labels = np.digitize(ratios, edges[1:-1], right=False)  # 0..n_clusters-1
# cmap = plt.cm.get_cmap("tab10", n_clusters)

# plt.figure(figsize=(9, 6))
# plt.scatter(speeds, rpms, c=labels, cmap=cmap, s=12, alpha=0.75)

# legend_handles = []
# for i in range(n_clusters):
#     lo = edges[i]
#     hi = edges[i + 1]
#     handle = Line2D(
#         [0], [0],
#         marker="o",
#         color="w",
#         label=f"Band {i+1}: {lo:.1f} to {hi:.1f} rpm/kmh",
#         markerfacecolor=cmap(i),
#         markersize=8
#     )
#     legend_handles.append(handle)

# plt.legend(handles=legend_handles, title="Ratio Bands", loc="best", fontsize=8)
# plt.xlabel("Speed (km/h)")
# plt.ylabel("RPM")
# plt.title("RPM vs Speed with Ratio-Color Clusters")
# plt.grid(True, alpha=0.3)
# plt.tight_layout()
# plt.show()
# Build rough data-driven bands using largest gaps in ratio values
n_clusters = 6

# Trim extreme outliers so one bad point does not create a fake band
q_lo, q_hi = np.quantile(ratios, [0.01, 0.99])
use_mask = (ratios >= q_lo) & (ratios <= q_hi)

speeds_f = speeds[use_mask]
rpms_f = rpms[use_mask]
ratios_f = ratios[use_mask]

if len(ratios_f) < n_clusters:
    raise RuntimeError("Not enough filtered points to form bands.")

# Sort ratios and find largest gaps
order = np.argsort(ratios_f)
r_sorted = ratios_f[order]
gaps = np.diff(r_sorted)

# Pick top (n_clusters - 1) split locations
split_count = n_clusters - 1
split_idx = np.argsort(gaps)[-split_count:]
split_idx = np.sort(split_idx)

# Convert split positions into band edges
mids = (r_sorted[split_idx] + r_sorted[split_idx + 1]) / 2.0
edges = np.concatenate(([r_sorted[0] - 1e-6], mids, [r_sorted[-1] + 1e-6]))

print("\nDetected ratio bands (rpm/kmh):")
for i in range(n_clusters):
    lo = edges[i]
    hi = edges[i + 1]
    count = int(np.sum((ratios_f >= lo) & (ratios_f < hi))) if i < n_clusters - 1 else int(np.sum((ratios_f >= lo) & (ratios_f <= hi)))
    print(f"Band {i+1}: {lo:.2f} to {hi:.2f}  | points: {count}")

# Label each point by edge interval
labels = np.digitize(ratios_f, edges[1:-1], right=False)
cmap = plt.cm.get_cmap("tab10", n_clusters)

plt.figure(figsize=(9, 6))
plt.scatter(speeds_f, rpms_f, c=labels, cmap=cmap, s=12, alpha=0.75)

legend_handles = []
for i in range(n_clusters):
    lo = edges[i]
    hi = edges[i + 1]
    handle = Line2D(
        [0], [0],
        marker="o",
        color="w",
        label=f"Band {i+1}: {lo:.1f} to {hi:.1f} rpm/kmh",
        markerfacecolor=cmap(i),
        markersize=8
    )
    legend_handles.append(handle)

plt.legend(handles=legend_handles, title="Ratio Bands", loc="best", fontsize=8)
plt.show()