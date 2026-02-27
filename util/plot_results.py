import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

df = pd.read_csv("../build/results.csv")
t = df["time"]

# =========================
# 3D Trajectory
# =========================
fig1 = plt.figure()
ax = fig1.add_subplot(111, projection='3d')
ax.plot(df["n"], df["e"], -df["d"])
ax.set_xlabel("North (m)")
ax.set_ylabel("East (m)")
ax.set_zlabel("Altitude (m)")
ax.set_title("3D Trajectory")

# =========================
# Position
# =========================
fig2 = plt.figure()
plt.plot(t, df["n"], label="North")
plt.plot(t, df["e"], label="East")
plt.plot(t, df["d"], label="Down")
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.title("Position (NED)")
plt.legend()
plt.grid()

# =========================
# Body Velocity
# =========================
fig3 = plt.figure()
plt.plot(t, df["u"], label="u")
plt.plot(t, df["v"], label="v")
plt.plot(t, df["w"], label="w")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.title("Body Velocity")
plt.legend()
plt.grid()

# =========================
# Euler Angles (deg)
# =========================
fig4 = plt.figure()
plt.plot(t, np.degrees(df["phi"]), label="Roll")
plt.plot(t, np.degrees(df["theta"]), label="Pitch")
plt.plot(t, np.degrees(df["psi"]), label="Yaw")
plt.xlabel("Time (s)")
plt.ylabel("Angle (deg)")
plt.title("Euler Angles")
plt.legend()
plt.grid()

# =========================
# Angular Rates
# =========================
fig5 = plt.figure()
plt.plot(t, df["p"], label="p")
plt.plot(t, df["q"], label="q")
plt.plot(t, df["r"], label="r")
plt.xlabel("Time (s)")
plt.ylabel("Angular Rate (rad/s)")
plt.title("Body Angular Rates")
plt.legend()
plt.grid()

# =========================
# Mass
# =========================
fig6 = plt.figure()
plt.plot(t, df["mass"])
plt.xlabel("Time (s)")
plt.ylabel("Mass (kg)")
plt.title("Mass vs Time")
plt.grid()

# =========================
# Speed
# =========================
fig7 = plt.figure()
plt.plot(t, df["V"])
plt.xlabel("Time (s)")
plt.ylabel("Speed (m/s)")
plt.title("Speed vs Time")
plt.grid()

# =========================
# Alpha & Beta (deg)
# =========================
fig8 = plt.figure()
plt.plot(t, np.degrees(df["alpha"]), label="Alpha")
plt.plot(t, np.degrees(df["beta"]), label="Beta")
plt.xlabel("Time (s)")
plt.ylabel("Angle (deg)")
plt.title("Angle of Attack & Sideslip")
plt.legend()
plt.grid()

# =========================
# Inertial Velocity
# =========================
fig9 = plt.figure()
plt.plot(t, df["Vn"], label="Vn")
plt.plot(t, df["Ve"], label="Ve")
plt.plot(t, df["Vd"], label="Vd")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.title("Inertial Velocity Components")
plt.legend()
plt.grid()

# Show ALL figures at once
plt.show()