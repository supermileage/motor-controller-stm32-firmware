import numpy as np
import matplotlib.pyplot as plt


STEP_MIN = 1
STEP_MAX = 8
STEP_RPM_MAX = 2000


rpm = np.linspace(0, 3000, 1000)
r = np.minimum(rpm, STEP_RPM_MAX)

# Smooth reference curve.
step_smooth = STEP_MIN + ((STEP_MAX - STEP_MIN) * r) / STEP_RPM_MAX

# Matches the integer truncation used in the STM32 code.
step_integer = STEP_MIN + np.floor(
    ((STEP_MAX - STEP_MIN) * r) / STEP_RPM_MAX
)


plt.figure(figsize=(8, 5))
plt.plot(rpm, step_smooth, label="Linear curve", linewidth=2)
plt.step(rpm, step_integer, where="post", label="Integer step used in C", linewidth=2)
plt.axvline(STEP_RPM_MAX, color="red", linestyle="--", label=f"Clamp @ {STEP_RPM_MAX} RPM")
plt.xlabel("RPM")
plt.ylabel("Step")
plt.title("Linear Duty Ramp Step vs RPM")
plt.grid(True, alpha=0.3)
plt.legend()
plt.tight_layout()
plt.show()
