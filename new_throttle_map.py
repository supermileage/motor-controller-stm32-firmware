import numpy as np
import matplotlib.pyplot as plt


x = np.linspace(0.0, 1.0, 1000)

linear = x
quadratic = x**2

old_map = (linear + 2 * quadratic) / 3
new_map = (2 * linear + quadratic) / 3
pure_linear = linear


plt.figure(figsize=(8, 5))
plt.plot(x * 100, old_map * 100, label="Old map: (lin + 2*quad) / 3", linewidth=2)
plt.plot(x * 100, new_map * 100, label="New map: (2*lin + quad) / 3", linewidth=2)
plt.plot(x * 100, pure_linear * 100, label="Pure linear", linestyle="--", linewidth=2)
plt.xlabel("Throttle input (%)")
plt.ylabel("Duty output (%)")
plt.title("Throttle Mapping Comparison")
plt.grid(True, alpha=0.3)
plt.legend()
plt.tight_layout()
plt.show()
