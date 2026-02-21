# tenso2
import numpy as np

# Tenso2
# points = [
#     (147070, 0),
#     (696759, 429),
#     (1101353, 744)
# ]


# Tenso3
points = [
    (87366, 0),
    (642572, 429),
    (1051450, 744)
]

# Convert to NumPy arrays
x = np.array([p[0] for p in points])
y = np.array([p[1] for p in points])

# Fit line y = a*x + b
a, b = np.polyfit(x, y, 1)

print(f"a (slope)     = {a}")
print(f"b (intercept) = {b}")

#tenso3: weight = 0.0007717730334784052 * rawVal -67.27574018734285
#tenso2: 0.0007796891947777704 * rawVal -114.54579575873767
