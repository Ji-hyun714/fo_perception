'''
// Left lane
y = 0.001749498536810279 * x*x*x
  - 0.04562922194600105 * x*x
  + 0.40893250703811646 * x
  - 0.23083217442035675;

// Right lane
y = -0.002768676495179534 * x*x*x
  + 0.06821615248918533 * x*x
  - 0.5181381106376648 * x
  - 0.42978039383888245;
'''

import numpy as np
import matplotlib.pyplot as plt

# -----------------------------
# Lane model coefficients
# y = ax^3 + bx^2 + cx + d
# -----------------------------

# ### 260309
# # Left lane
# la = 0.001749498536810279
# lb = -0.04562922194600105
# lc = 0.40893250703811646
# ld = -0.23083217442035675

# # Right lane
# ra = -0.002768676495179534
# rb = 0.06821615248918533
# rc = -0.5181381106376648
# rd = -0.42978039383888245

# # lane start/end (from ROS log)
# left_start = 6.842187404632568
# left_end   = 11.060937881469727

# right_start = 6.884375095367432
# right_end   = 10.681249618530273


### 260311
# Left lane
la = 0.0009602461359463632
lb = -0.026705686002969742
lc = 0.26545414328575134
ld = 0.2947007119655609

# Right lane
ra = -0.000925943604670465
rb = 0.022416001185774803
rc = -0.13478809595108032
rd = -1.3636255264282227

# lane start/end (from ROS log)
left_start = 7.1796875
left_end   = 11.735937118530273

right_start = 7.264062404632568
right_end   = 11.735937118530273

# -----------------------------
# polynomial function
# -----------------------------
def lane_poly(x, a, b, c, d):
    return a*x**3 + b*x**2 + c*x + d


# -----------------------------
# compute start/end y
# -----------------------------
y_ls = lane_poly(left_start, la, lb, lc, ld)
y_le = lane_poly(left_end,   la, lb, lc, ld)

y_rs = lane_poly(right_start, ra, rb, rc, rd)
y_re = lane_poly(right_end,   ra, rb, rc, rd)

print("Left lane")
print(f"x_start = {left_start:.6f}, y_start = {y_ls:.6f}")
print(f"x_end   = {left_end:.6f}, y_end   = {y_le:.6f}")
print()

print("Right lane")
print(f"x_start = {right_start:.6f}, y_start = {y_rs:.6f}")
print(f"x_end   = {right_end:.6f}, y_end   = {y_re:.6f}")


# -----------------------------
# visualize curve
# -----------------------------
x_left = np.linspace(left_start, left_end, 100)
y_left = lane_poly(x_left, la, lb, lc, ld)

x_right = np.linspace(right_start, right_end, 100)
y_right = lane_poly(x_right, ra, rb, rc, rd)

plt.figure(figsize=(6,6))
plt.plot(x_left, y_left, label="Left lane")
plt.plot(x_right, y_right, label="Right lane")

plt.scatter([left_start, left_end], [y_ls, y_le])
plt.scatter([right_start, right_end], [y_rs, y_re])

plt.xlabel("x (forward)")
plt.ylabel("y (lateral)")
plt.title("Lane model verification")
plt.legend()
plt.grid(True)
plt.axis("equal")

plt.show()