import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt

# Define fuzzy set
largest_dis = np.floor(np.sqrt(1700**2 + 1000**2))
leftSensor1Set = ctrl.Antecedent(np.arange(0, largest_dis, 1), 'Left Sensor 1')
leftSensor2Set = ctrl.Antecedent(np.arange(0, largest_dis, 1), 'Left Sensor 2')
# leftSensor1Set = ctrl.Antecedent(np.arange(0, 100, 1), 'Left Sensor 1')
# leftSensor2Set = ctrl.Antecedent(np.arange(0, 100, 1), 'Left Sensor 2')
output_speedDiff = ctrl.Consequent(np.arange(-3, 4, 0.01), 'Speed Diff')
# turnIntent = ctrl.Consequent(np.arange(-1, 1.01, 0.01), 'turn_intent')

# Anticident
leftSensor1Set['near'] = fuzz.trapmf(leftSensor1Set.universe, [0, 0, 30, 40])
# leftSensor1Set['good'] = fuzz.trimf(leftSensor1Set.universe, [40, 60, 80])
leftSensor1Set['good'] = fuzz.trapmf(leftSensor1Set.universe, [30, 40, 60, 70])
leftSensor1Set['far'] = fuzz.trapmf(leftSensor1Set.universe, [60, 70, largest_dis, largest_dis])

leftSensor2Set['near'] = fuzz.trapmf(leftSensor2Set.universe, [0, 0, 30, 40])
# leftSensor2Set['good'] = fuzz.trimf(leftSensor2Set.universe, [40, 60, 80])
leftSensor2Set['good'] = fuzz.trapmf(leftSensor2Set.universe, [30, 40, 60, 70])
leftSensor2Set['far'] = fuzz.trapmf(leftSensor2Set.universe, [60, 70, largest_dis, largest_dis])

# Consequence
# output_speedDiff['left']  = fuzz.trimf(output_speedDiff.universe, [-2, -1.5, 0])
# output_speedDiff['straight']  = fuzz.trimf(output_speedDiff.universe, [-0.5, 0, 0.5])
# output_speedDiff['right'] = fuzz.trimf(output_speedDiff.universe, [0, 1.5, 2])

output_speedDiff['left'] = fuzz.trapmf(output_speedDiff.universe, [-5, -5, -2, -1])
output_speedDiff['softleft'] = fuzz.trapmf(output_speedDiff.universe, [-2, -1, -0.2, -0.1])
output_speedDiff['straight'] = fuzz.trapmf(output_speedDiff.universe, [-0.2, -0.1, 0.1, 0.2])
output_speedDiff['softright'] = fuzz.trapmf(output_speedDiff.universe, [0.1, 0.2, 1, 2])
output_speedDiff['right'] = fuzz.trapmf(output_speedDiff.universe, [1, 2, 5, 5])

# turnIntent['left']  = fuzz.trapmf(turnIntent.universe, [-1.0, -1.0, -0.7, -0.4])
# turnIntent['softleft']    = fuzz.trimf(turnIntent.universe, [-0.6, -0.3, 0])
# turnIntent['straight']     = fuzz.trimf(turnIntent.universe, [-0.2, 0, 0.2])
# turnIntent['softright']   = fuzz.trimf(turnIntent.universe, [0, 0.3, 0.6])
# turnIntent['right'] = fuzz.trapmf(turnIntent.universe, [0.4, 0.7, 1.0, 1.0])

# rules = [
#     ctrl.Rule(leftSensor1Set['near'] & leftSensor2Set['far'], output_speedDiff['right']),
#     ctrl.Rule(leftSensor1Set['far'] & leftSensor2Set['near'], output_speedDiff['left']),
#     # ctrl.Rule(leftSensor1Set['good'] & leftSensor2Set['near'], output_speedDiff['left']),
#     ctrl.Rule(leftSensor1Set['far'] & leftSensor2Set['near'], output_speedDiff['left']),
#     ctrl.Rule(leftSensor1Set['near'] & leftSensor2Set['near'], output_speedDiff['right']),
#     ctrl.Rule(leftSensor1Set['far'] & leftSensor2Set['far'], output_speedDiff['zero']),
#     ctrl.Rule(leftSensor1Set['good'] & leftSensor2Set['good'], output_speedDiff['zero']),
#         ]

rules = [
ctrl.Rule(leftSensor1Set['near'] & leftSensor2Set['near'], output_speedDiff['right']),
ctrl.Rule(leftSensor1Set['near'] & leftSensor2Set['good'], output_speedDiff['softleft']),
ctrl.Rule(leftSensor1Set['near'] & leftSensor2Set['far'], output_speedDiff['left']),

ctrl.Rule(leftSensor1Set['good'] & leftSensor2Set['near'], output_speedDiff['softright']),
ctrl.Rule(leftSensor1Set['good'] & leftSensor2Set['good'], output_speedDiff['straight']),
ctrl.Rule(leftSensor1Set['good'] & leftSensor2Set['far'], output_speedDiff['softleft']),

ctrl.Rule(leftSensor1Set['far'] & leftSensor2Set['near'], output_speedDiff['right']),
ctrl.Rule(leftSensor1Set['far'] & leftSensor2Set['good'], output_speedDiff['softright']),
ctrl.Rule(leftSensor1Set['far'] & leftSensor2Set['far'], output_speedDiff['softleft']),
    ]

# rules = [
# ctrl.Rule(leftSensor1Set['near'] & leftSensor2Set['near'], turnIntent['right']),
# ctrl.Rule(leftSensor1Set['near'] & leftSensor2Set['good'], turnIntent['softleft']),
# ctrl.Rule(leftSensor1Set['near'] & leftSensor2Set['far'], turnIntent['left']),

# ctrl.Rule(leftSensor1Set['good'] & leftSensor2Set['near'], turnIntent['softright']),
# ctrl.Rule(leftSensor1Set['good'] & leftSensor2Set['good'], turnIntent['straight']),
# ctrl.Rule(leftSensor1Set['good'] & leftSensor2Set['far'], turnIntent['softleft']),

# ctrl.Rule(leftSensor1Set['far'] & leftSensor2Set['near'], turnIntent['right']),
# ctrl.Rule(leftSensor1Set['far'] & leftSensor2Set['good'], turnIntent['softright']),
# ctrl.Rule(leftSensor1Set['far'] & leftSensor2Set['far'], turnIntent['straight']),
#     ]

fuzzyControl = ctrl.ControlSystem(rules)
fuzzyControl_sim = ctrl.ControlSystemSimulation(fuzzyControl)

def fuzzy_wall_following(fuzzyControl_sim, leftSensor1Output, leftSensor2Output, base_speed=3.0):
    fuzzyControl_sim.input['Left Sensor 1'] = leftSensor1Output
    fuzzyControl_sim.input['Left Sensor 2'] = leftSensor2Output
    fuzzyControl_sim.compute()

    speedDiff = fuzzyControl_sim.output['Speed Diff']
    # print(speedDiff)
    speedLeft = base_speed + speedDiff
    speedRight = base_speed - speedDiff

    return speedLeft, speedRight

leftSensor1Set.view()
leftSensor2Set.view()
output_speedDiff.view()


fuzzyControl_sim.input['Left Sensor 1'] = 40
fuzzyControl_sim.input['Left Sensor 2'] = 80
fuzzyControl_sim.compute()

print(fuzzyControl_sim.output)

output_speedDiff.view(sim=fuzzyControl_sim)
# print(fuzzyControl_sim.output['speed_diff'])

x_range = np.arange(0, 200, 5)
y_range = np.arange(0, 200, 5)
x, y = np.meshgrid(x_range, y_range)
z = np.zeros_like(x)

for i in range(len(x_range)):
    for j in range(len(y_range)):
        fuzzyControl_sim.input['Left Sensor 1'] = x[i, j]
        fuzzyControl_sim.input['Left Sensor 2'] = y[i, j]
        fuzzyControl_sim.compute()
        z[i, j] = fuzzyControl_sim.output['Speed Diff']

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_surface(x, y, z, cmap='viridis')

ax.set_xlabel('Left Sensor 1')
ax.set_ylabel('Left Sensor 2')
ax.set_zlabel('Speed Diff')
ax.set_title('Fuzzy Control Surface')

# plt.tight_layout()
plt.show()
