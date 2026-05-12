# Trajectory and Metric Audit

- same trajectory generator references: True
- duration=1 waypoint times exactly 0,0.5,1: True
- duration scaling uses 0,T/2,T: True
- final_pos_err from last sample: True
- peak and duration baseline can be same run: True
- no post-run forced state before final metric: True

## Key Samples
- final sample time: 1.000000
- final desired xyz: [-0.25, -0.2, -0.1]
- final actual xyz: [0.2086674044427641, -0.14833943177033115, -0.12378324946549293]
- final position error: 0.462180
- peak sample time: 0.996000
- peak desired xyz: [-0.2499987353108477, -0.20000075881349139, -0.10000025293783046]
- peak actual xyz: [0.20905130649079573, -0.14828570388331674, -0.12369859796707258]
- peak position error: 0.462561

- final/peak inconsistency detected: False
