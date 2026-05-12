# IK Joint Tracking Summary

1. Does joint-space quintic tracking solve the path?
- best final EE error: 0.488379
2. Does longer duration improve tracking?
- best per duration: [0.48837942478470087, 0.48882060932085303, 0.4888803383310853, 0.4885623472437465]
3. Are torque limits sufficient?
- best per tau limit: [0.48837942478470087, 0.4885623472437465, 0.503063284591433]
4. Does computed torque improve tracking?
- best pd_bias final: 0.488602, best computed_torque final: 0.488379
5. Does task-space residual correction help or hurt?
- best final by lambda: l0=0.509130, l0.1=0.500728, l0.3=0.488379
6. Is this now ready to reintroduce q_act transmission?
- Reintroduce only if success criterion is met in arm-only baseline.
