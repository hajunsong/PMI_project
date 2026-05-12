# Hybrid no-collision model validation

- MJCF: `models/pmi_hybrid_no_collision.xml`
- `nv=8`, `nu=4`, `ngeom=10`

## Checks

| Check | Result |
|------|--------|
| All STL mesh geoms contype/conaffinity=0 (XML scan) | PASS |
| All mesh geoms contype/conaffinity=0 (compiled model) | PASS |
| `data.ncon` at initial consistent pose | 0 (PASS) |
| Equality `jnt ≈ ratio*q_act` (forward, abs err) | max=0.000e+00 (PASS) |
| Joints jnt*, q*_act exist | PASS |
| Site `end_effector` | PASS |
| `max abs qfrc_constraint` after forward | 56.8091 |

### Note: qfrc_constraint
이 모델은 `equality` 관절 4개가 있어, 중력·관성 없이도 제약 반력 때문에 `qfrc_constraint` 벡터가 완전히 0이 아닐 수 있습니다. 접촉 부재 여부는 `ncon` 으로 판단합니다.

### DOF table

| joint | ids |
|-------|-----|
| jnt1 | jid=0 | qposadr=0 | dofadr=0 |
| jnt2 | jid=1 | qposadr=1 | dofadr=1 |
| jnt3 | jid=2 | qposadr=2 | dofadr=2 |
| jnt4 | jid=3 | qposadr=3 | dofadr=3 |
| q1_act | jid=4 | qposadr=4 | dofadr=4 |
| q2_act | jid=5 | qposadr=5 | dofadr=5 |
| q3_act | jid=6 | qposadr=6 | dofadr=6 |
| q4_act | jid=7 | qposadr=7 | dofadr=7 |
