# qacc response audit

단일 관절에 `qfrc_applied` 토크를 걸고 `mj_forward` 후 해당 dof 의 `qacc` 부호가 토크 부호와 일치하는지 확인합니다.

**주의:** 메시 접촉이 있는 경우 접촉·제약이 가속도를 지배하면 부호 검증이 무의미할 수 있습니다. 이 경우 `pmi_arm_only_no_collision.xml` 결과를 참고합니다.

- CSV: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/arm_only_physics/qacc_response_audit.csv`
