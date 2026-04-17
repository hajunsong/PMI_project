#ifndef AMT21_MODEL_HPP
#define AMT21_MODEL_HPP

// 타깃 부품: Same Sky AMT212H-V
// - AMT21 시리즈, 해상도 키 H = 14-bit multi-turn, 가변 RS485 보드레이트 (AMT Viewpoint / 주문 옵션)
// - 키트 기본 보드레이트는 데이터시트 기준 115200 bps가 일반적
// - RS485 노드 주소는 4의 배수(기본 0x54)

namespace amt21::model {

inline constexpr char kPartNumber[] = "AMT212H-V";
inline constexpr int kPositionBits = 14;
inline constexpr bool kMultiTurn = true;

} // namespace amt21::model

#endif
