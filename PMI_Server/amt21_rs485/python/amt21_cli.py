import serial
import time


def amt21_checksum_ok(raw: int) -> bool:
    k1 = (raw >> 15) & 0x01
    k0 = (raw >> 14) & 0x01

    calc_k1 = int(not (
        ((raw >> 13) & 1) ^
        ((raw >> 11) & 1) ^
        ((raw >> 9)  & 1) ^
        ((raw >> 7)  & 1) ^
        ((raw >> 5)  & 1) ^
        ((raw >> 3)  & 1) ^
        ((raw >> 1)  & 1)
    ))

    calc_k0 = int(not (
        ((raw >> 12) & 1) ^
        ((raw >> 10) & 1) ^
        ((raw >> 8)  & 1) ^
        ((raw >> 6)  & 1) ^
        ((raw >> 4)  & 1) ^
        ((raw >> 2)  & 1) ^
        ((raw >> 0)  & 1)
    ))

    return (k1 == calc_k1) and (k0 == calc_k0)


def parse_amt21(raw: int, resolution_bits: int = 14):
    checksum_ok = amt21_checksum_ok(raw)
    data14 = raw & 0x3FFF

    if resolution_bits == 14:
        position = data14
        max_count = 16384
    elif resolution_bits == 12:
        position = data14 >> 2
        max_count = 4096
    else:
        raise ValueError("resolution_bits must be 12 or 14")

    angle_deg = position / max_count * 360.0

    return {
        "raw": raw,
        "k1": (raw >> 15) & 1,
        "k0": (raw >> 14) & 1,
        "data14": data14,
        "checksum_ok": checksum_ok,
        "position": position,
        "angle_deg": angle_deg,
    }


def read_amt21_once(
    ser: serial.Serial,
    node_address: int = 0x74,
    resolution_bits: int = 14,
):
    # 이전에 남은 RX 데이터 제거
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # Read position command: 1 byte only
    ser.write(bytes([node_address]))
    ser.flush()

    # AMT21 response: low byte first, high byte second
    rx = ser.read(2)

    if len(rx) != 2:
        print(f"RX length error: expected 2 bytes, got {len(rx)} byte(s): {rx.hex(' ').upper()}")
        return None

    low = rx[0]
    high = rx[1]
    raw = (high << 8) | low

    result = parse_amt21(raw, resolution_bits)

    print("----- AMT21 Read Result -----")
    print(f"TX command    : 0x{node_address:02X}")
    print(f"RX bytes      : {rx.hex(' ').upper()}  low-high")
    print(f"Low byte      : 0x{low:02X}")
    print(f"High byte     : 0x{high:02X}")
    print(f"Raw assembled : 0x{result['raw']:04X}")
    print(f"K1            : {result['k1']}")
    print(f"K0            : {result['k0']}")
    print(f"Data14        : 0x{result['data14']:04X} / {result['data14']}")
    print(f"Checksum OK   : {result['checksum_ok']}")

    if not result["checksum_ok"]:
        print("Invalid checksum. Discard this sample and retry.")
        return None

    print(f"Resolution    : {resolution_bits}-bit")
    print(f"Position      : {result['position']}")
    print(f"Angle         : {result['angle_deg']:.3f} deg")

    return result


def main():
    port = "/dev/ttyU2D2"
    baudrate = 115200
    node_address = 0x74
    resolution_bits = 14

    with serial.Serial(
        port=port,
        baudrate=baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.05,
        write_timeout=0.05,
    ) as ser:

        while True:
            read_amt21_once(
                ser=ser,
                node_address=node_address,
                resolution_bits=resolution_bits,
            )
            time.sleep(0.02)


if __name__ == "__main__":
    main()