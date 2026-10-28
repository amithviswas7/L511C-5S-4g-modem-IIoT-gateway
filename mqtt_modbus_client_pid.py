#!/usr/bin/env python3
"""Interactive Modbus RTU request builder and MQTT client.

MQTT payload:
    Bytes 0-3: Packet ID (uint32, big-endian)
    Bytes 4+:  Modbus RTU frame, including CRC

No version/status/length envelope fields.
"""

import os
import secrets
import struct
import threading
import time

import paho.mqtt.client as mqtt


# ============================================================
# MQTT CONFIGURATION
# ============================================================

MQTT_HOST = os.getenv("MQTT_HOST", "broker.emqx.io")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))
MQTT_USERNAME = os.getenv("MQTT_USERNAME", "1234")
MQTT_PASSWORD = os.getenv("MQTT_PASSWORD", "abcd")

COMMAND_TOPIC = os.getenv(
    "MQTT_COMMAND_TOPIC", "l511/device01/cmd"
)
RESPONSE_TOPIC = os.getenv(
    "MQTT_RESPONSE_TOPIC", "l511/device1/data"
)

RESPONSE_TIMEOUT = 5.0
CONNECT_TIMEOUT = 10.0
PUBLISH_TIMEOUT = 5.0
MAX_FRAME = 256

MAX_QUANTITY = {
    1: 2000, 2: 2000, 3: 125, 4: 125,
    15: 1968, 16: 123,
}

FUNCTION_NAMES = {
    1: "Read Coils",
    2: "Read Discrete Inputs",
    3: "Read Holding Registers",
    4: "Read Input Registers",
    5: "Write Single Coil",
    6: "Write Single Register",
    15: "Write Multiple Coils",
    16: "Write Multiple Registers",
}

READ_FUNCTIONS = {1, 2, 3, 4}


# ============================================================
# INPUT VALIDATION
# ============================================================

def read_integer(prompt, minimum, maximum):
    while True:
        try:
            value = int(input(prompt).strip(), 10)
        except ValueError:
            print("Invalid input. Enter a decimal whole number.")
            continue

        if not minimum <= value <= maximum:
            print(f"Enter a value from {minimum} to {maximum}.")
            continue

        return value


def read_choice(prompt, choices):
    while True:
        value = read_integer(prompt, min(choices), max(choices))
        if value in choices:
            return value
        print("Allowed values:", ", ".join(map(str, sorted(choices))))


def read_packet_id(pending):
    """Keep asking until the user enters an unused packet ID."""
    while True:
        packet_id = read_integer(
            "Enter packet ID (1-4294967295): ",
            1,
            0xFFFFFFFF,
        )

        with pending["lock"]:
            if packet_id in pending["requests"]:
                print(
                    f"INVALID: Packet ID {packet_id} already has "
                    "an outstanding request. Enter another ID."
                )
                continue

        return packet_id


# ============================================================
# MODBUS RTU CRC
# ============================================================

def modbus_crc16(data):
    crc = 0xFFFF

    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1

    return crc & 0xFFFF


def append_crc(payload):
    crc = modbus_crc16(payload)
    return payload + bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def check_crc(frame):
    if len(frame) < 4:
        raise ValueError("RTU frame is too short.")

    received = frame[-2] | (frame[-1] << 8)
    expected = modbus_crc16(frame[:-2])

    if received != expected:
        raise ValueError(
            f"CRC mismatch: received 0x{received:04X}, "
            f"expected 0x{expected:04X}."
        )


def format_frame(frame):
    return " ".join(f"{byte:02X}" for byte in frame)


# ============================================================
# MODBUS RTU REQUEST BUILDER
# ============================================================

def build_request():
    print("\n" + "=" * 60)
    print("MODBUS RTU REQUEST BUILDER")
    print("=" * 60)

    slave = read_integer("Slave ID (1-247): ", 1, 247)

    for code, name in FUNCTION_NAMES.items():
        print(f"  {code:2d} - {name}")

    function = read_choice(
        "Function code: ",
        set(FUNCTION_NAMES),
    )

    if function in (5, 6):
        address = read_integer("Address (0-65535): ", 0, 65535)

        if function == 5:
            state = read_choice("Coil state (0=OFF, 1=ON): ", {0, 1})
            value = 0xFF00 if state else 0
        else:
            value = read_integer("Register value (0-65535): ", 0, 65535)

        payload = bytes((
            slave,
            function,
            address >> 8,
            address & 0xFF,
            value >> 8,
            value & 0xFF,
        ))

        return append_crc(payload), slave, function

    address = read_integer(
        "Starting address (0-65535): ", 0, 65535
    )
    quantity = read_integer(
        f"Quantity (1-{MAX_QUANTITY[function]}): ",
        1,
        MAX_QUANTITY[function],
    )

    if address + quantity > 65536:
        raise ValueError("Address range exceeds Modbus address space.")

    payload = bytearray((
        slave,
        function,
        address >> 8,
        address & 0xFF,
        quantity >> 8,
        quantity & 0xFF,
    ))

    if function in READ_FUNCTIONS:
        return append_crc(bytes(payload)), slave, function

    if function == 15:
        packed = bytearray((quantity + 7) // 8)

        for i in range(quantity):
            bit = read_choice(
                f"Coil {i + 1}/{quantity} (0=OFF, 1=ON): ",
                {0, 1},
            )
            if bit:
                packed[i // 8] |= 1 << (i % 8)

        payload.append(len(packed))
        payload.extend(packed)

    elif function == 16:
        payload.append(quantity * 2)

        for i in range(quantity):
            value = read_integer(
                f"Register {i + 1}/{quantity} (0-65535): ",
                0,
                65535,
            )
            payload.extend(struct.pack(">H", value))

    return append_crc(bytes(payload)), slave, function


# ============================================================
# RESPONSE VALIDATION
# ============================================================

def validate_response(frame, slave, function, request):
    check_crc(frame)

    if len(frame) < 5:
        raise ValueError("Response is too short.")

    if frame[0] != slave:
        raise ValueError(
            f"Slave mismatch: expected {slave}, received {frame[0]}."
        )

    response_function = frame[1]

    if response_function == (function | 0x80):
        if len(frame) != 5:
            raise ValueError("Malformed exception response.")

        exceptions = {
            1: "Illegal Function",
            2: "Illegal Data Address",
            3: "Illegal Data Value",
            4: "Slave Device Failure",
            5: "Acknowledge",
            6: "Slave Device Busy",
            10: "Gateway Path Unavailable",
            11: "Gateway Target Failed to Respond",
        }

        code = frame[2]
        return (
            f"Modbus exception 0x{code:02X}: "
            f"{exceptions.get(code, 'Unknown exception')}"
        )

    if response_function != function:
        raise ValueError(
            f"Function mismatch: expected {function}, "
            f"received {response_function}."
        )

    if function in READ_FUNCTIONS:
        quantity = (request[4] << 8) | request[5]
        byte_count = (
            (quantity + 7) // 8
            if function in (1, 2)
            else quantity * 2
        )

        if len(frame) != byte_count + 5 or frame[2] != byte_count:
            raise ValueError("Read response length/byte count mismatch.")

        data = frame[3:-2]

        if function in (1, 2):
            values = [
                (data[i // 8] >> (i % 8)) & 1
                for i in range(quantity)
            ]
            return f"Bit values: {values}"

        values = [
            (data[i] << 8) | data[i + 1]
            for i in range(0, len(data), 2)
        ]
        return f"Register values: {values}"

    if len(frame) != 8 or frame[2:6] != request[2:6]:
        raise ValueError("Write acknowledgement does not match request.")

    return "Valid Modbus write acknowledgement."


# ============================================================
# MQTT CLIENT
# ============================================================

class ModbusMqttClient:

    def __init__(self):
        self.ready = threading.Event()
        self.closing = False

        # Packet IDs are reserved until a matching response arrives.
        self.pending = {
            "lock": threading.Lock(),
            "requests": {},
        }

        self.client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"modbus-pc-{secrets.token_hex(5)}",
            protocol=mqtt.MQTTv311,
        )

        if MQTT_USERNAME:
            self.client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)

        self.client.on_connect = self.on_connect
        self.client.on_subscribe = self.on_subscribe
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message

    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            print(f"[MQTT] Connection failed: {reason_code}")
            return

        client.subscribe(RESPONSE_TOPIC, qos=1)
        print(f"[MQTT] Connected: {MQTT_HOST}:{MQTT_PORT}")
        print(f"[MQTT] Subscribing to {RESPONSE_TOPIC}")

    def on_subscribe(self, client, userdata, mid, reason_codes, properties):
        if reason_codes and any(code.is_failure for code in reason_codes):
            print(f"[MQTT] Subscription rejected: {reason_codes}")
            return

        self.ready.set()
        print("[MQTT] Response subscription active.")

    def on_disconnect(
        self, client, userdata, disconnect_flags, reason_code, properties
    ):
        self.ready.clear()
        if not self.closing:
            print(f"\n[MQTT] Disconnected: {reason_code}")

    def on_message(self, client, userdata, message):
        # Response format: 4-byte packet ID + raw RTU response.
        payload = bytes(message.payload)

        if len(payload) < 4:
            print("[RX] Invalid payload: missing 4-byte packet ID.")
            return

        packet_id = struct.unpack(">I", payload[:4])[0]
        frame = payload[4:]

        with self.pending["lock"]:
            item = self.pending["requests"].pop(packet_id, None)

        if item is None:
            print(f"\n[RX] Unknown or late packet ID: {packet_id}")
            return

        print(f"\n[RX] Response received for packet ID {packet_id}")
        print(f"[RX] RTU frame: {format_frame(frame)}")

        try:
            result = validate_response(
                frame,
                item["slave"],
                item["function"],
                item["frame"],
            )
            print(f"[RX] {result}")
        except ValueError as exc:
            print(f"[RX] Response validation failed: {exc}")

        print(f"[ID] Packet ID {packet_id} is available again.")

    def connect(self):
        self.client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
        self.client.loop_start()

        if not self.ready.wait(CONNECT_TIMEOUT):
            raise TimeoutError("MQTT connection/subscription timed out.")

    def show_pending(self):
        with self.pending["lock"]:
            items = list(self.pending["requests"].items())

        if not items:
            print("No outstanding requests.")
            return

        print("\nOutstanding packet IDs:")
        for packet_id, item in items:
            print(
                f"  ID={packet_id} | "
                f"slave={item['slave']} | "
                f"function={item['function']}"
            )

    def submit(self, packet_id, frame, slave, function):
        if not self.ready.is_set():
            raise RuntimeError("MQTT response subscription is not active.")

        # Reserve the ID before publishing to avoid a fast-response race.
        item = {
            "frame": frame,
            "slave": slave,
            "function": function,
        }

        with self.pending["lock"]:
            if packet_id in self.pending["requests"]:
                raise ValueError(
                    f"Packet ID {packet_id} is already outstanding."
                )
            self.pending["requests"][packet_id] = item

        # ONLY packet ID + RTU frame. No version, status, or length.
        payload = struct.pack(">I", packet_id) + frame

        try:
            info = self.client.publish(
                COMMAND_TOPIC,
                payload=payload,
                qos=1,
                retain=False,
            )

            if info.rc != mqtt.MQTT_ERR_SUCCESS:
                raise RuntimeError(f"MQTT publish failed: rc={info.rc}")

            info.wait_for_publish(timeout=PUBLISH_TIMEOUT)

            if not info.is_published():
                raise TimeoutError(
                    "Publish acknowledgement timed out. Delivery is "
                    "uncertain; check before retrying a write."
                )

        except Exception:
            # Keep the ID reserved if its response already arrived.
            with self.pending["lock"]:
                if self.pending["requests"].get(packet_id) is item:
                    del self.pending["requests"][packet_id]
            raise

        print(f"\n[TX] Packet ID: {packet_id}")
        print(f"[TX] RTU frame: {format_frame(frame)}")
        print(f"[TX] Payload:   {format_frame(payload)}")
        print("[TX] Published. Packet ID is reserved until its response arrives.")

    def close(self):
        self.closing = True
        self.client.disconnect()
        self.client.loop_stop()


# ============================================================
# MAIN
# ============================================================

def main():
    print("=" * 60)
    print("MODBUS RTU OVER MQTT — L511")
    print("=" * 60)
    print("MQTT payload = 4-byte packet ID + RTU frame")
    print("No version/status/length fields.")

    app = ModbusMqttClient()

    try:
        app.connect()
    except Exception as exc:
        print(f"[FATAL] MQTT setup failed: {exc}")
        app.close()
        return

    try:
        while True:
            print("\n1 - Build and send request")
            print("2 - Show outstanding packet IDs")
            print("3 - Quit")

            choice = input("Select: ").strip()

            if choice == "1":
                # Duplicate IDs are rejected before accepting a frame.
                packet_id = read_packet_id(app.pending)

                try:
                    frame, slave, function = build_request()
                    if len(frame) > MAX_FRAME:
                        raise ValueError("RTU frame is too long.")
                    check_crc(frame)
                except (ValueError, KeyboardInterrupt, EOFError) as exc:
                    if isinstance(exc, (KeyboardInterrupt, EOFError)):
                        break
                    print(f"[REQUEST ERROR] {exc}")
                    continue

                print(f"\nPacket ID: {packet_id}")
                print(f"Function: {FUNCTION_NAMES[function]}")
                print(f"RTU frame: {format_frame(frame)}")

                if input("Publish request? (y/n): ").strip().lower() != "y":
                    print("Cancelled.")
                    continue

                try:
                    app.submit(packet_id, frame, slave, function)
                except (ValueError, RuntimeError, TimeoutError) as exc:
                    print(f"[REQUEST ERROR] {exc}")

            elif choice == "2":
                app.show_pending()

            elif choice == "3":
                break

            else:
                print("Choose 1, 2, or 3.")

    except (KeyboardInterrupt, EOFError):
        print("\nExiting.")

    finally:
        app.close()
        print("MQTT client stopped.")


if __name__ == "__main__":
    main()
