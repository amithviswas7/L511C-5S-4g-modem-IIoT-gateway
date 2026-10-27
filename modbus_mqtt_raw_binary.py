
#!/usr/bin/env python3
"""Interactive Modbus RTU request builder and MQTT client."""

import os
import secrets
import threading
import time

import paho.mqtt.client as mqtt


# ============================================================
# MQTT CONFIGURATION — match these with your L511 firmware.
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

MAX_QUANTITY = {
    1: 2000,
    2: 2000,
    3: 125,
    4: 125,
    15: 1968,
    16: 123,
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
    """Read a decimal integer within the inclusive range."""

    while True:
        text = input(prompt).strip()

        try:
            # Explicit base 10: input such as 010 means decimal 10.
            value = int(text, 10)
        except ValueError:
            print("Invalid input. Enter a whole number in decimal.")
            continue

        if not minimum <= value <= maximum:
            print(
                f"Out of range. Enter a value from "
                f"{minimum} to {maximum}."
            )
            continue

        return value


def read_choice(prompt, choices):
    """Read an integer that must be one of the allowed choices."""

    while True:
        value = read_integer(prompt, min(choices), max(choices))

        if value in choices:
            return value

        print("Invalid choice. Allowed values:", ", ".join(
            str(x) for x in choices
        ))


# ============================================================
# MODBUS RTU CRC
# ============================================================

def modbus_crc16(data):
    """
    Modbus CRC-16:
      Initial value: 0xFFFF
      Reflected polynomial: 0xA001
      Wire order: CRC low byte, then high byte
    """
\



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
    """Append CRC low byte followed by CRC high byte."""

    crc = modbus_crc16(payload)
    return payload + bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def format_frame(frame):
    return " ".join(f"{byte:02X}" for byte in frame)


# ============================================================
# BUILD A REQUEST FROM INDIVIDUAL INPUTS
# ============================================================

def build_request():
    print("\n" + "=" * 60)
    print("        MODBUS RTU REQUEST CONFIGURATION")
    print("=" * 60)

    slave = read_integer(
        "Enter slave ID (decimal, 1-247): ", 1, 247
    )

    print("\nFunction codes:")
    for code, name in FUNCTION_NAMES.items():
        print(f"  {code:2d} - {name}")

    function = read_choice(
        "\nEnter function code (decimal): ",
        set(FUNCTION_NAMES),
    )

    print(f"Selected: {FUNCTION_NAMES[function]}")

    # Function 0x05 and 0x06 each have a single address and value.
    if function == 5:
        address = read_integer(
            "Enter coil address (decimal, 0-65535): ",
            0, 65535,
        )

        state = read_choice(
            "Enter coil state (0=OFF, 1=ON): ", {0, 1}
        )

        # Modbus single-coil encoding: OFF=0x0000, ON=0xFF00.
        value = 0xFF00 if state == 1 else 0x0000

        payload = bytes([
            slave,
            function,
            (address >> 8) & 0xFF,
            address & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF,
        ])

        return append_crc(payload), slave, function

    if function == 6:
        address = read_integer(
            "Enter register address (decimal, 0-65535): ",
            0, 65535,
        )

        value = read_integer(
            "Enter register value (decimal, 0-65535): ",
            0, 65535,
        )

        payload = bytes([
            slave,
            function,
            (address >> 8) & 0xFF,
            address & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF,
        ])

        return append_crc(payload), slave, function

    # Read functions and multiple-write functions.
    address = read_integer(
        "Enter starting address (decimal, 0-65535): ",
        0, 65535,
    )

    quantity = read_integer(
        f"Enter quantity (decimal, 1-{MAX_QUANTITY[function]}): ",
        1, MAX_QUANTITY[function],
    )

    if address + quantity > 65536:
        raise ValueError(
            "Address range exceeds the Modbus 16-bit address space."
        )

    payload = bytearray([
        slave,
        function,
        (address >> 8) & 0xFF,
        address & 0xFF,
        (quantity >> 8) & 0xFF,
        quantity & 0xFF,
    ])

    if function in READ_FUNCTIONS:
        # Read request is 6 data bytes + 2 CRC bytes.
        return append_crc(bytes(payload)), slave, function

    if function == 15:
        # Each coil is entered as 0 or 1.
        bit_values = []

        print(
            f"\nEnter {quantity} coil states individually."
            "\n0 = OFF, 1 = ON."
        )

        for i in range(quantity):
            bit_values.append(
                read_choice(f"Coil {i + 1}/{quantity}: ", {0, 1})
            )

        # Modbus packs coil 0 into bit 0 of the first data byte.
        packed = bytearray((quantity + 7) // 8)

        for i, bit in enumerate(bit_values):
            if bit:
                packed[i // 8] |= 1 << (i % 8)

        payload.append(len(packed))
        payload.extend(packed)

    elif function == 16:
        # Each register is a 16-bit unsigned value.
        print(f"\nEnter {quantity} register values in decimal.")

        register_values = []

        for i in range(quantity):
            value = read_integer(
                f"Register {i + 1}/{quantity} "
                "(0-65535): ",
                0, 65535,
            )
            register_values.append(value)

        payload.append(quantity * 2)

        for value in register_values:
            # Register value is sent high byte first.
            payload.append((value >> 8) & 0xFF)
            payload.append(value & 0xFF)

    return append_crc(bytes(payload)), slave, function


# ============================================================
# RESPONSE CHECKING
# ============================================================

def validate_response(frame, slave, function, request):
    if len(frame) < 5:
        raise ValueError("Response is too short.")

    received_crc = frame[-2] | (frame[-1] << 8)
    calculated_crc = modbus_crc16(frame[:-2])

    if received_crc != calculated_crc:
        raise ValueError(
            f"Response CRC mismatch: received 0x{received_crc:04X}, "
            f"expected 0x{calculated_crc:04X}."
        )

    if frame[0] != slave:
        raise ValueError(
            f"Slave mismatch: expected {slave}, received {frame[0]}."
        )

    response_function = frame[1]

    if response_function == (function | 0x80):
        if len(frame) != 5:
            raise ValueError("Malformed Modbus exception response.")

        descriptions = {
            1: "Illegal Function",
            2: "Illegal Data Address",
            3: "Illegal Data Value",
            4: "Slave Device Failure",
            5: "Acknowledge",
            6: "Slave Device Busy",
            10: "Gateway Path Unavailable",
            11: "Gateway Target Device Failed to Respond",
        }

        code = frame[2]
        return (
            f"SLAVE EXCEPTION 0x{code:02X}: "
            f"{descriptions.get(code, 'Unknown exception')}"
        )

    if response_function != function:
        raise ValueError(
            f"Function mismatch: expected 0x{function:02X}, "
            f"received 0x{response_function:02X}."
        )

    if function in READ_FUNCTIONS:
        expected_bytes = (
            (request[4] << 8 | request[5]) + 7
        ) // 8 if function in (1, 2) else (
            (request[4] << 8 | request[5]) * 2
        )

        if len(frame) != expected_bytes + 5:
            raise ValueError("Read response length is inconsistent.")

        if frame[2] != expected_bytes:
            raise ValueError("Read response byte count is incorrect.")

    else:
        if len(frame) != 8:
            raise ValueError("Write response must be exactly 8 bytes.")

        if frame[2:6] != request[2:6]:
            raise ValueError(
                "Write response address/value/quantity does not "
                "match the request."
            )

    return "Response passed CRC and protocol checks."


# ============================================================
# MQTT CLIENT
# ============================================================

class ModbusMqttClient:
    def __init__(self):
        self.ready = threading.Event()
        self.pending_lock = threading.Lock()
        self.pending = None

        self.client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"modbus-pc-{secrets.token_hex(4)}",
            protocol=mqtt.MQTTv311,
        )

        if MQTT_USERNAME:
            self.client.username_pw_set(
                MQTT_USERNAME, MQTT_PASSWORD
            )

        self.client.on_connect = self.on_connect
        self.client.on_subscribe = self.on_subscribe
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message

    def on_connect(self, client, userdata, flags, reason_code,
                   properties):
        if reason_code.is_failure:
            print(f"MQTT connection failed: {reason_code}")
            return

        print(f"\nConnected to broker: {MQTT_HOST}:{MQTT_PORT}")
        client.subscribe(RESPONSE_TOPIC, qos=1)
        print(f"Subscribing to response topic: {RESPONSE_TOPIC}")

    def on_subscribe(self, client, userdata, mid, reason_codes,
                     properties):
        if reason_codes and reason_codes[0].is_failure:
            print("MQTT response subscription failed.")
            return

        self.ready.set()
        print("MQTT response subscription active.")

    def on_disconnect(self, client, userdata, disconnect_flags,
                      reason_code, properties):
        self.ready.clear()

        with self.pending_lock:
            if self.pending is not None:
                self.pending["error"] = "MQTT disconnected."
                self.pending["event"].set()

        print(f"\nMQTT disconnected: {reason_code}")

    def on_message(self, client, userdata, message):
        # The MQTT payload is the raw binary Modbus RTU response.
        # No JSON parsing or hexadecimal-text decoding is needed.
        frame = bytes(message.payload)

        if not frame:
            print("\\nIgnoring empty MQTT response.")
            return

        with self.pending_lock:
            pending = self.pending

            if pending is None:
                print(
                    f"Unsolicited MQTT message on {message.topic}; "
                    "no request is pending."
                )
                return

            try:
                result = validate_response(
                    frame,
                    pending["slave"],
                    pending["function"],
                    pending["request"],
                )
                pending["response"] = frame
                pending["result"] = result
            except ValueError as exc:
                pending["error"] = str(exc)

            pending["event"].set()

    def connect(self):
        self.client.connect(
            MQTT_HOST, MQTT_PORT, keepalive=60
        )
        self.client.loop_start()

        if not self.ready.wait(CONNECT_TIMEOUT):
            self.client.loop_stop()
            raise RuntimeError(
                "MQTT connection or response subscription timed out."
            )

    def close(self):
        self.ready.clear()
        self.client.disconnect()
        self.client.loop_stop()

    def transact(self, frame, slave, function):
        if not self.ready.is_set():
            print("MQTT is not connected; request not sent.")
            return

        event = threading.Event()

        pending = {
            "slave": slave,
            "function": function,
            "request": frame,
            "event": event,
            "response": None,
            "error": None,
            "result": None,
        }

        with self.pending_lock:
            if self.pending is not None:
                print("Another transaction is already pending.")
                return

            self.pending = pending

        try:
            # Publish the actual binary RTU frame as the MQTT payload.
            # Example payload bytes: 01 03 00 00 00 02 C4 0B
            info = self.client.publish(
                COMMAND_TOPIC,
                payload=frame,
                qos=1,
                retain=False,
            )

            if info.rc != mqtt.MQTT_ERR_SUCCESS:
                print(f"MQTT publish failed: {info.rc}")
                return

            try:
                info.wait_for_publish(timeout=5)
            except (RuntimeError, ValueError) as exc:
                print(f"MQTT publish error: {exc}")
                return

            if not info.is_published():
                print("MQTT publish acknowledgement timed out.")
                print(
                    "Delivery is uncertain. Do not automatically "
                    "retry a write request."
                )
                return

            print("\n" + "=" * 60)
            print("REQUEST SENT")
            print("=" * 60)
            print(f"Slave ID:     {slave} (0x{slave:02X})")
            print(f"Function:     {function} (0x{function:02X})")
            print(f"MQTT topic:   {COMMAND_TOPIC}")
            print(f"RTU frame:    {format_frame(frame)}")
            print(f"Frame length: {len(frame)} bytes")
            print("\nWaiting for response...")

            if not event.wait(RESPONSE_TIMEOUT):
                print(
                    f"\nTIMEOUT: no matching response within "
                    f"{RESPONSE_TIMEOUT:.1f} seconds."
                )
                return

            if pending["error"]:
                print(f"\nRESPONSE ERROR: {pending['error']}")
                return

            print("\nRESPONSE RECEIVED")
            print(f"RTU frame: {format_frame(pending['response'])}")
            print(pending["result"])

        finally:
            with self.pending_lock:
                if self.pending is pending:
                    self.pending = None


# ============================================================
# MAIN
# ============================================================

def main():
    print("=" * 60)
    print("  INTERACTIVE MODBUS RTU OVER MQTT")
    print("=" * 60)
    print("Enter parameters in decimal; frame is sent in hexadecimal.")

    app = ModbusMqttClient()

    try:
        app.connect()
    except Exception as exc:
        print(f"Could not connect to MQTT: {exc}")
        return

    try:
        while True:
            try:
                frame, slave, function = build_request()
            except (ValueError, KeyboardInterrupt, EOFError) as exc:
                if isinstance(exc, (KeyboardInterrupt, EOFError)):
                    print("\nExiting.")
                    break

                print(f"\nInvalid request: {exc}")
                continue

            print("\n" + "-" * 60)
            print("GENERATED MODBUS RTU REQUEST")
            print("-" * 60)
            print(f"Slave ID:     {slave} (decimal)")
            print(f"Function:     {function} - {FUNCTION_NAMES[function]}")
            print(f"RTU frame:    {format_frame(frame)}")
            print(f"Frame length: {len(frame)} bytes")
            print("CRC:          calculated and appended automatically")

            confirm = input(
                "\nPublish this request? (y/n): "
            ).strip().lower()

            if confirm == "y":
                app.transact(frame, slave, function)
            else:
                print("Request cancelled; nothing published.")

            again = input(
                "\nBuild another request? (y/n): "
            ).strip().lower()

            if again != "y":
                break

    finally:
        app.close()
        print("MQTT client stopped.")


if __name__ == "__main__":
    main()
