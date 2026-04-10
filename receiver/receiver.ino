/**
 * ============================================================================
 *  ESP32 P2P RECEIVER — Robot Motor/Servo Controller
 * ============================================================================
 *
 *  Receives ControlPacket from transmitter via ESP-NOW and executes
 *  motor/servo actions. Includes a safety watchdog that auto-stops
 *  if no command is received within a timeout window.
 *
 *  OUTPUT PINS (customize to your motor driver):
 *    GPIO 16  →  Motor A IN1  (Left motor forward)
 *    GPIO 17  →  Motor A IN2  (Left motor backward)
 *    GPIO 18  →  Motor B IN1  (Right motor forward)
 *    GPIO 19  →  Motor B IN2  (Right motor backward)
 *    GPIO 21  →  Gripper Servo PWM
 *    GPIO 22  →  Lift Servo / Motor PWM
 *
 *  STATUS LED:
 *    GPIO  2  →  Built-in LED (blink on receive)
 *
 * ============================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>  // Install via IDE Library Manager: "ESP32Servo"

// ─────────────────────────────────────────────────────────────────────────────
//  PACKET STRUCTURE — must match transmitter exactly
// ─────────────────────────────────────────────────────────────────────────────
#pragma pack(push, 1)
struct ControlPacket {
    uint8_t  header;
    uint8_t  command;
    uint8_t  seq;
    uint8_t  checksum;
};
#pragma pack(pop)

enum Command : uint8_t {
    CMD_STOP           = 0x00,
    CMD_FORWARD        = 0x01,
    CMD_BACKWARD       = 0x02,
    CMD_TURN_LEFT      = 0x03,
    CMD_TURN_RIGHT     = 0x04,
    CMD_GRIPPER_TOGGLE = 0x05,
    CMD_MOVE_UP        = 0x06,
    CMD_MOVE_DOWN      = 0x07,
};

// ─────────────────────────────────────────────────────────────────────────────
//  PIN DEFINITIONS — adjust these to match your motor driver board
// ─────────────────────────────────────────────────────────────────────────────
static constexpr uint8_t PIN_LED       = 2;

// H-Bridge motor driver pins (e.g., L298N, TB6612, DRV8833)
static constexpr uint8_t PIN_MOTOR_A1  = 16;  // Left motor forward
static constexpr uint8_t PIN_MOTOR_A2  = 17;  // Left motor backward
static constexpr uint8_t PIN_MOTOR_B1  = 18;  // Right motor forward
static constexpr uint8_t PIN_MOTOR_B2  = 19;  // Right motor backward

// Servo outputs
static constexpr uint8_t PIN_GRIPPER   = 21;
static constexpr uint8_t PIN_LIFT      = 22;

// ─────────────────────────────────────────────────────────────────────────────
//  SERVO ANGLES — tune these for your specific gripper mechanism
// ─────────────────────────────────────────────────────────────────────────────
static constexpr int GRIPPER_OPEN_ANGLE  = 10;
static constexpr int GRIPPER_CLOSE_ANGLE = 90;
static constexpr int LIFT_UP_ANGLE       = 150;
static constexpr int LIFT_DOWN_ANGLE     = 30;
static constexpr int LIFT_NEUTRAL_ANGLE  = 90;

// ─────────────────────────────────────────────────────────────────────────────
//  SAFETY — auto-stop if no commands received
// ─────────────────────────────────────────────────────────────────────────────
static constexpr unsigned long WATCHDOG_TIMEOUT_MS = 500;

// ─────────────────────────────────────────────────────────────────────────────
//  STATE
// ─────────────────────────────────────────────────────────────────────────────
static Servo gripperServo;
static Servo liftServo;

static volatile bool     g_packetReady = false;
static volatile uint8_t  g_lastCommand = CMD_STOP;
static volatile uint8_t  g_lastSeq     = 0;
static unsigned long     g_lastRecvMs  = 0;
static uint32_t          g_recvCount   = 0;
static uint32_t          g_badCount    = 0;
static bool              g_gripperOpen = true;  // Starts open

// ─────────────────────────────────────────────────────────────────────────────
//  MOTOR CONTROL HELPERS
// ─────────────────────────────────────────────────────────────────────────────
static void motorsStop() {
    digitalWrite(PIN_MOTOR_A1, LOW);
    digitalWrite(PIN_MOTOR_A2, LOW);
    digitalWrite(PIN_MOTOR_B1, LOW);
    digitalWrite(PIN_MOTOR_B2, LOW);
}

static void motorsForward() {
    digitalWrite(PIN_MOTOR_A1, HIGH);
    digitalWrite(PIN_MOTOR_A2, LOW);
    digitalWrite(PIN_MOTOR_B1, HIGH);
    digitalWrite(PIN_MOTOR_B2, LOW);
}

static void motorsBackward() {
    digitalWrite(PIN_MOTOR_A1, LOW);
    digitalWrite(PIN_MOTOR_A2, HIGH);
    digitalWrite(PIN_MOTOR_B1, LOW);
    digitalWrite(PIN_MOTOR_B2, HIGH);
}

static void motorsTurnLeft() {
    // Left motor backward, right motor forward → pivot left
    digitalWrite(PIN_MOTOR_A1, LOW);
    digitalWrite(PIN_MOTOR_A2, HIGH);
    digitalWrite(PIN_MOTOR_B1, HIGH);
    digitalWrite(PIN_MOTOR_B2, LOW);
}

static void motorsTurnRight() {
    // Left motor forward, right motor backward → pivot right
    digitalWrite(PIN_MOTOR_A1, HIGH);
    digitalWrite(PIN_MOTOR_A2, LOW);
    digitalWrite(PIN_MOTOR_B1, LOW);
    digitalWrite(PIN_MOTOR_B2, HIGH);
}

// ─────────────────────────────────────────────────────────────────────────────
//  COMMAND DISPATCHER
// ─────────────────────────────────────────────────────────────────────────────
static void executeCommand(uint8_t cmd) {
    switch (cmd) {
        case CMD_STOP:
            motorsStop();
            liftServo.write(LIFT_NEUTRAL_ANGLE);
            break;

        case CMD_FORWARD:       motorsForward();    break;
        case CMD_BACKWARD:      motorsBackward();   break;
        case CMD_TURN_LEFT:     motorsTurnLeft();   break;
        case CMD_TURN_RIGHT:    motorsTurnRight();  break;

        case CMD_GRIPPER_TOGGLE:
            g_gripperOpen = !g_gripperOpen;
            gripperServo.write(g_gripperOpen ? GRIPPER_OPEN_ANGLE : GRIPPER_CLOSE_ANGLE);
            Serial.printf("[RX]   Gripper now: %s\n", g_gripperOpen ? "OPEN" : "CLOSED");
            break;

        case CMD_MOVE_UP:
            liftServo.write(LIFT_UP_ANGLE);
            break;

        case CMD_MOVE_DOWN:
            liftServo.write(LIFT_DOWN_ANGLE);
            break;

        default:
            Serial.printf("[RX] ⚠ Unknown command: 0x%02X\n", cmd);
            motorsStop();
            break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  ESP-NOW RECEIVE CALLBACK — runs in WiFi task context, keep it fast
// ─────────────────────────────────────────────────────────────────────────────
static void onDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
    if (len != sizeof(ControlPacket)) {
        g_badCount++;
        return;
    }

    const ControlPacket* pkt = reinterpret_cast<const ControlPacket*>(data);

    // Validate header magic byte
    if (pkt->header != 0xFB) {
        g_badCount++;
        return;
    }

    // Validate checksum
    uint8_t expectedChecksum = pkt->header ^ pkt->command ^ pkt->seq;
    if (pkt->checksum != expectedChecksum) {
        g_badCount++;
        return;
    }

    // Packet is valid — store for main loop processing
    g_lastCommand  = pkt->command;
    g_lastSeq      = pkt->seq;
    g_packetReady  = true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  INITIALIZATION
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("╔══════════════════════════════════════════════════╗");
    Serial.println("║   FiboCLUBS — ESP32 P2P Robot Receiver          ║");
    Serial.println("╚══════════════════════════════════════════════════╝");

    // ── Configure motor pins ─────────────────────────────────────────
    const uint8_t motorPins[] = {PIN_MOTOR_A1, PIN_MOTOR_A2, PIN_MOTOR_B1, PIN_MOTOR_B2};
    for (auto pin : motorPins) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }

    // ── Configure servos ─────────────────────────────────────────────
    gripperServo.attach(PIN_GRIPPER);
    liftServo.attach(PIN_LIFT);
    gripperServo.write(GRIPPER_OPEN_ANGLE);
    liftServo.write(LIFT_NEUTRAL_ANGLE);

    // ── Status LED ───────────────────────────────────────────────────
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);

    // ── Initialize WiFi + ESP-NOW ────────────────────────────────────
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // ** IMPORTANT: The transmitter needs this MAC address **
    Serial.printf("[RX] ╔═══════════════════════════════════════════╗\n");
    Serial.printf("[RX] ║  THIS DEVICE MAC: %-22s ║\n", WiFi.macAddress().c_str());
    Serial.printf("[RX] ╚═══════════════════════════════════════════╝\n");
    Serial.println("[RX] Copy this MAC into the transmitter's RECEIVER_MAC[]");

    if (esp_now_init() != ESP_OK) {
        Serial.println("[RX] ✘ FATAL: ESP-NOW init failed!");
        digitalWrite(PIN_LED, HIGH);
        while (true) { delay(1000); }
    }

    esp_now_register_recv_cb(onDataRecv);

    Serial.println("[RX] ✔ Listening for commands...");
    Serial.println("────────────────────────────────────────────────────");
}

// ─────────────────────────────────────────────────────────────────────────────
//  MAIN LOOP
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    unsigned long now = millis();

    // ── Process received packet ──────────────────────────────────────
    if (g_packetReady) {
        g_packetReady = false;
        g_recvCount++;
        g_lastRecvMs = now;

        // Brief LED flash
        digitalWrite(PIN_LED, HIGH);

        // Log and execute
        static const char* CMD_NAMES[] = {
            "STOP", "FORWARD", "BACKWARD", "TURN LEFT", "TURN RIGHT",
            "GRIPPER TOGGLE", "MOVE UP", "MOVE DOWN"
        };
        uint8_t cmd = g_lastCommand;
        if (cmd <= CMD_MOVE_DOWN) {
            Serial.printf("[RX] ◀ %s  (seq:%u  total:%u)\n",
                CMD_NAMES[cmd], g_lastSeq, g_recvCount);
        }

        executeCommand(cmd);

        // LED off after brief flash
        delay(10);
        digitalWrite(PIN_LED, LOW);
    }

    // ── Safety watchdog: auto-stop if signal lost ────────────────────
    // Why: If the transmitter goes out of range or loses power, the robot
    // must not continue moving indefinitely. This is a critical safety feature.
    if (g_lastRecvMs > 0 && (now - g_lastRecvMs) > WATCHDOG_TIMEOUT_MS) {
        motorsStop();
        liftServo.write(LIFT_NEUTRAL_ANGLE);
        g_lastRecvMs = 0;  // Prevent repeated stop calls
        Serial.println("[RX] ⚠ WATCHDOG: No signal — motors stopped!");
    }

    delay(1);
}
