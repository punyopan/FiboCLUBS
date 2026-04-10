/**
 * ============================================================================
 *  ESP32 P2P TRANSMITTER — Robot Control Buttons
 * ============================================================================
 *
 *  Protocol:  ESP-NOW (peer-to-peer, no WiFi router required)
 *  Latency:   ~1-3ms typical
 *  Range:     Up to 200m line-of-sight (with default TX power)
 *
 *  BUTTONS (directly to GND, internal pull-up used):
 *    GPIO 13  →  FORWARD
 *    GPIO 12  →  BACKWARD
 *    GPIO 14  →  TURN LEFT
 *    GPIO 27  →  TURN RIGHT
 *    GPIO 26  →  GRIPPER CLOSE
 *    GPIO 25  →  GRIPPER OPEN
 *    GPIO 33  →  MOVE UP
 *    GPIO 32  →  MOVE DOWN
 *
 *  STATUS LED:
 *    GPIO  2  →  Built-in LED (blink on send, solid on error)
 *
 *  WIRING:
 *    Each button connects the GPIO pin to GND when pressed.
 *    No external pull-up resistors needed (internal pull-ups enabled).
 *
 *  HOW TO SET RECEIVER MAC:
 *    1. Flash the receiver with the receiver.ino sketch.
 *    2. Open Serial Monitor — it prints its MAC address on boot.
 *    3. Copy that MAC into RECEIVER_MAC below.
 *
 * ============================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ─────────────────────────────────────────────────────────────────────────────
//  RECEIVER MAC ADDRESS — *** CHANGE THIS TO YOUR RECEIVER'S MAC ***
// ─────────────────────────────────────────────────────────────────────────────
static uint8_t RECEIVER_MAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
// Example: {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC}
// Set to broadcast (FF:FF:FF:FF:FF:FF) for testing without knowing the MAC.

// ─────────────────────────────────────────────────────────────────────────────
//  COMMAND PROTOCOL — single-byte commands for minimum latency
// ─────────────────────────────────────────────────────────────────────────────
enum Command : uint8_t {
    CMD_STOP          = 0x00,
    CMD_FORWARD       = 0x01,
    CMD_BACKWARD      = 0x02,
    CMD_TURN_LEFT     = 0x03,
    CMD_TURN_RIGHT    = 0x04,
    CMD_GRIPPER_CLOSE = 0x05,
    CMD_GRIPPER_OPEN  = 0x06,
    CMD_MOVE_UP       = 0x07,
    CMD_MOVE_DOWN     = 0x08,
};

// ─────────────────────────────────────────────────────────────────────────────
//  PACKET STRUCTURE — 4 bytes total for integrity & future extensibility
// ─────────────────────────────────────────────────────────────────────────────
//  Why structured packet instead of raw byte:
//    - Header byte guards against stray/corrupt data on the receiver
//    - Sequence number lets the receiver detect missed packets
//    - Fixed struct alignment is cache-friendly and memcpy-safe
#pragma pack(push, 1)
struct ControlPacket {
    uint8_t  header;      // Magic header: 0xFB ("FiboCLUBS")
    uint8_t  command;     // Command enum value
    uint8_t  seq;         // Rolling sequence number (0-255)
    uint8_t  checksum;    // XOR of header ^ command ^ seq
};
#pragma pack(pop)

static_assert(sizeof(ControlPacket) == 4, "Packet must be exactly 4 bytes");

// ─────────────────────────────────────────────────────────────────────────────
//  PIN DEFINITIONS
// ─────────────────────────────────────────────────────────────────────────────
static constexpr uint8_t PIN_LED = 2;  // Built-in LED on most ESP32 DevKit boards

struct ButtonDef {
    uint8_t pin;
    Command command;
    const char* label;  // For serial debug output
};

// Why these specific GPIOs: they are all safe-to-use input pins on ESP32
// that don't interfere with boot strapping or flash operations.
static const ButtonDef BUTTONS[] = {
    {13, CMD_FORWARD,       "FORWARD"},
    {12, CMD_BACKWARD,      "BACKWARD"},
    {14, CMD_TURN_LEFT,     "TURN LEFT"},
    {27, CMD_TURN_RIGHT,    "TURN RIGHT"},
    {26, CMD_GRIPPER_CLOSE, "GRIPPER CLOSE"},
    {25, CMD_GRIPPER_OPEN,  "GRIPPER OPEN"},
    {33, CMD_MOVE_UP,       "MOVE UP"},
    {32, CMD_MOVE_DOWN,     "MOVE DOWN"},
};

static constexpr size_t NUM_BUTTONS = sizeof(BUTTONS) / sizeof(BUTTONS[0]);

// ─────────────────────────────────────────────────────────────────────────────
//  TIMING CONSTANTS
// ─────────────────────────────────────────────────────────────────────────────
static constexpr unsigned long DEBOUNCE_MS        = 50;   // Software debounce window
static constexpr unsigned long REPEAT_INTERVAL_MS = 100;  // How often to re-send while held
static constexpr unsigned long STOP_TIMEOUT_MS    = 150;  // Send STOP after all buttons released
static constexpr unsigned long LED_BLINK_MS       = 30;   // Visual feedback duration per send

// ─────────────────────────────────────────────────────────────────────────────
//  STATE VARIABLES
// ─────────────────────────────────────────────────────────────────────────────
static uint8_t  g_seq            = 0;
static bool     g_peerAdded      = false;
static bool     g_lastSendOk     = true;
static uint32_t g_sendCount      = 0;
static uint32_t g_failCount      = 0;

// Per-button debounce state
struct ButtonState {
    bool     pressed;
    unsigned long lastChangeMs;
    unsigned long lastSendMs;
};
static ButtonState g_btnState[NUM_BUTTONS] = {};

static unsigned long g_lastAnyPressMs = 0;
static bool          g_stopSent       = true;
static unsigned long g_ledOffMs       = 0;

// ─────────────────────────────────────────────────────────────────────────────
//  ESP-NOW SEND CALLBACK — confirms delivery at MAC layer
// ─────────────────────────────────────────────────────────────────────────────
static void onSendComplete(const uint8_t* mac, esp_now_send_status_t status) {
    g_lastSendOk = (status == ESP_NOW_SEND_SUCCESS);
    if (!g_lastSendOk) {
        g_failCount++;
        Serial.println("[TX] ⚠ Delivery FAILED — is the receiver powered on?");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  SEND COMMAND — builds packet and transmits via ESP-NOW
// ─────────────────────────────────────────────────────────────────────────────
static void sendCommand(Command cmd) {
    ControlPacket pkt;
    pkt.header   = 0xFB;
    pkt.command  = static_cast<uint8_t>(cmd);
    pkt.seq      = g_seq++;
    pkt.checksum = pkt.header ^ pkt.command ^ pkt.seq;

    esp_err_t result = esp_now_send(RECEIVER_MAC, reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt));

    if (result == ESP_OK) {
        g_sendCount++;
        // Brief LED flash for visual feedback
        digitalWrite(PIN_LED, HIGH);
        g_ledOffMs = millis() + LED_BLINK_MS;
    } else {
        g_failCount++;
        Serial.printf("[TX] ✘ esp_now_send() error: %s\n", esp_err_to_name(result));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  INITIALIZATION
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);  // Let serial stabilize

    Serial.println();
    Serial.println("╔══════════════════════════════════════════════════╗");
    Serial.println("║   FiboCLUBS — ESP32 P2P Robot Transmitter       ║");
    Serial.println("╚══════════════════════════════════════════════════╝");

    // ── Configure GPIO ──────────────────────────────────────────────
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);

    for (size_t i = 0; i < NUM_BUTTONS; i++) {
        pinMode(BUTTONS[i].pin, INPUT_PULLUP);
        g_btnState[i] = {false, 0, 0};
    }

    // ── Initialize WiFi in STA mode (required for ESP-NOW) ─────────
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();  // Ensure we're not connected to any AP

    // Print this device's MAC for reference
    Serial.printf("[TX] This device MAC: %s\n", WiFi.macAddress().c_str());
    Serial.printf("[TX] Target receiver:  %02X:%02X:%02X:%02X:%02X:%02X\n",
        RECEIVER_MAC[0], RECEIVER_MAC[1], RECEIVER_MAC[2],
        RECEIVER_MAC[3], RECEIVER_MAC[4], RECEIVER_MAC[5]);

    // ── Optional: Set WiFi channel (must match receiver) ───────────
    // esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

    // ── Initialize ESP-NOW ──────────────────────────────────────────
    if (esp_now_init() != ESP_OK) {
        Serial.println("[TX] ✘ FATAL: ESP-NOW init failed!");
        // Solid LED = error state
        digitalWrite(PIN_LED, HIGH);
        while (true) { delay(1000); }
    }

    esp_now_register_send_cb(onSendComplete);

    // ── Register peer ───────────────────────────────────────────────
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, RECEIVER_MAC, 6);
    peerInfo.channel = 0;     // 0 = use current channel
    peerInfo.encrypt = false; // No encryption for low-latency control

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[TX] ⚠ Failed to add peer — check MAC address");
        // Non-fatal: broadcast mode may still work
    } else {
        g_peerAdded = true;
        Serial.println("[TX] ✔ Peer registered successfully");
    }

    Serial.println("[TX] ✔ Ready — press buttons to send commands");
    Serial.println("────────────────────────────────────────────────────");
}

// ─────────────────────────────────────────────────────────────────────────────
//  MAIN LOOP — scan buttons, debounce, send commands
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    unsigned long now = millis();
    bool anyPressed = false;

    // ── Scan all buttons ─────────────────────────────────────────────
    for (size_t i = 0; i < NUM_BUTTONS; i++) {
        // Buttons are active-LOW (pulled up, grounded when pressed)
        bool rawPressed = (digitalRead(BUTTONS[i].pin) == LOW);

        // Debounce: only register state change after DEBOUNCE_MS stable
        if (rawPressed != g_btnState[i].pressed) {
            if ((now - g_btnState[i].lastChangeMs) >= DEBOUNCE_MS) {
                g_btnState[i].pressed     = rawPressed;
                g_btnState[i].lastChangeMs = now;

                if (rawPressed) {
                    // Button just pressed — send immediately
                    Serial.printf("[TX] ▶ %s\n", BUTTONS[i].label);
                    sendCommand(BUTTONS[i].command);
                    g_btnState[i].lastSendMs = now;
                    g_stopSent = false;
                }
            }
        }

        // Continuous send while held (for smooth motion control)
        if (g_btnState[i].pressed) {
            anyPressed = true;
            g_lastAnyPressMs = now;

            if ((now - g_btnState[i].lastSendMs) >= REPEAT_INTERVAL_MS) {
                sendCommand(BUTTONS[i].command);
                g_btnState[i].lastSendMs = now;
            }
        }
    }

    // ── Auto-STOP when all buttons released ──────────────────────────
    // Why a timeout instead of immediate: prevents spurious STOP commands
    // during rapid button transitions (e.g., switching from LEFT to RIGHT).
    if (!anyPressed && !g_stopSent) {
        if ((now - g_lastAnyPressMs) >= STOP_TIMEOUT_MS) {
            Serial.println("[TX] ■ STOP");
            sendCommand(CMD_STOP);
            g_stopSent = true;
        }
    }

    // ── LED management ───────────────────────────────────────────────
    if (g_ledOffMs > 0 && now >= g_ledOffMs) {
        digitalWrite(PIN_LED, LOW);
        g_ledOffMs = 0;
    }

    // Small yield to prevent watchdog timeout
    delay(1);
}
