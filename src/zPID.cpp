#include <Arduino.h>
#include <ESP32Encoder.h>
#include "config_Z.h"
#include "homing_flags.h"

// Create encoder instance
ESP32Encoder encoder;

float target_z_Pos = 00.0; // target in mm
volatile bool movement_z_done = true;
volatile bool homingDone_z = false;

unsigned long lastTime = 0;
long lastCount = 0;

void setupPWM()
{
    ledcSetup(0, 20000, 8); // channel 0, 20kHz, 8-bit
    ledcAttachPin(ENA, 0);  // attach ch 0 to ENA pin
    ledcWrite(0, 0);
}

float computeDistanceMM(long count)
{
    float rev = (count / (float)PPR); // revolutions
    return rev * screw_lead;          // linear mm
}

void setMotor(int pwm)
{

    ledcWrite(0, abs(pwm));
    if (pwm > 0)
    {
        // DOWN
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
    }
    else if (pwm < 0)
    {
        // UP
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    }
    else
    {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
    }
}
void home_z(void *pvParameters)
{
    Serial.println("Homing z axis...");
    setMotor(-180);
    vTaskDelay(pdMS_TO_TICKS(50)); // Let motor start
    while (digitalRead(limitSwitchPin_z) == HIGH)
    {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    setMotor(0);
    encoder.clearCount(); // Reset encoder count using library function

    homingDone_z = true;
    Serial.println("Z axis homed to position 0");

    vTaskDelete(NULL);
}

// ---------------- PID compute ----------------
float computePID(PID &pid, float setpoint, float measurement, float deltaTime)
{
    float error = setpoint - measurement;
    pid.integral += 0.5f * (error + pid.prevError) * deltaTime; // trapezoidal integration
    pid.integral = constrain(pid.integral, pid.outMin, pid.outMax);
    float derivative = (error - pid.prevError) / deltaTime;
    float out = pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;
    pid.prevError = error;
    return constrain(out, pid.outMin, pid.outMax);
}

void applyPID(void *parameter)
{
    while (!homingDone_x || !homingDone_y || !homingDone_z)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    Serial.println("Starting Z PID control loop...");

    lastTime = micros();
    lastCount = encoder.getCount(); // Get initial encoder count
    static float lastTarget = -999.0f;
    unsigned long lastDebugTime = 0; // For debug output timing

    while (1)
    {
        unsigned long now = micros();
        float deltaTime = (now - lastTime) / 1e6f;

        if (deltaTime < 0.001f)
        { // Minimum 1ms update rate
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        lastTime = now;

        // Read encoder count using ESP32Encoder library
        long currentCount = encoder.getCount();

        // Track target position changes
        if (target_z_Pos != lastTarget)
        {
            lastTarget = target_z_Pos;
            movement_z_done = false;
            Serial.println("Z target changed to: " + String(target_z_Pos) + " mm");
        }

        float currentDistance = computeDistanceMM(currentCount);
        float currentVelocity = computeDistanceMM(currentCount - lastCount) / deltaTime; // Convert to mm/s
        lastCount = currentCount;

        // outer loop displacement control
        float targetVelocity = computePID(posPID, target_z_Pos, currentDistance, deltaTime);
        // inner loop: velocity control
        int pwm = (int)computePID(velPID, targetVelocity, currentVelocity, deltaTime);

        // Debug output every 500ms
        if (now - lastDebugTime >= 500000) // 500ms in microseconds
        {
            Serial.print("Z Pos: ");
            Serial.print(currentDistance, 2);
            Serial.print(" mm | Target: ");
            Serial.print(target_z_Pos, 2);
            Serial.print(" mm | Count: ");
            Serial.print(currentCount);
            Serial.print(" | Vel: ");
            Serial.print(currentVelocity, 2);
            Serial.print(" mm/s | PWM: ");
            Serial.println(pwm);
            lastDebugTime = now;
        }

        // if error smaller than 0.5mm stop
        if (abs(target_z_Pos - currentDistance) <= 0.5f)
        {
            Serial.println("Z position reached: " + String(currentDistance) + " mm");
            pwm = 0;
            movement_z_done = true;
        }
        setMotor(pwm);
        vTaskDelay(pdMS_TO_TICKS(5)); // 200Hz update rate
    }
}

void startup_Z()
{
    Serial.begin(115200);

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);

    pinMode(limitSwitchPin_z, INPUT);
    // Enable internal pull-ups on encoder lines to avoid floating inputs
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);

    // setup Enable PWM for enable pin
    setupPWM();

    // Initialize ESP32Encoder library
    // ESP32Encoder::useInternalWeakPullups = UP;  // Use internal pullups
    encoder.attachFullQuad(ENC_A, ENC_B); // Attach both pins in full quadrature mode
    encoder.clearCount();                 // Start at 0

    Serial.print("ESP32Encoder initialized on GPIO");
    Serial.print(ENC_A);
    Serial.print(" (A) and GPIO");
    Serial.print(ENC_B);
    Serial.println(" (B)");

    lastTime = micros();

    Serial.println("Ready: PID position control (non-blocking).");
}
