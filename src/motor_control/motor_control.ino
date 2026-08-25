// =============================================================
// Arduino Uno
// Dual MD20A
// 20 kHz PWM
//
// LEFT MOTOR
// PWM = D9
// DIR = D8
//
// RIGHT MOTOR
// PWM = D10
// DIR = D12
//
// LEFT ENCODER
// A = D2
// B = D4
//
// RIGHT ENCODER
// A = D3
// B = D7
// =============================================================


// =============================================================
// Motor pins
// =============================================================

const int PWM_L = 9;
const int DIR_L = 8;

const int PWM_R = 10;
const int DIR_R = 12;


// =============================================================
// Encoder pins
// =============================================================

const int ENC_L_A = 2;
const int ENC_L_B = 4;

const int ENC_R_A = 3;
const int ENC_R_B = 7;


// =============================================================
// Encoder count
// =============================================================

volatile long encoderLeft = 0;
volatile long encoderRight = 0;


// =============================================================
// Serial
// =============================================================

String serialBuffer = "";


// =============================================================
// Watchdog
// =============================================================

unsigned long lastCommandTime = 0;

const unsigned long COMMAND_TIMEOUT = 500;


// =============================================================
// Encoder send
// =============================================================

unsigned long lastEncoderSend = 0;

const unsigned long ENCODER_INTERVAL = 200;


// =============================================================
// 20 kHz PWM
//
// Uno clock = 16 MHz
//
// 16,000,000 / 20,000
// = 800
//
// TOP = 799
// =============================================================

const uint16_t PWM_TOP = 799;


// =============================================================
// Encoder ISR
// =============================================================

void encoderLeftISR()
{
    bool a = digitalRead(ENC_L_A);
    bool b = digitalRead(ENC_L_B);

    if (a == b)
    {
        encoderLeft++;
    }
    else
    {
        encoderLeft--;
    }
}


void encoderRightISR()
{
    bool a = digitalRead(ENC_R_A);
    bool b = digitalRead(ENC_R_B);

    if (a == b)
    {
        encoderRight++;
    }
    else
    {
        encoderRight--;
    }
}


// =============================================================
// Timer1 20kHz setup
//
// D9  = OC1A
// D10 = OC1B
// =============================================================

void setupMotorPWM20kHz()
{
    pinMode(PWM_L, OUTPUT);
    pinMode(PWM_R, OUTPUT);

    // Timer1 reset
    TCCR1A = 0;
    TCCR1B = 0;

    TCNT1 = 0;

    // =========================================================
    // Fast PWM Mode 14
    //
    // TOP = ICR1
    // =========================================================

    TCCR1A |= (1 << WGM11);

    TCCR1B |=
        (1 << WGM13)
        |
        (1 << WGM12);

    // =========================================================
    // Non-inverting
    //
    // D9  = OC1A
    // D10 = OC1B
    // =========================================================

    TCCR1A |=
        (1 << COM1A1)
        |
        (1 << COM1B1);

    // TOP
    ICR1 = PWM_TOP;

    // Prescaler = 1
    TCCR1B |= (1 << CS10);

    OCR1A = 0;
    OCR1B = 0;
}


// =============================================================
// 0~255 -> 0~799
// =============================================================

uint16_t pwmToTimerValue(int pwm)
{
    pwm = constrain(
        pwm,
        0,
        255
    );

    unsigned long value =
        (
            (unsigned long)pwm
            *
            PWM_TOP
        )
        /
        255;

    return (uint16_t)value;
}


// =============================================================
// Left motor
// =============================================================

void setLeftMotor(int speed)
{
    speed = constrain(
        speed,
        -255,
        255
    );

    if (speed >= 0)
    {
        digitalWrite(
            DIR_L,
            LOW
        );

        OCR1A = pwmToTimerValue(
            speed
        );
    }

    else
    {
        digitalWrite(
            DIR_L,
            HIGH
        );

        OCR1A = pwmToTimerValue(
            -speed
        );
    }
}


// =============================================================
// Right motor
// =============================================================

void setRightMotor(int speed)
{
    speed = constrain(
        speed,
        -255,
        255
    );

    if (speed >= 0)
    {
        digitalWrite(
            DIR_R,
            LOW
        );

        OCR1B = pwmToTimerValue(
            speed
        );
    }

    else
    {
        digitalWrite(
            DIR_R,
            HIGH
        );

        OCR1B = pwmToTimerValue(
            -speed
        );
    }
}


// =============================================================
// Both motors
// =============================================================

void setMotors(
    int leftSpeed,
    int rightSpeed
)
{
    setLeftMotor(
        leftSpeed
    );

    setRightMotor(
        rightSpeed
    );
}


// =============================================================
// Serial command parser
//
// Pi:
// 65,65
//
// reverse:
// -45,-45
// =============================================================

void processCommand(
    String command
)
{
    command.trim();

    int commaIndex =
        command.indexOf(',');

    if (commaIndex < 0)
    {
        return;
    }

    String leftText =
        command.substring(
            0,
            commaIndex
        );

    String rightText =
        command.substring(
            commaIndex + 1
        );

    int leftSpeed =
        leftText.toInt();

    int rightSpeed =
        rightText.toInt();

    leftSpeed = constrain(
        leftSpeed,
        -255,
        255
    );

    rightSpeed = constrain(
        rightSpeed,
        -255,
        255
    );

    setMotors(
        leftSpeed,
        rightSpeed
    );

    lastCommandTime = millis();
}


// =============================================================
// Setup
// =============================================================

void setup()
{
    Serial.begin(
        9600
    );

    // =========================================================
    // DIR
    // =========================================================

    pinMode(
        DIR_L,
        OUTPUT
    );

    pinMode(
        DIR_R,
        OUTPUT
    );

    digitalWrite(
        DIR_L,
        LOW
    );

    digitalWrite(
        DIR_R,
        LOW
    );

    // =========================================================
    // Encoder
    // =========================================================

    pinMode(
        ENC_L_A,
        INPUT_PULLUP
    );

    pinMode(
        ENC_L_B,
        INPUT_PULLUP
    );

    pinMode(
        ENC_R_A,
        INPUT_PULLUP
    );

    pinMode(
        ENC_R_B,
        INPUT_PULLUP
    );

    attachInterrupt(
        digitalPinToInterrupt(
            ENC_L_A
        ),
        encoderLeftISR,
        CHANGE
    );

    attachInterrupt(
        digitalPinToInterrupt(
            ENC_R_A
        ),
        encoderRightISR,
        CHANGE
    );

    // =========================================================
    // 20kHz PWM
    // =========================================================

    setupMotorPWM20kHz();

    // =========================================================
    // Stop
    // =========================================================

    setMotors(
        0,
        0
    );

    lastCommandTime = millis();

    lastEncoderSend = millis();

    Serial.println(
        "READY,20KHZ"
    );
}


// =============================================================
// Loop
// =============================================================

void loop()
{
    // =========================================================
    // Pi -> Uno
    // =========================================================

    while (
        Serial.available()
        >
        0
    )
    {
        char c = Serial.read();

        if (c == '\n')
        {
            processCommand(
                serialBuffer
            );

            serialBuffer = "";
        }

        else if (c != '\r')
        {
            serialBuffer += c;

            if (
                serialBuffer.length()
                >
                50
            )
            {
                serialBuffer = "";
            }
        }
    }

    unsigned long now = millis();

    // =========================================================
    // Watchdog
    // =========================================================

    if (
        now
        -
        lastCommandTime
        >
        COMMAND_TIMEOUT
    )
    {
        setMotors(
            0,
            0
        );
    }

    // =========================================================
    // Encoder
    // =========================================================

    if (
        now
        -
        lastEncoderSend
        >=
        ENCODER_INTERVAL
    )
    {
        lastEncoderSend = now;

        long leftCopy;
        long rightCopy;

        noInterrupts();

        leftCopy = encoderLeft;
        rightCopy = encoderRight;

        interrupts();

        Serial.print(
            "ENC,"
        );

        Serial.print(
            leftCopy
        );

        Serial.print(
            ","
        );

        Serial.println(
            rightCopy
        );
    }
}
