#include "StellaUWB.h"

// =====================================================
// Stella 사이 간격
// =====================================================

const float BASELINE = 28.0;   // cm


// =====================================================
// 거리 변수
// =====================================================

float rawLeft  = 0.0;
float rawRight = 0.0;

float medianLeft  = 0.0;
float medianRight = 0.0;

float correctedLeft  = 0.0;
float correctedRight = 0.0;

bool haveLeft  = false;
bool haveRight = false;


// =====================================================
// Median Filter
// =====================================================

const int FILTER_SIZE = 5;

float leftBuffer[FILTER_SIZE]  = {0};
float rightBuffer[FILTER_SIZE] = {0};

int leftIndex  = 0;
int rightIndex = 0;

int leftCount  = 0;
int rightCount = 0;


// =====================================================
// LEFT / RIGHT 거리 보정
//
// 이 값은 절대거리 표시용
// 방향 판정에는 사용하지 않음!
// =====================================================

float correctLeft(float raw)
{
    float corrected =
        0.9629 * raw - 35.36;

    if (corrected < 0)
        corrected = 0;

    return corrected;
}


float correctRight(float raw)
{
    float corrected =
        0.8247 * raw - 22.04;

    if (corrected < 0)
        corrected = 0;

    return corrected;
}


// =====================================================
// Median 계산
// =====================================================

float calculateMedian(float buffer[], int count)
{
    float temp[FILTER_SIZE];

    for (int i = 0; i < count; i++)
    {
        temp[i] = buffer[i];
    }

    for (int i = 0; i < count - 1; i++)
    {
        for (int j = i + 1; j < count; j++)
        {
            if (temp[j] < temp[i])
            {
                float t = temp[i];

                temp[i] = temp[j];
                temp[j] = t;
            }
        }
    }

    if (count % 2 == 1)
    {
        return temp[count / 2];
    }
    else
    {
        return
            (temp[count / 2 - 1] +
             temp[count / 2]) / 2.0;
    }
}


// =====================================================
// LEFT Median 업데이트
// =====================================================

float updateLeftMedian(float value)
{
    leftBuffer[leftIndex] = value;

    leftIndex++;

    if (leftIndex >= FILTER_SIZE)
        leftIndex = 0;

    if (leftCount < FILTER_SIZE)
        leftCount++;

    return calculateMedian(
        leftBuffer,
        leftCount
    );
}


// =====================================================
// RIGHT Median 업데이트
// =====================================================

float updateRightMedian(float value)
{
    rightBuffer[rightIndex] = value;

    rightIndex++;

    if (rightIndex >= FILTER_SIZE)
        rightIndex = 0;

    if (rightCount < FILTER_SIZE)
        rightCount++;

    return calculateMedian(
        rightBuffer,
        rightCount
    );
}


// =====================================================
// MAC 주소 판별
// =====================================================

bool isRightStella(uint8_t *addr)
{
    return
        addr[0] == 0x22 &&
        addr[1] == 0x22;
}


bool isLeftStella(uint8_t *addr)
{
    return
        addr[0] == 0x33 &&
        addr[1] == 0x33;
}


// =====================================================
// 위치 계산
//
// 주의:
// 서로 다른 거리 보정식을 사용하기 때문에
// 현재 X/Y/ANGLE은 참고용.
// 모터 방향제어에는 사용하지 않음.
// =====================================================

void calculateUserPosition()
{
    float dL = correctedLeft;
    float dR = correctedRight;

    float x =
        (dL * dL - dR * dR)
        / (2.0 * BASELINE);


    float temp =
        dL * dL
        -
        (x + BASELINE / 2.0)
        *
        (x + BASELINE / 2.0);


    if (temp < 0)
    {
        Serial.println(
            "POSITION CALC = INVALID"
        );

        return;
    }


    float y = sqrt(temp);

    float centerDistance =
        sqrt(x * x + y * y);


    float angleDeg =
        atan2(x, y)
        * 180.0 / PI;


    Serial.println(
        "======== POSITION (REFERENCE) ========"
    );


    Serial.print(
        "X POSITION      = "
    );

    Serial.print(x, 1);

    Serial.println(" cm");


    Serial.print(
        "FORWARD Y       = "
    );

    Serial.print(y, 1);

    Serial.println(" cm");


    Serial.print(
        "CENTER DISTANCE = "
    );

    Serial.print(
        centerDistance,
        1
    );

    Serial.println(" cm");


    Serial.print(
        "ANGLE           = "
    );

    Serial.print(
        angleDeg,
        1
    );

    Serial.println(" deg");


    Serial.println(
        "======================================"
    );
}


// =====================================================
// UWB Callback
// =====================================================

void rangingHandler(UWBRangingData &rangingData)
{
    if (rangingData.measureType() !=
        (uint8_t)uwb::MeasurementType::TWO_WAY)
    {
        return;
    }


    RangingMeasures twr =
        rangingData.twoWayRangingMeasure();


    bool newLeft  = false;
    bool newRight = false;


    // =================================================
    // 거리 데이터 읽기
    // =================================================

    for (int j = 0;
         j < rangingData.available();
         j++)
    {
        if (twr[j].status != 0)
            continue;

        if (twr[j].distance == 0xFFFF)
            continue;


        float raw =
            (float)twr[j].distance;


        // =============================================
        // RIGHT Stella = 22:22
        // =============================================

        if (isRightStella(
                twr[j].peer_addr))
        {
            rawRight = raw;

            medianRight =
                updateRightMedian(
                    rawRight
                );

            correctedRight =
                correctRight(
                    medianRight
                );

            haveRight = true;
            newRight  = true;
        }


        // =============================================
        // LEFT Stella = 33:33
        // =============================================

        else if (isLeftStella(
                     twr[j].peer_addr))
        {
            rawLeft = raw;

            medianLeft =
                updateLeftMedian(
                    rawLeft
                );

            correctedLeft =
                correctLeft(
                    medianLeft
                );

            haveLeft = true;
            newLeft  = true;
        }


        // MAC 확인용
        Serial.print("Peer = ");

        Serial.print(
            twr[j].peer_addr[0],
            HEX
        );

        Serial.print(":");

        Serial.print(
            twr[j].peer_addr[1],
            HEX
        );

        Serial.print(
            "   RAW = "
        );

        Serial.print(raw);

        Serial.println(" cm");
    }


    // =================================================
    // 좌우 데이터 확보
    // =================================================

    if (haveLeft && haveRight)
    {
        Serial.println();
        Serial.println(
            "--------------------------------------"
        );


        // =============================================
        // LEFT
        // =============================================

        Serial.print(
            "LEFT RAW        = "
        );

        Serial.print(
            rawLeft,
            1
        );

        Serial.println(" cm");


        Serial.print(
            "LEFT MEDIAN     = "
        );

        Serial.print(
            medianLeft,
            1
        );

        Serial.println(" cm");


        Serial.print(
            "LEFT CORRECTED  = "
        );

        Serial.print(
            correctedLeft,
            1
        );

        Serial.println(" cm");


        // =============================================
        // RIGHT
        // =============================================

        Serial.print(
            "RIGHT RAW       = "
        );

        Serial.print(
            rawRight,
            1
        );

        Serial.println(" cm");


        Serial.print(
            "RIGHT MEDIAN    = "
        );

        Serial.print(
            medianRight,
            1
        );

        Serial.println(" cm");


        Serial.print(
            "RIGHT CORRECTED = "
        );

        Serial.print(
            correctedRight,
            1
        );

        Serial.println(" cm");


        // =================================================
        //
        // ★ 핵심 변경 부분 ★
        //
        // 방향 판단에는 corrected 값을 사용하지 않는다.
        //
        // =================================================

        float directionError =
            medianLeft -
            medianRight;


        Serial.println();
        Serial.println(
            "========== DIRECTION =========="
        );


        Serial.print(
            "DIRECTION ERROR = "
        );

        Serial.print(
            directionError,
            1
        );

        Serial.println(" cm");


        // =================================================
        // 방향 Deadband
        // =================================================

        const float DIRECTION_DEADBAND =
            10.0;


        if (directionError <
            -DIRECTION_DEADBAND)
        {
            Serial.println(
                "USER POSITION  = LEFT"
            );
        }

        else if (directionError >
                 DIRECTION_DEADBAND)
        {
            Serial.println(
                "USER POSITION  = RIGHT"
            );
        }

        else
        {
            Serial.println(
                "USER POSITION  = CENTER"
            );
        }


        Serial.println(
            "==============================="
        );


        // =================================================
        // 데이터 갱신 확인
        // =================================================

        Serial.print(
            "NEW LEFT/RIGHT = "
        );

        Serial.print(
            newLeft ? "YES" : "NO"
        );

        Serial.print(" / ");

        Serial.println(
            newRight ? "YES" : "NO"
        );


        // =================================================
        // 위치/각도는 참고용
        // =================================================

        calculateUserPosition();


        Serial.println(
            "--------------------------------------"
        );

        Serial.println();
    }
}


// =====================================================
// SETUP
// =====================================================

void setup()
{
    Serial.begin(115200);

    delay(2000);


    Serial.println();
    Serial.println(
        "HAND STELLA CONTROLLER START"
    );


    Serial.print(
        "BASELINE = "
    );

    Serial.print(BASELINE);

    Serial.println(" cm");


    // =================================================
    // HAND Stella = 11:11
    // =================================================

    uint8_t handAddr[] =
    {
        0x11,
        0x11
    };


    UWBMacAddress srcAddr(
        UWBMacAddress::Size::SHORT,
        handAddr
    );


    // =================================================
    // RIGHT = 22:22
    // =================================================

    uint8_t rightAddr[] =
    {
        0x22,
        0x22
    };


    UWBMacAddress right(
        UWBMacAddress::Size::SHORT,
        rightAddr
    );


    // =================================================
    // LEFT = 33:33
    // =================================================

    uint8_t leftAddr[] =
    {
        0x33,
        0x33
    };


    UWBMacAddress left(
        UWBMacAddress::Size::SHORT,
        leftAddr
    );


    // =================================================
    // 측정 대상
    // =================================================

    UWBMacAddressList destination(
        UWBMacAddress::Size::SHORT
    );


    destination.add(right);
    destination.add(left);


    // =================================================
    // Callback
    // =================================================

    UWB.registerRangingCallback(
        rangingHandler
    );


    // =================================================
    // UWB 시작
    // =================================================

    UWB.begin();


    Serial.println(
        "Starting UWB..."
    );


    while (UWB.state() != 0)
    {
        delay(10);
    }


    Serial.println(
        "UWB initialized."
    );


    // =================================================
    // One-To-Many
    // =================================================

    UWBRangingOneToMany controller(
        0x11223344,
        srcAddr,
        destination
    );


    UWBSessionManager.addSession(
        controller
    );


    controller.init();

    controller.start();


    Serial.println(
        "Ranging started."
    );

    Serial.println();
}


// =====================================================
// LOOP
// =====================================================

void loop()
{
    delay(100);
}
