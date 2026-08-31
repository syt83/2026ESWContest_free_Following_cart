#include "StellaUWB.h"

void rangingHandler(UWBRangingData &rangingData)
{
    if (rangingData.measureType() ==
        (uint8_t)uwb::MeasurementType::TWO_WAY)
    {
        RangingMeasures twr =
            rangingData.twoWayRangingMeasure();

        for (int j = 0; j < rangingData.available(); j++)
        {x
            if (twr[j].status == 0 &&
                twr[j].distance != 0xFFFF)
            {
                Serial.print("LEFT distance = ");
                Serial.println(twr[j].distance);
            }
        }
    }
}

void setup()
{
    Serial.begin(115200);

    // 좌측 Stella = 2번
    uint8_t devAddr[] = {0x33, 0x33};

    // 손에 들고 있는 Controller
    uint8_t controllerAddr[] = {0x11, 0x11};

    UWBMacAddress srcAddr(
        UWBMacAddress::Size::SHORT,
        devAddr
    );

    UWBMacAddress dstAddr(
        UWBMacAddress::Size::SHORT,
        controllerAddr
    );

    UWB.registerRangingCallback(rangingHandler);

    UWB.begin();

    Serial.println("Starting LEFT Stella...");

    while (UWB.state() != 0)
    {
        delay(10);
    }

    MulticastResponder responder(
        0x11223344,
        srcAddr,
        dstAddr
    );

    UWBSessionManager.addSession(responder);

    responder.init();
    responder.start();
}

void loop()
{
    delay(1000);
}
