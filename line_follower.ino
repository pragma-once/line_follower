
#define FLOAT_TYPE float

// PINS

const int LED = 13;

const int START_BUTTON = 2;
const int STOP_BUTTON  = 4;
const int BACKGROUND_CALIBRATION_BUTTON = 7;
const int LINE_CALIBRATION_BUTTON       = 8;

const int LEFT_MOTOR  = 9;
const int RIGHT_MOTOR = 10;

// SETTINGS

const int UPDATE_DELAY = 1; // ms
const FLOAT_TYPE ACCELERATION = (FLOAT_TYPE)UPDATE_DELAY / 50.0;

const FLOAT_TYPE TURNING_DEADZONE         = 0.00;
const FLOAT_TYPE DIRECTION_TURNING_SIGN   = 0.60;
const FLOAT_TYPE LINE_MINIMUM_CONTRAST    = 0.50;
const FLOAT_TYPE LINE_ACCEPTABLE_CONTRAST = 0.80;

const int SensorsCount = 5;

#define USE_SENSOR_DIRECTIONS 0 // 1: use the following
                                // 0: linear distribution
#if USE_SENSOR_DIRECTIONS
FLOAT_TYPE SensorDirections[] = { -1, -0.6, -0.2, 0.2, 0.6, 1 };
#else
FLOAT_TYPE SensorDirections[6];
#endif

#define USE_DIRECTION_MAPPING 0

#if USE_DIRECTION_MAPPING
void MapDirection(FLOAT_TYPE& Direction)
{
    Direction = Direction < 0 ? -(Direction * Direction) : Direction * Direction;
}
#endif

#define TEST 0

uint16_t SensorArray[6];
uint16_t SensorBackgroundValues[] = { 700, 700, 700, 700, 700, 700 };
uint16_t SensorLineValues[] = { 400, 400, 400, 400, 400, 400 };

#define DIRECTION_METHOD 2 // 1: Avg method (smooth)
                           // 2: Smooth line finder
                           // 3: max Value => line
                           // 4: Value > Threashold => line (to make sure it's facing the line (real value), for noisy environments)

#if DIRECTION_METHOD == 4
    const FLOAT_TYPE LINE_THRESHOLD = 0.5; // Calibrate BG on the darkest side if 0.5
#endif

FLOAT_TYPE Direction = 0;

FLOAT_TYPE Speed = 0;
FLOAT_TYPE LeftMotorSpeed  = 0;
FLOAT_TYPE RightMotorSpeed = 0;

bool Running = false;

void CountDown(int Speed = 40)
{
    // Software PWM
    for (int32_t i = 8000000 / 100 / Speed; i >= 0; i--)
        for (int32_t j = 0; j < 100; j++)
        {
            int amount = i / (8000000 / 100 / Speed / 100);
            if (j < amount)
                digitalWrite(LED, HIGH);
            else
                digitalWrite(LED, LOW);
        }
}

void setup()
{
    pinMode(LED, OUTPUT);

    pinMode(START_BUTTON,     INPUT_PULLUP);
    pinMode(STOP_BUTTON,      INPUT_PULLUP);
    pinMode(BACKGROUND_CALIBRATION_BUTTON, INPUT_PULLUP);
    pinMode(LINE_CALIBRATION_BUTTON, INPUT_PULLUP);

    pinMode(LEFT_MOTOR,  OUTPUT);
    pinMode(RIGHT_MOTOR, OUTPUT);

#if !USE_SENSOR_DIRECTIONS
    FLOAT_TYPE Fragment = (FLOAT_TYPE)2 / (FLOAT_TYPE)(SensorsCount - 1);
    for (int i = 0; i < SensorsCount; i++)
        SensorDirections[i] = -1 + Fragment * i;
#endif
#if TEST
    Serial.begin(9600);
#endif
}

void UpdateSensors()
{
    SensorArray[0] = analogRead(A0);
    SensorArray[1] = analogRead(A1);
    SensorArray[2] = analogRead(A2);
    SensorArray[3] = analogRead(A3);
    SensorArray[4] = analogRead(A4);
    SensorArray[5] = analogRead(A5);
}

void ProcessSensors()
{
    FLOAT_TYPE Amount[6]; // CAUTION: may have values less than zero!
#if DIRECTION_METHOD == 1
    FLOAT_TYPE Sum = 0;
#endif
    FLOAT_TYPE Minimum = 100;
    FLOAT_TYPE Maximum = 0;
    int MaximumIndex = 0;
    for (int i = 0; i < SensorsCount; i++)
    {
        Amount[i] = ((FLOAT_TYPE)SensorArray[i] - (FLOAT_TYPE)SensorBackgroundValues[i])
                  / ((FLOAT_TYPE)SensorLineValues[i] - (FLOAT_TYPE)SensorBackgroundValues[i]);
        if (Amount[i] < Minimum) Minimum = Amount[i];
        if (Amount[i] > Maximum) { Maximum = Amount[i]; MaximumIndex = i; }
#if DIRECTION_METHOD == 1
        Sum += Amount[i];
#endif
    }
    FLOAT_TYPE Contrast = Maximum - Minimum;
#if TEST
        Serial.println("Contrast:");
        Serial.println(Contrast);
#endif
    if (Contrast < LINE_MINIMUM_CONTRAST)
    {
#if TEST
        Serial.println("Rescue Mode!");
#endif
        if (Direction < -DIRECTION_TURNING_SIGN)
            Direction = -1;
        else if (Direction > DIRECTION_TURNING_SIGN)
            Direction = 1;
        // else, keep the previous Direction
    }
    else
    {
        bool ForceRescue = false;
        FLOAT_TYPE RescueDirection = Direction;
#if DIRECTION_METHOD == 1
        Direction = 0;
        for (int i = 0; i < SensorsCount; i++)
            Direction += (SensorDirections[i] - Minimum) * Amount[i];
        Sum -= Minimum * SensorsCount;
        Direction /= Sum; // Avg
#elif DIRECTION_METHOD == 2
        int Deviation;
        bool InEnd = true;
        if (MaximumIndex == 0) Deviation = 1;
        else if (MaximumIndex == SensorsCount - 1) Deviation = -1;
        else
        {
            Deviation = Amount[MaximumIndex - 1] < Amount[MaximumIndex + 1] ? 1 : -1;
            InEnd = false;
        }
        Direction = SensorDirections[MaximumIndex]
                  + (SensorDirections[MaximumIndex + Deviation] - SensorDirections[MaximumIndex]) / 2
                  * (InEnd ? (Amount[MaximumIndex + Deviation] - Minimum) / Contrast
                           : (Amount[MaximumIndex + Deviation] - Amount[MaximumIndex - Deviation])
                             / (Maximum - Amount[MaximumIndex - Deviation])
                    );
#elif DIRECTION_METHOD == 3
        Direction = SensorDirections[MaximumIndex];
#elif DIRECTION_METHOD == 4
        if (Maximum >= LINE_THRESHOLD)
            Direction = SensorDirections[MaximumIndex];
        else ForceRescue = true;
#endif
#if USE_DIRECTION_MAPPING
        MapDirection(Direction);
#endif
        if (ForceRescue)
        {
#if TEST
            Serial.println("Force Rescue Mode!");
            Serial.println("Max (line) Sensor Value:");
            Serial.println(Maximum);
#endif
            if (Direction < -DIRECTION_TURNING_SIGN)
                Direction = -1;
            else if (Direction > DIRECTION_TURNING_SIGN)
                Direction = 1;
            else Direction = RescueDirection;
        }
        else if (Contrast < LINE_ACCEPTABLE_CONTRAST)
        {
#if TEST
            Serial.println("Semi-Rescue Mode!");
#endif
            Contrast = (Contrast - LINE_MINIMUM_CONTRAST) / (LINE_ACCEPTABLE_CONTRAST - LINE_MINIMUM_CONTRAST);
            //Contrast = 1 - Contrast;
            //Contrast *= Contrast;
            //Contrast = 1 - Contrast;
            if (RescueDirection < -DIRECTION_TURNING_SIGN)
                RescueDirection = -1;
            else if (RescueDirection > DIRECTION_TURNING_SIGN)
                RescueDirection = 1;
            Direction = Contrast * Direction + (1 - Contrast) * RescueDirection;
        }
#if TEST
        else Serial.println("Sensing Mode!");
#endif
    }
#if TEST
        Serial.println("Direction:");
        Serial.println(Direction);
#endif
}

void Process()
{
    if (Running)
    {
        ProcessSensors();
        if (Speed < 1) Speed += ACCELERATION;
        if (Speed >= 1) Speed = 1;

        if (Direction < -TURNING_DEADZONE) // Go left
        {
            LeftMotorSpeed = Speed * (Direction + 1); // (0, -1) => (1, 0)
            RightMotorSpeed = Speed;
        }
        else if (Direction > TURNING_DEADZONE) // Go right
        {
            RightMotorSpeed = Speed * (1 - Direction); // (0, 1) => (1, 0)
            LeftMotorSpeed = Speed;
        }
        else
        {
            LeftMotorSpeed = Speed;
            RightMotorSpeed = Speed;
        }

        if (LeftMotorSpeed  < 0) LeftMotorSpeed  = 0;
        if (RightMotorSpeed < 0) RightMotorSpeed = 0;
        if (LeftMotorSpeed  > 1) LeftMotorSpeed  = 1;
        if (RightMotorSpeed > 1) RightMotorSpeed = 1;
    }
    else
    {
        Speed = 0;
        LeftMotorSpeed = 0;
        RightMotorSpeed = 0;
    }
}

void UpdateMotors()
{
    analogWrite(LEFT_MOTOR, (int)(LeftMotorSpeed * 255));
    analogWrite(RIGHT_MOTOR, (int)(RightMotorSpeed * 255));
}

void CalibrateBackground()
{
    UpdateSensors();
#if TEST
    Serial.println("Background Values:");
#endif
    for (int i = 0; i < SensorsCount; i++)
    {
        SensorBackgroundValues[i] = SensorArray[i];
#if TEST
        Serial.println(SensorBackgroundValues[i]);
#endif
    }
    for (int i = 0; i < 4; i++)
    {
        digitalWrite(LED, HIGH);
        delay(100);
        digitalWrite(LED, LOW);
        delay(100);
    }
}

void CalibrateLine()
{
    UpdateSensors();
#if TEST
    Serial.println("Line Values:");
#endif
    for (int i = 0; i < SensorsCount; i++)
    {
        SensorLineValues[i] = SensorArray[i];
#if TEST
        Serial.println(SensorLineValues[i]);
#endif
    }
    for (int i = 0; i < 8; i++)
    {
        digitalWrite(LED, HIGH);
        delay(50);
        digitalWrite(LED, LOW);
        delay(50);
    }
}

void ProcessInput()
{
    if (Running)
    {
        if (digitalRead(STOP_BUTTON) == LOW)
        {
            Running = false;
            digitalWrite(LED, LOW);
        }
    }
    else
    {
        if (digitalRead(START_BUTTON) == LOW)
        {
            delay(1000);
            CountDown();
            CountDown();
            CountDown();
            Running = true;
            digitalWrite(LED, HIGH);
        }
        else if (digitalRead(BACKGROUND_CALIBRATION_BUTTON) == LOW)
            CalibrateBackground();
        else if (digitalRead(LINE_CALIBRATION_BUTTON) == LOW)
            CalibrateLine();
    }
}

void loop()
{
    ProcessInput();

    UpdateSensors();
    Process();
    UpdateMotors();

    delay(UPDATE_DELAY);
}
