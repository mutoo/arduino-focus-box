#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROMWearLevel.h>

// please update the layout version if more variables are added
#define EEPROM_LAYOUT_VERSION 0
// currently we have only two values, 0 for sec, 1 for min
#define AMOUNT_OF_INDEXES 2

#define INDEX_CONFIGURATION_SEC 0
#define INDEX_CONFIGURATION_MIN 1

// #define DEBUG

// rotary encoder with button
const int rotClkPin = 3;
const int rotDtPin = 4;
const int rotBtnPin = 5;

#define POS_SEC 0
#define POS_MIN 1
int prevClk = 1;
int dt[2] = {0, 0};
int tVal[2] = {0, 0};
int currentPos = 0;
bool rotBtnPrevPressed = false;
unsigned long rotBtnPressedStartTime;

// lock
const int lockTriggerPin = 12;
const int lockStatePin = 11;

// timer
char time[6];
char remaingTime[6];

// FSM
#define STATE_CHECK_LOCKED 0
#define STATE_READ_TIMER 1
#define STATE_SET_TIMER 2
#define STATE_PREPARE 3
#define STATE_COUNTDOWN 4
#define STATE_TIMES_UP 5
#define STATE_UNLOCK 6
#define STATE_RETRY 7
#define STATE_ERROR 8
// else - error;
int state = STATE_CHECK_LOCKED;
long targetTimeMS = 0;
int min, sec, rCol, buzzerIdx, attampt;
const int maxRetry = 5;

// lcd
LiquidCrystal_I2C lcd(0x27, 16, 2); // 0x27 - PCF8574A; 0x3F - PCF8574AT

void setup()
{
#ifdef DEBUG
    Serial.begin(9600);
#endif

    // setup eeprom
    EEPROMwl.begin(EEPROM_LAYOUT_VERSION, AMOUNT_OF_INDEXES);

    // setup rotray encoder
    pinMode(rotDtPin, INPUT_PULLUP);
    pinMode(rotBtnPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(rotClkPin), updateRotVal, CHANGE);

    // setup locker
    pinMode(lockStatePin, INPUT_PULLUP);
    pinMode(lockTriggerPin, OUTPUT);

#ifdef NYAN_CAT
    // create char for nyan cat
    lcd.createChar(0, nyanCatFrame0);
    lcd.createChar(1, nyanCatFrame1);
    lcd.createChar(2, nyanCatFrame2);
    lcd.createChar(3, nyanCatFrame3);
    lcd.createChar(4, nyanCatFrame4);
    lcd.createChar(5, nyanCatFrame5);
    lcd.createChar(6, nyanCatFrame6);
    lcd.createChar(7, nyanCatFrame7);
#endif

    // start lcd
    lcd.init();
    lcd.backlight();
    lcd.clear();

    // check recover from state
    state = 0;
}

void updateRotVal()
{
    delayMicroseconds(1000); // bypass some noise
    int clk = digitalRead(rotClkPin);
    if (prevClk == clk)
    {
        // there might be noise occurs to trigger interupts
        // ignore it if the state is unchanged
        return;
    }
    if (clk == LOW)
    {
        // we only focus on the falling signal
        int dtVal = digitalRead(rotDtPin);
        dt[currentPos] += dtVal == HIGH ? -1 : 1;
    }
    prevClk = clk;
}

int readValue(int pos, int min, int max)
{
#ifdef DEBUG
    if (dt[pos] != 0)
    {
        Serial.print("pos: ");
        Serial.print(pos);
        Serial.print(" dt: ");
        Serial.print(dt[pos]);
        Serial.print(" val: ");
        Serial.println(tVal[pos]);
    }
#endif
    int v = constrain(dt[pos], -4, 4);
    int delta = (v < 0 ? -1 : 1) * pow(v, 2); // increase fastly
    tVal[pos] = constrain(tVal[pos] + delta, min, max);
    dt[pos] = 0;
    return tVal[pos];
}

void loop()
{
    if (state == STATE_CHECK_LOCKED)
    {
        bool isLocked = digitalRead(lockStatePin) == LOW;
        if (isLocked)
        {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.blink();
            state = STATE_READ_TIMER;
        }
        else
        {
            lcd.setCursor(0, 0);
            lcd.print("Close the door");
            lcd.setCursor(0, 1);
            lcd.print("to start...");
        }
        delay(200);
    }
    else if (state == STATE_READ_TIMER)
    { // read timer from eeprom
        sec = EEPROMwl.read(INDEX_CONFIGURATION_SEC);
        min = EEPROMwl.read(INDEX_CONFIGURATION_MIN);
        if (sec != 0 || min != 0)
        {
            sec = sec % 60;
            min = min % 60;
            lcd.clear();
            lcd.noBlink();
            state = STATE_PREPARE;
        } else {
            state = STATE_SET_TIMER;
        }

    }
    else if (state == STATE_SET_TIMER)
    { // set timer
        min = readValue(POS_MIN, 0, 59);
        sec = readValue(POS_SEC, 0, 59);
        sprintf(time, "%02d:%02d", min, sec);
#ifdef DEBUG
        Serial.print("timer: ");
        Serial.println(time);
#endif
        lcd.setCursor(0, 0);
        lcd.print("Set Timer:");
        lcd.setCursor(0, 1);
        lcd.print(time);
        if (currentPos == POS_MIN)
        {
            lcd.setCursor(1, 1);
        }
        else
        {
            lcd.setCursor(4, 1);
        }
        delay(20);

        bool rotBtnCurrentPressed = digitalRead(rotBtnPin) == LOW;
        if (!rotBtnPrevPressed && rotBtnCurrentPressed)
        {
            rotBtnPressedStartTime = millis();
        }
        if (rotBtnPrevPressed && !rotBtnCurrentPressed)
        {
            unsigned long currentTime = millis();
            if (currentTime - rotBtnPressedStartTime >= 800)
            {
                lcd.clear();
                lcd.noBlink();
                // save the timer to eeprom
                EEPROMwl.write(INDEX_CONFIGURATION_SEC, sec);
                EEPROMwl.write(INDEX_CONFIGURATION_MIN, min);
                state = STATE_PREPARE;
            }
            else
            {
                currentPos = 1 - currentPos;
            }
        }
        rotBtnPrevPressed = rotBtnCurrentPressed;
    }
    else if (state == STATE_PREPARE)
    { // prepare
        lcd.setCursor(0, 0);
        lcd.print("Starting...");

        delay(1000);

        long currentMS = millis();
        long durationMS = (min * 60l + sec) * 1000l;

#ifdef DEBUG
        Serial.print("Current ms: ");
        Serial.println(currentMS);
        Serial.print("Min: ");
        Serial.print(min);
        Serial.print(" Sec: ");
        Serial.print(sec);
        Serial.print(" Duration in ms: ");
        Serial.println(durationMS);
        Serial.print("Set target time in ms: ");
        Serial.println(targetTimeMS);
#endif

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Counting down...");
        targetTimeMS = currentMS + durationMS;
        rCol = 0;
        state = STATE_COUNTDOWN;
    }
    else if (state == STATE_COUNTDOWN)
    { // counting down
        long diffMS = targetTimeMS - millis();

        lcd.setCursor(0, 1);
        int rMin = diffMS / 1000 / 60;
        int rSec = (diffMS % 60000) / 1000;
        sprintf(remaingTime, "%02d%c%02d", rMin, rCol ? ':' : ' ', rSec);
        lcd.print(remaingTime);
        rCol = 1 - rCol;

#ifdef NYAN_CAT
        lcd.setCursor(6, 1);
        if (rCol == 0)
        {
            lcd.write(byte(0));
            lcd.write(byte(1));
            lcd.write(byte(0));
            lcd.write(byte(1));
            lcd.write(byte(0));
            lcd.write(byte(1));
            lcd.write(byte(0));
            lcd.write(byte(2));
            lcd.write(byte(3));
            lcd.write(byte(4));
        }
        else
        {
            lcd.write(byte(1));
            lcd.write(byte(0));
            lcd.write(byte(1));
            lcd.write(byte(0));
            lcd.write(byte(1));
            lcd.write(byte(0));
            lcd.write(byte(1));
            lcd.write(byte(5));
            lcd.write(byte(6));
            lcd.write(byte(7));
        }
#endif

#ifdef DEBUG
        Serial.print("diff in ms: ");
        Serial.println(diffMS % 500);
#endif
        int delayMS = diffMS % 500;

        if (diffMS < 100)
        {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Time's up,");
            lcd.setCursor(0, 1);
            lcd.print("Click to unlock!");
            state = STATE_TIMES_UP;
        }
        else
        {
            if (rSec % 10 == 0)
            {
                // save the timer to eeprom every 10 seconds
                EEPROMwl.write(INDEX_CONFIGURATION_SEC, rSec);
                EEPROMwl.write(INDEX_CONFIGURATION_MIN, rMin);
            }

            bool isLocked = digitalRead(lockStatePin) == LOW;
            if (!isLocked)
            {
                // the lock is forced to open
                // clear the timer in the eeprom
                EEPROMwl.write(INDEX_CONFIGURATION_SEC, 0);
                EEPROMwl.write(INDEX_CONFIGURATION_MIN, 0);
                state = STATE_CHECK_LOCKED;
            }

            delay(delayMS);
        }
    }
    else if (state == STATE_TIMES_UP)
    { // time's up
        delay(100);

        int act = digitalRead(rotBtnPin);
        if (act == LOW)
        {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Unlocking...");
            state = STATE_UNLOCK;
            attampt = 0;
        }
    }
    else if (state == STATE_UNLOCK)
    { // unlocking
        attampt = attampt + 1;
        digitalWrite(lockTriggerPin, HIGH);
        delay(500);
        digitalWrite(lockTriggerPin, LOW);
        delay(1000); // waiting for the lock fully unlocked
        bool unlocked = digitalRead(lockStatePin) == HIGH;
        if (unlocked)
        {
            lcd.clear();
            lcd.setCursor(0, 0);
            // clear the timer in the eeprom
            EEPROMwl.write(INDEX_CONFIGURATION_SEC, 0);
            EEPROMwl.write(INDEX_CONFIGURATION_MIN, 0);
            state = STATE_CHECK_LOCKED;
        }
        else if (attampt <= maxRetry)
        {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Failed (");
            lcd.print(attampt);
            lcd.print("/");
            lcd.print(maxRetry);
            lcd.print("),");
            lcd.setCursor(0, 1);
            lcd.print("Click to unlock!");
            state = STATE_RETRY;
        }
        else
        {
            state = STATE_ERROR;
        }
    }
    else if (state == STATE_RETRY)
    {
        delay(100);
        int act = digitalRead(rotBtnPin);
        if (act == LOW)
        {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Unlocking...");
            state = STATE_UNLOCK;
        }
    }
    else
    { // error
        lcd.clear();
        lcd.print("Error occurs,");
        lcd.setCursor(0, 1);
        lcd.print("Please reset!");
        while (true)
            delay(200);
    }
}
