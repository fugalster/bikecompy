#include "Wire.h"
#include <MeetAndroid.h>
MeetAndroid phone;


// Configuration //
const int min_wheel_tick = 1; // minimum wheel ticks between sending to phone
const int min_pedal_tick = 1; // minimum pedal ticks between sending to phone
#define DATA_SEND_INTERVAL 1000
#define FORCE_SEND_INTERVAL 4000

const unsigned char oss = 3; //oversamplig for measurement
const int EOC_pin = 4;

/* be careful that bmp085 is wired to no more than 3.3V!! */
#define BMP085_ADDRESS 0x77 // i2c address of pressure sensor
//just taken from the bmp085 datasheet
int ac1;
int ac2;
int ac3;
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1;
int b2;
int mb;
int mc;
int md;

int       diameter = 679;     // my road bike, millimeters. will need to be in
                              // EEPROM and programable from the phone
const int speed_pin = 2;      // interrupt 0
const int cadence_pin = 3;    // interrupt 1

// function prototypes
void bmp085_get_cal_data();
long bmp085_read_ut();
long bmp085_read_up();
void bmp085_read_temperature_and_pressure(int& temperature, long& pressure);
void write_register(unsigned char r, unsigned char v);
char read_register(unsigned char r);
int  read_int_register(unsigned char r);
void wheel();
    volatile unsigned long wheel_time = 0;
    volatile int           wheel_tick = 0;
void pedal();
    volatile unsigned long pedal_time = 0;
    volatile int           pedal_tick = 0;
void phone_send();
void phone_send_all();
float speed();
    float circ;
float cadence();

void setup() {

    Wire.begin(); // i2c communication
    Serial.begin(115200); // bluetooth communication
    pinMode(EOC_pin, INPUT); // bmp085 end-of-conversion signal
    bmp085_get_cal_data(); // calibrate bmp085
    circ = 3.14159 * diameter;
    attachInterrupt(speed_pin-2, wheel, FALLING); // speed HES
    //attachInterrupt(cadence_pin-2, pedal, FALLING); // cadence HES
 
}

void loop() {

    static unsigned long force_phone_send = FORCE_SEND_INTERVAL;
    static unsigned long next_phone_send = DATA_SEND_INTERVAL;

    unsigned long current_millis = millis();

    if ( current_millis >= next_phone_send ) {
        if ( wheel_tick >= min_wheel_tick ) {
            phone_send();
            force_phone_send = current_millis + FORCE_SEND_INTERVAL;
        }
        next_phone_send = current_millis + DATA_SEND_INTERVAL;
    }
    else if ( current_millis > force_phone_send ) {
        phone_send();
        next_phone_send = current_millis + DATA_SEND_INTERVAL;
        force_phone_send = current_millis + FORCE_SEND_INTERVAL;
    }

}

void phone_send() {

    static int temperature;
    static long pressure;
    static double altitude;

    bmp085_read_temperature_and_pressure(&temperature,&pressure);

    phone.send("");
    phone.send(temperature*0.1);
    phone.send(pressure*0.001);
    phone.send(speed());
    //phone.send(cadence());

}

void wheel() {

    static unsigned long prev_time = 0;
    unsigned long time = millis();

    wheel_time += time - prev_time; 
    wheel_tick++;
    prev_time = time;

}

void pedal() {

    static unsigned long prev_time = 0;
    unsigned long time = millis();

    pedal_time += time - prev_time; 
    pedal_tick++;
    prev_time = time;

}

float speed() {
 
    // return the speed in m/s
    float meters_per_second = circ*wheel_tick/wheel_time; // mm/ms == m/s
    wheel_tick = 0;
    wheel_time = 0;

    return meters_per_second;
    
    // mph
    //return meters_per_second * 2.2369;

}

float cadence() {

    // return the cadence in rev/min
    float rpm = pedal_tick*1000*60/pedal_time;
    pedal_tick = 0;
    pedal_time = 0;

    return rpm;
    
}

void bmp085_get_cal_data() {

    ac1 = read_int_register(0xAA);
    ac2 = read_int_register(0xAC);
    ac3 = read_int_register(0xAE);
    ac4 = read_int_register(0xB0);
    ac5 = read_int_register(0xB2);
    ac6 = read_int_register(0xB4);
    b1 = read_int_register(0xB6);
    b2 = read_int_register(0xB8);
    mb = read_int_register(0xBA);
    mc = read_int_register(0xBC);
    md = read_int_register(0xBE);

}

long bmp085_read_ut() {

    write_register(0xF4,0x2E);
    while ( !digitalRead(EOC_pin) ) {}
    return read_int_register(0xF6);

}

long bmp085_read_up() {

    write_register(0xF4, 0x34+(oss<<6));
    while ( !digitalRead(EOC_pin) ) {}

    unsigned char msb, lsb, xlsb;
    Wire.beginTransmission(BMP085_ADDRESS);
    Wire.send(0xF6); // register to read
    Wire.endTransmission();

    Wire.requestFrom(BMP085_ADDRESS, 3); // read a byte
    while(!Wire.available()) { }

    msb = Wire.receive();
    while(!Wire.available()) { }

    lsb |= Wire.receive();
    while(!Wire.available()) { }

    xlsb |= Wire.receive();
    return (((long)msb<<16) | ((long)lsb<<8) | ((long)xlsb)) >>(8-oss);

}

void bmp085_read_temperature_and_pressure(int* temperature, long* pressure) {

    long ut= bmp085_read_ut();
    long up = bmp085_read_up();
    long x1, x2, x3, b3, b5, b6, p;
    unsigned long b4, b7;

    //calculate the temperature
    x1 = ((long)ut - ac6) * ac5 >> 15;
    x2 = ((long) mc << 11) / (x1 + md);
    b5 = x1 + x2;
    *temperature = (b5 + 8) >> 4;

    //calculate the pressure
    b6 = b5 - 4000;
    x1 = (b2 * (b6 * b6 >> 12)) >> 11;
    x2 = ac2 * b6 >> 11;
    x3 = x1 + x2;

    //b3 = (((int32_t) ac1 * 4 + x3)<> 2;

    if (oss == 3) b3 = ((int32_t) ac1 * 4 + x3 + 2) << 1;
    if (oss == 2) b3 = ((int32_t) ac1 * 4 + x3 + 2);
    if (oss == 1) b3 = ((int32_t) ac1 * 4 + x3 + 2) >> 1;
    if (oss == 0) b3 = ((int32_t) ac1 * 4 + x3 + 2) >> 2;

    x1 = ac3 * b6 >> 13;
    x2 = (b1 * (b6 * b6 >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (ac4 * (uint32_t) (x3 + 32768)) >> 15;
    b7 = ((uint32_t) up - b3) * (50000 >> oss);
    p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    *pressure = p + ((x1 + x2 + 3791) >> 4);

}

void write_register(unsigned char r, unsigned char v) {

    Wire.beginTransmission(BMP085_ADDRESS);
    Wire.send(r);
    Wire.send(v);
    Wire.endTransmission();

}

char read_register(unsigned char r) {
    
    unsigned char v;

    Wire.beginTransmission(BMP085_ADDRESS);
    Wire.send(r); // register to read
    Wire.endTransmission();

    Wire.requestFrom(BMP085_ADDRESS, 1); // read a byte
    while(!Wire.available()) { }
    v = Wire.receive();

    return v;

}

int read_int_register(unsigned char r) {
    
    unsigned char msb, lsb;
    Wire.beginTransmission(BMP085_ADDRESS);
    Wire.send(r); // register to read
    Wire.endTransmission();

    Wire.requestFrom(BMP085_ADDRESS, 2); // read a byte
    while(!Wire.available()) { }
    msb = Wire.receive();

    while(!Wire.available()) { }
    lsb = Wire.receive();

    return (((int)msb<<8) | ((int)lsb));

}
