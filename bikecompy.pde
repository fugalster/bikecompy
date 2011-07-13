// BMP08 with Arduino

// DANGER: The BMP08 accepts 1.8 to 3.6 Volts – so no chance to connect it directly to 5 Volts.

// Connect VCC to VCC and GND to GND, SCL goes to analogue pin 5, SDA to analogue pin4.
// Notice! Sparkfun breakoutboard contains already 4.7K pull ups,
// If not using pre-built pull-ups:
// --> Add some pull up resistors (1K to 20K, most often something like 4.7K) between SDA, SCL and VCC finishes the setup.

// References: http://interactive-matter.org/2009/12/arduino-barometric-pressure-sensor-bmp085/ and http://news.jeelabs.org/2009/02/19/hooking-up-a-bmp085-sensor/
// Specification: http://www.bosch-sensortec.com/content/language1/downloads/BST-BMP085-DS000-05.pdf
// SparkFun breakout board: http://www.sparkfun.com/commerce/product_info.php?products_id=9694

#include "Wire.h"

//#include <LiquidCrystal.h>
//LiquidCrystal lcd(12,11,5,4,3,2);

#include <MeetAndroid.h>
MeetAndroid phone;

#define I2C_ADDRESS 0x77

const unsigned char oss = 3; //oversamplig for measurement
const unsigned char pressure_waittime[4] = { 5, 8, 14, 26 };



//just taken from the BMP085 datasheet
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





//for calculating altitude

// main loop variables
int hall_effect_pin = 2;
bool sense;
int pulse = 0;
bool old_poll_sense;
int poll_count = 0;
int max_poll_count = 5000;
float avg_speed = 0.0;
int min_pulse = 1;
const int max_phone_send = 4000; // maximum time between sending to phone in milliseconds

// function prototypes and their variables
void bmp085_get_cal_data();
long bmp085_read_ut();
long bmp085_read_up();
void bmp085_read_temperature_and_pressure(int& temperature, long& pressure);
void write_register(unsigned char r, unsigned char v);
char read_register(unsigned char r);
int read_int_register(unsigned char r);

float hall_effect_sense();
    unsigned long overflow = 0 - 1;
    unsigned long current_clock;
    unsigned long last_speed_clock = 0;
    unsigned long diff;
    float speed;
    float alpha = 0.9;
    float alpha_minus = 1 - alpha;

void phone_send_all();
    int temperature;
    long pressure;
    double altitude;

float calculate_speed(float circ_speed);
    float circ;

//void compute_altitude(double* altitude, long pressure) {
//    float exponent = 1/5.255;
//    float P_0 = 101325;


void setup() {

    Wire.begin();
    bmp085_get_cal_data();
    Serial.begin(115200);
    float diameter = 678.656;
    float circ = 3.14159 * diameter;
    pinMode(hall_effect_pin, INPUT);

}



void loop() {

    static unsigned long next_phone_send = max_phone_send;

    sense = digitalRead(hall_effect_pin);

    if ( !sense && old_poll_sense ) {
        avg_speed += hall_effect_sense();
        pulse++;
    }
    
    if ( poll_count >= max_poll_count && pulse >= min_pulse ) {
        avg_speed = avg_speed/pulse;
        phone_send_all();
        next_phone_send = millis() + max_phone_send;
        poll_count = 0;
        pulse = 0;
        avg_speed = 0;
    }
    else if ( millis() > next_phone_send ) {
        avg_speed = 0;
        phone_send_all();
        next_phone_send = millis() + max_phone_send;
        poll_count = 0;
        pulse = 0;
        avg_speed = 0;
    }

    poll_count++;
    old_poll_sense = sense;

    delayMicroseconds(200);

}



void phone_send_all() {

    bmp085_read_temperature_and_pressure(&temperature,&pressure);

    phone.send("");
    phone.send(temperature*0.1);
    phone.send(pressure*0.001);
    phone.send(pulse);
    phone.send(calculate_speed(avg_speed));

}



float hall_effect_sense() {

    current_clock = millis();                // time when fuction is called
    diff = current_clock - last_speed_clock; // time to complete one rotation of wheel

    // overflow only happens once every 50 days
    // ignore unless there's a good reason not to
    //if ( current_clock < last_speed_clock ) {
    //    diff = current_clock + (overflow - last_speed_clock);
    //}

    last_speed_clock = current_clock; // reset so it works next time

    speed = 1.0/diff;                 // speed in rotations per milliseconds

    return speed;
    //return alpha*speed + alpha_minus*avg_speed;

}



float calculate_speed(float circ_speed) {

    // return the speed in mph
    return circ_speed * circ * 2.2369 * 1000;

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
    delay(5); //longer than 4.5 ms
    return read_int_register(0xF6);

}



long bmp085_read_up() {

    write_register(0xF4, 0x34+(oss<<6));
    delay(pressure_waittime[oss]);

    unsigned char msb, lsb, xlsb;
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.send(0xF6); // register to read
    Wire.endTransmission();

    Wire.requestFrom(I2C_ADDRESS, 3); // read a byte
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


// leave this up to the phone
//void compute_altitude(double* altitude, long pressure) {
//
//    *altitude = 44330*(1-pow(float(pressure)/P_0, exponent));
//
//}

void write_register(unsigned char r, unsigned char v) {

    Wire.beginTransmission(I2C_ADDRESS);
    Wire.send(r);
    Wire.send(v);
    Wire.endTransmission();

}



char read_register(unsigned char r) {
    
    unsigned char v;

    Wire.beginTransmission(I2C_ADDRESS);
    Wire.send(r); // register to read
    Wire.endTransmission();

    Wire.requestFrom(I2C_ADDRESS, 1); // read a byte
    while(!Wire.available()) { }
    v = Wire.receive();

    return v;

}



int read_int_register(unsigned char r) {
    
    unsigned char msb, lsb;
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.send(r); // register to read
    Wire.endTransmission();

    Wire.requestFrom(I2C_ADDRESS, 2); // read a byte
    while(!Wire.available()) { }
    msb = Wire.receive();

    while(!Wire.available()) { }
    lsb = Wire.receive();

    return (((int)msb<<8) | ((int)lsb));

}
