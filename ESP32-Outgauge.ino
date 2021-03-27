#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>

#include "esp_err.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/periph_ctrl.h"
#include "soc/ledc_reg.h"

#define LEDC_TIMER_RES          LEDC_TIMER_10_BIT
#define DUTY_MAX                ((1 << LEDC_TIMER_10_BIT) -1 )

#define PWR_PIN                 2
#define H_BEAM_PIN              4
#define OIL_PIN                 18
#define BATTERY_PIN             17
#define MIL_PIN                 16
#define BRAKE_PIN               25
#define ABS_PIN                 33
#define DIMM_PIN                13
#define TURN_L_PIN              27
#define TURN_R_PIN              26
#define TEMP_PIN                19
#define RPM_PIN                 21
#define SPEED_PIN               22
#define FUEL_PIN                23

const char* host        = "ESP32";
const char* ssid        = "SSID";
const char* pwd         = "WPA";

const char* udpAddress  = "IP";
const int udpPort       = 8080;

unsigned int temp_duty_cycle = 0;
int speed_Hz = 0;
int rpm_Hz = 0;
unsigned int fuel_duty_cycle = 0;

WiFiUDP udp;

void ledc_init(uint8_t pin, float freq_Hz, ledc_channel_t channel, ledc_timer_t timer) {
    const char * ME = __func__;

    esp_err_t err;
    periph_module_enable(PERIPH_LEDC_MODULE);

    uint32_t precision = DUTY_MAX + 1;
    uint32_t div_param = ((uint64_t) LEDC_REF_CLK_HZ << 8) / freq_Hz / precision;
    if (div_param < 256 || div_param > LEDC_DIV_NUM_HSTIMER0_V)
    {
        ESP_LOGE(ME, "requested frequency and duty resolution can not be achieved, try increasing freq_hz or duty_resolution. div_param=%d", (uint32_t ) div_param);
    }
    /*
    ledc_timer_config_t ledc_timer;
    memset(&ledc_timer, 0, sizeof(ledc_timer_config_t));
    ledc_timer.duty_resolution = LEDC_TIMER_RES;
    ledc_timer.freq_hz = freq_Hz;
    ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_timer.timer_num = timer;
    
    ledc_timer_config(&ledc_timer);
    */
    ledc_channel_config_t ledc_channel = {
      .gpio_num   = pin,
      .speed_mode = LEDC_HIGH_SPEED_MODE,
      .channel    = channel,
      .intr_type  = LEDC_INTR_DISABLE,
      .timer_sel  = timer,
      .duty       = DUTY_MAX,
      .hpoint     = 0         // TODO: AD 10.11.2018: new, does 0 work (0xfffff does not work)
    };
    err = ledc_channel_config(&ledc_channel);
    ESP_LOGD(ME,"ledc_channel_config returned %d",err);
    
    err = ledc_timer_set(LEDC_HIGH_SPEED_MODE, timer, div_param, LEDC_TIMER_RES, LEDC_REF_TICK);
    if (err)
    {
        ESP_LOGE(ME, "ledc_timer_set returned %d",err);
    }
    
    ledc_timer_rst(LEDC_HIGH_SPEED_MODE, timer);
    ESP_LOGD(ME, "ledc_timer_set: divider: 0x%05x duty_resolution: %d\n", (uint32_t) div_param, LEDC_TIMER_RES);
}


void setup() {
    Serial.begin(115200);
    
    WiFi.begin(ssid, pwd);
    Serial.println("");

    unsigned int reset = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        reset++;
        if (reset > 10)
        {
            Serial.println("Restarting");
            ESP.restart();
        }
    }
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    
    udp.begin(udpPort);

    pinMode(PWR_PIN, OUTPUT);
    pinMode(TURN_L_PIN, OUTPUT);
    pinMode(TURN_R_PIN, OUTPUT);
    pinMode(H_BEAM_PIN, OUTPUT);
    pinMode(OIL_PIN, OUTPUT);
    pinMode(BATTERY_PIN, OUTPUT);
    pinMode(MIL_PIN, OUTPUT);
    pinMode(BRAKE_PIN, OUTPUT);
    pinMode(ABS_PIN, OUTPUT);
    pinMode(DIMM_PIN, OUTPUT);

    digitalWrite(TURN_L_PIN, LOW);
    digitalWrite(TURN_R_PIN, LOW);
    digitalWrite(H_BEAM_PIN, LOW);
    digitalWrite(OIL_PIN, LOW);
    digitalWrite(BATTERY_PIN, LOW);
    digitalWrite(MIL_PIN, LOW);
    digitalWrite(BRAKE_PIN, HIGH);
    digitalWrite(ABS_PIN, HIGH);
    digitalWrite(DIMM_PIN, LOW);

    ledc_init(TEMP_PIN, 2.4, LEDC_CHANNEL_0, LEDC_TIMER_0);
    ledc_init(RPM_PIN, 1, LEDC_CHANNEL_1, LEDC_TIMER_1);
    ledc_init(SPEED_PIN, 1, LEDC_CHANNEL_2, LEDC_TIMER_2);
    ledc_init(FUEL_PIN, 490, LEDC_CHANNEL_3, LEDC_TIMER_3);
    
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 211);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, 512);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
    
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, 512);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);
    
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3, DUTY_MAX);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3);
    
    delay(1000);
    uint32_t dut_0 = ledc_get_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    uint32_t dut_1 = ledc_get_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
    uint32_t dut_2 = ledc_get_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);
    uint32_t dut_3 = ledc_get_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3);
    Serial.println(dut_0, DEC);
    Serial.println(dut_1, DEC);
    Serial.println(dut_2, DEC);
    Serial.println(dut_3, DEC);

    digitalWrite(PWR_PIN, HIGH);
}

static union {
    uint32_t i;
    uint8_t b[sizeof (float)];
    float f;
} nums;

void loop(){
    char packet_buffer[128];
    int packet_size = udp.parsePacket();
    if (packet_size)
    {
        int len = udp.read(packet_buffer, 127);
        if (len > 0)
        {
            /*
            Serial.print("Received (IP/Size/Data): ");
            Serial.print(udp.remoteIP());Serial.print(" / ");
            Serial.print(packetSize);Serial.print(" / ");
            for (byte i=0; i<len;i++)
            {
                Serial.println(packet_buffer[i], DEC);
            }
            */
            /* Data Packet in detail
             * Time         [0 - 3]
             * AutoName     [4 - 7]
             * Flags        [8 - 9]
             * Gang         [10]
             * SpareB       [11]
             * Speed        [12 - 15]
             * RPM          [16 - 19]
             * Turbo        [20 - 23]
             * EngTemp      [24 - 27]
             * Fuel         [28 - 31]
             * Öldruck      [32 - 35]
             * ÖlTemp       [36 - 39]
             * Dashlight    [40 - 43]
             * ShowLights   [44 - 47]
             * Throttle     [48 - 51]
             * Brake        [52 - 55]
             * Clutch       [56 - 59]
             * Display1     [60 - 75]
             * Display2     [76 - 91]
             * ID           [92 - 95]
            */

            /* Temperature */
            /*
             * 48°C = 88ms low
             * 60°C = 144,5ms low
             * 70°C = 184,5ms low
             * 80°C = 224,5ms low
             * 90°C = 272,5ms low
             * 99°C = 304,5ms low
             * 111°C = 352,5ms low
             * 122°C = 392,5ms low
             * 
             * f(x) = mx+b;
             * ms = 4.115 * temp_degC - 109.514
             * duty cycle = ms * 2.4
            */
            
            nums.b[0] = packet_buffer[24];
            nums.b[1] = packet_buffer[25];
            nums.b[2] = packet_buffer[26];
            nums.b[3] = packet_buffer[27];
            Serial.print("Temp degC: ");
            Serial.print(nums.f);
            if (nums.f >= 129)
            {
                nums.f = 129;
            }
            temp_duty_cycle = ((4.115*nums.f)-109.514)*2.4;
            Serial.print(" @ ");
            Serial.print(temp_duty_cycle);
            Serial.println(" Duty Cycle");
            
            /* RPM */
            nums.b[0] = packet_buffer[16];
            nums.b[1] = packet_buffer[17];
            nums.b[2] = packet_buffer[18];
            nums.b[3] = packet_buffer[19];
            Serial.print("RPM: ");
            Serial.print(nums.f);
            rpm_Hz = nums.f / 28.99;
            Serial.print(" @ ");
            Serial.print(rpm_Hz);
            Serial.println(" Hz");

            /* Speed */
            nums.b[0] = packet_buffer[12];
            nums.b[1] = packet_buffer[13];
            nums.b[2] = packet_buffer[14];
            nums.b[3] = packet_buffer[15];
            nums.f = nums.f * 3.6;
            Serial.print("Speed km/h: ");
            Serial.print(nums.f);
            speed_Hz = nums.f / 1.5;
            Serial.print(" @ ");
            Serial.print(speed_Hz);
            Serial.println(" Hz");

            /* Fuel */
            nums.b[0] = packet_buffer[28];
            nums.b[1] = packet_buffer[29];
            nums.b[2] = packet_buffer[30];
            nums.b[3] = packet_buffer[31];
            Serial.print("Fuel %: ");
            nums.f = nums.f * 100.0;
            Serial.print(nums.f);
            if (nums.f > 0.0 && nums.f <= 25.0)
            {
                fuel_duty_cycle = 7.4 * nums.f;
            }
            else if (nums.f > 25.0 && nums.f <= 50.0)
            {
                fuel_duty_cycle = 2.6 * nums.f + 120;
            }
            else if (nums.f > 50.0 && nums.f <= 75.0)
            {
                fuel_duty_cycle = 4.8 * nums.f + 10;
            }
            else if (nums.f > 75.0 && nums.f <= 100.0)
            {
                fuel_duty_cycle = 26.12 * nums.f - 1598; // 1023
                //fuel_duty_cycle = 19.20 * nums.f - 1070; // 850
                //fuel_duty_cycle = 20.04 * nums.f - 1160; // 880
            }
            Serial.print(" @ ");
            Serial.print(fuel_duty_cycle);
            Serial.println(" Duty Cycle");

            /* Lights */
            nums.b[0] = packet_buffer[44];
            nums.b[1] = packet_buffer[45];
            nums.b[2] = packet_buffer[46];
            nums.b[3] = packet_buffer[47];

            if ((nums.i & 0x0040) != 0)
            {
                Serial.println("Turn right");
                digitalWrite(TURN_R_PIN, HIGH);
            }
            else
            {
                digitalWrite(TURN_R_PIN, LOW);
            }
            if ((nums.i & 0x0020) != 0)
            {
                Serial.println("Turn left");
                digitalWrite(TURN_L_PIN, HIGH);
            }
            else
            {
                digitalWrite(TURN_L_PIN, LOW);
            }

            if ((nums.i & 0x0002) != 0)
            {
                Serial.println("High beams");
                digitalWrite(H_BEAM_PIN, HIGH);
            }
            else
            {
                digitalWrite(H_BEAM_PIN, LOW);
            }

            if ((nums.i & 0x0010) != 0)
            {
                Serial.println("Traction control");
            }

            if ((nums.i & 0x0400) != 0)
            {
                Serial.println("ABS");
                digitalWrite(ABS_PIN, LOW);
            }
            else
            {
                digitalWrite(ABS_PIN, HIGH);
            }

            if ((nums.i & 0x0004) != 0)
            {
                Serial.println("Handbrake");
                digitalWrite(BRAKE_PIN, LOW);
            }
            else
            {
                digitalWrite(BRAKE_PIN, HIGH);
            }

            if ((nums.i & 0x0100) != 0)
            {
                Serial.println("Oil pressure warning");
                digitalWrite(OIL_PIN, HIGH);
            }
            else
            {
                digitalWrite(OIL_PIN, LOW);
            }

            if ((nums.i & 0x0200) != 0)
            {
                Serial.println("Battery warning");
                digitalWrite(BATTERY_PIN, HIGH);
            }
            else
            {
                digitalWrite(BATTERY_PIN, LOW);
            }
        }
        
        if (temp_duty_cycle <= DUTY_MAX)
        {
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, temp_duty_cycle);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        }

        if (rpm_Hz < 1)
        {
            rpm_Hz = 1;
        }
        ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_1, rpm_Hz);
    
        if (speed_Hz < 1)
        {
            speed_Hz = 1;
        }
        ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_2, speed_Hz);

        if (fuel_duty_cycle <= DUTY_MAX)
        {
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3, fuel_duty_cycle);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3);
        }
    
        udp.flush();
    }
}
