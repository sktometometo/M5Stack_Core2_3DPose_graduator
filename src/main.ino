#include <M5Core2.h>
#include <driver/i2s.h>


#define CONFIG_I2S_BCK_PIN 12
#define CONFIG_I2S_LRCK_PIN 0
#define CONFIG_I2S_DATA_PIN 2
#define CONFIG_I2S_DATA_IN_PIN 34

#define Speak_I2S_NUMBER I2S_NUM_0

#define MODE_MIC 0
#define MODE_SPK 1

extern volatile float q0, q1, q2, q3;
float quaternion_base2fixed[] = {1,0,0,0};
float quaternion_print[] = {1,0,0,0};

void multipyQuaternion( const float* quat_q, const float* quat_p, float* quat_result )
{
    quat_result[0] = quat_q[0] * quat_p[0] - quat_q[1] * quat_p[1] - quat_q[2] * quat_p[2] - quat_q[3] * quat_p[3];
    quat_result[1] = quat_q[1] * quat_p[0] + quat_q[0] * quat_p[1] - quat_q[3] * quat_p[2] + quat_q[2] * quat_p[3];
    quat_result[2] = quat_q[2] * quat_p[0] + quat_q[3] * quat_p[1] + quat_q[0] * quat_p[2] - quat_q[1] * quat_p[3];
    quat_result[3] = quat_q[3] * quat_p[0] - quat_q[2] * quat_p[1] + quat_q[1] * quat_p[2] + quat_q[0] * quat_p[3];
}

void calcInverseQuaternion( const float* from, float* to )
{
    to[0] =  1.0 * from[0];
    to[1] = -1.0 * from[1];
    to[2] = -1.0 * from[2];
    to[3] = -1.0 * from[3];
}

void convertQuaternionToRPY( const float* quat, float* rpy )
{
    rpy[0] = 0;
    rpy[1] = 0;
    rpy[2] = 0;
}

void DisplayInit()
{
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(GREEN , BLACK);
    M5.Lcd.setTextSize(2);
}

void SpeakInit()
{
    M5.Axp.SetSpkEnable(true);
    InitI2SSpeakOrMic(MODE_SPK);
}

bool InitI2SSpeakOrMic(int mode)
{
    esp_err_t err = ESP_OK;

    i2s_driver_uninstall(Speak_I2S_NUMBER);
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER),
        .sample_rate = 44100,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // is fixed at 12bit, stereo, MSB
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 2,
        .dma_buf_len = 128,
    };
    if (mode == MODE_MIC)
    {
        i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM);
    }
    else
    {
        i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
        i2s_config.use_apll = false;
        i2s_config.tx_desc_auto_clear = true;
    }
    err += i2s_driver_install(Speak_I2S_NUMBER, &i2s_config, 0, NULL);
    i2s_pin_config_t tx_pin_config;

    tx_pin_config.bck_io_num = CONFIG_I2S_BCK_PIN;
    tx_pin_config.ws_io_num = CONFIG_I2S_LRCK_PIN;
    tx_pin_config.data_out_num = CONFIG_I2S_DATA_PIN;
    tx_pin_config.data_in_num = CONFIG_I2S_DATA_IN_PIN;
    err += i2s_set_pin(Speak_I2S_NUMBER, &tx_pin_config);
    err += i2s_set_clk(Speak_I2S_NUMBER, 44100, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);

    return true;
}

bool flag_hold = false;

void setup() {
    M5.begin(true,true,true,true);
    M5.IMU.Init();
    DisplayInit();

    M5.Lcd.setTextSize(2);

    M5.Lcd.setCursor(20,220);
    M5.Lcd.print("Hold");

    M5.Lcd.setCursor(150,220);
    M5.Lcd.print("Zero");
}

void loop() {
    M5.update();

    float pitch = 0.0F;
    float roll  = 0.0F;
    float yaw   = 0.0F;
    M5.IMU.getAhrsData(&pitch,&roll,&yaw);

    float quaternion_base2current[] = {1,0,0,0};
    quaternion_base2current[0] = q0;
    quaternion_base2current[1] = q1;
    quaternion_base2current[2] = q2;
    quaternion_base2current[3] = q3;

    if ( M5.BtnB.wasReleased() || M5.BtnB.pressedFor(1000, 200) ) {
        quaternion_base2fixed[0] = quaternion_base2current[0];
        quaternion_base2fixed[1] = quaternion_base2current[1];
        quaternion_base2fixed[2] = quaternion_base2current[2];
        quaternion_base2fixed[3] = quaternion_base2current[3];
    }
    if ( M5.BtnA.wasReleased() || M5.BtnA.pressedFor(1000, 200) ) {
        flag_hold = not flag_hold;
        if ( flag_hold ) {
            M5.Lcd.setTextColor(RED , BLACK);
        } else {
            M5.Lcd.setTextColor(GREEN , BLACK);
        }
        M5.Lcd.setCursor(20,220);
        M5.Lcd.print("Hold");
        M5.Lcd.setTextColor(GREEN , BLACK);
    }

    float quaternion_fixed2base[4];
    calcInverseQuaternion( quaternion_base2fixed, quaternion_fixed2base );

    float quaternion_fixed2current[4];
    multipyQuaternion( quaternion_fixed2base, quaternion_base2current, quaternion_fixed2current );

    if ( not flag_hold ) {
        quaternion_print[0] = quaternion_fixed2current[0];
        quaternion_print[1] = quaternion_fixed2current[1];
        quaternion_print[2] = quaternion_fixed2current[2];
        quaternion_print[3] = quaternion_fixed2current[3];
    }

    M5.Lcd.setCursor(0,20);
    M5.Lcd.print("Quaternions:");
    M5.Lcd.setCursor(0,42);
    M5.Lcd.printf(" %5.2f %5.2f %5.2f %5.2f",
                            quaternion_print[0],
                            quaternion_print[1],
                            quaternion_print[2],
                            quaternion_print[3] );

    M5.Lcd.setCursor(0,65);
    M5.Lcd.print("Buttons:");
    M5.Lcd.setCursor(0,87);
    M5.Lcd.printf(" A: %2d, B: %2d, C: %2d",
            M5.BtnA.isPressed(),
            M5.BtnB.isPressed(),
            M5.BtnC.isPressed() );

    delay(10);
}
