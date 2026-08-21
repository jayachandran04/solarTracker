#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/printk.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>

#define I2C_NODE DT_NODELABEL(i2c1)

#define DS1307_ADDR   0x68
#define MPU6050_ADDR  0x69

#define RAD_TO_DEG 57.2957795f

/* Complementary filter coefficient */
#define ALPHA 0.98f

/* Sampling period (100 Hz) */
#define DT 0.01f
#define PI_F           3.14159265f
#define DEG_TO_RAD_F   (PI_F / 180.0f)

/* Puducherry, India */
#define SITE_LAT       11.9139f     // degrees N
#define SITE_LON       79.8145f     // degrees E
#define TIMEZONE_OFFSET 5.5f

#define ACTUATOR_MIN   60.0f
#define ACTUATOR_MAX   130.0f
#define ACTUATOR_FLAT  95.0f   // <-- CALIBRATE: raw actuator reading when panel is perfectly flat

#define MPU_ROLL_AT_FLAT           0.13f   // from your flat reading
#define MPU_ROLL_TO_ACTUATOR_SIGN  -1.0f   // <-- CALIBRATE: verify against real end-stops

typedef enum { MOTOR_STOP, MOTOR_FORWARD, MOTOR_REVERSE } motor_cmd_t;

/* FIX: this takes roll, not pitch - roll is the axis confirmed by your end-stop test */
float mpu_current_actuator_angle(float roll_now)
{
    float delta = roll_now - MPU_ROLL_AT_FLAT;
    float angle = ACTUATOR_FLAT + (MPU_ROLL_TO_ACTUATOR_SIGN * delta);

    if (angle < ACTUATOR_MIN) angle = ACTUATOR_MIN;
    if (angle > ACTUATOR_MAX) angle = ACTUATOR_MAX;

    return angle;
}

static const struct device *i2c;

/* Filtered angles */
float roll = 0.0f;
float pitch = 0.0f;

/* Gyroscope offsets (can be calibrated later) */
static float gyro_x_offset = 0.0f;
static float gyro_y_offset = 0.0f;
static float gyro_z_offset = 0.0f;

/* Sun position */
typedef struct
{
    float elevation;   // degrees above horizon (negative = below, sun down)
    float azimuth;     // degrees clockwise from true north
    float hour_angle;  // degrees, negative=morning, 0=solar noon, positive=afternoon
} sun_pos_t;


/* ---------------- RTC Structure ---------------- */

typedef struct
{
    uint8_t sec;
    uint8_t min;
    uint8_t hour;
    uint8_t day;
    uint8_t date;
    uint8_t month;
    uint8_t year;
} rtc_time_t;

/* ---------------- MPU Structure ---------------- */

typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t temp;

    int16_t gx;
    int16_t gy;
    int16_t gz;

} mpu6050_raw_t;


/* ---------------- BCD Conversion ---------------- */

static uint8_t bcd_to_dec(uint8_t value)
{
    return ((value >> 4) * 10) + (value & 0x0F);
}

static uint8_t dec_to_bcd(uint8_t value)
{
    return ((value / 10) << 4) | (value % 10);
}


/* ---------------- DS1307 Read ---------------- */

int rtc_read(rtc_time_t *t)
{
    uint8_t reg = 0x00;
    uint8_t data[7];

    if (i2c_write_read(i2c,
                       DS1307_ADDR,
                       &reg,
                       1,
                       data,
                       7))
    {
        return -1;
    }

    t->sec   = bcd_to_dec(data[0] & 0x7F);
    t->min   = bcd_to_dec(data[1]);
    t->hour  = bcd_to_dec(data[2] & 0x3F);
    t->day   = bcd_to_dec(data[3]);
    t->date  = bcd_to_dec(data[4]);
    t->month = bcd_to_dec(data[5]);
    t->year  = bcd_to_dec(data[6]);

    return 0;
}

/* ---------------- DS1307 Write ---------------- */

int rtc_write(rtc_time_t *t)
{
    uint8_t data[8];

    data[0] = 0x00;                       /* starting register address */
    data[1] = dec_to_bcd(t->sec) & 0x7F;  /* bit7=CH, cleared here to enable oscillator */
    data[2] = dec_to_bcd(t->min);
    data[3] = dec_to_bcd(t->hour) & 0x3F; /* forces 24-hour mode */
    data[4] = dec_to_bcd(t->day);         /* day-of-week, unused by sun calc */
    data[5] = dec_to_bcd(t->date);
    data[6] = dec_to_bcd(t->month);
    data[7] = dec_to_bcd(t->year);

    return i2c_write(i2c, data, 8, DS1307_ADDR);
}

/* ---------------- Shell Command: setrtc ---------------- */

static int cmd_setrtc(const struct shell *shell, size_t argc, char **argv)
{
    if (argc != 2)
    {
        shell_error(shell, "Usage: setrtc year,mon,day,hour,min,sec");
        return -EINVAL;
    }

    int year, mon, day, hour, min, sec;

    int n = sscanf(argv[1], "%d,%d,%d,%d,%d,%d",
                   &year, &mon, &day, &hour, &min, &sec);

    if (n != 6)
    {
        shell_error(shell, "Parse error - need 6 comma-separated values");
        return -EINVAL;
    }

    if (mon < 1 || mon > 12 || day < 1 || day > 31 ||
        hour > 23 || min > 59 || sec > 59)
    {
        shell_error(shell, "Value out of range");
        return -EINVAL;
    }

    rtc_time_t t = {
        .year  = (uint8_t)(year % 100),
        .month = (uint8_t)mon,
        .date  = (uint8_t)day,
        .hour  = (uint8_t)hour,
        .min   = (uint8_t)min,
        .sec   = (uint8_t)sec,
        .day   = 1   /* day-of-week, not used by sun_calculate() */
    };

    if (rtc_write(&t) == 0)
    {
        shell_print(shell, "RTC set: 20%02d-%02d-%02d %02d:%02d:%02d",
                    t.year, t.month, t.date, t.hour, t.min, t.sec);
    }
    else
    {
        shell_error(shell, "RTC write failed - check I2C");
    }

    return 0;
}

SHELL_CMD_REGISTER(setrtc, NULL,
                    "Set RTC time. Usage: setrtc year,mon,day,hour,min,sec",
                    cmd_setrtc);


/* ---------------- MPU6050 Wakeup ---------------- */

int mpu_init(void)
{
    uint8_t buf[2];

    buf[0] = 0x6B;
    buf[1] = 0x00;

    return i2c_write(i2c,
                     buf,
                     2,
                     MPU6050_ADDR);
}
/* ---------------- MPU6050 Read Raw Data ---------------- */

int mpu_read_raw(mpu6050_raw_t *raw)
{
    uint8_t reg = 0x3B;
    uint8_t data[14];

    if (i2c_write_read(i2c,
                       MPU6050_ADDR,
                       &reg,
                       1,
                       data,
                       14))
    {
        return -1;
    }

    raw->ax = (int16_t)((data[0] << 8) | data[1]);
    raw->ay = (int16_t)((data[2] << 8) | data[3]);
    raw->az = (int16_t)((data[4] << 8) | data[5]);

    raw->temp = (int16_t)((data[6] << 8) | data[7]);

    raw->gx = (int16_t)((data[8] << 8) | data[9]);
    raw->gy = (int16_t)((data[10] << 8) | data[11]);
    raw->gz = (int16_t)((data[12] << 8) | data[13]);

    return 0;
}


/* ---------------- Convert Raw Data ---------------- */

void calculate_angles(mpu6050_raw_t *raw,
                      float *roll_angle,
                      float *pitch_angle)
{
    /* Accelerometer values in g */
    float ax = raw->ax / 16384.0f;
    float ay = raw->ay / 16384.0f;
    float az = raw->az / 16384.0f;

    /* Gyroscope values in deg/s */
    float gx = (raw->gx / 131.0f) - gyro_x_offset;
    float gy = (raw->gy / 131.0f) - gyro_y_offset;

    /* Accelerometer angles */
    float accel_roll =
        atan2f(ay, az) * RAD_TO_DEG;

    float accel_pitch =
        atan2f(-ax,
               sqrtf((ay * ay) + (az * az)))
        * RAD_TO_DEG;

    /* Complementary Filter */

    *roll_angle =
        ALPHA *
        (*roll_angle + gx * DT)
        +
        (1.0f - ALPHA)
        *
        accel_roll;

    *pitch_angle =
        ALPHA *
        (*pitch_angle + gy * DT)
        +
        (1.0f - ALPHA)
        *
        accel_pitch;
}


/* ---------------- Temperature ---------------- */

float mpu_temperature(mpu6050_raw_t *raw)
{
    return (raw->temp / 340.0f) + 36.53f;
}


/* ---------------- Print Data ---------------- */

void print_status(rtc_time_t *rtc,
                  mpu6050_raw_t *raw)
{
    float temp = mpu_temperature(raw);

    printk("\n========================================\n");

    printk("Date : %02d/%02d/20%02d\n",
           rtc->date,
           rtc->month,
           rtc->year);

    printk("Time : %02d:%02d:%02d\n",
           rtc->hour,
           rtc->min,
           rtc->sec);

    printk("\n");

    printf("Roll x100  : %d\n", (int)(roll * 100));
    printf("Pitch x100 : %d\n", (int)(pitch * 100));
    printk("\n");

    printf("Temp x100  : %d\n", (int)(temp * 100));
    printk("\n");

    printk("Accel X : %6d\n", raw->ax);
    printk("Accel Y : %6d\n", raw->ay);
    printk("Accel Z : %6d\n", raw->az);

    printk("\n");

    printk("Gyro X : %6d\n", raw->gx);
    printk("Gyro Y : %6d\n", raw->gy);
    printk("Gyro Z : %6d\n", raw->gz);

    printk("========================================\n");
}

/* Day-of-year from date, using RTC's 2-digit year (assumes 20xx) */
static int day_of_year(uint8_t day_of_month, uint8_t month, uint8_t year_2digit)
{
    static const int cum_days[12] =
        {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

    int year = 2000 + year_2digit;
    int doy = cum_days[month - 1] + day_of_month;

    /* Leap year correction (valid 2000-2099) */
    int is_leap = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
    if (is_leap && month > 2)
    {
        doy += 1;
    }

    return doy;
}


void sun_calculate(rtc_time_t *t, sun_pos_t *out)
{
    int doy = day_of_year(t->date, t->month, t->year);

    float hour_decimal = t->hour + (t->min / 60.0f) + (t->sec / 3600.0f);

    /* Fractional year gamma, in radians */
    float gamma = (2.0f * PI_F / 365.0f) *
                  (doy - 1 + (hour_decimal - 12.0f) / 24.0f);

    /* Equation of time, in minutes */
    float eqtime = 229.18f * (0.000075f
                  + 0.001868f * cosf(gamma)
                  - 0.032077f * sinf(gamma)
                  - 0.014615f * cosf(2.0f * gamma)
                  - 0.040849f * sinf(2.0f * gamma));

    /* Solar declination, in radians */
    float decl = 0.006918f
                - 0.399912f * cosf(gamma)
                + 0.070257f * sinf(gamma)
                - 0.006758f * cosf(2.0f * gamma)
                + 0.000907f * sinf(2.0f * gamma)
                - 0.002697f * cosf(3.0f * gamma)
                + 0.001480f * sinf(3.0f * gamma);

    /* True solar time, in minutes */
    float time_offset = eqtime + 4.0f * SITE_LON - 60.0f * TIMEZONE_OFFSET;
    float tst = hour_decimal * 60.0f + time_offset;

    /* Hour angle, in degrees (-180 to 180, 0 = solar noon) */
    float ha_deg = (tst / 4.0f) - 180.0f;
    float ha_rad = ha_deg * DEG_TO_RAD_F;

    out->hour_angle = ha_deg;

    float lat_rad = SITE_LAT * DEG_TO_RAD_F;

    /* Solar zenith angle */
    float cos_zenith = sinf(lat_rad) * sinf(decl)
                      + cosf(lat_rad) * cosf(decl) * cosf(ha_rad);

    if (cos_zenith > 1.0f)  cos_zenith = 1.0f;
    if (cos_zenith < -1.0f) cos_zenith = -1.0f;

    float zenith_rad = acosf(cos_zenith);

    out->elevation = 90.0f - (zenith_rad / DEG_TO_RAD_F);

    /* Azimuth (clockwise from north, 0-360) */
    float cos_az = (sinf(decl) - sinf(lat_rad) * cos_zenith)
                   / (cosf(lat_rad) * sinf(zenith_rad));

    if (cos_az > 1.0f)  cos_az = 1.0f;
    if (cos_az < -1.0f) cos_az = -1.0f;

    float az_rad = acosf(cos_az);
    float az_deg = az_rad / DEG_TO_RAD_F;

    if (ha_deg > 0.0f)
    {
        out->azimuth = 360.0f - az_deg;
    }
    else
    {
        out->azimuth = az_deg;
    }
}

/* Below this elevation, tracking gains negligible energy but costs motor wear -
 * stop actively chasing the sun and hold position instead. Tune to taste. */
#define ELEVATION_CUTOFF   5.0f

/* Hour angle runs ~-120 (sunrise) to 0 (noon) to ~+120 (sunset) per your
 * clock-position description (60 -> 180 -> wraps to -60/300). This scale
 * compresses that ~120 deg half-swing into the actuator's real half-range
 * (ACTUATOR_MAX - ACTUATOR_FLAT). <-- CALIBRATE: 120 is approximate: if your
 * actuator hits its end-stop noticeably before/after true sunrise-sunset,
 * adjust this number up/down to match. */
#define HOUR_ANGLE_HALF_SWING  120.0f
#define HOUR_ANGLE_SCALE  ((ACTUATOR_MAX - ACTUATOR_FLAT) / HOUR_ANGLE_HALF_SWING)

float sun_to_actuator_angle(sun_pos_t *sun)
{
    float actuator_angle;

    if (sun->elevation < ELEVATION_CUTOFF)
    {
        /* Sun near/below horizon - hold at whichever end-stop matches
         * morning (before solar noon) or evening (after solar noon),
         * instead of always defaulting to one side. */
        actuator_angle = (sun->hour_angle < 0.0f) ? ACTUATOR_FLAT : ACTUATOR_FLAT;
    }
    else
    {
        actuator_angle = ACTUATOR_FLAT + (sun->hour_angle * HOUR_ANGLE_SCALE);

        if (actuator_angle < ACTUATOR_MIN) actuator_angle = ACTUATOR_FLAT;
        if (actuator_angle > ACTUATOR_MAX) actuator_angle = ACTUATOR_FLAT;
    }

    return actuator_angle;
}

motor_cmd_t tracker_control_step(float target_angle, float current_angle)
{
    int target_deg  = (int)roundf(target_angle);
    int current_deg = (int)roundf(current_angle);

    if (target_deg == current_deg)
    {
        return MOTOR_STOP;
    }
    else if (target_deg > current_deg)
    {
        return MOTOR_FORWARD;   // <-- CALIBRATE: confirm this drives toward 130 (sunset), not away
    }
    else
    {
        return MOTOR_REVERSE;
    }
}

/* ---------------- SmartElex 30S Motor Driver (DIR=PB15, PWM=PA0) ---------------- */

#define MOTOR_DIR_PORT_NODE   DT_NODELABEL(gpiob)
#define MOTOR_DIR_PIN         15

#define MOTOR_PWM_PERIOD_NS   PWM_KHZ(1)   // 1 kHz - fine for a DC brushed driver, adjust if the datasheet specifies otherwise
#define MOTOR_SPEED_PERCENT   70            // fixed drive duty (0-100) - bang-bang doesn't need variable speed

static const struct device *motor_dir_port;
static const struct pwm_dt_spec motor_pwm = PWM_DT_SPEC_GET(DT_ALIAS(motor_pwm));

int motor_init(void)
{
    motor_dir_port = DEVICE_DT_GET(MOTOR_DIR_PORT_NODE);

    if (!device_is_ready(motor_dir_port))
    {
        printk("Motor DIR GPIO port not ready!\n");
        return -1;
    }

    if (!device_is_ready(motor_pwm.dev))
    {
        printk("Motor PWM device not ready!\n");
        return -1;
    }

    return gpio_pin_configure(motor_dir_port, MOTOR_DIR_PIN, GPIO_OUTPUT_INACTIVE);
}

void motor_drive(motor_cmd_t cmd)
{
    uint32_t pulse = (MOTOR_PWM_PERIOD_NS * MOTOR_SPEED_PERCENT) / 100;
    int ret;

    switch (cmd)
    {
        case MOTOR_FORWARD:
            gpio_pin_set(motor_dir_port, MOTOR_DIR_PIN, 1);   /* <-- CALIBRATE: confirm this drives toward 130 deg */
            ret = pwm_set_dt(&motor_pwm, MOTOR_PWM_PERIOD_NS, pulse);
            if (ret) printk("pwm_set_dt FORWARD failed: %d\n", ret);
            break;

        case MOTOR_REVERSE:
            gpio_pin_set(motor_dir_port, MOTOR_DIR_PIN, 0);
            ret = pwm_set_dt(&motor_pwm, MOTOR_PWM_PERIOD_NS, pulse);
            if (ret) printk("pwm_set_dt REVERSE failed: %d\n", ret);
            break;

        case MOTOR_STOP:
        default:
            ret = pwm_set_dt(&motor_pwm, MOTOR_PWM_PERIOD_NS, 0);   /* 0% duty - motor off */
            if (ret) printk("pwm_set_dt STOP failed: %d\n", ret);
            break;
    }
}

/* ---------------- Main ---------------- */

int main(void)
{
    rtc_time_t rtc;
    mpu6050_raw_t raw;

    i2c = DEVICE_DT_GET(I2C_NODE);

    if (!device_is_ready(i2c))
    {
        printk("I2C device not ready!\n");
        return 0;
    }

    printk("\n");
    printk("=====================================\n");
    printk(" DS1307 + MPU6050 Demo\n");
    printk(" STM32F411E-DISCO + Zephyr\n");
    printk("=====================================\n");

    /* Wake up MPU6050 */

    if (mpu_init())
    {
        printk("Failed to initialize MPU6050\n");
        return 0;
    }

    printk("MPU6050 Initialized\n");

    if (motor_init())
    {
        printk("Failed to initialize motor driver\n");
        return 0;
    }

    printk("Motor Driver Initialized\n");

    /* Allow sensor to stabilize */

    k_sleep(K_MSEC(500));

    /* -------- Gyroscope Calibration -------- */

    printk("Calibrating Gyroscope...\n");

    float gx_sum = 0.0f;
    float gy_sum = 0.0f;
    float gz_sum = 0.0f;

    for (int i = 0; i < 500; i++)
    {
        if (mpu_read_raw(&raw) == 0)
        {
            gx_sum += raw.gx;
            gy_sum += raw.gy;
            gz_sum += raw.gz;
        }

        k_sleep(K_MSEC(2));
    }

    gyro_x_offset = (gx_sum / 500.0f) / 131.0f;
    gyro_y_offset = (gy_sum / 500.0f) / 131.0f;
    gyro_z_offset = (gz_sum / 500.0f) / 131.0f;

    printk("Calibration Complete\n");

    /* Initialize complementary filter */

    if (mpu_read_raw(&raw) == 0)
    {
        float ax = raw.ax / 16384.0f;
        float ay = raw.ay / 16384.0f;
        float az = raw.az / 16384.0f;

        roll =
            atan2f(ay, az) * RAD_TO_DEG;

        pitch =
            atan2f(-ax,
                   sqrtf(ay * ay + az * az))
            * RAD_TO_DEG;
    }

    int counter = 0;

    while (1)
    {
        if (rtc_read(&rtc) == 0 &&
            mpu_read_raw(&raw) == 0)
        {
            calculate_angles(&raw,
                             &roll,
                             &pitch);

            /* Print every 1 second */

            counter++;

            if (counter >= 100)
            {
                counter = 0;

                sun_pos_t sun;
                sun_calculate(&rtc, &sun);

                print_status(&rtc, &raw);

                float target_angle = sun_to_actuator_angle(&sun);

                printf("Target Actuator Angle x100 : %d\n", (int)(target_angle * 100));

                printf("Sun Elev x100 : %d\n", (int)(sun.elevation * 100));
                printf("Sun Azim x100 : %d\n", (int)(sun.azimuth * 100));

                /* FIX: use roll, not pitch - confirmed by end-stop test */
                float current_angle = mpu_current_actuator_angle(roll);

                printf("Current Actuator Angle x100 : %d\n", (int)(current_angle * 100));

                motor_cmd_t cmd = tracker_control_step(target_angle, current_angle);

                motor_drive(cmd);
            }
        }

        /* Complementary filter update at 100 Hz */

        k_sleep(K_MSEC(10));
    }

    return 0;
}
