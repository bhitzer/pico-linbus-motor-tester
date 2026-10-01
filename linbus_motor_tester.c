#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

#define UART_ID uart0
#define BAUD_RATE 9600
#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define MCP2004_EN_PIN 2
#define MCP2004_NSLP_PIN 3

#define MAX_MOTORS 13
#define WINDOW_SPEC 100
#define MOT_VER_SPEC 7

#define NORMAL_MODE '\xFC'
#define SERVICE_MODE '\xFD'
#define STOP_MODE '\xFE'

static const char Z3RH[] = "\x2B\x25\x27\x2F\x2E\x28\x2A\x24\x26\x31\x33\x22\x00";
static const char Z3LH[] = "\x22\x33\x2B\x25\x27\x2F\x2E\x31\x26\x24\x2A\x28\x00";
static const char Z2RH[] = "\x2B\x2A\x25\x24\x22\x00";
static const char Z2LH[] = "\x22\x2B\x2A\x25\x24\x00";

static const int Z3RHOSpec[12] = {2065,1528,1353,1398,1395,823,2071,1532,1338,1841,1614,1731};
static const int Z3RHCSpec[12] = {2084,1528,1347,1396,1389,822,2084,1532,1332,1843,1615,1753};
static const int Z3LHOSpec[12] = {1711,1572,2070,1524,1353,1396,1396,1837,1336,1537,2070,818};
static const int Z3LHCSpec[12] = {1721,1581,2081,1525,1349,1393,1388,1839,1335,1536,2079,817};
static const int Z2RHOSpec[5] = {1889,1870,5488,1533,2237};
static const int Z2RHCSpec[5] = {1904,1879,5479,1529,2247};
static const int Z2LHOSpec[5] = {2289,1895,1879,5489,1527};
static const int Z2LHCSpec[5] = {2301,1907,1888,5482,1522};

static const char Z3RHDir[12] = {0x00,0x00,0x01,0x00,0x01,0x01,0x00,0x01,0x00,0x01,0x01,0x00};
static const char Z3LHDir[12] = {0x01,0x00,0x00,0x00,0x01,0x00,0x01,0x01,0x00,0x01,0x00,0x01};
static const char Z2RHDir[5] = {0x00,0x01,0x01,0x01,0x00};
static const char Z2LHDir[5] = {0x01,0x00,0x01,0x01,0x01};

static const char Z3RHSafeDir[12] = {0x01,0x01,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x00,0x00,0x00};
static const char Z3LHSafeDir[12] = {0x00,0x00,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00};
static const char Z2RHSafeDir[5] = {0x01,0x01,0x00,0x01,0x00};
static const char Z2LHSafeDir[5] = {0x00,0x01,0x01,0x00,0x01};

static const char Z3RHSafeEnable[12] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00};
static const char Z3LHSafeEnable[12] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00};
static const char Z2RHSafeEnable[5] = {0x00,0x00,0x00,0x00,0x01};
static const char Z2LHSafeEnable[5] = {0x01,0x00,0x00,0x00,0x00};

static int CurOSpec[MAX_MOTORS];
static int CurCSpec[MAX_MOTORS];
static int ResultOpen[MAX_MOTORS];
static int ResultClose[MAX_MOTORS];
static int MotVer[MAX_MOTORS];
static char MotModeStat[MAX_MOTORS];
static char MotDirStat[MAX_MOTORS];
static char MotSafeStat[MAX_MOTORS];
static char MotSafeDirStat[MAX_MOTORS];
static int MotPosOk[MAX_MOTORS];
static int MotOpenOk[MAX_MOTORS];
static int MotCloseOk[MAX_MOTORS];
static int MotVerOk[MAX_MOTORS];
static int MotDirOk[MAX_MOTORS];
static int MotSafeOk[MAX_MOTORS];
static int MotSafeDirOk[MAX_MOTORS];

static void lin_uart_init(void) {
    uart_init(UART_ID, BAUD_RATE);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    gpio_init(MCP2004_EN_PIN);
    gpio_init(MCP2004_NSLP_PIN);
    gpio_set_dir(MCP2004_EN_PIN, GPIO_OUT);
    gpio_set_dir(MCP2004_NSLP_PIN, GPIO_OUT);

    gpio_put(MCP2004_EN_PIN, 1);
    gpio_put(MCP2004_NSLP_PIN, 1);
}

static void lin_send_break(void) {
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(UART_TX_PIN, GPIO_OUT);
    gpio_put(UART_TX_PIN, 0);
    sleep_us(13000);
    gpio_put(UART_TX_PIN, 1);
    sleep_us(1000);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
}

static uint8_t lin_checksum_payload(const uint8_t *data, size_t len) {
    uint16_t sum = 0;
    for (size_t i = 0; i < len; ++i) {
        sum += data[i];
    }
    return (uint8_t)(0xFF - (sum & 0xFF));
}

static int lin_send_frame(const uint8_t *payload, size_t len, uint8_t *response, size_t response_cap, bool expect_response) {
    uint8_t frame[32];
    size_t frame_len = 0;

    frame[frame_len++] = 0x55;
    for (size_t i = 0; i < len; ++i) {
        frame[frame_len++] = payload[i];
    }
    frame[frame_len++] = lin_checksum_payload(payload, len);

    lin_send_break();
    for (size_t i = 0; i < frame_len; ++i) {
        uart_putc_raw(UART_ID, frame[i]);
    }

    sleep_us(15000);

    if (!expect_response) {
        return 0;
    }

    size_t r = 0;
    absolute_time_t end = make_timeout_time_us(500000);
    while (!time_reached(end) && r < response_cap) {
        if (uart_is_readable(UART_ID)) {
            response[r++] = uart_getc(UART_ID);
        }
    }
    return (int)r;
}

static void motor_open(char addr) {
    uint8_t data1[5] = {0xEC, (uint8_t)addr, 0xD0, 0x87, 0xFF};
    uint8_t data2[5] = {0x6F, (uint8_t)addr, 0xFF, 0xFF, 0x10};
    uint8_t dummy[16] = {0};
    lin_send_frame(data1, 5, dummy, sizeof(dummy), false);
    sleep_ms(10);
    lin_send_frame(data2, 5, dummy, sizeof(dummy), false);
}

static void motor_open_all(const char *addrs) {
    size_t len = strlen(addrs);
    for (size_t i = 0; i < len; ++i) {
        motor_open(addrs[i]);
        sleep_ms(50);
    }
}

static void motor_close(char addr) {
    uint8_t data1[5] = {0xEC, (uint8_t)addr, 0xFF, 0xFF, 0xFF};
    uint8_t data2[5] = {0x6F, (uint8_t)addr, 0xD0, 0x87, 0x10};
    uint8_t dummy[16] = {0};
    lin_send_frame(data1, 5, dummy, sizeof(dummy), false);
    sleep_ms(10);
    lin_send_frame(data2, 5, dummy, sizeof(dummy), false);
}

static void motor_close_all(const char *addrs) {
    size_t len = strlen(addrs);
    for (size_t i = 0; i < len; ++i) {
        motor_close(addrs[i]);
        sleep_ms(50);
    }
}

static void motor_init_pos(char addr) {
    uint8_t data[5] = {0xEC, (uint8_t)addr, 0xFF, 0xFF, 0xFF};
    uint8_t dummy[16] = {0};
    lin_send_frame(data, 5, dummy, sizeof(dummy), false);
}

static int motor_get_pos(char addr) {
    uint8_t req1[3] = {0x5B, (uint8_t)addr, 0xFF};
    uint8_t req2[1] = {0xA3};
    uint8_t resp[16] = {0};
    int n = lin_send_frame(req1, sizeof(req1), resp, sizeof(resp), false);
    (void)n;
    sleep_ms(10);
    int r = lin_send_frame(req2, sizeof(req2), resp, sizeof(resp), true);
    if (r >= 5) {
        return ((int)resp[4] << 8) | resp[3];
    }
    return 0;
}

static int motor_await_block(char addr) {
    uint8_t req1[3] = {0x5B, (uint8_t)addr, 0xFF};
    uint8_t req2[1] = {0xA3};
    uint8_t resp[16] = {0};
    absolute_time_t end = make_timeout_time_us(500000);
    while (!time_reached(end)) {
        lin_send_frame(req1, sizeof(req1), resp, sizeof(resp), false);
        sleep_ms(10);
        int r = lin_send_frame(req2, sizeof(req2), resp, sizeof(resp), true);
        if (r >= 6) {
            uint8_t status = resp[5];
            if (status == 0x40) {
                return ((int)resp[4] << 8) | resp[3];
            }
        }
        sleep_ms(50);
    }
    return 0;
}

static int *motor_stall_check(const char *addrs, char direction, int timeout_seconds) {
    static int positions[MAX_MOTORS];
    size_t len = strlen(addrs);
    memset(positions, 0, sizeof(positions));

    absolute_time_t end = make_timeout_time_us((uint64_t)timeout_seconds * 1000000ULL);
    int done = 0;

    while (!time_reached(end) && !done) {
        done = 1;
        for (size_t i = 0; i < len; ++i) {
            uint8_t req1[3] = {0x5B, (uint8_t)addrs[i], 0xFF};
            uint8_t req2[1] = {0xA3};
            uint8_t resp[16] = {0};
            lin_send_frame(req1, sizeof(req1), resp, sizeof(resp), false);
            sleep_ms(10);
            int r = lin_send_frame(req2, sizeof(req2), resp, sizeof(resp), true);
            if (r >= 6 && resp[5] == 0x40) {
                int pos = ((int)resp[4] << 8) | resp[3];
                positions[i] = (direction == 'O') ? (pos - 34768) : (65535 - pos);
            } else {
                positions[i] = 0;
                done = 0;
            }
            sleep_ms(50);
        }
    }
    return positions;
}

static void motor_set_mode(char mode) {
    uint8_t data[5] = {0x2E, 0x20, (uint8_t)mode, 0xFF, 0xFF};
    uint8_t dummy[16] = {0};
    lin_send_frame(data, sizeof(data), dummy, sizeof(dummy), false);
}

static void motor_addr_service(char addr) {
    uint8_t data[3] = {0x97, 0x20, (uint8_t)addr};
    uint8_t dummy[16] = {0};
    lin_send_frame(data, sizeof(data), dummy, sizeof(dummy), false);
}

static void motor_program(char addr, char version, char mode_byte) {
    uint8_t data[5] = {0xA8, (uint8_t)addr, (uint8_t)addr, (uint8_t)mode_byte, (uint8_t)version};
    uint8_t dummy[16] = {0};
    lin_send_frame(data, sizeof(data), dummy, sizeof(dummy), false);
}

static int motor_get_status(char addr, char *status_buf) {
    uint8_t req1[3] = {0xD8, (uint8_t)addr, 0xFF};
    uint8_t req2[1] = {0x20};
    uint8_t resp[16] = {0};
    lin_send_frame(req1, sizeof(req1), resp, sizeof(resp), false);
    sleep_ms(10);
    int r = lin_send_frame(req2, sizeof(req2), resp, sizeof(resp), true);
    if (r >= 7) {
        memcpy(status_buf, resp, (size_t)r);
        return r;
    }
    return 0;
}

static void motor_get_status_all(const char *addrs) {
    size_t len = strlen(addrs);
    for (size_t i = 0; i < len; ++i) {
        char status[16] = {0};
        int r = motor_get_status(addrs[i], status);
        if (r >= 6) {
            MotVer[i] = (unsigned char)status[5];
            MotModeStat[i] = status[4];
        }
        sleep_ms(50);
    }
}

static char motor_mode_byte(int model, int motor_idx) {
    char mode = 0xC0;
    switch (model) {
        case 20:
            mode |= (Z2LHDir[motor_idx] << 2);
            mode |= Z2LHSafeDir[motor_idx];
            mode |= (Z2LHSafeEnable[motor_idx] << 1);
            break;
        case 21:
            mode |= (Z2RHDir[motor_idx] << 2);
            mode |= Z2RHSafeDir[motor_idx];
            mode |= (Z2RHSafeEnable[motor_idx] << 1);
            break;
        case 30:
            mode |= (Z3LHDir[motor_idx] << 2);
            mode |= Z3LHSafeDir[motor_idx];
            mode |= (Z3LHSafeEnable[motor_idx] << 1);
            break;
        case 31:
            mode |= (Z3RHDir[motor_idx] << 2);
            mode |= Z3RHSafeDir[motor_idx];
            mode |= (Z3RHSafeEnable[motor_idx] << 1);
            break;
        default:
            break;
    }
    return mode;
}

static void motor_addr_safe_mode(const char *addrs, int model) {
    size_t len = strlen(addrs);
    char status[16] = {0};
    int retry = 0;

retry_addr:
    motor_set_mode(SERVICE_MODE);
    sleep_ms(100);

    for (size_t i = 0; i < len; ++i) {
        motor_addr_service(addrs[i]);
        sleep_ms(50);
        int r = motor_get_status(addrs[i], status);
        if (r < 5) {
            if (retry < 5) {
                retry++;
                goto retry_addr;
            }
            goto end_safe;
        }
    }

    retry = 0;

retry_prg:
    for (size_t i = 0; i < len; ++i) {
        char mode = motor_mode_byte(model, (int)i);
        motor_program(addrs[i], 0x07, mode);
        sleep_ms(50);
        int r = motor_get_status(addrs[i], status);
        if (r < 5) {
            if (retry < 5) {
                retry++;
                goto retry_prg;
            }
            goto end_safe;
        }
    }

end_safe:
    motor_set_mode(NORMAL_MODE);
    sleep_ms(100);
}

static int motor_validate_results(const char *addrs, int model) {
    size_t len = strlen(addrs);
    int pass_count = 0;
    for (size_t i = 0; i < len; ++i) {
        MotPosOk[i] = 1;
        MotOpenOk[i] = 1;
        MotCloseOk[i] = 1;
        MotVerOk[i] = 1;
        MotDirOk[i] = 1;
        MotSafeOk[i] = 1;
        MotSafeDirOk[i] = 1;

        if (ResultOpen[i] < CurOSpec[i] - WINDOW_SPEC || ResultOpen[i] > CurOSpec[i] + WINDOW_SPEC) {
            MotPosOk[i] = 0;
            MotOpenOk[i] = 0;
        }
        if (ResultClose[i] < CurCSpec[i] - WINDOW_SPEC || ResultClose[i] > CurCSpec[i] + WINDOW_SPEC) {
            MotPosOk[i] = 0;
            MotCloseOk[i] = 0;
        }
        if (MotVer[i] != MOT_VER_SPEC) {
            MotPosOk[i] = 0;
            MotVerOk[i] = 0;
        }

        MotDirStat[i] = (MotModeStat[i] >> 2) & 0x01;
        MotSafeStat[i] = (MotModeStat[i] >> 1) & 0x01;
        MotSafeDirStat[i] = MotModeStat[i] & 0x01;

        switch (model) {
            case 20:
                if (Z2LHDir[i] != MotDirStat[i]) MotDirOk[i] = 0;
                if (Z2LHSafeDir[i] != MotSafeDirStat[i]) MotSafeDirOk[i] = 0;
                if (Z2LHSafeEnable[i] != MotSafeStat[i]) MotSafeOk[i] = 0;
                break;
            case 21:
                if (Z2RHDir[i] != MotDirStat[i]) MotDirOk[i] = 0;
                if (Z2RHSafeDir[i] != MotSafeDirStat[i]) MotSafeDirOk[i] = 0;
                if (Z2RHSafeEnable[i] != MotSafeStat[i]) MotSafeOk[i] = 0;
                break;
            case 30:
                if (Z3LHDir[i] != MotDirStat[i]) MotDirOk[i] = 0;
                if (Z3LHSafeDir[i] != MotSafeDirStat[i]) MotSafeDirOk[i] = 0;
                if (Z3LHSafeEnable[i] != MotSafeStat[i]) MotSafeOk[i] = 0;
                break;
            case 31:
                if (Z3RHDir[i] != MotDirStat[i]) MotDirOk[i] = 0;
                if (Z3RHSafeDir[i] != MotSafeDirStat[i]) MotSafeDirOk[i] = 0;
                if (Z3RHSafeEnable[i] != MotSafeStat[i]) MotSafeOk[i] = 0;
                break;
            default:
                break;
        }

        if (MotSafeOk[i] == 0 || MotSafeDirOk[i] == 0 || MotDirOk[i] == 0) {
            MotPosOk[i] = 0;
        }

        if (MotPosOk[i]) {
            pass_count++;
        }
    }
    return pass_count;
}

static void run_motor_test(const char *addrs, int model) {
    size_t len = strlen(addrs);
    int *positions;
    int pass_count;
    int fail_count;

    printf("\r\n=== Starting motor test ===\r\n");
    printf("Model=%d, motor_count=%zu\r\n", model, len);

    motor_get_status_all(addrs);
    sleep_ms(100);

    printf("Close all and settle...\r\n");
    motor_close_all(addrs);
    positions = motor_stall_check(addrs, 'C', 20);
    for (size_t i = 0; i < len; ++i) {
        ResultClose[i] = positions[i];
    }
    sleep_ms(1000);

    printf("Open all and measure...\r\n");
    motor_open_all(addrs);
    positions = motor_stall_check(addrs, 'O', 20);
    for (size_t i = 0; i < len; ++i) {
        ResultOpen[i] = positions[i];
    }
    sleep_ms(1000);

    printf("Open results: ");
    for (size_t i = 0; i < len; ++i) {
        printf("%d ", ResultOpen[i]);
    }
    printf("\r\n");

    printf("Close results: ");
    for (size_t i = 0; i < len; ++i) {
        printf("%d ", ResultClose[i]);
    }
    printf("\r\n");

    pass_count = motor_validate_results(addrs, model);
    fail_count = (int)len - pass_count;
    printf("Passed: %d, Failed: %d\r\n", pass_count, fail_count);

    for (size_t i = 0; i < len; ++i) {
        printf("Motor %02X: Open=%d Close=%d Status=%s\r\n",
            (unsigned char)addrs[i], ResultOpen[i], ResultClose[i], MotPosOk[i] ? "PASS" : "FAIL");
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);

    printf("\r\n=== Pico LIN Motor Tester ===\r\n");
    printf("Using UART0 on GPIO0 (TX) and GPIO1 (RX)\r\n");

    lin_uart_init();
    sleep_ms(500);

    printf("Tests: 1=Z2LH, 2=Z2RH, 3=Z3LH, 4=Z3RH\r\n");

    while (true) {
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            switch (c) {
                case '1':
                    for (int i = 0; i < 5; ++i) {
                        CurOSpec[i] = Z2LHOSpec[i];
                        CurCSpec[i] = Z2LHCSpec[i];
                    }
                    run_motor_test(Z2LH, 20);
                    break;
                case '2':
                    for (int i = 0; i < 5; ++i) {
                        CurOSpec[i] = Z2RHOSpec[i];
                        CurCSpec[i] = Z2RHCSpec[i];
                    }
                    run_motor_test(Z2RH, 21);
                    break;
                case '3':
                    for (int i = 0; i < 12; ++i) {
                        CurOSpec[i] = Z3LHOSpec[i];
                        CurCSpec[i] = Z3LHCSpec[i];
                    }
                    run_motor_test(Z3LH, 30);
                    break;
                case '4':
                    for (int i = 0; i < 12; ++i) {
                        CurOSpec[i] = Z3RHOSpec[i];
                        CurCSpec[i] = Z3RHCSpec[i];
                    }
                    run_motor_test(Z3RH, 31);
                    break;
                default:
                    printf("Unknown command: %c\r\n", (char)c);
                    break;
            }
        }
        sleep_ms(100);
    }

    return 0;
}
