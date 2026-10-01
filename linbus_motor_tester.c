/**
 * Raspberry Pi Pico port of RCM3700 LINBus Motor Vehicle Tester
 * Original: Z-World, 2003
 * Port: 2026
 *
 * Hardware:
 * - Pico UART1: TX=GPIO8, RX=GPIO9 (for LIN communication)
 * - MCP2004 LIN Transceiver
 * - Motor control pins (configurable)
 *
 * This program tests LINBus motor actuators by:
 * 1. Initializing motor nodes
 * 2. Commanding open/close sequences
 * 3. Measuring motor position and stall detection
 * 4. Validating against specification windows
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

/* ============================================================================
 * PIN CONFIGURATION
 * ============================================================================ */
#define UART_ID uart1
#define BAUD_RATE 9600
#define UART_TX_PIN 8
#define UART_RX_PIN 9

/* MCP2004 Control Pins */
#define MCP2004_EN_PIN 10   /* Enable pin (active high) */
#define MCP2004_NSLP_PIN 11 /* Sleep pin (active low) */

/* Keypad input pins (if used) */
#define KEYPAD_A_PIN 12
#define KEYPAD_B_PIN 13
#define KEYPAD_C_PIN 14
#define KEYPAD_IN1_PIN 15
#define KEYPAD_IN2_PIN 16
#define KEYPAD_IN3_PIN 17
#define KEYPAD_IN4_PIN 18
#define KEYPAD_IN5_PIN 19

/* ============================================================================
 * MOTOR CONFIGURATION - ZONE 2 LH (can be expanded)
 * ============================================================================ */
#define MAX_MOTORS 13

/* Motor addresses for different configurations */
const char Z3RH[13] = {"\x2B\x25\x27\x2F\x2E\x28\x2A\x24\x26\x31\x33\x22"};
const char Z3LH[13] = {"\x22\x33\x2B\x25\x27\x2F\x2E\x31\x26\x24\x2A\x28"};
const char Z2RH[13] = {"\x2B\x2A\x25\x24\x22"};
const char Z2LH[13] = {"\x22\x2B\x2A\x25\x24"};

/* Motor open specifications */
const int Z3RHOSpec[13] = {2065,1528,1353,1398,1395,823,2071,1532,1338,1841,1614,1731};
const int Z3RHCSpec[13] = {2084,1528,1347,1396,1389,822,2084,1532,1332,1843,1615,1753};
const int Z3LHOSpec[13] = {1711,1572,2070,1524,1353,1396,1396,1837,1336,1537,2070,818};
const int Z3LHCSpec[13] = {1721,1581,2081,1525,1349,1393,1388,1839,1335,1536,2079,817};
const int Z2RHOSpec[13] = {1889,1870,5488,1533,2237};
const int Z2RHCSpec[13] = {1904,1879,5479,1529,2247};
const int Z2LHOSpec[13] = {2289,1895,1879,5489,1527};
const int Z2LHCSpec[13] = {2301,1907,1888,5482,1522};

const int MotVerSpec = 7;
const int WindowSpec = 100; /* +/- tolerance */

/* Motor direction & safe mode configuration */
const char Z3RHDir[13] = {"\x00\x00\x01\x00\x01\x01\x00\x01\x00\x01\x01\x00"};
const char Z3LHDir[13] = {"\x01\x00\x00\x00\x01\x00\x01\x01\x00\x01\x00\x01"};
const char Z2RHDir[13] = {"\x00\x01\x01\x01\x00"};
const char Z2LHDir[13] = {"\x01\x00\x01\x01\x01"};

const char Z3RHSafeDir[13] = {"\x01\x01\x00\x00\x00\x00\x01\x01\x00\x00\x00\x00"};
const char Z3LHSafeDir[13] = {"\x00\x00\x01\x01\x00\x00\x00\x00\x00\x01\x01\x00"};
const char Z2RHSafeDir[13] = {"\x01\x01\x00\x01\x00"};
const char Z2LHSafeDir[13] = {"\x00\x01\x01\x00\x01"};

const char Z3RHSafeEnable[13] = {"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\x00\x00"};
const char Z3LHSafeEnable[13] = {"\x00\x00\x00\x00\x00\x00\x00\x01\x00\x00\x00\x00"};
const char Z2RHSafeEnable[13] = {"\x00\x00\x00\x00\x01"};
const char Z2LHSafeEnable[13] = {"\x01\x00\x00\x00\x00"};

/* Motor mode constants */
#define NORMAL_MODE '\xFC'
#define SERVICE_MODE '\xFD'
#define STOP_MODE '\xFE'

/* ============================================================================
 * GLOBAL STATE
 * ============================================================================ */

/* Current test configuration */
char *CurrentAddrList = NULL;
int CurrentZoneModel = 0;
int CurrentMotorCount = 0;
int CurOSpec[MAX_MOTORS];
int CurCSpec[MAX_MOTORS];

/* Motor test results */
int ResultOpen[MAX_MOTORS];
int ResultClose[MAX_MOTORS];
int MotVer[MAX_MOTORS];
char MotModeStat[MAX_MOTORS];
char MotDirStat[MAX_MOTORS];
char MotSafeStat[MAX_MOTORS];
char MotSafeDirStat[MAX_MOTORS];

/* Motor status flags */
int MotPosOk[MAX_MOTORS];
int MotOpenOk[MAX_MOTORS];
int MotCloseOk[MAX_MOTORS];
int MotVerOk[MAX_MOTORS];
int MotDirOk[MAX_MOTORS];
int MotSafeOk[MAX_MOTORS];
int MotSafeDirOk[MAX_MOTORS];

/* ============================================================================
 * DELAY FUNCTIONS
 * ============================================================================ */

/**
 * Delay in microseconds
 */
void delay_us(uint32_t us) {
    busy_wait_us(us);
}

/**
 * Delay in milliseconds
 */
void delay_ms(uint32_t ms) {
    sleep_ms(ms);
}

/* ============================================================================
 * UART / LIN COMMUNICATION
 * ============================================================================ */

/**
 * Initialize UART1 for LIN communication (9600 baud)
 */
void lin_uart_init(void) {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    /* Initialize MCP2004 transceiver */
    gpio_init(MCP2004_EN_PIN);
    gpio_init(MCP2004_NSLP_PIN);
    gpio_set_dir(MCP2004_EN_PIN, GPIO_OUT);
    gpio_set_dir(MCP2004_NSLP_PIN, GPIO_OUT);
    
    /* Enable transceiver */
    gpio_put(MCP2004_EN_PIN, 1);
    gpio_put(MCP2004_NSLP_PIN, 1);
}

/**
 * Transmit LIN break (low for ~13 bit times at 9600 baud)
 */
void lin_send_break(void) {
    /* Disable UART and bit-bang the break */
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(UART_TX_PIN, GPIO_OUT);
    gpio_put(UART_TX_PIN, 0);
    delay_us(13000); /* ~13 bit times */
    gpio_put(UART_TX_PIN, 1);
    delay_us(1000);
    /* Re-enable UART */
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
}

/**
 * Calculate LIN checksum (inverted sum of bytes)
 */
uint8_t lin_checksum(const char *data, int len) {
    int checksum = 0;
    for (int i = 0; i < len; i++) {
        checksum += (unsigned char)data[i];
    }
    checksum = checksum % 256;
    return (checksum ^ 0xFF);
}

/**
 * Send LIN frame with optional checksum
 * @param data: payload bytes
 * @param len: payload length
 * @param response_expected: if true, read response
 * @param response_buf: buffer for response (must be 30 bytes)
 * @return: number of bytes read (if response_expected)
 */
int lin_send_frame(const char *data, int len, int response_expected, char *response_buf) {
    char frame[35];
    int frame_len = 0;
    uint8_t checksum;
    
    /* Build frame: 0x55 (PID) + payload + checksum */
    frame[frame_len++] = 0x55;
    memcpy(&frame[frame_len], data, len);
    frame_len += len;
    
    /* Calculate and append checksum */
    checksum = lin_checksum(data, len);
    frame[frame_len++] = checksum;
    
    /* Send LIN break */
    lin_send_break();
    
    /* Send frame */
    delay_us(100);
    for (int i = 0; i < frame_len; i++) {
        uart_putc(UART_ID, frame[i]);
        delay_us(100);
    }
    
    /* Inter-frame delay */
    delay_us(15000);
    
    /* Read response if expected */
    if (response_expected) {
        int bytes_read = 0;
        uint32_t timeout = time_us_32() + 500000; /* 500ms timeout */
        
        while (time_us_32() < timeout && bytes_read < 30) {
            if (uart_is_readable(UART_ID)) {
                response_buf[bytes_read++] = uart_getc(UART_ID);
            }
        }
        return bytes_read;
    }
    
    return 0;
}

/* ============================================================================
 * MOTOR CONTROL COMMANDS
 * ============================================================================ */

/**
 * Open a single motor
 */
void motor_open(char addr) {
    char data1[5] = {0xEC, addr, 0xD0, 0x87, 0xFF};
    char data2[5] = {0x6F, addr, 0xFF, 0xFF, 0x10};
    char response[30];
    
    lin_send_frame(data1, 5, 0, response);
    delay_ms(10);
    lin_send_frame(data2, 5, 0, response);
}

/**
 * Open all motors in address list
 */
void motor_open_all(const char *addr_list) {
    int len = strlen(addr_list);
    for (int i = 0; i < len; i++) {
        motor_open(addr_list[i]);
        delay_ms(50);
    }
}

/**
 * Close a single motor
 */
void motor_close(char addr) {
    char data1[5] = {0xEC, addr, 0xFF, 0xFF, 0xFF};
    char data2[5] = {0x6F, addr, 0xD0, 0x87, 0x10};
    char response[30];
    
    lin_send_frame(data1, 5, 0, response);
    delay_ms(10);
    lin_send_frame(data2, 5, 0, response);
}

/**
 * Close all motors in address list
 */
void motor_close_all(const char *addr_list) {
    int len = strlen(addr_list);
    for (int i = 0; i < len; i++) {
        motor_close(addr_list[i]);
        delay_ms(50);
    }
}

/**
 * Initialize motor position
 */
void motor_init_pos(char addr) {
    char data[5] = {0xEC, addr, 0xFF, 0xFF, 0xFF};
    char response[30];
    lin_send_frame(data, 5, 0, response);
}

/**
 * Get motor position
 * @return: position value (16-bit)
 */
int motor_get_pos(char addr) {
    char data1[3] = {0x5B, addr, 0xFF};
    char data2[1] = {0xA3};
    char response[30];
    int bytes_read;
    int position = 0;
    
    lin_send_frame(data1, 3, 0, response);
    delay_ms(10);
    bytes_read = lin_send_frame(data2, 1, 1, response);
    
    if (bytes_read >= 5) {
        position = (unsigned char)response[4] * 256 + (unsigned char)response[3];
    }
    
    return position;
}

/**
 * Extract position from response data
 */
int lin_pos_calc(const char *rx_data) {
    return (unsigned char)rx_data[4] * 256 + (unsigned char)rx_data[3];
}

/**
 * Extract status from response data
 */
char lin_stat_calc(const char *rx_data) {
    return rx_data[5];
}

/**
 * Await motor blocking (motor stalled)
 * @return: position when stalled
 */
int motor_await_block(char addr) {
    char data1[3] = {0x5B, addr, 0xFF};
    char data2[1] = {0xA3};
    char response[30];
    int bytes_read;
    int position = 0;
    char status = 0;
    uint32_t timeout = time_us_32() + 500000; /* 500ms timeout */
    
    while (time_us_32() < timeout) {
        lin_send_frame(data1, 3, 0, response);
        delay_ms(10);
        bytes_read = lin_send_frame(data2, 1, 1, response);
        
        if (bytes_read >= 6) {
            status = lin_stat_calc(response);
            if (status == 0x40) { /* Motor stalled */
                position = lin_pos_calc(response);
                break;
            }
        }
        delay_ms(50);
    }
    
    return position;
}

/**
 * Await all motors in list to stall
 */
void motor_await_all(const char *addr_list, char direction) {
    int len = strlen(addr_list);
    int stalled_count = 0;
    uint32_t timeout = time_us_32() + 20000000; /* 20 second timeout */
    
    while (time_us_32() < timeout && stalled_count < len) {
        stalled_count = 0;
        for (int i = 0; i < len; i++) {
            int pos = motor_await_block(addr_list[i]);
            if (pos > 0) {
                stalled_count++;
            }
            delay_ms(50);
        }
    }
}

/**
 * Stall check: measure position during open/close sequence
 * @param addr_list: motors to check
 * @param direction: 'O' for open, 'C' for close
 * @param timeout_sec: timeout in seconds
 * @return: array of positions
 */
int* motor_stall_check(const char *addr_list, char direction, int timeout_sec) {
    static int positions[MAX_MOTORS];
    int len = strlen(addr_list);
    char response[30];
    int bytes_read;
    int status_count = 0;
    
    memset(positions, 0, sizeof(positions));
    
    uint32_t timeout = time_us_32() + (timeout_sec * 1000000);
    
    while (time_us_32() < timeout && status_count < len) {
        status_count = 0;
        for (int i = 0; i < len; i++) {
            char data1[3] = {0x5B, addr_list[i], 0xFF};
            char data2[1] = {0xA3};
            
            lin_send_frame(data1, 3, 0, response);
            delay_ms(10);
            bytes_read = lin_send_frame(data2, 1, 1, response);
            
            if (bytes_read >= 6) {
                char status = lin_stat_calc(response);
                if (status == 0x40) { /* Motor stalled */
                    int pos = lin_pos_calc(response);
                    if (direction == 'O') {
                        positions[i] = pos - 34768;
                    } else if (direction == 'C') {
                        positions[i] = 65535 - pos;
                    }
                    status_count++;
                    printf("Motor %c stalled: pos=%d\r\n", addr_list[i], positions[i]);
                }
            }
            delay_ms(50);
        }
    }
    
    return positions;
}

/**
 * Set motor mode (normal, service, stop)
 */
void motor_set_mode(char mode) {
    char data[5] = {0x2E, 0x20, mode, 0xFF, 0xFF};
    char response[30];
    lin_send_frame(data, 5, 0, response);
}

/**
 * Set motor address (service mode)
 */
void motor_addr_service(char addr) {
    char data[3] = {0x97, 0x20, addr};
    char response[30];
    lin_send_frame(data, 3, 0, response);
}

/**
 * Program motor with mode byte
 */
void motor_program(char addr, char version, char mode_byte) {
    char data[5] = {0xA8, addr, addr, mode_byte, version};
    char response[30];
    lin_send_frame(data, 5, 0, response);
}

/**
 * Get motor status
 */
int motor_get_status(char addr, char *status_buf) {
    char data1[3] = {0xD8, addr, 0xFF};
    char data2[1] = {0x20};
    char response[30];
    int bytes_read;
    
    lin_send_frame(data1, 3, 0, response);
    delay_ms(10);
    bytes_read = lin_send_frame(data2, 1, 1, response);
    
    if (bytes_read >= 7) {
        memcpy(status_buf, response, bytes_read);
        printf("Motor %02X: ", addr);
        for (int i = 0; i < bytes_read; i++) {
            printf("%02X ", (unsigned char)response[i]);
        }
        printf("\r\n");
    }
    
    return bytes_read;
}

/**
 * Get status of all motors
 */
void motor_get_status_all(const char *addr_list) {
    int len = strlen(addr_list);
    char status[30];
    
    for (int i = 0; i < len; i++) {
        int bytes = motor_get_status(addr_list[i], status);
        if (bytes >= 6) {
            MotVer[i] = (unsigned char)status[5];
            MotModeStat[i] = status[4];
        }
        delay_ms(50);
    }
}

/**
 * Calculate mode byte from configuration
 */
char motor_mode_byte(int model, int motor_idx) {
    char mode = 0xC0;
    
    switch (model) {
        case 20: /* Z2LH */
            mode |= (Z2LHDir[motor_idx] << 2);
            mode |= Z2LHSafeDir[motor_idx];
            mode |= (Z2LHSafeEnable[motor_idx] << 1);
            break;
        case 21: /* Z2RH */
            mode |= (Z2RHDir[motor_idx] << 2);
            mode |= Z2RHSafeDir[motor_idx];
            mode |= (Z2RHSafeEnable[motor_idx] << 1);
            break;
        case 30: /* Z3LH */
            mode |= (Z3LHDir[motor_idx] << 2);
            mode |= Z3LHSafeDir[motor_idx];
            mode |= (Z3LHSafeEnable[motor_idx] << 1);
            break;
        case 31: /* Z3RH */
            mode |= (Z3RHDir[motor_idx] << 2);
            mode |= Z3RHSafeDir[motor_idx];
            mode |= (Z3RHSafeEnable[motor_idx] << 1);
            break;
    }
    
    return mode;
}

/**
 * Address/safe mode configuration sequence
 */
void motor_addr_safe_mode(const char *addr_list, int model) {
    int len = strlen(addr_list);
    char status[30];
    int retry_count = 0;
    
retry_addr:
    motor_set_mode(SERVICE_MODE);
    delay_ms(100);
    
    for (int i = 0; i < len; i++) {
        motor_addr_service(addr_list[i]);
        delay_ms(50);
        int bytes = motor_get_status(addr_list[i], status);
        if (bytes < 5) {
            if (retry_count < 5) {
                retry_count++;
                goto retry_addr;
            }
            goto end_addr_safe;
        }
    }
    
    retry_count = 0;
    
retry_prg:
    for (int i = 0; i < len; i++) {
        char mode = motor_mode_byte(model, i);
        motor_program(addr_list[i], 0x07, mode);
        delay_ms(50);
        int bytes = motor_get_status(addr_list[i], status);
        if (bytes < 5) {
            if (retry_count < 5) {
                retry_count++;
                goto retry_prg;
            }
            goto end_addr_safe;
        }
    }
    
end_addr_safe:
    motor_set_mode(NORMAL_MODE);
    delay_ms(100);
}

/* ============================================================================
 * MOTOR TEST & VALIDATION
 * ============================================================================ */

/**
 * Validate motor test results
 * @return: number of motors that passed
 */
int motor_validate_results(const char *addr_list, int model) {
    int len = strlen(addr_list);
    int pass_count = 0;
    
    for (int i = 0; i < len; i++) {
        MotPosOk[i] = 1;
        MotOpenOk[i] = 1;
        MotCloseOk[i] = 1;
        MotVerOk[i] = 1;
        MotDirOk[i] = 1;
        MotSafeOk[i] = 1;
        MotSafeDirOk[i] = 1;
        
        /* Check position specs */
        if (ResultOpen[i] < CurOSpec[i] - WindowSpec || 
            ResultOpen[i] > CurOSpec[i] + WindowSpec) {
            MotPosOk[i] = 0;
            MotOpenOk[i] = 0;
        }
        
        if (ResultClose[i] < CurCSpec[i] - WindowSpec || 
            ResultClose[i] > CurCSpec[i] + WindowSpec) {
            MotPosOk[i] = 0;
            MotCloseOk[i] = 0;
        }
        
        /* Check motor version */
        if (MotVer[i] != MotVerSpec) {
            MotPosOk[i] = 0;
            MotVerOk[i] = 0;
        }
        
        /* Extract and check direction/safe mode bits */
        MotDirStat[i] = (MotModeStat[i] >> 2) & 0x01;
        MotSafeStat[i] = (MotModeStat[i] >> 1) & 0x01;
        MotSafeDirStat[i] = MotModeStat[i] & 0x01;
        
        /* Validate based on model */
        switch (model) {
            case 20: /* Z2LH */
                if (Z2LHDir[i] != MotDirStat[i]) MotDirOk[i] = 0;
                if (Z2LHSafeDir[i] != MotSafeDirStat[i]) MotSafeDirOk[i] = 0;
                if (Z2LHSafeEnable[i] != MotSafeStat[i]) MotSafeOk[i] = 0;
                break;
            case 21: /* Z2RH */
                if (Z2RHDir[i] != MotDirStat[i]) MotDirOk[i] = 0;
                if (Z2RHSafeDir[i] != MotSafeDirStat[i]) MotSafeDirOk[i] = 0;
                if (Z2RHSafeEnable[i] != MotSafeStat[i]) MotSafeOk[i] = 0;
                break;
            case 30: /* Z3LH */
                if (Z3LHDir[i] != MotDirStat[i]) MotDirOk[i] = 0;
                if (Z3LHSafeDir[i] != MotSafeDirStat[i]) MotSafeDirOk[i] = 0;
                if (Z3LHSafeEnable[i] != MotSafeStat[i]) MotSafeOk[i] = 0;
                break;
            case 31: /* Z3RH */
                if (Z3RHDir[i] != MotDirStat[i]) MotDirOk[i] = 0;
                if (Z3RHSafeDir[i] != MotSafeDirStat[i]) MotSafeDirOk[i] = 0;
                if (Z3RHSafeEnable[i] != MotSafeStat[i]) MotSafeOk[i] = 0;
                break;
        }
        
        /* Fail if any mode check fails */
        if (MotSafeOk[i] == 0 || MotSafeDirOk[i] == 0 || MotDirOk[i] == 0) {
            MotPosOk[i] = 0;
        }
        
        if (MotPosOk[i] == 1) {
            pass_count++;
        }
    }
    
    return pass_count;
}

/**
 * Run full motor test sequence
 */
void run_motor_test(const char *addr_list, int model) {
    int len = strlen(addr_list);
    int *pos;
    int pass_count, fail_count;
    
    printf("\r\n=== Starting Motor Test ===");
    printf("\r\nMotor Count: %d", len);
    printf("\r\nModel: %d\r\n", model);
    
    CurrentAddrList = (char *)addr_list;
    CurrentZoneModel = model;
    CurrentMotorCount = len;
    
    /* Get initial status */
    printf("Getting initial status...\r\n");
    motor_get_status_all(addr_list);
    delay_ms(100);
    
    /* Await motors to settle */
    printf("Awaiting motor settling...\r\n");
    motor_await_all(addr_list, 'N');
    delay_ms(500);
    
    /* Set initial position (close all) */
    printf("Setting initial position (closing)...\r\n");
    motor_close_all(addr_list);
    pos = motor_stall_check(addr_list, 'C', 20);
    delay_ms(1000);
    
    /* Open all motors and measure */
    printf("Opening motors...\r\n");
    motor_open_all(addr_list);
    pos = motor_stall_check(addr_list, 'O', 20);
    memcpy(ResultOpen, pos, len * sizeof(int));
    delay_ms(1000);
    
    printf("Open results: ");
    for (int i = 0; i < len; i++) {
        printf("%d ", ResultOpen[i]);
    }
    printf("\r\n");
    
    /* Close all motors and measure */
    printf("Closing motors...\r\n");
    motor_close_all(addr_list);
    pos = motor_stall_check(addr_list, 'C', 20);
    memcpy(ResultClose, pos, len * sizeof(int));
    delay_ms(1000);
    
    printf("Close results: ");
    for (int i = 0; i < len; i++) {
        printf("%d ", ResultClose[i]);
    }
    printf("\r\n");
    
    /* Validate results */
    printf("Validating results...\r\n");
    pass_count = motor_validate_results(addr_list, model);
    fail_count = len - pass_count;
    
    printf("\r\n=== Test Complete ===");
    printf("\r\nPassed: %d", pass_count);
    printf("\r\nFailed: %d\r\n", fail_count);
    
    /* Print per-motor results */
    for (int i = 0; i < len; i++) {
        printf("Motor %02X: Open=%d Close=%d Status=%s\r\n",
               (unsigned char)addr_list[i],
               ResultOpen[i],
               ResultClose[i],
               MotPosOk[i] ? "PASS" : "FAIL");
    }
}

/* ============================================================================
 * MAIN
 * ============================================================================ */

int main(void) {
    stdio_init_all();
    delay_ms(2000);
    
    printf("\r\n\r\n=== Raspberry Pi Pico LINBus Motor Tester ===");
    printf("\r\nInitializing UART and LIN transceiver...\r\n");
    
    lin_uart_init();
    delay_ms(500);
    
    printf("System ready.\r\n");
    printf("Available tests:\r\n");
    printf("  1: Z2LH (2-zone left-hand)\r\n");
    printf("  2: Z2RH (2-zone right-hand)\r\n");
    printf("  3: Z3LH (3-zone left-hand)\r\n");
    printf("  4: Z3RH (3-zone right-hand)\r\n");
    printf("\r\nEnter test selection (1-4): ");
    
    while (1) {
        if (stdio_usb_connected()) {
            int ch = getchar();
            
            switch (ch) {
                case '1':
                    printf("1\r\nRunning Z2LH test...\r\n");
                    /* Copy specs for Z2LH */
                    for (int i = 0; i < 5; i++) {
                        CurOSpec[i] = Z2LHOSpec[i];
                        CurCSpec[i] = Z2LHCSpec[i];
                    }
                    run_motor_test(Z2LH, 20);
                    break;
                    
                case '2':
                    printf("2\r\nRunning Z2RH test...\r\n");
                    for (int i = 0; i < 5; i++) {
                        CurOSpec[i] = Z2RHOSpec[i];
                        CurCSpec[i] = Z2RHCSpec[i];
                    }
                    run_motor_test(Z2RH, 21);
                    break;
                    
                case '3':
                    printf("3\r\nRunning Z3LH test...\r\n");
                    for (int i = 0; i < 13; i++) {
                        CurOSpec[i] = Z3LHOSpec[i];
                        CurCSpec[i] = Z3LHCSpec[i];
                    }
                    run_motor_test(Z3LH, 30);
                    break;
                    
                case '4':
                    printf("4\r\nRunning Z3RH test...\r\n");
                    for (int i = 0; i < 13; i++) {
                        CurOSpec[i] = Z3RHOSpec[i];
                        CurCSpec[i] = Z3RHCSpec[i];
                    }
                    run_motor_test(Z3RH, 31);
                    break;
                    
                default:
                    if (ch >= 32 && ch < 127) {
                        printf("%c", ch);
                    }
                    break;
            }
            
            printf("\r\nEnter test selection (1-4): ");
        }
        delay_ms(100);
    }
    
    return 0;
}
