#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

#include "FreeRTOS.h"
#include "task.h"

/* Pins */
#define MOTOR1A_PIN 11
#define MOTOR1B_PIN 10
#define MOTOR2A_PIN 12
#define MOTOR2B_PIN 13

#define START_BUTTON 7
#define RESET_BUTTON 27

#define ADC_IN_PIN 28
#define MUXA_PIN 18
#define MUXB_PIN 19
#define MUXC_PIN 20

/* Legacy behavior constants */
#define PWM_MAX 240
#define PWM_WRAP 255
#define NOMINAL_SPEED 140
#define FOLLOW_SPEED 150
#define MOTOR_MIN_EFFECTIVE_PWM 100
#define FOLLOW_KP 0.12f
#define FOLLOW_KI 0.0f
#define FOLLOW_KD 1.0f
#define ALIGN_FACTOR 1.5f

#define TURN_TIME_U_MS  1560/2
#define TURN_TIME_L_MS  720/2
#define TURN_TIME_R_MS  720/2
#define U_TURN_POST_STOP_MS 40000
#define SMALL_FWD_TIME_MS 90
#define ALIGN_AFTER_UTURN_MS 150
#define ALIGN_AFTER_TURN_MS 140
#define SOLVE_START_DELAY_MS 5000

/* Temporary debug switch: keep robot stopped at END_MAP and keep printing stacks. */
#define DEBUG_HOLD_AT_END_MAP 0
#define DEBUG_STACK_PRINT_PERIOD_MS 1000

#define NODE_DETECTION_DEFAULT 2
#define NODE_DETECTION_WHITE 2

#define IRSENSORS_COUNT 5
#define SENSOR_PERIOD_MS 3
#define MAP_PERIOD_MS 10
#define MOTOR_CTRL_PERIOD_MS 20
#define DEBUG_PERIOD_MS 120

#define IR_THRESHOLD 500

typedef enum {
    ROBOT_STOP,
    ROBOT_FORWARD,
    ROBOT_REVERSE,
    ROBOT_TURN_LEFT,
    ROBOT_TURN_RIGHT
} robot_cmd_t;

typedef enum {
    IDLE_MAP,
    FOLLOW_LINE_MAP,
    U_TURN,
    LEFT_TURN_MAP,
    RIGHT_TURN_MAP,
    REVERSE_MAP,
    SMALL_FORWARD,
    FORWARD_MAP,
    END_MAP
} map_state_t;

typedef enum {
    MODE_MAP,
    MODE_WAIT_SOLVE,
    MODE_SOLVE,
    MODE_SOLVE_DONE
} run_mode_t;

typedef enum {
    IDLE_SOLVE,
    FOLLOW_LINE_SOLVE,
    GET_INSTRUCTION,
    RIGHT_TURN_SOLVE,
    LEFT_TURN_SOLVE,
    FORWARD_SOLVE,
    FINISH_SOLVE
} solve_state_t;

typedef struct {
    int left;
    int right;
} motor_pair_t;

typedef struct {
    uint16_t ir[IRSENSORS_COUNT];
    char node;
} sensor_data_t;

static volatile motor_pair_t g_target  = {0, 0};
static volatile motor_pair_t g_current = {0, 0};
static volatile sensor_data_t g_sensor = {{0}, 'N'};
static volatile robot_cmd_t g_last_cmd = ROBOT_STOP;
static volatile map_state_t g_map_state = IDLE_MAP;
static volatile run_mode_t g_run_mode = MODE_MAP;
static volatile solve_state_t g_solve_state = IDLE_SOLVE;

static char g_node_stack[256];
static int  g_node_stack_len = 0;
static int  g_node_count     = 0;
static char g_past_node      = ' ';

static char g_solve_stack[256];
static int  g_solve_stack_len = 0;
static int  g_solve_stack_pos = 0;
static TickType_t g_solve_delay_start = 0;
static bool g_end_captured = false;

/* ================================================================
 * Hardware init
 * ================================================================ */
static void pwm_pin_init(uint pin) {
    static bool slice_inited[8] = { false };
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    if (slice < 8 && !slice_inited[slice]) {
        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_wrap(&cfg, PWM_WRAP);
        pwm_init(slice, &cfg, true);
        slice_inited[slice] = true;
    }
    pwm_set_gpio_level(pin, PWM_WRAP);
}

static void motors_init(void) {
    pwm_pin_init(MOTOR1A_PIN);
    pwm_pin_init(MOTOR1B_PIN);
    pwm_pin_init(MOTOR2A_PIN);
    pwm_pin_init(MOTOR2B_PIN);
}

static void buttons_init(void) {
    gpio_init(START_BUTTON);
    gpio_set_dir(START_BUTTON, GPIO_IN);
    gpio_pull_up(START_BUTTON);

    gpio_init(RESET_BUTTON);
    gpio_set_dir(RESET_BUTTON, GPIO_IN);
    gpio_pull_up(RESET_BUTTON);
}

static void sensors_init(void) {
    adc_init();
    adc_gpio_init(ADC_IN_PIN);
    adc_select_input(2);

    gpio_init(MUXA_PIN); gpio_set_dir(MUXA_PIN, GPIO_OUT);
    gpio_init(MUXB_PIN); gpio_set_dir(MUXB_PIN, GPIO_OUT);
    gpio_init(MUXC_PIN); gpio_set_dir(MUXC_PIN, GPIO_OUT);
}

static inline bool button_pressed(uint pin) {
    return gpio_get(pin) == 0;
}

static inline void mux_set_channel(uint8_t channel) {
    gpio_put(MUXA_PIN, (channel >> 0u) & 1u);
    gpio_put(MUXB_PIN, (channel >> 1u) & 1u);
    gpio_put(MUXC_PIN, (channel >> 2u) & 1u);
}

static uint16_t read_mux_adc(uint8_t channel) {
    mux_set_channel(channel);
    sleep_us(40);
    return (uint16_t)((4095u - adc_read()) >> 2u);
}

/* ================================================================
 * Motor control
 * ================================================================ */
static void set_motor_pwm(int new_pwm, uint pin_a, uint pin_b) {
    if (new_pwm >  PWM_MAX) new_pwm =  PWM_MAX;
    if (new_pwm < -PWM_MAX) new_pwm = -PWM_MAX;

    if (new_pwm > 0 && new_pwm < MOTOR_MIN_EFFECTIVE_PWM)  new_pwm =  MOTOR_MIN_EFFECTIVE_PWM;
    if (new_pwm < 0 && new_pwm > -MOTOR_MIN_EFFECTIVE_PWM) new_pwm = -MOTOR_MIN_EFFECTIVE_PWM;

    if (new_pwm == 0) {
        pwm_set_gpio_level(pin_a, PWM_WRAP);
        pwm_set_gpio_level(pin_b, PWM_WRAP);
    } else if (new_pwm > 0) {
        pwm_set_gpio_level(pin_a, (uint16_t)(PWM_WRAP - new_pwm));
        pwm_set_gpio_level(pin_b, PWM_WRAP);
    } else {
        pwm_set_gpio_level(pin_a, PWM_WRAP);
        pwm_set_gpio_level(pin_b, (uint16_t)(PWM_WRAP + new_pwm));
    }
}

static void set_target_pwm(int left, int right) {
    taskENTER_CRITICAL();
    g_target.left  = left;
    g_target.right = right;
    taskEXIT_CRITICAL();
}

static void robot_apply_cmd(robot_cmd_t cmd) {
    g_last_cmd = cmd;
    switch (cmd) {
        case ROBOT_FORWARD:    set_target_pwm( NOMINAL_SPEED,  NOMINAL_SPEED); break;
        case ROBOT_REVERSE:    set_target_pwm(-NOMINAL_SPEED, -NOMINAL_SPEED); break;
        case ROBOT_TURN_LEFT:  set_target_pwm(-NOMINAL_SPEED,  NOMINAL_SPEED); break;
        case ROBOT_TURN_RIGHT: set_target_pwm( NOMINAL_SPEED, -NOMINAL_SPEED); break;
        case ROBOT_STOP:
        default:               set_target_pwm(0, 0); break; 
    }
}

/* ================================================================
 * Sensor / node detection
 * ================================================================ */
static char detect_node_from_ir(const uint16_t ir[IRSENSORS_COUNT]) {
    char pattern[6];
    for (int i = 0; i < IRSENSORS_COUNT; i++) {
        pattern[i] = (ir[i] > IR_THRESHOLD) ? '1' : '0';
    }
    pattern[5] = '\0';

    if ((strcmp(pattern, "11100") == 0) || (strcmp(pattern, "11000") == 0)) return 'L';
    if ((strcmp(pattern, "00111") == 0) || (strcmp(pattern, "00011") == 0)) return 'R';
    if  (strcmp(pattern, "11111") == 0)                                      return 'B';
    if  (strcmp(pattern, "00000") == 0)                                      return 'W';
    return 'N';
}

/* ================================================================
 * Node stack / solver
 * ================================================================ */
static void push_node(char n) {
    if (g_node_stack_len < (int)(sizeof(g_node_stack))) {
        g_node_stack[g_node_stack_len++] = n;
    }
}

static void solve_stack(char *stack, int *stack_len) {
    bool changed = true;
    while (changed && *stack_len >= 3) {
        changed = false;
        for (int i = 0; i <= *stack_len - 3; i++) {
            char a = stack[i];
            char b = stack[i + 1];
            char c = stack[i + 2];
            char repl = '\0';

            if      (a=='L' && b=='U' && c=='F') repl = 'R';
            else if (a=='R' && b=='U' && c=='F') repl = 'L';
            else if (a=='L' && b=='U' && c=='L') repl = 'F';
            else if (a=='L' && b=='U' && c=='R') repl = 'U';
            else if (a=='R' && b=='U' && c=='R') repl = 'F';
            else if (a=='L' && b=='U' && c=='R') repl = 'U';
            else if (a=='R' && b=='U' && c=='L') repl = 'U';
            else if (a=='F' && b=='U' && c=='L') repl = 'R';
            else if (a=='F' && b=='U' && c=='R') repl = 'L';

            if (repl != '\0') {
                stack[i] = repl;
                for (int j = i + 1; j + 2 < *stack_len; j++) {
                    stack[j] = stack[j + 2];
                }
                *stack_len -= 2;
                changed = true;
                break;
            }
        }
    }
}

static void print_node_stack(void) {
    printf("Node Stack: ");
    for (int i = 0; i < g_node_stack_len; i++) printf("%c ", g_node_stack[i]);
    printf("\n");
}

static void copy_node_stack_to_solve_stack(void) {
    g_solve_stack_len = g_node_stack_len;
    g_solve_stack_pos = 0;
    for (int i = 0; i < g_node_stack_len; i++) {
        g_solve_stack[i] = g_node_stack[i];
    }
}

static void build_solved_stack_from_node_stack(void) {
    copy_node_stack_to_solve_stack();
    solve_stack(g_solve_stack, &g_solve_stack_len);
}

static void print_solve_stack(void) {
    printf("Solve Stack: ");
    for (int i = 0; i < g_solve_stack_len; i++) printf("%c ", g_solve_stack[i]);
    printf("\n");
}

/* ================================================================
 * Line follower PID step
 * ================================================================ */
static void robot_follow_line_step() {
    static float prev_error = 0.0f;
    static float integral   = 0.0f;

    uint16_t ir[IRSENSORS_COUNT];
    taskENTER_CRITICAL();
    for (int i = 0; i < IRSENSORS_COUNT; i++) ir[i] = g_sensor.ir[i];
    taskEXIT_CRITICAL();

    int sum = 0;
    for (int i = 0; i < IRSENSORS_COUNT; i++) sum += ir[i];

    if (sum >= 2500) {
        integral    = 0.0f;
        g_last_cmd  = ROBOT_FORWARD;
        set_target_pwm(FOLLOW_SPEED, FOLLOW_SPEED);
        return;
    }

    float error = (-1.1f * (float)ir[0]) + (-1.0f * (float)ir[1]) +
                  ( 1.0f * (float)ir[3]) + ( 1.1f * (float)ir[4]);

    integral += error;
    float derivative = error - prev_error;
    prev_error = error;

    int correction = (int)((FOLLOW_KP * error) + (FOLLOW_KI * integral) + (FOLLOW_KD * derivative));
    if (correction >  100) correction =  100;
    if (correction < -100) correction = -100;

    g_last_cmd = ROBOT_FORWARD;
    set_target_pwm(FOLLOW_SPEED + correction, FOLLOW_SPEED - correction);
}

static void robot_align_line_step() {
    static float prev_error = 0.0f;
    static float integral   = 0.0f;

    uint16_t ir[IRSENSORS_COUNT];
    taskENTER_CRITICAL();
    for (int i = 0; i < IRSENSORS_COUNT; i++) ir[i] = g_sensor.ir[i];
    taskEXIT_CRITICAL();

    int sum = 0;
    for (int i = 0; i < IRSENSORS_COUNT; i++) sum += ir[i];

    if (sum >= 2500) {
        integral    = 0.0f;
        g_last_cmd  = ROBOT_FORWARD;
        set_target_pwm(FOLLOW_SPEED, FOLLOW_SPEED);
        return;
    }

    float error = (-1.1f * (float)ir[0]) + (-1.0f * (float)ir[1]) +
                  ( 1.0f * (float)ir[3]) + ( 1.1f * (float)ir[4]);

    integral += error;
    float derivative = error - prev_error;
    prev_error = error;

    int correction = (int)((FOLLOW_KP * error) + (FOLLOW_KI * integral) + (FOLLOW_KD * derivative));
    if (correction >  100) correction =  100;
    if (correction < -100) correction = -100;

    g_last_cmd = ROBOT_FORWARD;
    correction = (int)(ALIGN_FACTOR * correction);
    set_target_pwm(correction, -correction);
}

/* ================================================================
 * Name helpers (debug)
 * ================================================================ */
static const char *cmd_name(robot_cmd_t cmd) {
    switch (cmd) {
        case ROBOT_FORWARD:    return "FORWARD";
        case ROBOT_REVERSE:    return "REVERSE";
        case ROBOT_TURN_LEFT:  return "TURN_LEFT";
        case ROBOT_TURN_RIGHT: return "TURN_RIGHT";
        default:               return "STOP";
    }
}

static const char *map_state_name(map_state_t s) {
    switch (s) {
        case IDLE_MAP:       return "IDLE";
        case FOLLOW_LINE_MAP:return "FOLLOW";
        case U_TURN:         return "U_TURN";
        case LEFT_TURN_MAP:  return "LEFT_TURN";
        case RIGHT_TURN_MAP: return "RIGHT_TURN";
        case REVERSE_MAP:    return "REVERSE";
        case SMALL_FORWARD:  return "SMALL_FWD";
        case FORWARD_MAP:    return "FORWARD";
        case END_MAP:        return "END";
        default:             return "UNKNOWN";
    }
}

static const char *run_mode_name(run_mode_t m) {
    switch (m) {
        case MODE_MAP:        return "MAP";
        case MODE_WAIT_SOLVE: return "WAIT_SOLVE";
        case MODE_SOLVE:      return "SOLVE";
        case MODE_SOLVE_DONE: return "SOLVE_DONE";
        default:              return "UNKNOWN";
    }
}

static const char *solve_state_name(solve_state_t s) {
    switch (s) {
        case IDLE_SOLVE:       return "IDLE";
        case FOLLOW_LINE_SOLVE:return "FOLLOW";
        case GET_INSTRUCTION:  return "GET_INST";
        case RIGHT_TURN_SOLVE: return "RIGHT";
        case LEFT_TURN_SOLVE:  return "LEFT";
        case FORWARD_SOLVE:    return "FORWARD";
        case FINISH_SOLVE:     return "FINISH";
        default:               return "UNKNOWN";
    }
}

/* ================================================================
 * Tasks
 * ================================================================ */
static void led_task(void *params) {
    (void)params;
    while (true) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(250));
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

static void sensor_task(void *params) {
    (void)params;
    TickType_t last = xTaskGetTickCount();
    while (true) {
        uint16_t local_ir[IRSENSORS_COUNT];
        for (uint8_t i = 0; i < IRSENSORS_COUNT; i++) {
            local_ir[IRSENSORS_COUNT - 1 - i] = read_mux_adc((uint8_t)(3u + i));
        }
        char node = detect_node_from_ir(local_ir);
        taskENTER_CRITICAL();
        for (int i = 0; i < IRSENSORS_COUNT; i++) g_sensor.ir[i] = local_ir[i];
        g_sensor.node = node;
        taskEXIT_CRITICAL();
        vTaskDelayUntil(&last, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}

static void motor_control_task(void *params) {
    (void)params;
    TickType_t last = xTaskGetTickCount();
    while (true) {
        int tl, tr;
        taskENTER_CRITICAL();
        tl = g_target.left;
        tr = g_target.right;
        taskEXIT_CRITICAL();

        set_motor_pwm(tl, MOTOR1A_PIN, MOTOR1B_PIN);
        set_motor_pwm(tr, MOTOR2A_PIN, MOTOR2B_PIN);

        taskENTER_CRITICAL();
        g_current.left  = tl;
        g_current.right = tr;
        taskEXIT_CRITICAL();

        vTaskDelayUntil(&last, pdMS_TO_TICKS(MOTOR_CTRL_PERIOD_MS));
    }
}

static void map_fsm_task(void *params) {
    (void)params;
    TickType_t last = xTaskGetTickCount();
    TickType_t last_stack_print = 0;

    while (true) {
        /* Reset tem prioridade máxima */
        if (button_pressed(RESET_BUTTON)) {
            g_map_state      = IDLE_MAP;
            g_run_mode       = MODE_MAP;
            g_solve_state    = IDLE_SOLVE;
            g_node_count     = 0;
            g_past_node      = ' ';
            g_node_stack_len = 0;
            g_solve_stack_len = 0;
            g_solve_stack_pos = 0;
            g_end_captured    = false;
            robot_apply_cmd(ROBOT_STOP);
            vTaskDelayUntil(&last, pdMS_TO_TICKS(MAP_PERIOD_MS));
            continue;
        }

        char node;
        taskENTER_CRITICAL();
        node = g_sensor.node;
        taskEXIT_CRITICAL();

        if (g_run_mode == MODE_WAIT_SOLVE) {
            robot_apply_cmd(ROBOT_STOP);
            if ((xTaskGetTickCount() - g_solve_delay_start) >= pdMS_TO_TICKS(SOLVE_START_DELAY_MS)) {
                g_run_mode = MODE_SOLVE;
                g_solve_state = IDLE_SOLVE;
                g_node_count = 0;
                g_past_node = ' ';
                printf("SOLVE start after wait.\n");
            }
            vTaskDelayUntil(&last, pdMS_TO_TICKS(MAP_PERIOD_MS));
            continue;
        }

        if (g_run_mode == MODE_SOLVE) {
            switch (g_solve_state) {
                case IDLE_SOLVE:
                    g_solve_state = FOLLOW_LINE_SOLVE;
                    break;

                case FOLLOW_LINE_SOLVE: {
                    if (node == g_past_node) {
                        g_node_count++;
                    } else {
                        g_node_count = 0;
                        g_past_node = node;
                    }

                    if (g_node_count >= NODE_DETECTION_DEFAULT && node != 'N') {
                        g_solve_state = GET_INSTRUCTION;
                    }
                    break;
                }

                case GET_INSTRUCTION:
                    if (node == 'B') {
                        g_solve_state = FINISH_SOLVE;
                    } else if (g_solve_stack_pos < g_solve_stack_len) {
                        char inst = g_solve_stack[g_solve_stack_pos++];
                        if (inst == 'R') g_solve_state = RIGHT_TURN_SOLVE;
                        else if (inst == 'L') g_solve_state = LEFT_TURN_SOLVE;
                        else if (inst == 'F') g_solve_state = FORWARD_SOLVE;
                        else g_solve_state = FINISH_SOLVE;
                    } else {
                        g_solve_state = FINISH_SOLVE;
                    }
                    break;

                case RIGHT_TURN_SOLVE:
                case LEFT_TURN_SOLVE:
                case FORWARD_SOLVE:
                case FINISH_SOLVE:
                default:
                    break;
            }

            switch (g_solve_state) {
                case FOLLOW_LINE_SOLVE:
                    robot_follow_line_step();
                    break;

                case GET_INSTRUCTION:
                    robot_apply_cmd(ROBOT_FORWARD);
                    vTaskDelay(pdMS_TO_TICKS(SMALL_FWD_TIME_MS));
                    robot_apply_cmd(ROBOT_STOP);
                    break;

                case RIGHT_TURN_SOLVE:
                    robot_apply_cmd(ROBOT_TURN_RIGHT);
                    vTaskDelay(pdMS_TO_TICKS(TURN_TIME_R_MS));
                    robot_apply_cmd(ROBOT_STOP);
                    vTaskDelay(pdMS_TO_TICKS(50));
                    {
                        TickType_t align_start = xTaskGetTickCount();
                        while ((xTaskGetTickCount() - align_start) < pdMS_TO_TICKS(ALIGN_AFTER_TURN_MS)) {
                            robot_follow_line_step();
                            vTaskDelay(pdMS_TO_TICKS(10));
                        }
                    }
                    g_node_count = 0;
                    g_past_node = ' ';
                    g_solve_state = FOLLOW_LINE_SOLVE;
                    break;

                case LEFT_TURN_SOLVE:
                    robot_apply_cmd(ROBOT_TURN_LEFT);
                    vTaskDelay(pdMS_TO_TICKS(TURN_TIME_L_MS));
                    // robot_apply_cmd(ROBOT_STOP);
                    // vTaskDelay(pdMS_TO_TICKS(50));
                    {
                        TickType_t align_start = xTaskGetTickCount();
                        while ((xTaskGetTickCount() - align_start) < pdMS_TO_TICKS(ALIGN_AFTER_TURN_MS)) {
                            robot_follow_line_step();
                            vTaskDelay(pdMS_TO_TICKS(10));
                        }
                    }
                    g_node_count = 0;
                    g_past_node = ' ';
                    g_solve_state = FOLLOW_LINE_SOLVE;
                    break;

                case FORWARD_SOLVE:
                    robot_apply_cmd(ROBOT_FORWARD);
                    vTaskDelay(pdMS_TO_TICKS(SMALL_FWD_TIME_MS));
                    g_node_count = 0;
                    g_past_node = ' ';
                    g_solve_state = FOLLOW_LINE_SOLVE;
                    break;

                case FINISH_SOLVE:
                    robot_apply_cmd(ROBOT_STOP);
                    g_run_mode = MODE_SOLVE_DONE;
                    break;

                case IDLE_SOLVE:
                default:
                    break;
            }

            vTaskDelayUntil(&last, pdMS_TO_TICKS(MAP_PERIOD_MS));
            continue;
        }

        /* --- Transition switch (MAP mode) --- */
        switch (g_map_state) {

            case IDLE_MAP:
                g_map_state = FOLLOW_LINE_MAP;
                break;

            case FOLLOW_LINE_MAP: {
                if (node == g_past_node) {
                    g_node_count++;
                } else {
                    g_node_count = 0;
                    g_past_node  = node;
                }

                int required = (node == 'W') ? NODE_DETECTION_WHITE : NODE_DETECTION_DEFAULT;

                if (g_node_count >= required) {
                    if (node == 'W') {
                        g_map_state = U_TURN;
                    } else if (node == 'L' || node == 'R' || node == 'B') {
                        /* FIX: avança um pouco antes de decidir a viragem */
                        g_map_state = SMALL_FORWARD;
                    }
                }
                break;
            }

            /* Estes estados tratam a sua própria transição no output switch */
            case U_TURN:
            case LEFT_TURN_MAP:
            case RIGHT_TURN_MAP:
            case SMALL_FORWARD:
                break;

            case FORWARD_MAP:
                g_map_state = FOLLOW_LINE_MAP;
                break;

            case END_MAP:
                break;

            case REVERSE_MAP:
            default:
                break;
        }

        /* --- Output switch --- */
        switch (g_map_state) {

            case IDLE_MAP:
                robot_apply_cmd(ROBOT_STOP);
                break;

            case FOLLOW_LINE_MAP:
                robot_follow_line_step();
                break;

            case U_TURN:
                robot_apply_cmd(ROBOT_TURN_RIGHT);
                vTaskDelay(pdMS_TO_TICKS(TURN_TIME_U_MS));
                // robot_apply_cmd(ROBOT_STOP);
                // vTaskDelay(pdMS_TO_TICKS(10000000000));
                /* Check if line was found; if white, continue turning until line found */
                char node;
                taskENTER_CRITICAL();
                node = g_sensor.node;
                taskEXIT_CRITICAL();
                
                if (node == 'W') {
                    /* Line not found after initial U-turn, keep searching */
                    TickType_t search_start = xTaskGetTickCount();
                    TickType_t search_timeout = pdMS_TO_TICKS(3000);  /* Max 3 seconds search */
                    
                    while (node == 'W' && (xTaskGetTickCount() - search_start) < search_timeout) {
                        robot_apply_cmd(ROBOT_TURN_RIGHT);
                        vTaskDelay(pdMS_TO_TICKS(100));  /* Small increment, check frequently */
                        
                        taskENTER_CRITICAL();
                        node = g_sensor.node;
                        taskEXIT_CRITICAL();
                    }
                }
                
                // robot_apply_cmd(ROBOT_STOP);
                // vTaskDelay(pdMS_TO_TICKS(100));
                
                /* Align middle sensor on the line via brief follow-line */
                TickType_t align_start = xTaskGetTickCount();
                while ((xTaskGetTickCount() - align_start) < pdMS_TO_TICKS(ALIGN_AFTER_UTURN_MS)) {
                    robot_align_line_step();
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                
                push_node('U');
                g_node_count = 0;
                g_past_node  = ' ';
                g_map_state  = FOLLOW_LINE_MAP;
                break;

            case LEFT_TURN_MAP:
                robot_apply_cmd(ROBOT_TURN_LEFT);
                vTaskDelay(pdMS_TO_TICKS(TURN_TIME_L_MS));  /* 600ms */
                
                robot_apply_cmd(ROBOT_STOP);
                vTaskDelay(pdMS_TO_TICKS(50));
                
                /* Align middle sensor on the line via brief follow-line */
                TickType_t align_start_l = xTaskGetTickCount();
                while ((xTaskGetTickCount() - align_start_l) < pdMS_TO_TICKS(ALIGN_AFTER_TURN_MS)) {
                    robot_align_line_step();
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                
                push_node('L');
                g_node_count = 0;
                g_past_node  = ' ';
                g_map_state  = FOLLOW_LINE_MAP;
                break;

            case RIGHT_TURN_MAP:
                robot_apply_cmd(ROBOT_TURN_RIGHT);
                vTaskDelay(pdMS_TO_TICKS(TURN_TIME_R_MS));  /* 200ms */
                
                robot_apply_cmd(ROBOT_STOP);
                vTaskDelay(pdMS_TO_TICKS(50));
                
                /* Align middle sensor on the line via brief follow-line */
                TickType_t align_start_r = xTaskGetTickCount();
                while ((xTaskGetTickCount() - align_start_r) < pdMS_TO_TICKS(ALIGN_AFTER_TURN_MS)) {
                    robot_align_line_step();
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                
                push_node('R');
                g_node_count = 0;
                g_past_node  = ' ';
                g_map_state  = FOLLOW_LINE_MAP;
                break;

            case SMALL_FORWARD:
                /* FIX: para antes de avançar para garantir leitura limpa do nó */
                robot_apply_cmd(ROBOT_STOP);
                vTaskDelay(pdMS_TO_TICKS(20));

                robot_apply_cmd(ROBOT_FORWARD);
                vTaskDelay(pdMS_TO_TICKS(SMALL_FWD_TIME_MS));
                robot_apply_cmd(ROBOT_STOP);
                /* Relê o nó após avançar */
                taskENTER_CRITICAL();
                node = g_sensor.node;
                taskEXIT_CRITICAL();

                printf("SMALL_FWD: past=%c node_after=%c\n", g_past_node, node);

                if (g_past_node == 'L') {
                    g_map_state = LEFT_TURN_MAP;
                } else if (g_past_node == 'R') {
                    if (node == 'W' || node == 'R' || node == 'B') {
                        g_map_state = RIGHT_TURN_MAP;
                    } else {
                        /* Linha continua em frente — ignora o nó R */
                        g_map_state = FOLLOW_LINE_MAP;
                        push_node('F');
                    }
                } else if (g_past_node == 'B') {
                    if (node == 'B' || node == 'L' || node == 'R') {
                        g_map_state = END_MAP;
                    } else {
                        g_map_state = LEFT_TURN_MAP;
                    }
                } else {
                    g_map_state = FOLLOW_LINE_MAP;
                }

                g_node_count = 0;
                g_past_node  = ' ';
                break;

            case FORWARD_MAP:
                robot_apply_cmd(ROBOT_FORWARD);
                break;

            case END_MAP:
                robot_apply_cmd(ROBOT_STOP);
                if (!g_end_captured) {
                    print_node_stack();
                    build_solved_stack_from_node_stack();
                    print_solve_stack();
                    g_end_captured = true;
#if DEBUG_HOLD_AT_END_MAP
                    g_run_mode = MODE_SOLVE_DONE;
                    printf("END_MAP reached. DEBUG hold enabled: robot stopped, continuous stack print.\n");
#else
                    g_solve_delay_start = xTaskGetTickCount();
                    g_run_mode = MODE_WAIT_SOLVE;
                    printf("END_MAP reached. Waiting %d ms to start solve.\n", SOLVE_START_DELAY_MS);
#endif
                }

#if DEBUG_HOLD_AT_END_MAP
                if ((xTaskGetTickCount() - last_stack_print) >= pdMS_TO_TICKS(DEBUG_STACK_PRINT_PERIOD_MS)) {
                    printf("[END_MAP DEBUG] ");
                    print_node_stack();
                    printf("[END_MAP DEBUG] ");
                    print_solve_stack();
                    last_stack_print = xTaskGetTickCount();
                }
#endif
                break;

            case REVERSE_MAP:
                robot_apply_cmd(ROBOT_REVERSE);
                break;
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(MAP_PERIOD_MS));
    }
}

static void serial_debug_task(void *params) {
    (void)params;
    TickType_t last = xTaskGetTickCount();
    while (true) {
        uint16_t ir_local[IRSENSORS_COUNT];
        char node_local;
        int tl, tr, cl, cr;
        robot_cmd_t  last_cmd;
        map_state_t  map_state;
        run_mode_t run_mode;
        solve_state_t solve_state;
        int nc;
        char pn;

        taskENTER_CRITICAL();
        for (int i = 0; i < IRSENSORS_COUNT; i++) ir_local[i] = g_sensor.ir[i];
        node_local = g_sensor.node;
        tl         = g_target.left;
        tr         = g_target.right;
        cl         = g_current.left;
        cr         = g_current.right;
        last_cmd   = g_last_cmd;
        map_state  = g_map_state;
         run_mode   = g_run_mode;
         solve_state = g_solve_state;
        nc         = g_node_count;
        pn         = g_past_node;
        taskEXIT_CRITICAL();

         printf("DBG mode=%-10s map=%-10s solve=%-10s cmd=%-10s node=%c past=%c count=%3d "
               "ir=[%3u,%3u,%3u,%3u,%3u] tgt=[%4d,%4d] pwm=[%4d,%4d]\n",
             run_mode_name(run_mode), map_state_name(map_state), solve_state_name(solve_state), cmd_name(last_cmd),
               node_local, pn, nc,
               ir_local[0], ir_local[1], ir_local[2], ir_local[3], ir_local[4],
               tl, tr, cl, cr);

        vTaskDelayUntil(&last, pdMS_TO_TICKS(DEBUG_PERIOD_MS));
    }
}

/* ================================================================
 * Main
 * ================================================================ */
int main(void) {
    stdio_init_all();
    sleep_ms(200);

    if (cyw43_arch_init() != 0) {
        for (;;) tight_loop_contents();
    }

    motors_init();
    sensors_init();
    buttons_init();
    robot_apply_cmd(ROBOT_STOP);

    xTaskCreate(led_task,          "LED",       2048, NULL, 1, NULL);
    xTaskCreate(sensor_task,       "SENSOR",    2048, NULL, 3, NULL);
    xTaskCreate(map_fsm_task,      "MAP_FSM",   4096, NULL, 2, NULL);
    xTaskCreate(motor_control_task,"MOTOR_CTRL",2048, NULL, 3, NULL);
    xTaskCreate(serial_debug_task, "DEBUG",     3072, NULL, 1, NULL);

    vTaskStartScheduler();
    while (1) tight_loop_contents();
}