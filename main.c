#ifndef F_CPU
#define F_CPU 3333333UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdint.h>
#include <limits.h>
#include <string.h>

#define BAUD_RATE 9600UL
#define RX_ECHO_ENABLED 0
#define UART_RX_BUDGET_PER_TICK 32U

// ATmega4809 Curiosity-Nano style default VCP mapping is typically USART3 on PB0/PB1.
#define VCP_USART USART3
#define VCP_TX_PORT PORTB
#define VCP_RX_PORT PORTB
#define VCP_TX_PIN PIN0_bm
#define VCP_RX_PIN PIN1_bm

// 3x4 key matrix
// Coordinate mapping:
// row 0 -> PA7, row 1 -> PA6, row 2 -> PA5
// col 0 -> PC0, col 1 -> PC1, col 2 -> PC2, col 3 -> PC3
#define MATRIX_ROW_PORT PORTA
#define MATRIX_COL_PORT PORTC
#define MATRIX_ROWS 3U
#define MATRIX_COLS 4U
#define MATRIX_DEBOUNCE_MS 20U

#define MATRIX_ROW0_PIN PIN7_bm
#define MATRIX_ROW1_PIN PIN6_bm
#define MATRIX_ROW2_PIN PIN5_bm
#define MATRIX_ROWS_MASK (MATRIX_ROW0_PIN | MATRIX_ROW1_PIN | MATRIX_ROW2_PIN)

#define MATRIX_COL0_PIN PIN0_bm
#define MATRIX_COL1_PIN PIN1_bm
#define MATRIX_COL2_PIN PIN2_bm
#define MATRIX_COL3_PIN PIN3_bm
#define MATRIX_COLS_MASK (MATRIX_COL0_PIN | MATRIX_COL1_PIN | MATRIX_COL2_PIN | MATRIX_COL3_PIN)

// Rotary encoder pins
#define ENC_PORT PORTE
#define ENC_SW_PIN  PIN1_bm
#define ENC_DT_PIN  PIN2_bm
#define ENC_CLK_PIN PIN3_bm

// OLED I2C pins (TWI0 default route): SCL=PA3, SDA=PA2
#define OLED_ADDR_7BIT 0x3CU
#define I2C_FREQ_HZ 100000UL
#define OLED_ENABLED 1
#define OLED_BUFFER_SIZE 1024U
#define OLED_RENDER_WIDTH_PX 128U
#define OLED_RENDER_LINES 8U
#define OLED_FLUSH_CHUNK_BYTES 8U
#define OLED_TEXT_SEGMENTS 8U
#define OLED_TEXT_MAX_CHARS 64U
#define EEPROM_MAGIC 0xA5C35A7EUL
#define EEPROM_SAVE_DELAY_MS 750U

static uint32_t EEMEM ee_magic;
static uint32_t EEMEM ee_rx_enc_u32;
static uint8_t g_oled_buffer[OLED_BUFFER_SIZE];
static char g_oled_lines[OLED_TEXT_SEGMENTS][OLED_TEXT_MAX_CHARS + 1U] = {
    "READY",
    "TEXT FROM PC",
    "",
    ""
};
static bool g_oled_text_dirty = false;
static uint8_t g_oled_render_holdoff_ms = 0U;
static bool g_oled_flush_active = false;
static uint8_t g_oled_flush_page = 0U;
static uint8_t g_oled_flush_col = 0U;
static volatile int16_t g_enc_pending_steps = 0;
static volatile int8_t g_enc_accum = 0;
static volatile uint8_t g_enc_prev_state = 0;

static void oled_render_text(void);

static void uart_init(void)
{
#if defined(PORTMUX_USART3_gm) && defined(PORTMUX_USART3_DEFAULT_gc)
    PORTMUX.USARTROUTEA = (PORTMUX.USARTROUTEA & ~PORTMUX_USART3_gm) | PORTMUX_USART3_DEFAULT_gc;
#endif

    VCP_TX_PORT.DIRSET = VCP_TX_PIN;
    VCP_RX_PORT.DIRCLR = VCP_RX_PIN;

    // megaAVR 0-series async normal mode baud register value.
    VCP_USART.BAUD = (uint16_t)((((uint32_t)4U * F_CPU) + (BAUD_RATE / 2U)) / BAUD_RATE);
    VCP_USART.CTRLB = USART_TXEN_bm | USART_RXEN_bm;
    VCP_USART.CTRLC = USART_CHSIZE_8BIT_gc;
}

static void uart_write_byte(uint8_t b)
{
    while ((VCP_USART.STATUS & USART_DREIF_bm) == 0U) {;}
    VCP_USART.TXDATAL = b;
}

static void uart_write_str(const char *s)
{
    while (*s != '\0')
    {
        uart_write_byte((uint8_t)*s++);
    }
}

static bool uart_read_byte(uint8_t *out)
{
    if ((VCP_USART.STATUS & USART_RXCIF_bm) == 0U)
    {
        return false;
    }

    *out = VCP_USART.RXDATAL;
    return true;
}

static void io_init(void)
{
    // Matrix rows are driven outputs, idle high.
    MATRIX_ROW_PORT.DIRSET = MATRIX_ROWS_MASK;
    MATRIX_ROW_PORT.OUTSET = MATRIX_ROWS_MASK;

    // Matrix columns are inputs with internal pull-ups.
    MATRIX_COL_PORT.DIRCLR = MATRIX_COLS_MASK;
    MATRIX_COL_PORT.PIN0CTRL = PORT_PULLUPEN_bm;
    MATRIX_COL_PORT.PIN1CTRL = PORT_PULLUPEN_bm;
    MATRIX_COL_PORT.PIN2CTRL = PORT_PULLUPEN_bm;
    MATRIX_COL_PORT.PIN3CTRL = PORT_PULLUPEN_bm;

    // Rotary encoder inputs with internal pull-ups (SW=PE1, DT=PE2, CLK=PE3).
    ENC_PORT.DIRCLR = ENC_SW_PIN | ENC_DT_PIN | ENC_CLK_PIN;
    ENC_PORT.PIN1CTRL = PORT_PULLUPEN_bm;
    ENC_PORT.PIN2CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;
    ENC_PORT.PIN3CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;
}

static inline uint8_t matrix_row_pin_mask(uint8_t row)
{
    switch (row)
    {
        case 0U: return MATRIX_ROW0_PIN;
        case 1U: return MATRIX_ROW1_PIN;
        default: return MATRIX_ROW2_PIN;
    }
}

static inline uint8_t matrix_col_pin_mask(uint8_t col)
{
    switch (col)
    {
        case 0U: return MATRIX_COL0_PIN;
        case 1U: return MATRIX_COL1_PIN;
        case 2U: return MATRIX_COL2_PIN;
        default: return MATRIX_COL3_PIN;
    }
}

static void matrix_select_row(uint8_t row)
{
    MATRIX_ROW_PORT.OUTSET = MATRIX_ROWS_MASK;
    MATRIX_ROW_PORT.OUTCLR = matrix_row_pin_mask(row);
}

static uint8_t matrix_scan_row_pressed_cols(uint8_t row)
{
    uint8_t pressed_cols = 0U;
    uint8_t sample;

    matrix_select_row(row);
    _delay_us(3);
    sample = MATRIX_COL_PORT.IN;

    for (uint8_t col = 0U; col < MATRIX_COLS; col++)
    {
        if ((sample & matrix_col_pin_mask(col)) == 0U)
        {
            pressed_cols |= (uint8_t)(1U << col);
        }
    }

    return pressed_cols;
}

static void send_key_state(uint8_t row, uint8_t col, bool pressed)
{
    uart_write_str("KEY=");
    uart_write_byte((uint8_t)('0' + row));
    uart_write_byte(',');
    uart_write_byte((uint8_t)('0' + col));
    uart_write_byte(',');
    uart_write_byte(pressed ? '1' : '0');
    uart_write_str("\r\n");
}

static void send_encoder_step(int8_t step)
{
    if (step > 0)
    {
        uart_write_str("ENC=+1\r\n");
    }
    else if (step < 0)
    {
        uart_write_str("ENC=-1\r\n");
    }
}

static void send_encoder_button(bool pressed)
{
    uart_write_str("ENC_SW=");
    uart_write_byte(pressed ? '1' : '0');
    uart_write_str("\r\n");
}

#if RX_ECHO_ENABLED
static void uart_echo_raw_line(const char *line)
{
    uart_write_str("RAW:[");
    uart_write_str(line);
    uart_write_str("]\n");
}
#endif

static inline uint8_t encoder_state_2bit(void)
{
    uint8_t s = 0U;
    if ((ENC_PORT.IN & ENC_CLK_PIN) != 0U) s |= 0x02U;
    if ((ENC_PORT.IN & ENC_DT_PIN) != 0U)  s |= 0x01U;
    return s;
}

static int8_t encoder_transition_delta(uint8_t prev_state, uint8_t curr_state)
{
    // Quadrature state transition table (Gray code).
    switch ((uint8_t)((prev_state << 2) | curr_state))
    {
        case 0x01: // 00 -> 01
        case 0x07: // 01 -> 11
        case 0x0E: // 11 -> 10
        case 0x08: // 10 -> 00
            return +1;

        case 0x02: // 00 -> 10
        case 0x04: // 01 -> 00
        case 0x0D: // 11 -> 01
        case 0x0B: // 10 -> 11
            return -1;

        default:
            return 0;
    }
}

ISR(PORTE_PORT_vect)
{
    uint8_t flags = ENC_PORT.INTFLAGS;
    ENC_PORT.INTFLAGS = flags;

    if ((flags & (ENC_CLK_PIN | ENC_DT_PIN)) == 0U)
    {
        return;
    }

    {
        uint8_t enc_curr = encoder_state_2bit();
        int8_t enc_delta = encoder_transition_delta(g_enc_prev_state, enc_curr);
        g_enc_prev_state = enc_curr;

        if (enc_delta != 0)
        {
            g_enc_accum = (int8_t)(g_enc_accum + enc_delta);

            if (g_enc_accum >= 4)
            {
                if (g_enc_pending_steps < INT16_MAX)
                {
                    g_enc_pending_steps++;
                }
                g_enc_accum = 0;
            }
            else if (g_enc_accum <= -4)
            {
                if (g_enc_pending_steps > INT16_MIN)
                {
                    g_enc_pending_steps--;
                }
                g_enc_accum = 0;
            }
        }
    }
}

static bool parse_i32(const char *s, int32_t *out)
{
    bool neg = false;
    int64_t v = 0;

    if (*s == '\0') return false;

    if (*s == '+')
    {
        s++;
    }
    else if (*s == '-')
    {
        neg = true;
        s++;
    }

    if ((*s < '0') || (*s > '9')) return false;

    while ((*s >= '0') && (*s <= '9'))
    {
        v = (v * 10) + (int64_t)(*s - '0');
        if (v > 2147483648LL) return false;
        s++;
    }

    if (*s != '\0') return false;

    if (neg)
    {
        v = -v;
        if (v < (int64_t)INT32_MIN) return false;
    }
    else if (v > (int64_t)INT32_MAX)
    {
        return false;
    }

    *out = (int32_t)v;
    return true;
}

static void copy_oled_text_trimmed(char *dst, const char *src, uint8_t span_len, uint8_t max_chars)
{
    uint8_t start = 0U;
    uint8_t end = span_len;
    uint8_t n = 0U;

    while ((start < end) && (src[start] == ' '))
    {
        start++;
    }

    while ((end > start) && (src[end - 1U] == ' '))
    {
        end--;
    }

    // Remove matching single/double quotes around full span.
    if ((end > (uint8_t)(start + 1U)) &&
        ((src[start] == '\'' && src[end - 1U] == '\'') ||
         (src[start] == '"'  && src[end - 1U] == '"')))
    {
        start++;
        end--;
    }

    while ((start < end) && (n < max_chars))
    {
        char c = src[start++];
        if ((c < ' ') || (c > '~')) c = ' ';
        dst[n] = c;
        n++;
    }

    dst[n] = '\0';
}

static void oled_set_text_payload(const char *payload)
{
    uint8_t line_idx = 0U;
    const char *span_start = payload;
    const char *p = payload;

    while (line_idx < OLED_TEXT_SEGMENTS)
    {
        while ((*p != '\0') && (*p != '|'))
        {
            p++;
        }

        copy_oled_text_trimmed(
            g_oled_lines[line_idx],
            span_start,
            (uint8_t)(p - span_start),
            OLED_TEXT_MAX_CHARS
        );
        line_idx++;

        if (*p == '\0')
        {
            break;
        }

        p++;
        span_start = p;
    }

    while (line_idx < OLED_TEXT_SEGMENTS)
    {
        g_oled_lines[line_idx][0] = '\0';
        line_idx++;
    }

    g_oled_text_dirty = true;
    g_oled_render_holdoff_ms = 30U;
}

static void handle_command(const char *line, int32_t *rx_enc_value)
{
    if (strncmp(line, "ENC=", 4) == 0)
    {
        const char *v = &line[4];

        if ((v[0] == '+') && (v[1] == '1') && (v[2] == '\0'))
        {
            (*rx_enc_value)++;
            return;
        }

        if ((v[0] == '-') && (v[1] == '1') && (v[2] == '\0'))
        {
            (*rx_enc_value)--;
            return;
        }

        {
            int32_t parsed;
            if (parse_i32(v, &parsed))
            {
                *rx_enc_value = parsed;
            }
        }
        return;
    }

    if (strncmp(line, "TXT:", 4) == 0)
    {
        oled_set_text_payload(&line[4]);
        return;
    }

    if (strncmp(line, "OLED=", 5) == 0)
    {
        oled_set_text_payload(&line[5]);
        return;
    }

    if (strncmp(line, "TXT=", 4) == 0)
    {
        oled_set_text_payload(&line[4]);
        return;
    }

    if (strcmp(line, "CLR") == 0)
    {
        oled_set_text_payload("");
        return;
    }

    // Treat any other line as display text.
    if (line[0] != '\0')
    {
        oled_set_text_payload(line);
    }
}

static char rx_buffer[128] = {0};
static uint8_t rx_index = 0U;

static void serial_task(int32_t *rx_enc_value)
{
    uint8_t c;
    uint8_t budget = UART_RX_BUDGET_PER_TICK;

    while ((budget-- > 0U) && uart_read_byte(&c))
    {
        if (c == '\r')
        {
            continue;
        }

        if (c == '\n')
        {
            rx_buffer[rx_index] = '\0';
#if RX_ECHO_ENABLED
            uart_echo_raw_line(rx_buffer);
#endif
            handle_command(rx_buffer, rx_enc_value);

            rx_index = 0U;
            memset(rx_buffer, 0, sizeof(rx_buffer));
            continue;
        }

        if (rx_index < (uint8_t)(sizeof(rx_buffer) - 1U))
        {
            rx_buffer[rx_index++] = (char)c;
        }
    }
}

#if OLED_ENABLED
static void twi0_init(void)
{
#if defined(PORTMUX_TWI0_gm) && defined(PORTMUX_TWI0_DEFAULT_gc)
    PORTMUX.TWISPIROUTEA = (PORTMUX.TWISPIROUTEA & ~PORTMUX_TWI0_gm) | PORTMUX_TWI0_DEFAULT_gc;
#endif

    // Keep lines released high and enable weak pull-ups.
    PORTA.DIRSET = PIN2_bm | PIN3_bm;
    PORTA.OUTSET = PIN2_bm | PIN3_bm;
    PORTA.PIN2CTRL = PORT_PULLUPEN_bm;
    PORTA.PIN3CTRL = PORT_PULLUPEN_bm;

    {
        uint32_t baud = ((F_CPU / I2C_FREQ_HZ) - 10UL) / 2UL;
        if (baud > 255UL) baud = 255UL;
        TWI0.MBAUD = (uint8_t)baud;
    }

    TWI0.MCTRLA = TWI_ENABLE_bm;
    TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
}

static bool twi0_wait_wif(void)
{
    uint16_t timeout = 0;
    while ((TWI0.MSTATUS & TWI_WIF_bm) == 0U)
    {
        if ((TWI0.MSTATUS & (TWI_ARBLOST_bm | TWI_BUSERR_bm)) != 0U)
        {
            return false;
        }
        timeout++;
        if (timeout == 0U)
        {
            return false;
        }
    }
    return true;
}

static bool twi0_start_write(uint8_t addr7)
{
    TWI0.MADDR = (uint8_t)(addr7 << 1);
    if (!twi0_wait_wif()) return false;

    if ((TWI0.MSTATUS & TWI_RXACK_bm) != 0U)
    {
        TWI0.MCTRLB = TWI_MCMD_STOP_gc;
        return false;
    }

    return true;
}

static bool twi0_write_byte(uint8_t data)
{
    TWI0.MDATA = data;
    if (!twi0_wait_wif()) return false;

    if ((TWI0.MSTATUS & TWI_RXACK_bm) != 0U)
    {
        TWI0.MCTRLB = TWI_MCMD_STOP_gc;
        return false;
    }

    return true;
}

static void twi0_stop(void)
{
    TWI0.MCTRLB = TWI_MCMD_STOP_gc;
}

static bool ssd1306_cmds(const uint8_t *cmds, uint8_t len)
{
    if (!twi0_start_write(OLED_ADDR_7BIT)) return false;
    if (!twi0_write_byte(0x00U)) { twi0_stop(); return false; }

    while (len--)
    {
        if (!twi0_write_byte(*cmds++))
        {
            twi0_stop();
            return false;
        }
    }

    twi0_stop();
    return true;
}

static bool ssd1306_set_cursor(uint8_t page, uint8_t col)
{
    uint8_t cmds[3];
    cmds[0] = (uint8_t)(0xB0U | (page & 0x07U));
    cmds[1] = (uint8_t)(0x00U | (col & 0x0FU));
    cmds[2] = (uint8_t)(0x10U | ((col >> 4) & 0x0FU));
    return ssd1306_cmds(cmds, 3);
}

static const uint8_t *font5x7(char c)
{
    static const uint8_t blank[5] = {0x00,0x00,0x00,0x00,0x00};
    static const uint8_t exclam[5] = {0x00,0x00,0x5F,0x00,0x00};
    static const uint8_t quote[5] = {0x00,0x07,0x00,0x07,0x00};
    static const uint8_t hash[5] = {0x14,0x7F,0x14,0x7F,0x14};
    static const uint8_t dollar[5] = {0x24,0x2A,0x7F,0x2A,0x12};
    static const uint8_t percent[5] = {0x23,0x13,0x08,0x64,0x62};
    static const uint8_t ampersand[5] = {0x36,0x49,0x55,0x22,0x50};
    static const uint8_t apostrophe[5] = {0x00,0x05,0x03,0x00,0x00};
    static const uint8_t lparen[5] = {0x00,0x1C,0x22,0x41,0x00};
    static const uint8_t rparen[5] = {0x00,0x41,0x22,0x1C,0x00};
    static const uint8_t star[5] = {0x14,0x08,0x3E,0x08,0x14};
    static const uint8_t plus[5] = {0x08,0x08,0x3E,0x08,0x08};
    static const uint8_t comma[5] = {0x00,0x50,0x30,0x00,0x00};
    static const uint8_t period[5] = {0x00,0x60,0x60,0x00,0x00};
    static const uint8_t colon[5] = {0x00,0x36,0x36,0x00,0x00};
    static const uint8_t semicolon[5] = {0x00,0x56,0x36,0x00,0x00};
    static const uint8_t less[5] = {0x08,0x14,0x22,0x41,0x00};
    static const uint8_t equals[5] = {0x14,0x14,0x14,0x14,0x14};
    static const uint8_t greater[5] = {0x00,0x41,0x22,0x14,0x08};
    static const uint8_t question[5] = {0x02,0x01,0x51,0x09,0x06};
    static const uint8_t at[5] = {0x32,0x49,0x79,0x41,0x3E};
    static const uint8_t slash[5] = {0x20,0x10,0x08,0x04,0x02};
    static const uint8_t minus[5] = {0x08,0x08,0x08,0x08,0x08};
    static const uint8_t lbrack[5] = {0x00,0x7F,0x41,0x41,0x00};
    static const uint8_t backslash[5] = {0x02,0x04,0x08,0x10,0x20};
    static const uint8_t rbrack[5] = {0x00,0x41,0x41,0x7F,0x00};
    static const uint8_t caret[5] = {0x04,0x02,0x01,0x02,0x04};
    static const uint8_t underscore[5] = {0x40,0x40,0x40,0x40,0x40};
    static const uint8_t pipe[5] = {0x00,0x00,0x7F,0x00,0x00};
    static const uint8_t tilde[5] = {0x08,0x04,0x08,0x10,0x08};
    static const uint8_t d0[5] = {0x3E,0x51,0x49,0x45,0x3E};
    static const uint8_t d1[5] = {0x00,0x42,0x7F,0x40,0x00};
    static const uint8_t d2[5] = {0x42,0x61,0x51,0x49,0x46};
    static const uint8_t d3[5] = {0x21,0x41,0x45,0x4B,0x31};
    static const uint8_t d4[5] = {0x18,0x14,0x12,0x7F,0x10};
    static const uint8_t d5[5] = {0x27,0x45,0x45,0x45,0x39};
    static const uint8_t d6[5] = {0x3C,0x4A,0x49,0x49,0x30};
    static const uint8_t d7[5] = {0x01,0x71,0x09,0x05,0x03};
    static const uint8_t d8[5] = {0x36,0x49,0x49,0x49,0x36};
    static const uint8_t d9[5] = {0x06,0x49,0x49,0x29,0x1E};

    static const uint8_t A[5] = {0x7E,0x11,0x11,0x11,0x7E};
    static const uint8_t B[5] = {0x7F,0x49,0x49,0x49,0x36};
    static const uint8_t C[5] = {0x3E,0x41,0x41,0x41,0x22};
    static const uint8_t D[5] = {0x7F,0x41,0x41,0x22,0x1C};
    static const uint8_t E[5] = {0x7F,0x49,0x49,0x49,0x41};
    static const uint8_t F[5] = {0x7F,0x09,0x09,0x09,0x01};
    static const uint8_t G[5] = {0x3E,0x41,0x49,0x49,0x3A};
    static const uint8_t H[5] = {0x7F,0x08,0x08,0x08,0x7F};
    static const uint8_t I[5] = {0x00,0x41,0x7F,0x41,0x00};
    static const uint8_t J[5] = {0x20,0x40,0x41,0x3F,0x01};
    static const uint8_t K[5] = {0x7F,0x08,0x14,0x22,0x41};
    static const uint8_t L[5] = {0x7F,0x40,0x40,0x40,0x40};
    static const uint8_t M[5] = {0x7F,0x02,0x0C,0x02,0x7F};
    static const uint8_t N[5] = {0x7F,0x04,0x08,0x10,0x7F};
    static const uint8_t O[5] = {0x3E,0x41,0x41,0x41,0x3E};
    static const uint8_t P[5] = {0x7F,0x09,0x09,0x09,0x06};
    static const uint8_t Q[5] = {0x3E,0x41,0x51,0x21,0x5E};
    static const uint8_t R[5] = {0x7F,0x09,0x19,0x29,0x46};
    static const uint8_t S[5] = {0x46,0x49,0x49,0x49,0x31};
    static const uint8_t T[5] = {0x01,0x01,0x7F,0x01,0x01};
    static const uint8_t U[5] = {0x3F,0x40,0x40,0x40,0x3F};
    static const uint8_t V[5] = {0x1F,0x20,0x40,0x20,0x1F};
    static const uint8_t W[5] = {0x7F,0x20,0x18,0x20,0x7F};
    static const uint8_t X[5] = {0x63,0x14,0x08,0x14,0x63};
    static const uint8_t Y[5] = {0x03,0x04,0x78,0x04,0x03};
    static const uint8_t Z[5] = {0x61,0x51,0x49,0x45,0x43};
    static const uint8_t a[5] = {0x20,0x54,0x54,0x54,0x78};
    static const uint8_t b[5] = {0x7F,0x48,0x44,0x44,0x38};
    static const uint8_t c_lower[5] = {0x38,0x44,0x44,0x44,0x20};
    static const uint8_t d[5] = {0x38,0x44,0x44,0x48,0x7F};
    static const uint8_t e_lower[5] = {0x38,0x54,0x54,0x54,0x18};
    static const uint8_t f_lower[5] = {0x08,0x7E,0x09,0x01,0x02};
    static const uint8_t g[5] = {0x08,0x14,0x54,0x54,0x3C};
    static const uint8_t h_lower[5] = {0x7F,0x08,0x04,0x04,0x78};
    static const uint8_t i_lower[5] = {0x00,0x44,0x7D,0x40,0x00};
    static const uint8_t j_lower[5] = {0x20,0x40,0x44,0x3D,0x00};
    static const uint8_t k_lower[5] = {0x7F,0x10,0x28,0x44,0x00};
    static const uint8_t l_lower[5] = {0x00,0x41,0x7F,0x40,0x00};
    static const uint8_t m_lower[5] = {0x7C,0x04,0x18,0x04,0x78};
    static const uint8_t n_lower[5] = {0x7C,0x08,0x04,0x04,0x78};
    static const uint8_t o_lower[5] = {0x38,0x44,0x44,0x44,0x38};
    static const uint8_t p_lower[5] = {0x7C,0x14,0x14,0x14,0x08};
    static const uint8_t q_lower[5] = {0x08,0x14,0x14,0x18,0x7C};
    static const uint8_t r_lower[5] = {0x7C,0x08,0x04,0x04,0x08};
    static const uint8_t s_lower[5] = {0x48,0x54,0x54,0x54,0x20};
    static const uint8_t t_lower[5] = {0x04,0x3F,0x44,0x40,0x20};
    static const uint8_t u_lower[5] = {0x3C,0x40,0x40,0x20,0x7C};
    static const uint8_t v_lower[5] = {0x1C,0x20,0x40,0x20,0x1C};
    static const uint8_t w_lower[5] = {0x3C,0x40,0x30,0x40,0x3C};
    static const uint8_t x_lower[5] = {0x44,0x28,0x10,0x28,0x44};
    static const uint8_t y_lower[5] = {0x0C,0x50,0x50,0x50,0x3C};
    static const uint8_t z_lower[5] = {0x44,0x64,0x54,0x4C,0x44};

    switch (c)
    {
        case ' ': return blank;
        case '!': return exclam;
        case '"': return quote;
        case '#': return hash;
        case '$': return dollar;
        case '%': return percent;
        case '&': return ampersand;
        case '\'': return apostrophe;
        case '(': return lparen;
        case ')': return rparen;
        case '*': return star;
        case '+': return plus;
        case ',': return comma;
        case '.': return period;
        case ':': return colon;
        case ';': return semicolon;
        case '<': return less;
        case '=': return equals;
        case '>': return greater;
        case '?': return question;
        case '@': return at;
        case '/': return slash;
        case '-': return minus;
        case '[': return lbrack;
        case '\\': return backslash;
        case ']': return rbrack;
        case '^': return caret;
        case '_': return underscore;
        case '|': return pipe;
        case '~': return tilde;
        case '0': return d0;
        case '1': return d1;
        case '2': return d2;
        case '3': return d3;
        case '4': return d4;
        case '5': return d5;
        case '6': return d6;
        case '7': return d7;
        case '8': return d8;
        case '9': return d9;
        case 'A': return A;
        case 'B': return B;
        case 'C': return C;
        case 'D': return D;
        case 'E': return E;
        case 'F': return F;
        case 'G': return G;
        case 'H': return H;
        case 'I': return I;
        case 'J': return J;
        case 'K': return K;
        case 'L': return L;
        case 'M': return M;
        case 'N': return N;
        case 'O': return O;
        case 'P': return P;
        case 'Q': return Q;
        case 'R': return R;
        case 'S': return S;
        case 'T': return T;
        case 'U': return U;
        case 'V': return V;
        case 'W': return W;
        case 'X': return X;
        case 'Y': return Y;
        case 'Z': return Z;
        case 'a': return a;
        case 'b': return b;
        case 'c': return c_lower;
        case 'd': return d;
        case 'e': return e_lower;
        case 'f': return f_lower;
        case 'g': return g;
        case 'h': return h_lower;
        case 'i': return i_lower;
        case 'j': return j_lower;
        case 'k': return k_lower;
        case 'l': return l_lower;
        case 'm': return m_lower;
        case 'n': return n_lower;
        case 'o': return o_lower;
        case 'p': return p_lower;
        case 'q': return q_lower;
        case 'r': return r_lower;
        case 's': return s_lower;
        case 't': return t_lower;
        case 'u': return u_lower;
        case 'v': return v_lower;
        case 'w': return w_lower;
        case 'x': return x_lower;
        case 'y': return y_lower;
        case 'z': return z_lower;
        default:  return blank;
    }
}

static uint8_t font_glyph_span(const uint8_t *glyph, char c, uint8_t *start_col)
{
    uint8_t first = 0U;
    uint8_t last = 0U;
    bool found = false;
    uint8_t i;

    if (c == ' ')
    {
        *start_col = 0U;
        return 3U;
    }

    for (i = 0U; i < 5U; i++)
    {
        if (glyph[i] != 0U)
        {
            if (!found)
            {
                first = i;
                found = true;
            }
            last = i;
        }
    }

    if (!found)
    {
        *start_col = 0U;
        return 3U;
    }

    *start_col = first;
    return (uint8_t)(last - first + 1U);
}

static void ssd1306_clear(void)
{
    for (uint8_t page = 0; page < 8U; page++)
    {
        if (!ssd1306_set_cursor(page, 0U)) continue;
        if (!twi0_start_write(OLED_ADDR_7BIT)) continue;
        if (!twi0_write_byte(0x40U)) { twi0_stop(); continue; }

        for (uint8_t i = 0; i < 128U; i++)
        {
            if (!twi0_write_byte(0x00U)) break;
        }

        twi0_stop();
    }
}

static void oled_flush_buffer(void)
{
    g_oled_flush_active = true;
    g_oled_flush_page = 0U;
    g_oled_flush_col = 0U;
}

static void oled_flush_step(void)
{
    uint8_t page;
    uint8_t col;
    uint8_t chunk;
    uint16_t base;

    if (!g_oled_flush_active)
    {
        return;
    }

    page = g_oled_flush_page;
    col = g_oled_flush_col;

    chunk = OLED_FLUSH_CHUNK_BYTES;
    if ((uint16_t)col + chunk > OLED_RENDER_WIDTH_PX)
    {
        chunk = (uint8_t)(OLED_RENDER_WIDTH_PX - col);
    }

    base = ((uint16_t)page * OLED_RENDER_WIDTH_PX) + col;

    if (ssd1306_set_cursor(page, col) &&
        twi0_start_write(OLED_ADDR_7BIT) &&
        twi0_write_byte(0x40U))
    {
        for (uint8_t i = 0U; i < chunk; i++)
        {
            if (!twi0_write_byte(g_oled_buffer[base + i]))
            {
                break;
            }
        }
        twi0_stop();
    }

    col = (uint8_t)(col + chunk);
    if (col >= OLED_RENDER_WIDTH_PX)
    {
        col = 0U;
        page++;
    }

    if (page >= OLED_RENDER_LINES)
    {
        g_oled_flush_active = false;
        return;
    }

    g_oled_flush_page = page;
    g_oled_flush_col = col;
}

static uint8_t oled_char_advance(char c)
{
    const uint8_t *glyph = font5x7(c);
    uint8_t start_col = 0U;
    uint8_t span = font_glyph_span(glyph, c, &start_col);

    if (c == ' ')
    {
        return span;
    }

    return (uint8_t)(span + 1U);
}

static void oled_draw_char_to_buffer(uint8_t page, uint8_t x, char c)
{
    const uint8_t *glyph = font5x7(c);
    uint8_t start_col = 0U;
    uint8_t span = font_glyph_span(glyph, c, &start_col);
    uint16_t base;

    if ((page >= OLED_RENDER_LINES) || (x >= OLED_RENDER_WIDTH_PX) || (c == ' '))
    {
        return;
    }

    base = (uint16_t)page * OLED_RENDER_WIDTH_PX;

    for (uint8_t i = 0U; (i < span) && ((uint16_t)x + i < OLED_RENDER_WIDTH_PX); i++)
    {
        g_oled_buffer[base + x + i] = glyph[start_col + i];
    }
}

static void ssd1306_init(void)
{
    static const uint8_t init_cmds[] = {
        0xAE,       // display off
        0x20, 0x00, // horizontal addressing mode
        0xB0,       // page start
        0xC8,       // COM scan dec
        0x00,       // low column
        0x10,       // high column
        0x40,       // start line
        0x81, 0x7F, // contrast
        0xA1,       // segment remap
        0xA6,       // normal display
        0xA8, 0x3F, // multiplex 1/64
        0xA4,       // output follows RAM
        0xD3, 0x00, // offset
        0xD5, 0x80, // clock divide
        0xD9, 0xF1, // pre-charge
        0xDA, 0x12, // COM pins
        0xDB, 0x40, // VCOMH
        0x8D, 0x14, // charge pump on
        0xAF        // display on
    };

    (void)ssd1306_cmds(init_cmds, (uint8_t)sizeof(init_cmds));
    ssd1306_clear();
}

static void oled_render_text(void)
{
    uint8_t render_line = 0U;
    bool first_segment = true;

    memset(g_oled_buffer, 0, sizeof(g_oled_buffer));

    for (uint8_t seg = 0U; (seg < OLED_TEXT_SEGMENTS) && (render_line < OLED_RENDER_LINES); seg++)
    {
        const char *s = g_oled_lines[seg];
        uint8_t x = 0U;

        if (!first_segment)
        {
            render_line++;
            if (render_line >= OLED_RENDER_LINES)
            {
                break;
            }
        }
        first_segment = false;

        while ((*s != '\0') && (render_line < OLED_RENDER_LINES))
        {
            uint8_t adv = oled_char_advance(*s);

            if ((x > 0U) && ((uint16_t)x + adv > OLED_RENDER_WIDTH_PX))
            {
                render_line++;
                x = 0U;
                if (render_line >= OLED_RENDER_LINES)
                {
                    break;
                }
            }

            oled_draw_char_to_buffer(render_line, x, *s);
            x = (uint8_t)(x + adv);
            s++;
        }
    }

    oled_flush_buffer();
}
#else
static void oled_render_text(void)
{
}

static void oled_flush_step(void)
{
}
#endif

static void settings_load(int32_t *rx_enc_value)
{
    if (eeprom_read_dword(&ee_magic) == EEPROM_MAGIC)
    {
        *rx_enc_value = (int32_t)eeprom_read_dword(&ee_rx_enc_u32);
        return;
    }

    // First run / invalid EEPROM contents.
    *rx_enc_value = 0;
    eeprom_update_dword(&ee_rx_enc_u32, (uint32_t)*rx_enc_value);
    eeprom_update_dword(&ee_magic, EEPROM_MAGIC);
}

static void settings_save(int32_t rx_enc_value)
{
    eeprom_update_dword(&ee_rx_enc_u32, (uint32_t)rx_enc_value);
    eeprom_update_dword(&ee_magic, EEPROM_MAGIC);
}

int main(void)
{
    uart_init();
    io_init();
    g_enc_prev_state = encoder_state_2bit();
    sei();
#if OLED_ENABLED
    twi0_init();
    ssd1306_init();
#endif

    uart_write_str("\r\nATmega4809 ready. MATRIX R=PA7/PA6/PA5 C=PC0..PC3, ENC SW=PE1 DT=PE2 CLK=PE3, OLED TXT RX enabled\r\n");

    int32_t rx_enc_value;
    settings_load(&rx_enc_value);
#if OLED_ENABLED
    oled_render_text();
    g_oled_text_dirty = false;
    g_oled_render_holdoff_ms = 0U;
#endif
    bool nv_dirty = false;
    uint16_t nv_dirty_ms = 0;

    // Matrix key debounce state.
    bool key_state[MATRIX_ROWS][MATRIX_COLS] = {{false}};
    uint8_t key_debounce_ms[MATRIX_ROWS][MATRIX_COLS] = {{0U}};

    bool enc_sw_last_sample = true; // pull-up idle high
    bool enc_sw_stable = true;
    uint8_t enc_sw_stable_ms = 0;

    while (1)
    {
        int32_t enc_before = rx_enc_value;
        serial_task(&rx_enc_value);
        if (rx_enc_value != enc_before)
        {
            nv_dirty = true;
            nv_dirty_ms = 0;
        }

        if (g_oled_render_holdoff_ms > 0U)
        {
            g_oled_render_holdoff_ms--;
        }

#if OLED_ENABLED
        if (g_oled_text_dirty && (g_oled_render_holdoff_ms == 0U) && !g_oled_flush_active)
        {
            oled_render_text();
            g_oled_text_dirty = false;
        }

        oled_flush_step();
#endif

        // Drain encoder steps captured by interrupt and send to PC.
        {
            int16_t pending_steps;
            cli();
            pending_steps = g_enc_pending_steps;
            g_enc_pending_steps = 0;
            sei();

            if (pending_steps > 0)
            {
                rx_enc_value += pending_steps;
                while (pending_steps-- > 0)
                {
                    send_encoder_step(+1);
                }
                nv_dirty = true;
                nv_dirty_ms = 0;
            }
            else if (pending_steps < 0)
            {
                rx_enc_value += pending_steps;
                while (pending_steps++ < 0)
                {
                    send_encoder_step(-1);
                }
                nv_dirty = true;
                nv_dirty_ms = 0;
            }
        }

        // Rotary encoder push button (PE1): pressed=LOW.
        {
            bool sample_high = (ENC_PORT.IN & ENC_SW_PIN) != 0U;

            if (sample_high != enc_sw_last_sample)
            {
                enc_sw_last_sample = sample_high;
                enc_sw_stable_ms = 0U;
            }
            else if (enc_sw_stable_ms < 20U)
            {
                enc_sw_stable_ms++;
            }
            else if (enc_sw_stable != sample_high)
            {
                enc_sw_stable = sample_high;
                send_encoder_button(!enc_sw_stable);
            }
        }

        // Matrix scan and per-key debounce.
        {
            for (uint8_t row = 0U; row < MATRIX_ROWS; row++)
            {
                uint8_t pressed_cols = matrix_scan_row_pressed_cols(row);

                for (uint8_t col = 0U; col < MATRIX_COLS; col++)
                {
                    bool sample_pressed = (pressed_cols & (uint8_t)(1U << col)) != 0U;
                    bool stable_pressed = key_state[row][col];

                    if (sample_pressed == stable_pressed)
                    {
                        key_debounce_ms[row][col] = 0U;
                        continue;
                    }

                    if (key_debounce_ms[row][col] < MATRIX_DEBOUNCE_MS)
                    {
                        key_debounce_ms[row][col]++;
                    }

                    if (key_debounce_ms[row][col] >= MATRIX_DEBOUNCE_MS)
                    {
                        key_debounce_ms[row][col] = 0U;
                        key_state[row][col] = sample_pressed;

                        send_key_state(row, col, sample_pressed);
                    }
                }
            }

            // Return all rows high between scans.
            MATRIX_ROW_PORT.OUTSET = MATRIX_ROWS_MASK;
        }

        if (nv_dirty)
        {
            if (nv_dirty_ms < EEPROM_SAVE_DELAY_MS)
            {
                nv_dirty_ms++;
            }
            else
            {
                settings_save(rx_enc_value);
                nv_dirty = false;
            }
        }

        _delay_ms(1);
    }
}
