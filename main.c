#ifndef F_CPU
#define F_CPU 3333333UL
#endif

#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdint.h>
#include <limits.h>

#define BAUD_RATE 9600UL

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
#define ENC_CLK_PIN PIN1_bm
#define ENC_DT_PIN  PIN2_bm
#define ENC_SW_PIN  PIN3_bm

// OLED I2C pins (TWI0 default route): SCL=PA3, SDA=PA2
#define OLED_ADDR_7BIT 0x3CU
#define I2C_FREQ_HZ 100000UL
#define EEPROM_MAGIC 0xA5C35A7EUL
#define EEPROM_SAVE_DELAY_MS 750U

static uint32_t EEMEM ee_magic;
static uint32_t EEMEM ee_rx_enc_u32;
static uint8_t EEMEM ee_sw_u8;

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

    // Rotary encoder inputs with internal pull-ups (CLK=PE1, DT=PE2, SW=PE3).
    ENC_PORT.DIRCLR = ENC_CLK_PIN | ENC_DT_PIN | ENC_SW_PIN;
    ENC_PORT.PIN1CTRL = PORT_PULLUPEN_bm;
    ENC_PORT.PIN2CTRL = PORT_PULLUPEN_bm;
    ENC_PORT.PIN3CTRL = PORT_PULLUPEN_bm;
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

static void send_sw_state(bool sw_state)
{
    uart_write_str("SW=");
    uart_write_byte(sw_state ? '1' : '0');
    uart_write_str("\r\n");
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

static void process_rx_line(const char *line, bool *sw_state, int32_t *rx_enc_value, bool *oled_dirty)
{
    if ((line[0] == 'S') && (line[1] == 'W') && (line[2] == '=') &&
        ((line[3] == '0') || (line[3] == '1')) && (line[4] == '\0'))
    {
        *sw_state = (line[3] == '1');
        *oled_dirty = true;
        return;
    }

    if ((line[0] == 'E') && (line[1] == 'N') && (line[2] == 'C') && (line[3] == '='))
    {
        const char *v = &line[4];

        if ((v[0] == '+') && (v[1] == '1') && (v[2] == '\0'))
        {
            (*rx_enc_value)++;
            *oled_dirty = true;
            return;
        }

        if ((v[0] == '-') && (v[1] == '1') && (v[2] == '\0'))
        {
            (*rx_enc_value)--;
            *oled_dirty = true;
            return;
        }

        {
            int32_t parsed;
            if (parse_i32(v, &parsed))
            {
                *rx_enc_value = parsed;
                *oled_dirty = true;
            }
        }
    }
}

static void uart_process_rx_commands(bool *sw_state, int32_t *rx_enc_value, bool *oled_dirty)
{
    static char line[24];
    static uint8_t line_len = 0;
    uint8_t b;

    while (uart_read_byte(&b))
    {
        if ((b == '\n') || (b == '\r'))
        {
            if (line_len > 0U)
            {
                line[line_len] = '\0';
                process_rx_line(line, sw_state, rx_enc_value, oled_dirty);
                line_len = 0U;
            }
        }
        else
        {
            if (line_len < (uint8_t)(sizeof(line) - 1U))
            {
                line[line_len++] = (char)b;
            }
            else
            {
                line_len = 0U;
            }
        }
    }
}

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
    static const uint8_t colon[5] = {0x00,0x36,0x36,0x00,0x00};
    static const uint8_t minus[5] = {0x08,0x08,0x08,0x08,0x08};
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

    if ((c >= 'a') && (c <= 'z')) c = (char)(c - ('a' - 'A'));

    switch (c)
    {
        case ' ': return blank;
        case ':': return colon;
        case '-': return minus;
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
        default:  return blank;
    }
}

static bool ssd1306_write_text(uint8_t page, uint8_t col, const char *s)
{
    if (!ssd1306_set_cursor(page, col)) return false;
    if (!twi0_start_write(OLED_ADDR_7BIT)) return false;
    if (!twi0_write_byte(0x40U)) { twi0_stop(); return false; }

    while ((*s != '\0') && (col <= 122U))
    {
        const uint8_t *g = font5x7(*s++);
        for (uint8_t i = 0; i < 5U; i++)
        {
            if (!twi0_write_byte(g[i]))
            {
                twi0_stop();
                return false;
            }
        }
        if (!twi0_write_byte(0x00U))
        {
            twi0_stop();
            return false;
        }
        col = (uint8_t)(col + 6U);
    }

    twi0_stop();
    return true;
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

static void i32_to_dec(int32_t value, char *out, uint8_t out_len)
{
    char tmp[12];
    uint8_t n = 0;
    int64_t v = (int64_t)value;
    bool neg = false;

    if (out_len == 0U) return;

    if (v < 0)
    {
        neg = true;
        v = -v;
    }

    do
    {
        tmp[n++] = (char)('0' + (v % 10));
        v /= 10;
    } while ((v > 0) && (n < (uint8_t)sizeof(tmp)));

    {
        uint8_t idx = 0;
        if (neg && (idx < (uint8_t)(out_len - 1U)))
        {
            out[idx++] = '-';
        }

        while ((n > 0U) && (idx < (uint8_t)(out_len - 1U)))
        {
            out[idx++] = tmp[--n];
        }
        out[idx] = '\0';
    }
}

static void oled_render_status(int32_t rx_enc_value, bool sw_state)
{
    char enc_text[16];
    char line1[24] = "RX ENC: ";
    const char *line2 = sw_state ? "SW: GREEN" : "SW: RED";
    char *p = &line1[8];

    i32_to_dec(rx_enc_value, enc_text, (uint8_t)sizeof(enc_text));

    for (uint8_t i = 0; (enc_text[i] != '\0') && (p < &line1[sizeof(line1) - 1U]); i++)
    {
        *p++ = enc_text[i];
    }
    *p = '\0';

    ssd1306_clear();
    (void)ssd1306_write_text(0U, 0U, line1);
    (void)ssd1306_write_text(2U, 0U, line2);
}

static void settings_load(int32_t *rx_enc_value, bool *sw_state)
{
    if (eeprom_read_dword(&ee_magic) == EEPROM_MAGIC)
    {
        *rx_enc_value = (int32_t)eeprom_read_dword(&ee_rx_enc_u32);
        *sw_state = (eeprom_read_byte(&ee_sw_u8) != 0U);
        return;
    }

    // First run / invalid EEPROM contents.
    *rx_enc_value = 0;
    *sw_state = false;
    eeprom_update_dword(&ee_rx_enc_u32, (uint32_t)*rx_enc_value);
    eeprom_update_byte(&ee_sw_u8, 0U);
    eeprom_update_dword(&ee_magic, EEPROM_MAGIC);
}

static void settings_save(int32_t rx_enc_value, bool sw_state)
{
    eeprom_update_dword(&ee_rx_enc_u32, (uint32_t)rx_enc_value);
    eeprom_update_byte(&ee_sw_u8, sw_state ? 1U : 0U);
    eeprom_update_dword(&ee_magic, EEPROM_MAGIC);
}

int main(void)
{
    uart_init();
    io_init();
    twi0_init();
    ssd1306_init();

    uart_write_str("\r\nATmega4809 ready. MATRIX R=PA7/PA6/PA5 C=PC0..PC3, ENC=PE1/PE2, OLED=PA3/PA2\r\n");

    bool sw_state;
    int32_t rx_enc_value;
    settings_load(&rx_enc_value, &sw_state);
    bool oled_dirty = true;
    bool nv_dirty = false;
    uint16_t nv_dirty_ms = 0;

    // Matrix key debounce state.
    bool key_state[MATRIX_ROWS][MATRIX_COLS] = {{false}};
    uint8_t key_debounce_ms[MATRIX_ROWS][MATRIX_COLS] = {{0U}};

    // Encoder quadrature decode state.
    uint8_t enc_prev = encoder_state_2bit();
    int8_t enc_accum = 0;

    while (1)
    {
        bool sw_before = sw_state;
        int32_t enc_before = rx_enc_value;
        uart_process_rx_commands(&sw_state, &rx_enc_value, &oled_dirty);
        if ((sw_state != sw_before) || (rx_enc_value != enc_before))
        {
            nv_dirty = true;
            nv_dirty_ms = 0;
        }

        // Rotary encoder decode and TX message to PC.
        {
            uint8_t enc_curr = encoder_state_2bit();
            int8_t enc_delta = encoder_transition_delta(enc_prev, enc_curr);
            enc_prev = enc_curr;

            if (enc_delta != 0)
            {
                enc_accum = (int8_t)(enc_accum + enc_delta);
                if (enc_accum >= 4)
                {
                    rx_enc_value++;
                    send_encoder_step(+1);
                    enc_accum = 0;
                    oled_dirty = true;
                    nv_dirty = true;
                    nv_dirty_ms = 0;
                }
                else if (enc_accum <= -4)
                {
                    rx_enc_value--;
                    send_encoder_step(-1);
                    enc_accum = 0;
                    oled_dirty = true;
                    nv_dirty = true;
                    nv_dirty_ms = 0;
                }
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

                        // Key (0,0) acts as the software toggle switch.
                        if (sample_pressed && (row == 0U) && (col == 0U))
                        {
                            sw_state = !sw_state;
                            send_sw_state(sw_state);
                            oled_dirty = true;
                            nv_dirty = true;
                            nv_dirty_ms = 0;
                        }

                        send_key_state(row, col, sample_pressed);
                    }
                }
            }

            // Return all rows high between scans.
            MATRIX_ROW_PORT.OUTSET = MATRIX_ROWS_MASK;
        }

        if (oled_dirty)
        {
            oled_render_status(rx_enc_value, sw_state);
            oled_dirty = false;
        }

        if (nv_dirty)
        {
            if (nv_dirty_ms < EEPROM_SAVE_DELAY_MS)
            {
                nv_dirty_ms++;
            }
            else
            {
                settings_save(rx_enc_value, sw_state);
                nv_dirty = false;
            }
        }

        _delay_ms(1);
    }
}
