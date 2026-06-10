#ifndef __UART_HAL__
#define __UART_HAL__
#include <stm32f4xx.h>
#include "gpio.hpp"
#include "pinConfig.hpp"
#include "common.hpp"


class UART {
public:
    enum class WordLength: uint8_t{
        B8    = 0,
        B9    = 1,
    };

    enum class StopBit: uint8_t{
        B1    = 0,
        B05   = 1,
        B2    = 2,
        B15   = 3,
    };

    enum class Parity: uint8_t{
        Disabled,
        Even,
        Odd,
    };

    enum class Mode: uint8_t{
        TX_RX,
        RX,
        TX,
    };

    enum class HwFlowCtl: uint8_t{
        None,
        RTS,
        CTS,
        RTS_CTS,
    };

    enum class OverSampling: uint8_t{
        O16,
        O8
    };


    UART(
        USART_TypeDef *uart, PinConfig tx = {nullptr, 0xFF}, 
        PinConfig rx = {nullptr, 0xFF}, uint32_t baud=115200,
        Mode mode = Mode::TX_RX, WordLength wl = WordLength::B8, 
        StopBit sb = StopBit::B1, Parity pt = Parity::Disabled,
        OverSampling os = OverSampling::O16, HwFlowCtl hfc = HwFlowCtl::None,
        PinConfig cts = {nullptr, 0xFF}, PinConfig rts = {nullptr, 0xFF}
    );
    UART(const UART&)            = delete;
    UART& operator=(const UART&) = delete;
    UART(UART&& other) noexcept;
    UART& operator=(UART&& other) noexcept;
    ~UART();
    void setBaud(uint32_t baud);
    static int8_t uartToIdx(USART_TypeDef *uart);
    bool send(uint8_t *data, uint16_t len, uint32_t timeout_ms);
    void setStdout();
    static void write_syscall(int fd, char *ptr, int len);

private:
    USART_TypeDef *_uart;
    int8_t _inuseIdx;
    GPIO _tx;
    GPIO _rx;
    GPIO _cts;
    GPIO _rts;
    static bool    inuse[6];
    static UART    *stdout_uart;

    static bool enableClock(USART_TypeDef *uart, int8_t &idx);
};


#endif //__UART_HAL__
