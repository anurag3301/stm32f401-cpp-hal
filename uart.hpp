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
    ):
    _uart(uart),
    _inuseIdx(-1),
    _tx (tx.port,  tx.pin,  GPIO::Mode::Alternate, GPIO::OutputType::PushPull,
         GPIO::Speed::High, GPIO::Pull::PullUp,  tx.af),
    _rx (rx.port,  rx.pin,  GPIO::Mode::Alternate, GPIO::OutputType::PushPull,
         GPIO::Speed::High, GPIO::Pull::PullUp,  rx.af),
    _cts(cts.port, cts.pin, GPIO::Mode::Alternate, GPIO::OutputType::PushPull,
         GPIO::Speed::High, GPIO::Pull::PullUp,  cts.af),
    _rts(rts.port, rts.pin, GPIO::Mode::Alternate, GPIO::OutputType::PushPull,
         GPIO::Speed::High, GPIO::Pull::PullUp,  rts.af)
    {
        if(!enableClock(_uart, _inuseIdx)) return;

        _uart->CR1 &= ~USART_CR1_UE; // Disable before config

        // word length
        _uart->CR1 &= ~USART_CR1_M;
        _uart->CR1 |= (static_cast<uint32_t>(wl) << USART_CR1_M_Pos);

        // parity
        _uart->CR1 &= ~(USART_CR1_PCE | USART_CR1_PS);
        switch(pt){
            case Parity::Disabled:break;
            case Parity::Even:_uart->CR1 |= USART_CR1_PCE; break;
            case Parity::Odd: _uart->CR1 |= USART_CR1_PCE | USART_CR1_PS; break;
        }

        // oversampling
        _uart->CR1 &= ~USART_CR1_OVER8;
        _uart->CR1 |= (static_cast<uint32_t>(os) << USART_CR1_OVER8_Pos);

        // rx/tx enable
        _uart->CR1 &= ~(USART_CR1_TE | USART_CR1_RE);
        switch (mode) {
            case Mode::TX_RX: _uart->CR1 |= USART_CR1_TE | USART_CR1_RE;  break;
            case Mode::TX:    _uart->CR1 |= USART_CR1_TE;                 break;
            case Mode::RX:    _uart->CR1 |= USART_CR1_RE;                 break;
        }

        // stopbit
        _uart->CR2 &= ~USART_CR2_STOP;
        _uart->CR2 |= (static_cast<uint32_t>(sb) << USART_CR2_STOP_Pos);
        _uart->CR3 &= ~(USART_CR3_CTSE | USART_CR3_RTSE);

        // hwflowctl
        switch (hfc) {
            case HwFlowCtl::RTS:     _uart->CR3 |= USART_CR3_RTSE; break;
            case HwFlowCtl::CTS:     _uart->CR3 |= USART_CR3_CTSE; break;
            case HwFlowCtl::RTS_CTS: _uart->CR3 |= USART_CR3_RTSE | USART_CR3_CTSE; break;
            default: break;
        }

        setBaud(baud);
        _uart->CR1 |= USART_CR1_UE; // reenable uart
    }

    UART(const UART&)            = delete;
    UART& operator=(const UART&) = delete;
    UART(UART&& other) noexcept
        : _uart(other._uart),
          _inuseIdx(other._inuseIdx),
          _tx(move(other._tx)),
          _rx(move(other._rx)),
          _cts(move(other._cts)),
          _rts(move(other._rts))
    {
        other._inuseIdx = -1;
        other._uart     = nullptr;
    }

    UART& operator=(UART&& other) noexcept {
        if (this != &other) {
            this->~UART();
            _uart          = other._uart;
            _inuseIdx      = other._inuseIdx;
            _tx            = move(other._tx);
            _rx            = move(other._rx);
            _cts           = move(other._cts);
            _rts           = move(other._rts);
            other._inuseIdx = -1;
            other._uart     = nullptr;
        }
        return *this;
    }

    ~UART() {
        if (_inuseIdx == -1) return;
        _uart->CR1 &= ~USART_CR1_UE;
        inuse[_inuseIdx] = false;
        _inuseIdx = -1;
    }

    void setBaud(uint32_t baud){
        uint32_t  pclk, hclk = SystemCoreClock; // CMSIS provides this

        if (_uart == USART1 || _uart == USART6) {
            // APB2
            uint32_t pre = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
            pclk = (pre & 0x4) ? (hclk >> ((pre & 0x3) + 1)) : hclk;
        } else {
            // APB1
            uint32_t pre = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
            pclk = (pre & 0x4) ? (hclk >> ((pre & 0x3) + 1)) : hclk;
        }

        if((_uart->CR1 & USART_CR1_OVER8) == 0){
            uint32_t div = (pclk + baud / 2) / baud;
            _uart->BRR = div;
        }
        else{
            uint32_t div  = (2 * pclk + baud / 2) / baud;
            uint32_t mant = div >> 4;
            uint32_t frac = div & 0x7;   // only 3 bits
            _uart->BRR = (mant << 4) | frac;
        }
    }

    static int8_t uartToIdx(USART_TypeDef *uart) {
        if      (uart == USART1) return 0;
        else if (uart == USART2) return 1;
        else if (uart == USART6) return 2;
        else                     return -1;
    }

    bool send(uint8_t *data, uint16_t len, uint32_t timeout_ms) {
        if (_inuseIdx == -1) return false;

        for (uint16_t i = 0; i < len; i++) {
            uint32_t t = timeout_ms * (SystemCoreClock / 1000);
            while (!(_uart->SR & USART_SR_TXE)) {
                if (--t == 0) return false;
            }
            _uart->DR = data[i];
        }

        // wait for last byte to fully shift out
        uint32_t t = timeout_ms * (SystemCoreClock / 1000);
        while (!(_uart->SR & USART_SR_TC)) {
            if (--t == 0) return false;
        }
        return true;
    }

    void setStdout(){
        if(stdout_uart == nullptr){
            stdout_uart = this;
        }
    }

    static void write_syscall(int fd, char *ptr, int len){
        if(stdout_uart == nullptr)return;
        (void)fd;
        stdout_uart->send((uint8_t*)ptr, len, 100);
    }

private:
    USART_TypeDef *_uart;
    int8_t _inuseIdx;
    GPIO _tx;
    GPIO _rx;
    GPIO _cts;
    GPIO _rts;
    static bool    inuse[6];
    static UART    *stdout_uart;

    static bool enableClock(USART_TypeDef *uart, int8_t &idx) {
        idx = uartToIdx(uart);
        if (idx == -1)        return false;
        if (inuse[idx])       return false; // already instantiated

        if      (uart == USART1) RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
        else if (uart == USART2) RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
        else if (uart == USART6) RCC->APB2ENR |= RCC_APB2ENR_USART6EN;

        inuse[idx] = true;
        return true;
    }

};


#endif //__UART_HAL__
