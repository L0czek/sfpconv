#ifndef __SFP_CONV_HPP__
#define __SFP_CONV_HPP__

#include "TCA9535.hpp"
#include "sfp_conv_io.hpp"
#include "si5351.h"
#include "stm32g474xx.h"
#include "stm32g4xx_hal_def.h"
#include "stm32g4xx_hal_dma.h"
#include "stm32g4xx_hal_i2c.h"
#include <cstdint>
#include <expected>

class SfpConv {
public:
    enum class ClockSource {
        Xtal,
        Clkin
    };

    struct Config {
        GPIO_TypeDef *SyncPinGpio;
        uint32_t SyncPin;

        GPIO_TypeDef *LockPinGpio;
        uint32_t LockPin;

        ClockSource ClkSrc;
        uint32_t ClkSrc_Hz;
        Si5351_XTALLoadTypeDef XtalLoad;

        uint32_t Pll_Hz;
        uint32_t RefClk_Hz;
        uint32_t Tclk_Hz;

        I2C_HandleTypeDef *i2c;
        uint8_t i2c_io_exp_addr;
        uint8_t i2c_si5351_addr;
    };

    enum class Mode {
        Off,
        Tx,
        Rx,
        FullDuplex,
        Sync,
    };

    static std::expected<SfpConv, HAL_StatusTypeDef> configure(Config config);
    std::expected<void, HAL_StatusTypeDef> switch_mode(Mode mode, SFPConvIOExp::LoopbackMode lo) noexcept;
    Mode get_mode() const noexcept;
    bool is_synced() const noexcept;
    std::expected<SFPConvIOExp::Status, HAL_StatusTypeDef> get_board_status() noexcept;
private:
    SfpConv(Config config, TCA9535 ioexp);

    std::expected<void, HAL_StatusTypeDef> config_clock() noexcept;

    Config config;
    SFPConvIOExp io;
    Si5351_ConfigTypeDef si5351;

    Mode mode;
};

#endif
