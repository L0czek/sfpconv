#include "sfpconv.hpp"
#include "TCA9535.hpp"
#include "sfp_conv_io.hpp"
#include "si5351.h"
#include "stm32g4xx_hal_def.h"
#include "stm32g4xx_hal_gpio.h"
#include <expected>
#include <cstring>



SfpConv::SfpConv(Config config, TCA9535 ioexp) :config(config), io(ioexp) {
    memset(&si5351, 0, sizeof(si5351));

}

std::expected<void, HAL_StatusTypeDef> SfpConv::config_clock() noexcept {
    Si5351_PLLClockSourceTypeDef pll_src;

    switch (config.ClkSrc) {
        case ClockSource::Clkin:
            si5351.f_CLKIN = config.ClkSrc_Hz;
            si5351.Fanout_CLKIN_EN = EnableState::ON;
            pll_src = PLL_Clock_Source_CLKIN;
            break;
        case ClockSource::Xtal:
            si5351.f_XTAL  = config.ClkSrc_Hz;
            si5351.Fanout_XO_EN = EnableState::ON;
            pll_src = PLL_Clock_Source_XTAL;
            break;
    }

    si5351.I2Cx = config.i2c;
    si5351.HW_I2C_Address = config.i2c_si5351_addr;

    si5351.OSC = Si5351_OSCConfigTypeDef {
        .OSC_XTAL_Load = config.XtalLoad,
        .CLKIN_Div = Si5351_CLKINDivTypeDef::CLKINDiv_Div1,
        .VCXO_Pull_Range_ppm = 0,
    };

    si5351.PLL[0] = Si5351_PLLConfigTypeDef {
        .PLL_Multiplier_Integer = (config.Pll_Hz / config.ClkSrc_Hz),
        .PLL_Multiplier_Numerator = 0,
        .PLL_Multiplier_Denominator = 1,
        .PLL_Clock_Source = pll_src,
        .PLL_Capacitive_Load = PLL_Capacitive_Load_0,
    };

    // RefCLK
    si5351.MS[0] = Si5351_MSConfigTypeDef {
        .MS_Clock_Source = Si5351_MSClockSourceTypeDef::MS_Clock_Source_PLLA,
        .MS_Divider_Integer = (config.RefClk_Hz / config.Pll_Hz),
        .MS_Divider_Numerator = 0,
        .MS_Divider_Denominator = 1
    };

    // TCLK
    si5351.MS[1] = Si5351_MSConfigTypeDef {
        .MS_Clock_Source = Si5351_MSClockSourceTypeDef::MS_Clock_Source_PLLA,
        .MS_Divider_Integer = (config.Tclk_Hz / config.Pll_Hz),
        .MS_Divider_Numerator = 0,
        .MS_Divider_Denominator = 1
    };

    si5351.CLK[0] = Si5351_CLKConfigTypeDef {
        .CLK_Clock_Source = Si5351_CLKClockSourceTypeDef::CLK_Clock_Source_MS_Own,
        .CLK_QuarterPeriod_Offset = 0,
        .CLK_R_Div = Si5351_CLKRDivTypeDef::CLK_R_Div1,
        .CLK_Invert = EnableState::OFF,
        .CLK_Enable = EnableState::ON,
        .CLK_PowerDown = EnableState::OFF,
        .CLK_Disable_State = Si5351_CLKDisableStateTypeDef::CLK_Disable_State_LOW,
        .CLK_I_Drv = Si5351_CLKIDrvTypeDef::CLK_I_Drv_2mA,
        .CLK_Use_OEB_Pin = EnableState::ON
    };

    si5351.CLK[1] = Si5351_CLKConfigTypeDef {
        .CLK_Clock_Source = Si5351_CLKClockSourceTypeDef::CLK_Clock_Source_MS_Own,
        .CLK_QuarterPeriod_Offset = 0,
        .CLK_R_Div = Si5351_CLKRDivTypeDef::CLK_R_Div1,
        .CLK_Invert = EnableState::OFF,
        .CLK_Enable = EnableState::ON,
        .CLK_PowerDown = EnableState::OFF,
        .CLK_Disable_State = Si5351_CLKDisableStateTypeDef::CLK_Disable_State_LOW,
        .CLK_I_Drv = Si5351_CLKIDrvTypeDef::CLK_I_Drv_2mA,
        .CLK_Use_OEB_Pin = EnableState::ON
    };

    return {};
}

std::expected<SfpConv, HAL_StatusTypeDef> SfpConv::configure(Config config) {
    auto ioexp = TCA9535::configure(config.i2c, config.i2c_io_exp_addr);
    if (!ioexp)
        return std::unexpected(ioexp.error());

    auto self = SfpConv(std::move(config), std::move(*ioexp));

    auto result = self.config_clock();
    if (!result)
        return std::unexpected(result.error());

    return self;
}

std::expected<void, HAL_StatusTypeDef> SfpConv::switch_mode(Mode new_mode, SFPConvIOExp::LoopbackMode lo) noexcept {
    SFPConvIOExp::Mode io_exp_mode;

    switch (new_mode) {
        case Mode::Off:        io_exp_mode = SFPConvIOExp::Mode::OFF;            break;
        case Mode::Rx:         io_exp_mode = SFPConvIOExp::Mode::RX;             break;
        case Mode::Tx:         io_exp_mode = SFPConvIOExp::Mode::TX;             break;
        case Mode::Sync:
        case Mode::FullDuplex: io_exp_mode = SFPConvIOExp::Mode::OFF;            break;
    }

    auto cfg = SFPConvIOExp::Config::from_mode(io_exp_mode);
    auto result = io.configure_all(cfg);

    if (!result)
        return std::unexpected(result.error());

    result = io.set_loopback_mode(lo);

    if (!result)
        return std::unexpected(result.error());

    if (new_mode == Mode::Sync) {
        HAL_GPIO_WritePin(config.SyncPinGpio, config.SyncPin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(config.SyncPinGpio, config.SyncPin, GPIO_PIN_RESET);
    }

    return {};
}

SfpConv::Mode SfpConv::get_mode() const noexcept {
    return mode;
}

bool SfpConv::is_synced() const noexcept {
    return HAL_GPIO_ReadPin(config.LockPinGpio, config.LockPin) == GPIO_PIN_SET;
}

std::expected<SFPConvIOExp::Status, HAL_StatusTypeDef> SfpConv::get_board_status() noexcept {
    return io.get_status();
}
