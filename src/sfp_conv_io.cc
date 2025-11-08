#include "sfp_conv_io.hpp"

SFPConvIOExp::SFPConvIOExp(TCA9535 io)
    : io(std::move(io)) {}

std::expected<void, HAL_StatusTypeDef>
SFPConvIOExp::start_transmitter(bool enable, bool power_down, bool disable_tx_pin) {
    return io.transact(TCA9535::REG_OUTPUT_0, [&](uint8_t regval) -> uint8_t {
        regval = set_bit(regval, TRANSMITTER_ENABLE, enable);
        regval = set_bit(regval, TRANSMITTER_PWDN, !power_down); // Active low
        regval = set_bit(regval, TX_DISABLE, disable_tx_pin);
        return regval;
    });
}

std::expected<void, HAL_StatusTypeDef>
SFPConvIOExp::start_receiver(bool enable, bool power_down) {
    return io.transact(TCA9535::REG_OUTPUT_0, [&](uint8_t regval) -> uint8_t {
        regval = set_bit(regval, RECEIVER_ENABLE, enable);
        regval = set_bit(regval, RECEIVER_PWDN, !power_down); // Active low
        return regval;
    });
}

std::expected<void, HAL_StatusTypeDef>
SFPConvIOExp::set_loopback_mode(LoopbackMode mode) {
    return io.transact(TCA9535::REG_OUTPUT_0, [&](uint8_t regval) -> uint8_t {
        regval = clear_pin(regval, LOCAL_LE);
        regval = clear_pin(regval, LINE_LE);
        switch (mode) {
            case LoopbackMode::LOCAL: return set_bit(regval, LOCAL_LE, true);
            case LoopbackMode::LINE:  return set_bit(regval, LINE_LE, true);
            default: return regval;
        }
    });
}

std::expected<void, HAL_StatusTypeDef>
SFPConvIOExp::configure_all(const Config& cfg) {
    // Configure port 0 atomically (RX/TX/Loopback)
    auto r0 = io.transact(TCA9535::REG_OUTPUT_0, [&](uint8_t regval) -> uint8_t {
        // Transmitter
        regval = set_bit(regval, TRANSMITTER_ENABLE, cfg.transmitter_enable);
        regval = set_bit(regval, TRANSMITTER_PWDN, !cfg.transmitter_power_down);
        regval = set_bit(regval, TX_DISABLE, cfg.tx_disable);

        // Receiver
        regval = set_bit(regval, RECEIVER_ENABLE, cfg.receiver_enable);
        regval = set_bit(regval, RECEIVER_PWDN, !cfg.receiver_power_down);

        // Loopback
        regval = clear_pin(regval, LOCAL_LE);
        regval = clear_pin(regval, LINE_LE);
        switch (cfg.loopback) {
            case LoopbackMode::LOCAL: regval = set_bit(regval, LOCAL_LE, true); break;
            case LoopbackMode::LINE:  regval = set_bit(regval, LINE_LE, true);  break;
            default: break;
        }

        return regval;
    });

    if (!r0)
        return r0;

    // Configure port 1 atomically (Rate select + PLL)
    auto r1 = io.transact(TCA9535::REG_OUTPUT_1, [&](uint8_t regval) -> uint8_t {
        regval = set_bit(regval, RATE_SELECT, cfg.high_speed);
        regval = set_bit(regval, PLL_OEB, !cfg.pll_output_enable); // active low
        return regval;
    });

    if (!r1)
        return r1;

    return {};
}

std::expected<SFPConvIOExp::Status, HAL_StatusTypeDef>
SFPConvIOExp::get_status() {
    Status s{};

    auto port0 = io.read_port(TCA9535::Port::PORT_0);
    if (!port0)
        return std::unexpected(port0.error());

    auto port1 = io.read_port(TCA9535::Port::PORT_1);
    if (!port1)
        return std::unexpected(port1.error());

    uint8_t p0 = *port0;
    uint8_t p1 = *port1;

    s.tx_fault             = get_bit(p0, TX_FAULT);
    s.receiver_enabled     = get_bit(p0, RECEIVER_ENABLE);
    s.transmitter_enabled  = get_bit(p0, TRANSMITTER_ENABLE);
    s.receiver_powered_down = !get_bit(p0, RECEIVER_PWDN);
    s.transmitter_powered_down = !get_bit(p0, TRANSMITTER_PWDN);
    s.loopback_mode = get_bit(p0, LOCAL_LE) ? LoopbackMode::LOCAL :
                      get_bit(p0, LINE_LE)  ? LoopbackMode::LINE :
                                              LoopbackMode::DISABLED;

    s.sfp_present        = !get_bit(p1, SFP_PRESENT); // active low
    s.rx_los             = get_bit(p1, RX_LOS);
    s.pll_interrupt      = get_bit(p1, PLL_INTR);
    s.pll_output_enabled = !get_bit(p1, PLL_OEB); // active low

    return s;
}

