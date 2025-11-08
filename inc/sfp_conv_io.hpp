#ifndef __SFP_CONV_IO_HPP__
#define __SFP_CONV_IO_HPP__

#include "TCA9535.hpp"
#include <expected>
#include <utility>
#include <cstdint>

class SFPConvIOExp {
public:
    enum class LoopbackMode : uint8_t {
        DISABLED,
        LOCAL,
        LINE
    };

    enum class Mode : uint8_t {
        OFF,
        TX,
        RX,
        FULL_DUPLEX
    };

    struct Status {
        bool sfp_present;
        bool tx_fault;
        bool rx_los;
        bool pll_interrupt;
        bool pll_output_enabled;
        bool receiver_enabled;
        bool transmitter_enabled;
        bool receiver_powered_down;
        bool transmitter_powered_down;
        LoopbackMode loopback_mode;
    };

    struct Config {
        bool transmitter_enable;
        bool transmitter_power_down;
        bool tx_disable;
        bool receiver_enable;
        bool receiver_power_down;
        bool pll_output_enable;
        bool high_speed;
        LoopbackMode loopback;

        /// Returns a default, powered-down configuration.
        static constexpr Config defaults() noexcept {
            return Config{
                .transmitter_enable = false,
                .transmitter_power_down = true,
                .tx_disable = true,
                .receiver_enable = false,
                .receiver_power_down = true,
                .pll_output_enable = true,
                .high_speed = false,
                .loopback = LoopbackMode::DISABLED
            };
        }

        static constexpr Config from_mode(Mode mode) noexcept {
            Config cfg = defaults();
            switch (mode) {
                case Mode::OFF:
                    break; // keep everything powered down
                case Mode::TX:
                    cfg.transmitter_enable = true;
                    cfg.transmitter_power_down = false;
                    cfg.tx_disable = false;
                    break;
                case Mode::RX:
                    cfg.receiver_enable = true;
                    cfg.receiver_power_down = false;
                    break;
                case Mode::FULL_DUPLEX:
                    cfg.transmitter_enable = true;
                    cfg.transmitter_power_down = false;
                    cfg.tx_disable = false;
                    cfg.receiver_enable = true;
                    cfg.receiver_power_down = false;
                    break;
            }
            return cfg;
        }
    };

    explicit SFPConvIOExp(TCA9535 io);

    // --- High-level configuration ---
    std::expected<void, HAL_StatusTypeDef> start_transmitter(bool enable, bool power_down, bool disable_tx_pin);
    std::expected<void, HAL_StatusTypeDef> start_receiver(bool enable, bool power_down);
    std::expected<void, HAL_StatusTypeDef> set_loopback_mode(LoopbackMode mode);
    std::expected<void, HAL_StatusTypeDef> configure_all(const Config& cfg);

    // --- Status retrieval ---
    std::expected<Status, HAL_StatusTypeDef> get_status();

private:
    TCA9535 io; // Owned I/O expander instance

    // --- Pin operations (inline, pure helpers) ---
    static constexpr uint8_t clear_pin(uint8_t val, TCA9535::Pin pin) {
        return val & ~static_cast<uint8_t>(pin);
    }

    static constexpr uint8_t set_bit(uint8_t val, TCA9535::Pin pin, bool set) {
        uint8_t mask = static_cast<uint8_t>(pin);
        return set ? (val | mask) : (val & ~mask);
    }

    static constexpr bool get_bit(uint8_t val, TCA9535::Pin pin) {
        return (val & static_cast<uint8_t>(pin)) != 0;
    }

    // --- Pin mapping ---
    static constexpr TCA9535::Pin RECEIVER_ENABLE    = TCA9535::Pin::PIN_0;  // REN
    static constexpr TCA9535::Pin TRANSMITTER_ENABLE = TCA9535::Pin::PIN_1;  // DEN
    static constexpr TCA9535::Pin RECEIVER_PWDN      = TCA9535::Pin::PIN_2;  // /RPWDN
    static constexpr TCA9535::Pin TRANSMITTER_PWDN   = TCA9535::Pin::PIN_3;  // /TPWDN
    static constexpr TCA9535::Pin LOCAL_LE           = TCA9535::Pin::PIN_4;
    static constexpr TCA9535::Pin LINE_LE            = TCA9535::Pin::PIN_5;
    static constexpr TCA9535::Pin TX_FAULT           = TCA9535::Pin::PIN_6;
    static constexpr TCA9535::Pin TX_DISABLE         = TCA9535::Pin::PIN_7;

    // Port 1
    static constexpr TCA9535::Pin SFP_PRESENT = TCA9535::Pin::PIN_0; // active low
    static constexpr TCA9535::Pin RATE_SELECT = TCA9535::Pin::PIN_1;
    static constexpr TCA9535::Pin RX_LOS      = TCA9535::Pin::PIN_2;
    static constexpr TCA9535::Pin PLL_INTR    = TCA9535::Pin::PIN_3;
    static constexpr TCA9535::Pin PLL_OEB     = TCA9535::Pin::PIN_4; // active low
};

#endif
