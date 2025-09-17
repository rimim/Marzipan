// Copyright 2025 Mimir Reynisson
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the “Software”),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include "pd/Actuator.h"

namespace pd::motor::cubemars {

class AK60 : public pd::Actuator {
public:
    // ===== Limits match AK packet ranges =====
    static constexpr float P_MIN = -12.5f;
    static constexpr float P_MAX =  12.5f;
    static constexpr float V_MIN = -45.0f;   // AK allows ±45 in your original
    static constexpr float V_MAX =  45.0f;
    static constexpr float T_MIN = -18.0f;
    static constexpr float T_MAX =  18.0f;
    static constexpr float KP_MIN =   0.0f;
    static constexpr float KP_MAX = 500.0f;
    static constexpr float KD_MIN =   0.0f;
    static constexpr float KD_MAX =   5.0f;

    static constexpr float GEAR_RATIO = 6.33f; // keep if you need deg output

    enum RunMode {
        MODE_UNKNOWN  = -1,
        MODE_MOTION   = 0,
        MODE_POSITION = 1,
        MODE_SPEED    = 2,
        MODE_CURRENT  = 3
    };

    struct MotorStatus {
        uint8_t  motor_id = 0;
        float    position = 0;     // [-4π, 4π]
        float    velocity = 0;     // [-45, 45] rad/s
        float    torque   = 0;     // [-18, 18] N·m
        float    temperature = 0;  // not provided by AK reply; keep 0

        // raw (for sync/debug)
        uint16_t raw_position = 0;
        uint16_t raw_velocity = 0;
        uint16_t raw_torque   = 0;
        uint16_t raw_temperature = 0;

        // AK simple reply has no flags/mode bits; default them
        bool hasCalibrationError = false;
        bool hasHallEncoderError = false;
        bool hasMagneticEncodingError = false;
        bool hasOverTemperature = false;
        bool hasOverCurrent = false;
        bool hasUnderVoltage = false;
        RunMode mode = MODE_MOTION;
    };

    // ===== Constructors (match target API) =====
    AK60() {
    }

    AK60(uint8_t id, pd::String name) :
        pd::Actuator(id, pd::AK60, name.c_str())
    {
    }

    AK60(uint8_t id, const char* name = nullptr) :
        pd::Actuator(id, pd::AK60, name)
    {
    }

    virtual bool update() override {
        if (shouldIgnore()) {
            return true;
        }
        if (fBus == nullptr) {
            PDLOG_ERROR("UNRESOLVED ACTUATOR BUS\n");
            return false;
        }
        switch (fRunMode) {
            case MODE_UNKNOWN:
                if (initialize() && requestStatus()) {
                    return process();
                }
                return false;

            case MODE_MOTION:
                if (fActive) {
                    if (pd::Log::isVerboseMove()) {
                        PDLOG_INFO("[%s] %f\n", getName().c_str(), fPosNow);
                    }
                    // position in radians expected by control()
                    if (!control(degreesToRadians(fPosNow), 0.0f, fKP, fKD, fTau))
                        return false;
                } else if (!control(0, 0, 0, 0, 0)) {
                    return false;
                }
                return process();

            case MODE_POSITION:
                PDLOG_ERROR("NYI MODE_POSITION\n");
                return false;
            case MODE_SPEED:
                PDLOG_ERROR("NYI MODE_SPEED\n");
                return false;
            case MODE_CURRENT:
                PDLOG_ERROR("NYI MODE_CURRENT\n");
                return false;
        }
        return false;
    }

    int getBusID() const {
        return (fBus != nullptr) ? fBus->ID() : 0;
    }

    bool initialize() {
        fRunMode = MODE_MOTION;
        return enableMotor();
    }

    bool initMotor(RunMode mode) {
        if (resetMotor()) {
            return setMode(mode);
        }
        return false;
    }

    bool enableMotor() {
        // AK ENABLE = FF FF FF FF FF FF FF FC
        uint8_t data[8] = { 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC };
        return sendRaw(fMotorID, data, sizeof(data));
    }

    bool resetMotor() {
        // AK DISABLE maps closer to "reset/disable"
        uint8_t data[8] = { 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD };
        return sendRaw(fMotorID, data, sizeof(data));
    }

    bool setMode(RunMode mode) {
        // AK has no modes; accept and store for state machine
        fRunMode = mode;
        return true;
    }

    RunMode getMode() const { return fRunMode; }

    bool requestStatus() {
        // AK has no explicit status request; just rely on async reply
        // You can trigger a zero-motion command to provoke a reply.
        return control(0, 0, 0, 0, 0);
    }

    bool setZeroPosition() {
        // AK ZERO = FF FF FF FF FF FF FF FE
        uint8_t data[8] = { 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE };
        return sendRaw(fMotorID, data, sizeof(data));
    }

    bool control(float pos, float speed, float kp, float kd, float tau) {
        // clamp to AK ranges
        const float p_des = clamp(pos,   P_MIN, P_MAX);
        const float v_des = clamp(speed, V_MIN, V_MAX);
        const float k_p   = clamp(kp,    KP_MIN, KP_MAX);
        const float k_d   = clamp(kd,    KD_MIN, KD_MAX);
        const float t_ff  = clamp(tau,   T_MIN, T_MAX);

        // quantize (AK: p 16b, v/kp/kd/t 12b)
        const uint16_t p_int  = float_to_uint(p_des, P_MIN, P_MAX, 16);
        const uint16_t v_int  = float_to_uint(v_des, V_MIN, V_MAX, 12);
        const uint16_t kp_int = float_to_uint(k_p,   KP_MIN, KP_MAX, 12);
        const uint16_t kd_int = float_to_uint(k_d,   KD_MIN, KD_MAX, 12);
        const uint16_t t_int  = float_to_uint(t_ff,  T_MIN,  T_MAX,  12);

        uint8_t pkt[8] = {
            uint8_t(p_int >> 8),
            uint8_t(p_int & 0xFF),
            uint8_t(v_int >> 4),
            uint8_t(((v_int & 0xF) << 4) | (kp_int >> 8)),
            uint8_t(kp_int & 0xFF),
            uint8_t(kd_int >> 4),
            uint8_t(((kd_int & 0xF) << 4) | (t_int >> 8)),
            uint8_t(t_int & 0xFF)
        };
        return sendRaw(fMotorID, pkt, sizeof(pkt));
    }

    bool process() {
        bool any = false;
        while (true) {
            uint32_t id = 0;
            uint8_t data[8] = {0};
            if (fBus->read(data, sizeof(data), &id) != sizeof(data)) {
                break; // no more frames
            }
            if (parseAKReply(id, data, fStatus) == 1) {
                any = true;
                fResponded = true;
                // Feedback: convert to your app’s expectations
                setFeedback(
                    radiansToDegrees(fStatus.position / GEAR_RATIO),
                    fStatus.velocity,
                    fStatus.torque,
                    fStatus.temperature
                );
                clearError(); // AK reply has no error flags; clear for now
            }
        }
        return any;
    }

    MotorStatus getStatus() const { return fStatus; }
    bool responded() const { return fResponded; }

    static unsigned process(AK60* motors[], unsigned count) {
        if (count == 0) return 0;
        auto* bus = motors[0]->getBus();
        for (unsigned i = 0; i < count; ++i) motors[i]->fResponded = false;

        unsigned responses = 0;
        while (true) {
            uint32_t id = 0;
            uint8_t data[8] = {0};
            if (bus->read(data, sizeof(data), &id) != sizeof(data)) break;
            for (unsigned i = 0; i < count; ++i) {
                auto* m = motors[i];
                if (m->parseAKReply(id, data, m->fStatus) == 1) {
                    m->fResponded = true;
                    m->setFeedback(
                        radiansToDegrees(m->fStatus.position / GEAR_RATIO),
                        m->fStatus.velocity,
                        m->fStatus.torque,
                        m->fStatus.temperature
                    );
                    m->clearError();
                    ++responses;
                }
            }
        }
        return responses;
    }

private:
    RunMode     fRunMode    = MODE_UNKNOWN;
    MotorStatus fStatus     {};
    bool        fResponded  = false;

    static float clamp(float x, float lo, float hi) {
        return (x < lo) ? lo : (x > hi) ? hi : x;
    }

    static uint16_t float_to_uint(float x, float x_min, float x_max, int bits) {
        const float span = x_max - x_min;
        if (x < x_min) x = x_min;
        else if (x > x_max) x = x_max;
        const float scale = (float)((1u << bits) - (bits == 16 ? 1u : 0u)) / span;
        // For 16-bit AK position we want 0..65535, for 12-bit 0..4095
        const uint32_t maxv = (bits == 16) ? 0xFFFFu : ((1u << bits) - 1u);
        uint32_t ui = (uint32_t)((x - x_min) * scale + 0.5f);
        if (ui > maxv) ui = maxv;
        return (uint16_t)ui;
    }

    static float uint_to_float(uint16_t x, float x_min, float x_max, int bits) {
        const float span = x_max - x_min;
        const float denom = (float)((1u << bits) - 1u);
        return ((float)x) * span / denom + x_min;
    }

    bool sendRaw(uint8_t can_id, const uint8_t* data, uint8_t len) {
        // Standard (11-bit) ID, no RTR; base Bus uses (data,len,id,extended)
        return fBus->write(const_cast<uint8_t*>(data), len, can_id, false) == len;
    }

    // Parse AK status reply:
    //   ID must be 0, 8-byte frame:
    //   [0]=motor_id, [1..2]=pos16, [3..4]=vel12, [5..6]=torque12, [7]=unused (per your AK code)
    int parseAKReply(uint32_t id, const uint8_t* buf, MotorStatus& mot) {
        if (id != 0) return -1;
        const uint8_t mid = buf[0];
        if (mid != fMotorID) return -1;

        const uint16_t p_int = (uint16_t(buf[1]) << 8) | buf[2];
        const uint16_t v_int = (uint16_t(buf[3]) << 4) | (buf[4] >> 4);
        const uint16_t i_int = (uint16_t(buf[4] & 0x0F) << 8) | buf[5];

        mot.motor_id     = mid;
        mot.raw_position = p_int;
        mot.raw_velocity = v_int;
        mot.raw_torque   = i_int;

        mot.position = uint_to_float(p_int, P_MIN, P_MAX, 16);
        mot.velocity = uint_to_float(v_int, V_MIN, V_MAX, 12);
        mot.torque   = uint_to_float(i_int, T_MIN, T_MAX, 12);
        mot.temperature = 0.0f; // not supplied

        // No fault bits in AK simple reply
        mot.hasUnderVoltage = false;
        mot.hasOverCurrent = false;
        mot.hasOverTemperature = false;
        mot.hasMagneticEncodingError = false;
        mot.hasHallEncoderError = false;
        mot.hasCalibrationError = false;
        mot.mode = MODE_MOTION;

        return 1;
    }
};

}
