/******************************************************************************
 * LazyGun.cpp
 * 
 * Main program file (and support routines) for RP PICO Lazy Gun board. This
 * supports a momentary switch (debounced in SW) to switch between different
 * patterns on 4 driven LED groups and control a laser diode (pointer) via
 * a power switch peripheral board. Because Riff Raff needs some fucking
 * cool tools...
 * ****************************************************************************
 * Code Copyright(C) 2022 Don Goodeve. Open source - no restrictions
 * ***************************************************************************/

#include "pico/stdlib.h"
#include <stdio.h>
#include <list>
#include <math.h>
#include "pico/time.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"

// Defines
//-----------------------------------------------------------------------------
#define kPicoClockHz        (125.0e6f)
#define kClockDivider       (4.0f)
#define kPWMClockHz         (kPicoClockHz / kClockDivider)
#define kPWMClkSec          (1.0f / kPWMClock)
#define kPWMCountMax        (0x10000)
#define kPWMFullFreq        (kPWMClockHz / (float)kPWMCountMax)
#define kPWMWrapDuration    (1.0f / kPWMFullFreq)
#define kPWMWrapHz          (kPWMFullFreq)

#define kLEDs3  (16)
#define kLEDs2  (18)
#define kLaser  (19)
#define kLEDs1  (20)
#define kLEDs4  (27)
#define kAliveLED   (PICO_DEFAULT_LED_PIN)
#define kButton (28)
#define kDoublePressUsec   (300000)

class _PWMManager {
    public:
        virtual void ISRUpdate(void) = 0;
};
static _PWMManager *spPWMManager = nullptr;

static
void _stepPWM(void) {
    if (spPWMManager != nullptr) {
        spPWMManager->ISRUpdate();
    }
    /*
    static int fade = 0;
    static bool going_up = true;
    // Clear the interrupt flag that brought us here

    if (going_up) {
        ++fade;
        if (fade > 128) {
            fade = 128;
            going_up = false;
        }
    } else {
        --fade;
        if (fade < 0) {
            fade = 0;
            going_up = true;
        }
    }
    // Square the fade value to make the LED's brightness appear more linear
    // Note this range matches with the wrap value
    pwm_set_gpio_level(kLEDs1, fade * fade);
    pwm_set_gpio_level(kLEDs2, fade * fade);
    pwm_set_gpio_level(kLEDs3, fade * fade);
    pwm_set_gpio_level(kLEDs4, fade * fade);
    */
}

class PWMManager : public _PWMManager {
    public:
        class PWMStep {
            public:
                float mfDurationSec;
                float mfTargetLevel;
        };

    private:
        class PWM {
            private:
                uint muGPIO;
                std::list<PWMStep> mlNext;
                bool mbRepeat;
                float mfCurrentLevel;
                float mfTargetLevel;
                uint muStepsRemaining;

            public:
                PWM(uint uGPIO) :
                    muGPIO(uGPIO),
                    mfCurrentLevel(0.0f), mfTargetLevel(0.0f), muStepsRemaining(0.0f) {
                    gpio_set_function(muGPIO, GPIO_FUNC_PWM);
                    pwm_set_gpio_level(muGPIO, 0x0);    // Off
                    // Get some sensible defaults for the slice configuration. By default, the
                    // counter is allowed to wrap over its maximum range (0 to 2**16-1)
                    pwm_config cPWMConfig(pwm_get_default_config());
                    // Set divider, reduces counter clock to sysclock/this value
                    pwm_config_set_clkdiv(&cPWMConfig, (float)kClockDivider);
                    pwm_init(pwm_gpio_to_slice_num(muGPIO), &cPWMConfig, true);
                }

                ~PWM() {
                }

                uint getGPIO(void) const {
                    return muGPIO;
                }

                void setProgram(const PWMStep *pSteps, uint uSteps, uint uOffset, bool bRepeat) {
                    std::list<PWMStep> lProgram;
                    for(uint i=0; i<uSteps; i++) {
                        lProgram.push_back(pSteps[(i+uOffset)%uSteps]);
                    }
                    
                    uint32_t cStatus(save_and_disable_interrupts());
                    mlNext = lProgram;
                    mbRepeat = bRepeat;
                    muStepsRemaining = 0;
                    restore_interrupts(cStatus);
                }

                void update(void) {
                    if (0 == muStepsRemaining) {
                        if (false == mlNext.empty()) {
                            // Grab next step
                            PWMStep cStep(mlNext.front());
                            mlNext.pop_front();
                            if (true == mbRepeat) {
                                mlNext.push_back(cStep);
                            }

                            // Interpret it
                            muStepsRemaining = (uint)roundf(kPWMWrapHz * cStep.mfDurationSec);
                            mfTargetLevel = cStep.mfTargetLevel;
                            //printf("Target level %.2f in steps %d (kWrapsPerSec=%f)\r\n", mfTargetLevel, muStepsRemaining, kWrapsPerSec);
                        }
                    }
                    if (muStepsRemaining > 0) {
                        float fDiff(mfTargetLevel - mfCurrentLevel);
                        float fDelta(fDiff / (float)muStepsRemaining);
                        mfCurrentLevel += fDelta;
                        muStepsRemaining--;

                        uint uRangedLevel(mfCurrentLevel * 255.0f);
                        uRangedLevel = (uRangedLevel > 255)?255:uRangedLevel;
                        pwm_set_gpio_level(muGPIO, uRangedLevel*uRangedLevel);
                    }
                }
        };
        
        uint muPWMIRQSlice;
        std::list<PWM> mlPWMs;

    public:
        PWMManager(const uint *pGPIOSet, uint uGPIOs) {
            if (0 == uGPIOs) {
                return;
            }
            for(uint i=0; i<uGPIOs; i++) {
                mlPWMs.push_back(PWM(pGPIOSet[i]));

            }

            uint uBasePWM(pGPIOSet[0]);
            // Figure out which slice we just connected to the uBasePWM. We use this block
            // as the interrupt source (on wrap). Note there is just one PWM IRQ.
            muPWMIRQSlice = pwm_gpio_to_slice_num(uBasePWM);

            // Mask our slice's IRQ output into the PWM block's single interrupt line,
            // register the interrupt handler and enable it.
            pwm_clear_irq(muPWMIRQSlice);
            pwm_set_irq_enabled(muPWMIRQSlice, true);
            irq_set_exclusive_handler(PWM_IRQ_WRAP, _stepPWM);
            irq_set_enabled(PWM_IRQ_WRAP, true);
        }

        ~PWMManager() {
            pwm_set_irq_enabled(muPWMIRQSlice, false);
        }

        void setProgram(uint uGPIO, const PWMStep *pSteps, uint uSteps, bool bRepeat, uint uOffset=0) {
            for(std::list<PWM>::iterator cPWM = mlPWMs.begin(); cPWM != mlPWMs.end(); ++cPWM) {
                if ((*cPWM).getGPIO() == uGPIO) {
                    (*cPWM).setProgram(pSteps, uSteps, uOffset, bRepeat);
                    break;
                }
            }
        }

        void ISRUpdate(void) {
            pwm_clear_irq(muPWMIRQSlice);
            for(std::list<PWM>::iterator cPWM = mlPWMs.begin(); cPWM != mlPWMs.end(); ++cPWM) {
                (*cPWM).update();
            }
        }
};

#define _arraysize(a)  (sizeof(a)/sizeof(a[0]))

// LED output programs
static const PWMManager::PWMStep pOff[] = {{0.01f, 0.0f}, {1.0f, 0.00f}};
static const PWMManager::PWMStep pHeartbeat[] = {{0.1f, 0.8f}, {0.1f, 0.0f}, {0.2f, 0.0f}, {0.1f, 1.0f}, {0.1f, 0.2f}, {0.3f, 0.0f}};
static const PWMManager::PWMStep pMalevolent[] = {
    {0.6f, 0.1f}, {0.2f, 0.05f}, {0.2f, 0.05f}, {0.2f, 0.05f}, {0.2f, 0.05f}, {1.0f, 0.5f},
};
static const PWMManager::PWMStep pPointerWave[] = {{0.2f, 0.5f}, {0.2f, 0.5f}, {0.2f, 0.25f}, {0.2f, 0.1f}, {0.2f, 0.0f}};
static const PWMManager::PWMStep pLaserPulse[] = {{0.04f, 1.0f}, {0.4f, 0.8f}, {0.2f, 0.4f}};
static const PWMManager::PWMStep pLaserFiring[] = {{0.01f, 0.0f}, {0.5f, 1.0f}, {0.5f, 0.0f}};
static const PWMManager::PWMStep pFiringSequence[] = {
    {0.04f, 0.5f}, {0.04f, 0.5f}, {0.04f, 0.25f}, {0.04f, 0.1f}, {0.04f, 0.0f}, {0.04f, 0.0f},
    {0.02f, 0.5f}, {0.02f, 0.5f}, {0.02f, 0.25f}, {0.02f, 0.1f}, {0.02f, 0.0f}, {0.02f, 0.0f},
    {0.01f, 0.5f}, {0.01f, 0.5f}, {0.01f, 0.25f}, {0.01f, 0.1f}, {0.01f, 0.0f}, {0.01f, 0.0f},
    {0.005f, 0.5f}, {0.005f, 0.75f}, {0.005f, 0.5f}, {0.005f, 0.3}, {0.005f, 0.2f}, {0.005f, 0.2f},
    {0.005f, 0.6f}, {0.005f, 0.85f}, {0.005f, 0.6f}, {0.005f, 0.4f}, {0.005f, 0.3f}, {0.005f, 0.3f},
    {0.005f, 0.7f}, {0.005f, 0.95f}, {0.005f, 0.8f}, {0.005f, 0.5f}, {0.005f, 0.4f}, {0.005f, 0.4f},
    {0.005f, 0.9f}, {0.005f, 1.0f}, {0.005f, 0.9f}, {0.005f, 1.0f}, {0.005f, 0.9f}, {0.005f, 1.0f},
    {0.01f, 0.0f}, {1.0f, 0.0f}
};

static PWMManager *spFullPWMManager = nullptr;

static void _triggerDown(void) {
    if (nullptr == spFullPWMManager) {
        return;
    }
    spFullPWMManager->setProgram(kLaser, pLaserPulse, _arraysize(pLaserPulse), true, 0);
    spFullPWMManager->setProgram(kLEDs1, pPointerWave, _arraysize(pPointerWave), true, 0);
    spFullPWMManager->setProgram(kLEDs2, pPointerWave, _arraysize(pPointerWave), true, 1);
    spFullPWMManager->setProgram(kLEDs3, pPointerWave, _arraysize(pPointerWave), true, 2);
    spFullPWMManager->setProgram(kLEDs4, pPointerWave, _arraysize(pPointerWave), true, 3);
}

static void _triggerUp(void) {
    if (nullptr == spFullPWMManager) {
        return;
    }
    spFullPWMManager->setProgram(kLaser, pOff, _arraysize(pOff), true, 0);
    spFullPWMManager->setProgram(kLEDs1, pMalevolent, _arraysize(pMalevolent), true, 0);
    spFullPWMManager->setProgram(kLEDs2, pMalevolent, _arraysize(pMalevolent), true, 1);
    spFullPWMManager->setProgram(kLEDs3, pMalevolent, _arraysize(pMalevolent), true, 2);
    spFullPWMManager->setProgram(kLEDs4, pMalevolent, _arraysize(pMalevolent), true, 1);
}

int64_t _relaxToMalevolent(alarm_id_t id, void *user_data) {
    _triggerUp();
    return 0;
}

static void _triggerDoublePress(void) {
    if (nullptr == spFullPWMManager) {
        return;
    }
    spFullPWMManager->setProgram(kLaser, pLaserFiring, _arraysize(pLaserFiring), false, 0);
    spFullPWMManager->setProgram(kLEDs1, pFiringSequence, _arraysize(pFiringSequence), false, 1);
    spFullPWMManager->setProgram(kLEDs2, pFiringSequence, _arraysize(pFiringSequence), false, 2);
    spFullPWMManager->setProgram(kLEDs3, pFiringSequence, _arraysize(pFiringSequence), false, 3);
    spFullPWMManager->setProgram(kLEDs4, pFiringSequence, _arraysize(pFiringSequence), false, 4);
    add_alarm_in_ms(2000, _relaxToMalevolent, nullptr, false);
}

// Button is active-low (pull down)
static bool _buttonCallback(struct repeating_timer *pTimer) {
    static uint uLastEventTicks(0);
    static bool sbLastState(false), sbCurrentState(false);
    static uint suLastDownTime(0);

    bool bButtonState(gpio_get(kButton));  // pressed is 'true'
    if (bButtonState == sbLastState) {      // Detected steady
        if (sbCurrentState != sbLastState) {    // Detected change
            if (true == sbCurrentState) {
                uint uNow(time_us_32());
                uint uDelay(uNow - suLastDownTime);
                printf("uDelay = %dusec\r\n",uDelay);
                if (uDelay < kDoublePressUsec) {
                    _triggerDoublePress();
                } else {
                    _triggerDown();
                }
                suLastDownTime = time_us_32();
            } else {
                _triggerUp();
            }
            sbCurrentState = sbLastState;
        }
    }
    sbLastState = bButtonState;
    return true;
}

int main() {
    // Initialize stdio functionality
    stdio_init_all();

    const uint pPWMPins[] = {kLEDs1, kLEDs2, kLEDs3, kLEDs4, kLaser, kAliveLED};
    PWMManager cPWMManager(pPWMPins, _arraysize(pPWMPins));
    spPWMManager = spFullPWMManager = &cPWMManager;    // Link for ISR and button callback routines
    
    _triggerUp();   // Initial state
    cPWMManager.setProgram(kAliveLED, pHeartbeat, _arraysize(pHeartbeat), true, 0);

    // Set up GPIO and add timer for button detection
    gpio_init(kButton);
    //gpio_set_function(kButton, GPIO_FUNC_XIP);
    gpio_set_input_enabled(kButton, true);
    gpio_set_input_hysteresis_enabled(kButton, true);
    struct repeating_timer cButtonTimer;
    add_repeating_timer_ms(10, _buttonCallback, NULL, &cButtonTimer);

    // Everything after this point happens in the PWM interrupt handler, so we
    // can twiddle our thumbs
    while (1)
        tight_loop_contents();
}
