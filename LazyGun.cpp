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
#include <memory.h>
#include <math.h>
#include "pico/time.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/sync.h"

#include "PWMDriver.h"
#include "resources/bwowww.cpp"
#include "resources/LaserShot1.cpp"
#include "resources/LaserShot2.cpp"
#include "resources/LaserShot4.cpp"


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

#define kClkFrequency       (125.0e6f)
#define k8BitPWMUpdateFreq  (kClkFrequency/256)
#define _min(a,b)           (((a)<(b))?(a):(b))

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

// Sample player PWMDriver::Source, PWMDriver::Group
//-------------------------------------
#define kSampleBufferSize   (1024)
class SamplePlayer :  public PWMDriver::Source,
                      public PWMDriver::Group {
    private:
        // Source sample information
        const uint8_t *mpSample;
        uint muSamples;
        uint muSampleRate;

        // DMA channel and timer
        static SamplePlayer *mspInstance;
        uint muDMAChannel;
        uint muDMATimer;

        // Double-buffer (16-bit)
        uint16_t mpBuffer[2 * kSampleBufferSize];
        const uint16_t *mpActiveBuffer;
        const uint16_t *mpPreparedBuffer;

        // Read state over samples
        uint muSamplesRemaining;
        const uint8_t *mpSampleRead;
        bool mbRepeat;
        bool mbPlaying;

        // If exhausted sample stream, returns false
        inline bool _copyTo16BitBuffer(uint16_t *pDest, uint uCopySize) {
            //printf("_copyTo16BitBuffer(pDest=0x%08x, uCopySize=%d)\r\n", (uint)pDest, uCopySize);
            if (0 == uCopySize) {
                return true;
            }

            if ((0 == muSamplesRemaining) && (false == mbRepeat)) {
                return false;
            }

            uint uThisCopySize(_min(uCopySize, muSamplesRemaining));
            uint16_t *pWr(pDest);
            for(uint i=0; i<uThisCopySize; i++) {
                *(pWr++) = (uint16_t)*(mpSampleRead++);
            }
            muSamplesRemaining -= uThisCopySize;

            // Handle buffer exhaustion - always return a complete buffer
            if (uThisCopySize < uCopySize) {
                uint uRemainder(uCopySize-uThisCopySize);
                if (false == mbRepeat) {
                    memset(pWr, 0x00, sizeof(uint16_t)*uRemainder);
                } else {
                    mpSampleRead = mpSample;
                    muSamplesRemaining = muSamples;
                    return _copyTo16BitBuffer(pWr, uRemainder);
                }
            }

            return true;
        }

        const uint16_t *prepareNextBuffer(void) {
            uint16_t *pPreparedBuffer((mpActiveBuffer != mpBuffer)?mpBuffer:&mpBuffer[kSampleBufferSize]);
            if (false == _copyTo16BitBuffer(pPreparedBuffer, kSampleBufferSize)) {
                return nullptr;
            }
            return pPreparedBuffer;
        }

        inline void handleIRQ(void) {
           mpActiveBuffer = mpPreparedBuffer;
           if (mpActiveBuffer != nullptr) {
                dma_channel_set_read_addr(muDMAChannel, mpActiveBuffer, true);
                mpPreparedBuffer = prepareNextBuffer();
           } else {
                //giCalls2++;
                halt();
           }
        }

        // Handle interrupt on completion of transfer sequence.
        static void _DMAISR(void) {
            //giCalls++;
            if (mspInstance != nullptr) {
                dma_channel_acknowledge_irq0(mspInstance->muDMAChannel);
                mspInstance->handleIRQ();
            }
        }

        static void _computeDMATimerNumDen(uint &uSampleRate, uint16_t &uNum, uint16_t &uDen) {
            float fTarget((float)uSampleRate), fMinError(1000.0f);
            int iBestDen(0), iBestNum(0);
            for (int iNum = 1; iNum < 65536; iNum++) {
                int iDen((int)roundf((kClkFrequency * (float)iNum) / fTarget));
                if (iDen >= 0x10000) {
                    break;
                }
                float fResult((kClkFrequency * iNum) / iDen);
                float fError(fabs(fTarget - fResult));
                if (fError < fMinError) {
                    fMinError = fError;
                    iBestDen = iDen;
                    iBestNum = iNum;
                }
            }
            uNum = (uint)iBestNum;
            uDen = (uint)iBestDen;
        }

    public:
        SamplePlayer(uint uGPIO, uint uSampleRate) :
            PWMDriver::Source(uGPIO),
            mpSample(nullptr), muSamples(0), muSampleRate(uSampleRate), mbRepeat(false),
            mbPlaying(false), mpActiveBuffer(nullptr), mpPreparedBuffer(nullptr) {

            addSource(this);
            mspInstance = this;
        }

        ~SamplePlayer(void) {
            PWMDriver::instance()->removeGroup(this);
            removeSource(this);

            mspInstance = nullptr;
        }

        static SamplePlayer *instance(void) {
            return mspInstance;
        }

        void playSample(const uint8_t *pSample, uint uSamples, bool bRepeat) {
            halt();
            mpSample = pSample;
            muSamples = uSamples;
            mbRepeat = bRepeat;
            
            // Prepare sample buffer (from the top)
            mpSampleRead = mpSample;
            muSamplesRemaining = muSamples;
            mpActiveBuffer = mpPreparedBuffer = nullptr;
            mpActiveBuffer = prepareNextBuffer();
            mpPreparedBuffer = prepareNextBuffer();
            printf("mpActiveBuffer = 0x%08x, mpPreparedBuffer = 0x%08x\r\n", (uint)mpActiveBuffer, (uint)mpPreparedBuffer);

            PWMDriver::instance()->addGroup(this);  // Only fires first time...

            start();
        }

        //void stopPlaying(void) {
        //    if (true == mbPlaying) {
        //        PWMDriver::instance()->removeGroup(this);
        //        mbPlaying = false;
        //    }
        //}

        // Subclass required implementations
        virtual float getDesiredUpdateFrequency(void) {
            return k8BitPWMUpdateFreq;
        }

        // Override ensures that IRQ is not enabled
        virtual void configure(void) {
            printf("SamplePlayer configure\r\n");
            pwm_set_irq_enabled(muSlice, false);    // Ensure Wrap DMA for this slice is disabled
            PWMDriver::Source::configure(); // Setup PWM GPIO configuration and timing

            // Grab and setup DMA channel
            muDMAChannel = dma_claim_unused_channel(true);
            dma_channel_config cDMAConfig(dma_channel_get_default_config(muDMAChannel));
            channel_config_set_transfer_data_size(&cDMAConfig, DMA_SIZE_16); //!!!!!
            channel_config_set_read_increment(&cDMAConfig, true);
            channel_config_set_write_increment(&cDMAConfig, false);

            // Grab and setup DMA pacing timer (at audio rate - 44.1kHz)
            muDMATimer = dma_claim_unused_timer(true);
            uint16_t uNum(0), uDen(0);
            _computeDMATimerNumDen(muSampleRate, uNum, uDen);
            float fActualSampleRate((kClkFrequency * (float)uNum) / (float)uDen);
            printf("Timer X/Y for required sample rate %d = %d/%d - giving %f\r\n", muSampleRate, uNum, uDen, fActualSampleRate);
            dma_timer_set_fraction(muDMATimer, uNum, uDen);  // Should be pretty damn exact...

            // Attach timer to DMA channel
            channel_config_set_dreq(&cDMAConfig, dma_get_timer_dreq(muDMATimer));
            volatile void *pPWMCountRegister(&(pwm_hw->slice[muSlice].cc));
            uint uChannel(pwm_gpio_to_channel(muGPIO));
            if (uChannel != 0) {    // Channel B on higher 16-bits
                pPWMCountRegister = (void*)((uint)pPWMCountRegister+2);
            }

            //for(uint i=0; i<4; i++) {
            //    mpActiveBuffer = mpPreparedBuffer;
            //    mpPreparedBuffer = prepareNextBuffer();
            //    printf("iter mpActiveBuffer = 0x%08x, mpPreparedBuffer = 0x%08x\r\n", (uint)mpActiveBuffer, (uint)mpPreparedBuffer);               
            //}

            printf("--GPIO=%d, channel=%d, pPWMCountRegister=0x%08x\r\n", muGPIO, uChannel, (uint)pPWMCountRegister);
            dma_channel_configure(
                muDMAChannel,               // Channel to be configured
                &cDMAConfig,                // The configuration we just created
                pPWMCountRegister,          // The initial write address 8-bits
                mpActiveBuffer,             // The initial read address
                kSampleBufferSize,          // Number of transfers; 16-bits each
                false                       // Do not start immediately.
            );

            // Completion interrupt via DMA_IRQ0
            dma_channel_set_irq0_enabled(muDMAChannel, true);
            irq_set_exclusive_handler(DMA_IRQ_0, _DMAISR);
            irq_set_enabled(DMA_IRQ_0, true);
        }

        virtual bool start(void) {
            PWMDriver::Source::start();
            pwm_set_enabled(muSlice, true);
            //pwm_set_gpio_level(muGPIO, 0);
            //dma_channel_start(muDMAChannel);    
            dma_channel_set_read_addr(muDMAChannel, mpActiveBuffer, true);
 
            return true;       
            //if (true == PWMDriver::Source::start()) {
            //    pwm_set_gpio_level(muGPIO, 0);
            //    dma_channel_start(muDMAChannel);
            //    return true;
            //}
            //return false;
        }

        virtual bool halt(void) {
            dma_channel_abort(muDMAChannel);
            pwm_set_enabled(muSlice, false);
            //if (true == PWMDriver::Source::halt()) {
            //    pwm_set_enabled(muSlice, false);
            //    dma_channel_abort(muDMAChannel);
            //    return true;
            //}
            //return false;
            return true;
        }

        // Overrides that manage DMA channel sequencing
        virtual void resetSequence(void) {
        }

        // Required - but not used (replaced by DMA)
        virtual float getNextSequence(void) {
            printf("GetNextSequence\r\n");
            return 0.5f;
        }
};
SamplePlayer *SamplePlayer::mspInstance = nullptr;


#define _arraysize(a)  (sizeof(a)/sizeof(a[0]))

// LED output programs
static const PWMManager::PWMStep pOff[] = {{0.01f, 0.0f}, {1.0f, 0.00f}};
static const PWMManager::PWMStep pHeartbeat[] = {{0.1f, 0.8f}, {0.1f, 0.0f}, {0.2f, 0.0f}, {0.1f, 1.0f}, {0.1f, 0.2f}, {0.3f, 0.0f}};
static const PWMManager::PWMStep pMalevolent[] = {
    {0.6f, 0.1f}, {0.2f, 0.05f}, {0.2f, 0.05f}, {0.2f, 0.05f}, {0.2f, 0.05f}, {1.0f, 0.5f},
};
static const PWMManager::PWMStep pPointerWave[] = {{0.2f, 0.5f}, {0.2f, 0.5f}, {0.2f, 0.25f}, {0.2f, 0.1f}, {0.2f, 0.0f}};
static const PWMManager::PWMStep pLaserPulse[] = {{0.04f, 1.0f}, {0.4f, 0.8f}, {0.2f, 0.4f}};
static const PWMManager::PWMStep pLaserFiring[] = {
    {0.05f, 0.0f}, {0.05f, 1.0f}, 
    {0.05f, 0.0f}, {0.05f, 1.0f}, 
    {0.05f, 0.0f}, {0.05f, 1.0f}, 
    {0.05f, 0.0f}, {0.05f, 1.0f}, 
    {0.5f, 0.0f}};
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
    SamplePlayer::instance()->playSample(pBwowww, kBwowwwSize, true);
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
    SamplePlayer::instance()->halt();
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
    SamplePlayer::instance()->playSample(pLaserShot4, kLaserShot4Size, false);
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

    SamplePlayer cAudioOut(14, 16000);
    
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
