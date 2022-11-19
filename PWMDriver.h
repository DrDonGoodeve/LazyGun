/******************************************************************************
 * PWMDriver.h
 * 
 * PWM Driver module. Provides functions for driving single and groups of
 * PWM outputs from signal source objects. Versatile enough to handle
 * servo control, light dimming, audio output etc...
 * 
 * ****************************************************************************
 * Code Copyright(C) 2022 Don Goodeve. Open source - no restrictions
 * ***************************************************************************/

#include "pico/stdlib.h"
#include <list>
#include <map>


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

#define _ignore(a)  


// PWM driver class. Singleton that provides control for groups and individual
// PWM outputs.
//-----------------------------------------------------------------------------
class PWMDriver {
    public:
        // A source creates a signal comprising a series of values over time. ::getNextValue will be
        // called inside an ISR and should therefore be *very* efficient. However the
        // default mechanism can be overriden to, for example, allow a source to utilize
        // DMA for higher efficiency operation without breaking the abstraction.
        class Source {
            private:
                friend class Group;
                float mfClkDiv;

            protected:
                uint muGPIO;
                uint muSlice;
                uint muWrapValue;
                float mfSampleRateHz;
                bool mbRunning;
                
            public:
                Source(uint uGPIO);
                ~Source();

                uint getGPIO(void) const;
                float getSampleRateHz(void) const;

                // Subclass overrideable
                virtual void setPWMConfiguration(float fSampleRateHz, float fClkDiv, uint uWrapValue);
                virtual void configure(void);
                virtual bool start(void);
                virtual bool halt(void);                
                
                void updateSource(void);

                // Subclass required implementations
                virtual float getDesiredUpdateFrequency(void) = 0;
                virtual void resetSequence(void) = 0;      // Reset to start of output sequence
                virtual float getNextSequence(void) = 0;   // Get next value to use in range (0.0,1.0]
        };

        // A group is a collection of sources that share a common sample rate and are updated in unison.
        class Group {
            private:
                bool mbRunning;
                uint muWrapCount;
                uint muClkDivider;

                friend class PWMDriver;
                uint muIRQSlice;                // Typically of the first source added to the group
                std::list<Source*> mlSources;
                void update(void);

            public:
                Group(void);
                ~Group();

                // Manage sources within group
                bool addSource(Source *pSource);
                bool removeSource(Source *pSource);

                // Control
                void start(void);
                void halt(void);

                // User implemented to describe parameters of the actual group
                float getDesiredUpdateFrequency(void) const;           // Return desired PWM frequency
                void setPWMConfiguration(float fSampleRateHz, float fClkDiv, uint uWrapValue);
        };

    private:
        static PWMDriver *mspInstance;

        std::list<Group*> mlGroups;

        static void pwmISRStatic(void);
        void pwmISR(void);

        PWMDriver(void);

        uint scaleValueToWrap(float fValue);
 
    public:
        ~PWMDriver();

        static PWMDriver *instance(void);

        bool addGroup(Group *pGroup);
        bool removeGroup(Group *pGroup);
};
