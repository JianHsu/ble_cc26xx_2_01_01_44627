/// \addtogroup module_scif_driver_setup
//@{
#include "scif.h"
#include "scif_framework.h"
#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <inc/hw_aon_event.h>
#include <inc/hw_aon_rtc.h>
#include <inc/hw_aon_wuc.h>
#include <inc/hw_aux_sce.h>
#include <inc/hw_aux_smph.h>
#include <inc/hw_aux_evctl.h>
#include <inc/hw_aux_aiodio.h>
#include <inc/hw_aux_timer.h>
#include <inc/hw_aux_wuc.h>
#include <inc/hw_event.h>
#include <inc/hw_ints.h>
#include <inc/hw_ioc.h>
#include <string.h>
#if defined(__IAR_SYSTEMS_ICC__)
    #include <intrinsics.h>
#endif


// OSAL function prototypes
uint32_t scifOsalEnterCriticalSection(void);
void scifOsalLeaveCriticalSection(uint32_t key);




/// Firmware image to be uploaded to the AUX RAM
static const uint16_t pAuxRamImage[] = {
    /*0x0000*/ 0x1408, 0x040C, 0x1408, 0x042C, 0x1408, 0x0447, 0x1408, 0x044D, 0x4436, 0x2437, 0xAEFE, 0xADB7, 0x6442, 0x7000, 0x7C6B, 0x6870, 
    /*0x0020*/ 0x0068, 0x1425, 0x6871, 0x0069, 0x1425, 0x6872, 0x006A, 0x1425, 0x786B, 0xF801, 0xFA01, 0xBEF2, 0x786E, 0x6870, 0xFD0E, 0x6872, 
    /*0x0040*/ 0xED92, 0xFD06, 0x7C6E, 0x642D, 0x0451, 0x786B, 0x8F1F, 0xED8F, 0xEC01, 0xBE01, 0xADB7, 0x8DB7, 0x6630, 0x6542, 0x0000, 0x186E, 
    /*0x0060*/ 0x9D88, 0x9C01, 0xB60D, 0x1067, 0xAF19, 0xAA00, 0xB609, 0xA8FF, 0xAF39, 0xBE06, 0x0C6B, 0x8869, 0x8F08, 0xFD47, 0x9DB7, 0x086B, 
    /*0x0080*/ 0x8801, 0x8A01, 0xBEEC, 0x262F, 0xAEFE, 0x4630, 0x0451, 0x5527, 0x6642, 0x0000, 0x0C6B, 0x140B, 0x0451, 0x6742, 0x86FF, 0x03FF, 
    /*0x00A0*/ 0x0C6D, 0x786C, 0xCD47, 0x686D, 0xCD06, 0xB605, 0x0000, 0x0C6C, 0x7C6F, 0x652D, 0x0C6D, 0x786D, 0xF801, 0xE92B, 0xFD0E, 0xBE01, 
    /*0x00C0*/ 0x6436, 0xBDB7, 0x241A, 0xA6FE, 0xADB7, 0x641A, 0xADB7, 0x0000, 0x0086, 0x0091, 0x0107, 0x0000, 0x0000, 0xFFFF, 0x0000, 0x0000, 
    /*0x00E0*/ 0x0000, 0x0000, 0x0000, 0x0005, 0x0006, 0x0007, 0x00F2, 0x00F3, 0x0079, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
    /*0x0100*/ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x5073, 0xFF1D, 0x4428, 0x1508, 0x8801, 0x8A03, 0xAEF9, 0x0001, 0x0C67, 
    /*0x0120*/ 0xADB7, 0x0876, 0x1877, 0x8D29, 0x0001, 0xBE05, 0x0000, 0x186C, 0x8601, 0x9200, 0x1C6C, 0x8A01, 0xBE67, 0x241D, 0xA6FE, 0x241B, 
    /*0x0140*/ 0xA6FE, 0x1510, 0x0061, 0x1462, 0x8B4A, 0xFD47, 0x8B4A, 0x1465, 0x1462, 0x0180, 0x86A0, 0x8B80, 0x0000, 0x86A0, 0x8B40, 0xFD47, 
    /*0x0160*/ 0x86A0, 0x8900, 0x1465, 0x0103, 0x8B08, 0x0000, 0x8640, 0x8B18, 0x8640, 0x641C, 0x1520, 0x0000, 0x5073, 0xFF1D, 0x1524, 0x7002, 
    /*0x0180*/ 0x1531, 0x5073, 0xFF1D, 0x153D, 0x6428, 0x700F, 0x1545, 0x5073, 0xFF1D, 0x4428, 0x1508, 0xD906, 0x6878, 0xE803, 0xDF3E, 0x6885, 
    /*0x01A0*/ 0xEA00, 0xBE03, 0x6878, 0xDF3E, 0x04E6, 0x5878, 0xEF1D, 0xEDAA, 0x5878, 0xDF1D, 0xDD1E, 0x6878, 0xDF3E, 0x5878, 0xD803, 0xEF1D, 
    /*0x01C0*/ 0xEDAA, 0x5878, 0xDF1D, 0xDD26, 0x6878, 0xDF3E, 0x8801, 0x8A03, 0xAED3, 0x0001, 0x0C85, 0x1551, 0x1462, 0x70FF, 0xFB52, 0xFD47, 
    /*0x01E0*/ 0xFB52, 0x1465, 0x1557, 0x641D, 0x641B, 0x0876, 0x880C, 0x8601, 0x8A0A, 0xAE02, 0x88E8, 0x8401, 0x1001, 0x8E00, 0x286C, 0xAD09, 
    /*0x0200*/ 0x2C6C, 0x0C76, 0x8DA9, 0x0C78, 0x0001, 0x0C67, 0xADB7, 0xADB7, 0xED47, 0xEDAB, 0xE816, 0xF007, 0x5001, 0xDD87, 0xDF26, 0xADB7, 
    /*0x0220*/ 0x643C, 0x243D, 0xA6FE, 0x1462, 0x700F, 0xFB50, 0x86FF, 0x7300, 0xFB53, 0x86C1, 0x7100, 0xFB4A, 0xFD47, 0xFB4A, 0x1465, 0xADB7, 
    /*0x0240*/ 0x6432, 0x2532, 0xA6FE, 0xADB7, 0xF007, 0x1462, 0x86FF, 0x63F0, 0xEB50, 0x8680, 0x6000, 0xED8F, 0xEB48, 0xFD47, 0xEB48, 0x1465, 
    /*0x0260*/ 0xADB7, 0x6003, 0xEB04, 0x600F, 0x8640, 0xEB0C, 0x6000, 0xEB04, 0xFB04, 0x7641, 0xA500, 0xFD47, 0xADB7, 0xED47, 0xEDAB, 0xE814, 
    /*0x0280*/ 0xF007, 0x5001, 0xDD87, 0xDF26, 0xADB7, 0xFB0C, 0x7070, 0xFB09, 0x7741, 0x640B, 0x8650, 0xF90C, 0xFCC0, 0xB6FC, 0x440B, 0x7741, 
    /*0x02A0*/ 0xADB7, 0x6003, 0xEB04, 0x4432, 0x2532, 0xAEFE, 0xADB7, 0x1462, 0x86FF, 0x7300, 0xFB52, 0xFD47, 0xFB52, 0x1465, 0x443C, 0x243D, 
    /*0x02C0*/ 0xAEFE, 0xADB7
};


/// Look-up table that converts from AUX I/O index to MCU IOCFG offset
static const uint8_t pAuxIoIndexToMcuIocfgOffsetLut[] = {
    120, 116, 112, 108, 104, 100, 96, 92, 28, 24, 20, 16, 12, 8, 4, 0
};


/** \brief Look-up table of data structure information for each task
  *
  * There is one entry per data structure (\c cfg, \c input, \c output and \c state) per task:
  * - [31:20] Data structure size (number of 16-bit words)
  * - [19:12] Buffer count (when 2+, first data structure is preceded by buffering control variables)
  * - [11:0] Address of the first data structure
  */
static const uint32_t pScifTaskDataStructInfoLut[] = {
//  cfg         input       output      state       
    0x003010E6, 0x00000000, 0x006020F2, 0x0010110A  // Capacitive Touch Data Logger
};




// No task-specific initialization functions




// No task-specific uninitialization functions




/** \brief Initilializes task resource hardware dependencies
  *
  * This function is called by the internal driver initialization function, \ref scifInit().
  */
static void scifTaskResourceInit(void) {
    scifInitIo(5, AUXIOMODE_OPEN_DRAIN, -1, 1);
    scifInitIo(6, AUXIOMODE_OPEN_DRAIN, -1, 1);
    scifInitIo(7, AUXIOMODE_OPEN_DRAIN, -1, 1);
} // scifTaskResourceInit




/** \brief Uninitilializes task resource hardware dependencies
  *
  * This function is called by the internal driver uninitialization function, \ref scifUninit().
  */
static void scifTaskResourceUninit(void) {
    scifUninitIo(5, -1);
    scifUninitIo(6, -1);
    scifUninitIo(7, -1);
} // scifTaskResourceUninit




/** \brief Re-initializes I/O pins used by the specified tasks
  *
  * It is possible to stop a Sensor Controller task and let the System CPU borrow and operate its I/O
  * pins. For example, the Sensor Controller can operate an SPI interface in one application state while
  * the System CPU with SSI operates the SPI interface in another application state.
  *
  * This function must be called before \ref scifExecuteTasksOnceNbl() or \ref scifStartTasksNbl() if
  * I/O pins belonging to Sensor Controller tasks have been borrowed System CPU peripherals.
  *
  * \param[in]      bvTaskIds
  *     Bit-vector of task IDs for the task I/Os to be re-initialized
  */
void scifReinitTaskIo(uint32_t bvTaskIds) {
    if (bvTaskIds & (1 << SCIF_CAPACITIVE_TOUCH_DATA_LOGGER_TASK_ID)) {
        scifReinitIo(5, -1);
        scifReinitIo(6, -1);
        scifReinitIo(7, -1);
    }
} // scifReinitTaskIo




/// Driver setup data, to be used in the call to \ref scifInit()
const SCIF_DATA_T scifDriverSetup = {
    (volatile SCIF_INT_DATA_T*) 0x400E00D6,
    (volatile SCIF_TASK_CTRL_T*) 0x400E00DC,
    (volatile uint16_t*) 0x400E00CE,
    0x0000,
    sizeof(pAuxRamImage),
    pAuxRamImage,
    pScifTaskDataStructInfoLut,
    pAuxIoIndexToMcuIocfgOffsetLut,
    scifTaskResourceInit,
    scifTaskResourceUninit
};




/** \brief Starts or modifies RTC-based task scheduling tick generation
  *
  * RTC-based tick generation will wake up the Sensor Controller first at the specified value of the RTC
  * and then periodically at the specified interval. The application must call this function after
  * calling \ref scifInit().
  *
  * The application must ensure that:
  * - \a tickStart is not in the past (prefer using \ref scifStartRtcTicksNow() to avoid this)
  * - \a tickPeriod is not too short. RTC ticks will be skipped silently if the Sensor Controller does
  *   not complete its tasks within a single tick interval.
  *
  * \param[in]      tickStart
  *     RTC value when the first tick is generated:
  *     - Bits 31:16 = seconds
  *     - Bits 15:0 = 1/65536 of a second
  * \param[in]      tickPeriod
  *     Interval at which subsequent ticks are generated:
  *     - Bits 31:16 = seconds
  *     - Bits 15:0 = 1/65536 of a second
  */
void scifStartRtcTicks(uint32_t tickStart, uint32_t tickPeriod) {
    HWREG(AON_RTC_BASE + AON_RTC_O_CH2CMP) = tickStart;
    HWREG(AON_RTC_BASE + AON_RTC_O_CH2CMPINC) = tickPeriod;
    HWREG(AON_RTC_BASE + AON_RTC_O_CHCTL) |= AON_RTC_CHCTL_CH2_EN_M | AON_RTC_CHCTL_CH2_CONT_EN_M;
    HWREG(AON_EVENT_BASE + AON_EVENT_O_AUXWUSEL) =
        (HWREG(AON_EVENT_BASE + AON_EVENT_O_AUXWUSEL) & ~AON_EVENT_AUXWUSEL_WU0_EV_M) |
        (AON_EVENT_AUXWUSEL_WU0_EV_RTC_CH2);
} // scifStartRtcTicks




/** \brief Starts or modifies RTC-based task scheduling tick generation
  *
  * RTC-based tick generation will wake up the Sensor Controller first after approximately 128 us and
  * then periodically at the specified interval. The application must call this function after calling
  * \ref scifInit().
  *
  * The application must ensure that \a tickPeriod is not too short. RTC ticks will be skipped silently
  * if the Sensor Controller does not complete its tasks within a single tick interval.
  *
  * \param[in]      tickPeriod
  *     Interval at which subsequent ticks are generated:
  *     - Bits 31:16 = seconds
  *     - Bits 15:0 = 1/65536 of a second
  */
void scifStartRtcTicksNow(uint32_t tickPeriod) {
    uint32_t key, sec, subsec;
    key = scifOsalEnterCriticalSection();
    sec = HWREG(AON_RTC_BASE + AON_RTC_O_SEC);
    subsec = HWREG(AON_RTC_BASE + AON_RTC_O_SUBSEC);
    scifStartRtcTicks(((sec << 16) | (subsec >> 16)) + 8, tickPeriod);
    scifOsalLeaveCriticalSection(key);
} // scifStartRtcTicksNow




/** \brief Stops RTC-based task scheduling tick generation
  *
  * The application must call this function before calling \ref scifUninit().
  */
void scifStopRtcTicks(void) {
    HWREG(AON_RTC_BASE + AON_RTC_O_CHCTL) &= ~(AON_RTC_CHCTL_CH2_EN_M | AON_RTC_CHCTL_CH2_CONT_EN_M);
    HWREG(AON_EVENT_BASE + AON_EVENT_O_AUXWUSEL) =
        (HWREG(AON_EVENT_BASE + AON_EVENT_O_AUXWUSEL) & ~AON_EVENT_AUXWUSEL_WU0_EV_M) |
        (AON_EVENT_AUXWUSEL_WU0_EV_NONE);
    HWREG(AON_RTC_BASE + AON_RTC_O_SYNC);
} // scifStopRtcTicks


//@}


// Generated by WOLF at 2016-12-26 14:10:54.040
