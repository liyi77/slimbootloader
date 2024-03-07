/** @file

  Copyright (c) 2020 - 2023, Intel Corporation. All rights reserved.<BR>
  SPDX-License-Identifier: BSD-2-Clause-Patent

**/


#include <PiPei.h>
#include <Library/BaseLib.h>
#include <Library/BoardInitLib.h>
#include <Library/SerialPortLib.h>
#include <Library/PlatformHookLib.h>
#include <Library/BootloaderCoreLib.h>
#include <PchAccess.h>
#include <CpuRegs.h>
#include <FsptUpd.h>
#include <PlatformData.h>
#include <Library/GpioLib.h>
#include <GpioConfig.h>
#include <GpioPinsVer4S.h>
#include <GpioPinsVer2Lp.h>
#include <Library/ConfigDataLib.h>
#include <Library/PchInfoLib.h>
#include <Library/TcoTimerLib.h>
#include <Register/Intel/ArchitecturalMsr.h>
#include <Library/PagingLib.h>

#define UCODE_REGION_BASE   FixedPcdGet32(PcdUcodeBase)
#define UCODE_REGION_SIZE   FixedPcdGet32(PcdUcodeSize)
//#define SG1B_REDB_BASE      (UINT32) ((2 * FixedPcdGet32(PcdTopSwapRegionSize)) + FixedPcdGet32(PcdRedundantRegionSize) + FixedPcdGet32(PcdStage1BSize))
#define CODE_REGION_SIZE    0x00400000
#define CODE_REGION_BASE    0xFFC00000

#define ADL_MAX_SERIALIO_UART_CONTROLLERS     3

CONST
FSPT_UPD TempRamInitParams = {
  .FspUpdHeader = {
    .Signature = FSPT_UPD_SIGNATURE,
    .Revision  = 2,
    .Reserved  = {0},
  },
  .FsptArchUpd = {
    .Revision = 1,
    .Length = 0x20,
    .Reserved  = {0},
  },
  .FsptCoreUpd = {
    .MicrocodeRegionBase    = UCODE_REGION_BASE,
    .MicrocodeRegionSize    = UCODE_REGION_SIZE,
    .CodeRegionBase         = CODE_REGION_BASE,
    .CodeRegionSize         = CODE_REGION_SIZE,
    .Reserved               = {0},
  },
  .FsptConfig = {
    .PcdSerialIoUartDebugEnable = 1,
    .PcdSerialIoUartNumber      = FixedPcdGet32 (PcdDebugPortNumber) < PCH_MAX_SERIALIO_UART_CONTROLLERS ? \
                                    FixedPcdGet32 (PcdDebugPortNumber) : 2,
    .PcdSerialIoUartMode        = 4, // SerialIoUartSkipInit, let SBL init UART
#if defined(PLATFORM_ADLN) || defined(PLATFORM_ASL)
    .PcdSerialIoUartPowerGating = 1,
#endif
    .PcdSerialIoUartBaudRate    = 115200,
    .PcdPciExpressBaseAddress   = FixedPcdGet32 (PcdPciMmcfgBase),
    .PcdPciExpressRegionLength  = 0x10000000,
    .PcdSerialIoUartParity      = 1, // NoParity
    .PcdSerialIoUartDataBits    = 8,
    .PcdSerialIoUartStopBits    = 1,
    .PcdSerialIoUartAutoFlow    = 0,
    .PcdSerialIoUartRxPinMux    = 0,
    .PcdSerialIoUartTxPinMux    = 0,
    .PcdSerialIoUartRtsPinMux   = 0,
    .PcdSerialIoUartCtsPinMux   = 0,
    .PcdLpcUartDebugEnable      = 1,
  },
  .UpdTerminator = 0x55AA,
};

CONST GPIO_INIT_CONFIG mAdlsUartGpioTable[] = {
  {GPIO_VER4_S_GPP_C8,  {GpioPadModeNative1, GpioHostOwnGpio, GpioDirNone,  GpioOutDefault, GpioIntDis, GpioHostDeepReset,  GpioTermNone}},//SERIALIO_UART0_RXD
  {GPIO_VER4_S_GPP_C9,  {GpioPadModeNative1, GpioHostOwnGpio, GpioDirNone,  GpioOutDefault, GpioIntDis, GpioHostDeepReset,  GpioTermNone}},//SERIALIO_UART0_TXD
  {GPIO_VER4_S_GPP_C12, {GpioPadModeNative1, GpioHostOwnGpio, GpioDirNone,  GpioOutDefault, GpioIntDis, GpioHostDeepReset,  GpioTermNone}},//SERIALIO_UART1_RXD
  {GPIO_VER4_S_GPP_C13, {GpioPadModeNative1, GpioHostOwnGpio, GpioDirNone,  GpioOutDefault, GpioIntDis, GpioHostDeepReset,  GpioTermNone}},//SERIALIO_UART1_TXD
  {GPIO_VER4_S_GPP_C20, {GpioPadModeNative1, GpioHostOwnGpio, GpioDirNone,  GpioOutDefault, GpioIntDis, GpioHostDeepReset,  GpioTermNone}},//SERIALIO_UART2_RXD
  {GPIO_VER4_S_GPP_C21, {GpioPadModeNative1, GpioHostOwnGpio, GpioDirNone,  GpioOutDefault, GpioIntDis, GpioHostDeepReset,  GpioTermNone}},//SERIALIO_UART2_TXD
};

CONST GPIO_INIT_CONFIG mAdlpUartGpioTable[] = {
  {GPIO_VER2_LP_GPP_H10, {GpioPadModeNative2, GpioHostOwnGpio, GpioDirNone,  GpioOutDefault, GpioIntDis, GpioHostDeepReset,  GpioTermNone}},//SERIALIO_UART0_RXD
  {GPIO_VER2_LP_GPP_H11, {GpioPadModeNative2, GpioHostOwnGpio, GpioDirNone,  GpioOutDefault, GpioIntDis, GpioHostDeepReset,  GpioTermNone}},//SERIALIO_UART0_TXD
  {GPIO_VER2_LP_GPP_D17, {GpioPadModeNative1, GpioHostOwnGpio, GpioDirNone,  GpioOutDefault, GpioIntDis, GpioHostDeepReset,  GpioTermNone}},//SERIALIO_UART1_RXD
  {GPIO_VER2_LP_GPP_D18, {GpioPadModeNative1, GpioHostOwnGpio, GpioDirNone,  GpioOutDefault, GpioIntDis, GpioHostDeepReset,  GpioTermNone}},//SERIALIO_UART1_TXD
  {GPIO_VER2_LP_GPP_F1,  {GpioPadModeNative2, GpioHostOwnGpio, GpioDirNone,  GpioOutDefault, GpioIntDis, GpioHostDeepReset,  GpioTermNone}},//SERIALIO_UART2_RXD
  {GPIO_VER2_LP_GPP_F2,  {GpioPadModeNative2, GpioHostOwnGpio, GpioDirNone,  GpioOutDefault, GpioIntDis, GpioHostDeepReset,  GpioTermNone}},//SERIALIO_UART2_TXD
};

/**
  Stitching process might pass some specific platform data to be
  consumed pretty early. This will be used to guide the platform initialization
  even before CFGDATA is available.

**/
VOID
EarlyPlatformDataCheck (
  VOID
)
{
  STITCH_DATA          *StitchData;

  // Stitching process might pass some platform specific data.
  StitchData = (STITCH_DATA *)(UINTN)(0xFFFFFFF4);

  if (StitchData->Marker != 0xAA) {
    // set default  as Debug UART
    // PlatformID will be deferred to be detected
    SetDebugPort (PcdGet8 (PcdDebugPortNumber));
  } else {
    SetDebugPort  (StitchData->DebugUart);
    SetPlatformId (StitchData->PlatformId);
  }
}

/**
  CAR created by ACM covers only Initial code region.
  This function will expand CAR to cover full flash to improve performance.

**/
VOID
EFIAPI
CacheFullFlash (
  VOID
)
{
  UINT32                            MsrIdx;
  UINT32                            ImgLen;
  UINT32                            AdjLen;
  UINT64                            MskLen;
  UINT32                            VariableMtrrCount;
  MSR_IA32_MTRRCAP_REGISTER         MtrrCap;
  MSR_IA32_MTRR_PHYSMASK_REGISTER   MtrrMaskReg;
  UINT64                            ValidMtrrAddressMask;
  UINT8                             PhysicalAddressBits;
  UINT8                             Index;

  // Enlarge the code cache region to cover full flash.
  // FSP-T does not allow to enable full flash code cache due to cache size restriction.
  // Here, MTRR is patched to enable full flash region cache to avoid performance penalty.
  // However, the SBL code flow should ensure only limited flash regions will be accessed
  // before FSP TempRamExit() is called. The combined DATA and CODE cache size should satisfy
  // the BWG requirement.
  if ((AsmReadMsr64(MSR_BOOT_GUARD_SACM_INFO) & B_BOOT_GUARD_SACM_INFO_NEM_ENABLED) == 0
      || PcdGetBool (PcdFastBootEnabled)) {
    MskLen = (AsmReadMsr64(MSR_CACHE_VARIABLE_MTRR_BASE + 1) | (SIZE_4GB - 1)) + 1;
    MsrIdx = MSR_CACHE_VARIABLE_MTRR_BASE + 1 * 2;
    ImgLen = PcdGet32(PcdFlashSize);
    // PCH only decodes max 16MB of SPI flash from the top down to MMIO.
    if (ImgLen > SIZE_16MB) {
      ImgLen = SIZE_16MB;
    }
    AdjLen = GetPowerOfTwo32(ImgLen);
    if (ImgLen > AdjLen) {
      AdjLen <<= 1;
    }
    AsmWriteMsr64(MsrIdx, (SIZE_4GB - AdjLen) | CACHE_WRITEPROTECTED);
    AsmWriteMsr64(MsrIdx + 1, (MskLen - AdjLen) | B_CACHE_MTRR_VALID);

    // Clear other used MTRRs set by ACM to avoid conflict.
    if ((AsmReadMsr64(MSR_BOOT_GUARD_SACM_INFO) & B_BOOT_GUARD_SACM_INFO_NEM_ENABLED) == 1) {
      // Get Valid MTRR Address Mask
      PhysicalAddressBits  = GetPhysicalAddressBits ();
      ValidMtrrAddressMask = (LShiftU64((UINT64) 1, PhysicalAddressBits) - 1) & (~(UINT64)0x0FFF);
      // Current the number of Variable MTRRs
      MtrrCap.Uint64 = AsmReadMsr64 (MSR_IA32_MTRRCAP);
      VariableMtrrCount = MtrrCap.Bits.VCNT;
      for (Index = 2; Index < VariableMtrrCount; Index++) {
        MsrIdx = MSR_CACHE_VARIABLE_MTRR_BASE + (Index << 1);
        MtrrMaskReg.Uint64 = AsmReadMsr64 (MsrIdx + 1);
        MskLen = ~(MtrrMaskReg.Uint64 & ValidMtrrAddressMask) + 1;
        if (MtrrMaskReg.Bits.V != 0 && MskLen != 0) {
          // If MtrrMaskReg.V != 0 and length of this MTRR != 0, clear this MTRR.
          AsmWriteMsr64(MsrIdx, 0);
          AsmWriteMsr64(MsrIdx + 1, 0);
        }
      }
    }

  }
}

/**
  Board specific hook points.

  Implement board specific initialization during the boot flow.

  @param[in] InitPhase             Current phase in the boot flow.

**/
VOID
EFIAPI
BoardInit (
  IN  BOARD_INIT_PHASE  InitPhase
  )
{
  UINT8                     DebugPort;
  GPIO_INIT_CONFIG          *UartGpioTable;

  switch (InitPhase) {
  case PostTempRamInit:
    // Initialize TCO timer in board-specific SG1A file
    // as not to interfere with other boards' disable TCO
    // timer functions which do the same thing
    InitTcoTimer ();
    EarlyPlatformDataCheck ();

    DebugPort = GetDebugPort ();
    if ((DebugPort != 0xFF) && (DebugPort < ADL_MAX_SERIALIO_UART_CONTROLLERS)) {
      if (IsPchLp ()) {
        UartGpioTable = (GPIO_INIT_CONFIG*) &mAdlpUartGpioTable;
      } else {
        UartGpioTable = (GPIO_INIT_CONFIG*) &mAdlsUartGpioTable;
      }
      ConfigureGpio (CDATA_NO_TAG, 2, (UINT8 *)(UartGpioTable + DebugPort * 2));
    }

    PlatformHookSerialPortInitialize ();
    SerialPortInitialize ();

    // Enlarge the code cache region to cover full flash for non-BootGuard case or fast boot enabled
    CacheFullFlash ();

    break;
  default:
    break;
  }
}


/**
  Get size of Platform Specific Data

  @param[in] none

  @retval    UINT32     Size of Platform Specific Data

**/
UINT32
EFIAPI
GetPlatformDataSize (
  IN  VOID
  )
{
  return sizeof (PLATFORM_DATA);
}
