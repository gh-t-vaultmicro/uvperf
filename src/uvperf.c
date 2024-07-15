/*!*********************************************************************
 *   uvperf.c
 *   Version : V1.0.6
 *   Author : gh-t-vaultmicro
 *   This is a simple utility to test the performance of USB transfers.
 *   It is designed to be used with the libusbK driver.
 *   The utility will perform a series of transfers to the specified endpoint
 *   and report the results.
 *
 *   Usage:
 *   uvperf -V VERBOSE-v VID -p PID -i INTERFACE -a AltInterface -e ENDPOINT -m TRANSFERMODE
 * -t TIMEOUT -b BUFFERCOUNT -l READLENGTH -w WRITELENGTH -r REPEAT -S
 *
 *   -VVERBOSE       Enable verbose output
 *   -vVID           USB Vendor ID
 *   -pPID           USB Product ID
 *   -iINTERFACE     USB Interface
 *   -aAltInterface  USB Alternate Interface
 *   -eENDPOINT      USB Endpoint
 *   -mTRANSFERMODE  0 = isochronous, 1 = bulk
 *   -tTIMEOUT       USB Transfer Timeout
 *   -bBUFFERCOUNT   Number of buffers to use
 *   -lREADLENGTH    Length of read transfers
 *   -wWRITELENGTH   Length of write transfers
 *   -rREPEAT        Number of transfers to perform
 *   -S              1 = Show transfer data, defulat = 0\n
 *
 *   Example:
 *   uvperf -v0x1004 -p0xa000 -i0 -a0 -e0x81 -m1 -t1000 -l1024 -r1000
 *
 *   This will perform 1000 bulk transfers of 1024 bytes to endpoint 0x81
 *   on interface 0, alternate setting 0 of a device with VID 0x1004 and PID 0xA000.
 *   The transfers will have a timeout of 1000ms.
 *
 ********************************************************************!*/
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include <wtypes.h>

#include <stdarg.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "libusbk.h"
#include "log.h"
#include "lusbk_linked_list.h"
#include "lusbk_shared.h"

//included libusb headerfile
#include "libusb.h"

#define MAX_OUTSTANDING_TRANSFERS 10

BOOL verbose = FALSE;

#define LOG_VERBOSE(format, ...)                                                                   \
    do {                                                                                           \
        if (verbose)                                                                               \
            printf(format, ##__VA_ARGS__);                                                         \
    } while (0)

#define VerifyListLock(mTest)                                                                      \
    while (InterlockedExchange(&((mTest)->verifyLock), 1) != 0)                                    \
    Sleep(0)
#define VerifyListUnlock(mTest) InterlockedExchange(&((mTest)->verifyLock), 0)

static LPCSTR DrvIdNames[8] = {"libusbK", "libusb0", "WinUSB", "libusb0 filter",
                               "Unknown", "Unknown", "Unknown"};
#define GetDrvIdString(DriverID)                                                                   \
    (DrvIdNames[((((LONG)(DriverID)) < 0) || ((LONG)(DriverID)) >= KUSB_DRVID_COUNT)               \
                    ? KUSB_DRVID_COUNT                                                             \
                    : (DriverID)])

KUSB_DRIVER_API K;


typedef struct _UVPERF_BUFFER {
    PUCHAR Data;
    LONG dataLenth;
    LONG syncFailed;

    struct _UVPERF_BUFFER *next;
    struct _UVPERF_BUFFER *prev;
} UVPERF_BUFFER, *PUVPERF_BUFFER;

typedef enum _BENCHMARK_DEVICE_COMMAND {
    SET_TEST = 0x0E,
    GET_TEST = 0x0F,
} UVPERF_DEVICE_COMMAND,
    *PUVPERF_DEVICE_COMMAND;

typedef enum _UVPERF_DEVICE_TRANSFER_TYPE {
    TestTypeNone = 0x00,
    TestTypeIn = 0x01,
    TestTypeOut = 0x02,
    TestTypeLoop = TestTypeIn | TestTypeOut,
} UVPERF_DEVICE_TRANSFER_TYPE,
    *PUVPERF_DEVICE_TRANSFER_TYPE;

typedef enum _UVPERF_TRANSFER_MODE {
    TRANSFER_MODE_SYNC,
    TRANSFER_MODE_ASYNC,
} UVPERF_TRANSFER_MODE;

typedef struct _BENCHMARK_ISOCH_RESULTS {
    UINT GoodPackets;
    UINT BadPackets;
    UINT Length;
    UINT TotalPackets;
} BENCHMARK_ISOCH_RESULTS;


typedef struct _UVPERF_PARAM {
    int vid;
    int pid;
    int intf;
    int altf;
    int endpoint;
    int Timer;
    int timeout;
    int refresh;
    int retry;
    int bufferlength;
    int allocBufferSize;
    int readlenth;
    int writelength;
    int bufferCount;
    int repeat;
    int fixedIsoPackets;
    int priority;
    BOOL fileIO;
    BOOL ShowTransfer;
    BOOL useList;
    BOOL verify;
    BOOL verifyDetails;
    UVPERF_DEVICE_TRANSFER_TYPE TestType;
    UVPERF_TRANSFER_MODE TransferMode;

    KLST_HANDLE DeviceList;
    KLST_DEVINFO_HANDLE SelectedDeviceProfile;
    HANDLE DeviceHandle;
    KUSB_HANDLE InterfaceHandle;
    USB_DEVICE_DESCRIPTOR DeviceDescriptor;
    USB_CONFIGURATION_DESCRIPTOR ConfigDescriptor;
    USB_INTERFACE_DESCRIPTOR InterfaceDescriptor;
    USB_ENDPOINT_DESCRIPTOR EndpointDescriptor;
    WINUSB_PIPE_INFORMATION_EX PipeInformation[32];
    BOOL isCancelled;
    BOOL isUserAborted;

    volatile long verifyLock;
    UVPERF_BUFFER *VerifyList;

    unsigned char verifyBuffer;
    unsigned short verifyBufferSize;
    BOOL use_UsbK_Init;
    BOOL listDevicesOnly;
    unsigned long deviceSpeed;

    FILE *BufferFile;
    FILE *LogFile;
    char BufferFileName[MAX_PATH];
    char LogFileName[MAX_PATH];

    UCHAR UseRawIO;
    UCHAR DefaultAltSetting;
    BOOL UseIsoAsap;
    BYTE *VerifyBuffer;

    unsigned char defaultAltSetting;
} UVPERF_PARAM, *PUVPERF_PARAM;



typedef struct _UVPERF_TRANSFER_HANDLE {
    KISOCH_HANDLE IsochHandle;
    OVERLAPPED Overlapped;
    BOOL InUse;
    PUCHAR Data;
    INT DataMaxLength;
    INT ReturnCode;
    BENCHMARK_ISOCH_RESULTS IsochResults;
} UVPERF_TRANSFER_HANDLE, *PUVPERF_TRANSFER_HANDLE;

typedef struct _UVPERF_TRANSFER_PARAM {
    PUVPERF_PARAM TestParams;
    unsigned int frameNumber;
    unsigned int numberOFIsoPackets;
    HANDLE ThreadHandle;
    DWORD ThreadId;
    WINUSB_PIPE_INFORMATION_EX Ep;
    USB_SUPERSPEED_ENDPOINT_COMPANION_DESCRIPTOR EpCompanionDescriptor;
    BOOL HasEpCompanionDescriptor;
    BOOL isRunning;

    LONGLONG TotalTransferred;
    LONG LastTransferred;

    LONG Packets;
    struct timespec StartTick;
    struct timespec LastTick;
    struct timespec LastStartTick;

    int shortTrasnferred;

    int TotalTimeoutCount;
    int RunningTimeoutCount;

    int totalErrorCount;
    int runningErrorCount;

    int TotalErrorCount;
    int RunningErrorCount;

    int shortTransferCount;

    int transferHandleNextIndex;
    int transferHandleWaitIndex;
    int outstandingTransferCount;

    UVPERF_TRANSFER_HANDLE TransferHandles[MAX_OUTSTANDING_TRANSFERS];
    BENCHMARK_ISOCH_RESULTS IsochResults;

    UCHAR Buffer[0];
} UVPERF_TRANSFER_PARAM, *PUVPERF_TRANSFER_PARAM;

#include <pshpack1.h>
typedef struct _KBENCH_CONTEXT_LSTK {
    BYTE Selected;
} KBENCH_CONTEXT_LSTK, *PKBENCH_CONTEXT_LSTK;
#include <poppack.h>

BOOL Bench_Open(__in PUVPERF_PARAM TestParams);

BOOL Bench_Configure(__in KUSB_HANDLE handle, __in UVPERF_DEVICE_COMMAND command, __in UCHAR intf,
                     __inout PUVPERF_DEVICE_TRANSFER_TYPE testType);

// Critical section for running status.
CRITICAL_SECTION DisplayCriticalSection;

// Internal function used by the benchmark application.
void ShowUsage();
void SetParamsDefaults(PUVPERF_PARAM TestParams);
int GetDeviceParam(PUVPERF_PARAM TestParams);
int GetDeviceInfoFromList(PUVPERF_PARAM TestParams);

int ParseArgs(PUVPERF_PARAM TestParams, int argc, char **argv);
void FreeTransferParam(PUVPERF_TRANSFER_PARAM *transferParamRef);

void ShowParams(PUVPERF_PARAM TestParams);
PUVPERF_TRANSFER_PARAM CreateTransferParam(PUVPERF_PARAM TestParams, int endpointID);
void GetAverageBytesSec(PUVPERF_TRANSFER_PARAM transferParam, DOUBLE *bps);
void GetCurrentBytesSec(PUVPERF_TRANSFER_PARAM transferParam, DOUBLE *bps);
void ShowRunningStatus(PUVPERF_TRANSFER_PARAM readParam, PUVPERF_TRANSFER_PARAM writeParam);
DWORD TransferThread(PUVPERF_TRANSFER_PARAM transferParam);

void FileIOOpen(PUVPERF_PARAM TestParams);
void FileIOBuffer(PUVPERF_PARAM TestParams, PUVPERF_TRANSFER_PARAM transferParam);
void FileIOLog(PUVPERF_PARAM TestParams);
void FileIOClose(PUVPERF_PARAM TestParams);

//showing descriptors
void ShowDeviceDescriptor(libusb_device *dev);
void ShowConfigurationDescriptor(libusb_device *dev);
void ShowInterfaceDescriptor(libusb_device *dev, int interface_index);
void ShowEndpointDescriptor(libusb_device *dev, int interface_index, int endpoint_index);


#define TRANSFER_DISPLAY(TransferParam, ReadingString, WritingString)                              \
    ((TransferParam->Ep.PipeId & USB_ENDPOINT_DIRECTION_MASK) ? ReadingString : WritingString)

#define INC_ROLL(IncField, RollOverValue)                                                          \
    if ((++IncField) >= RollOverValue)                                                             \
    IncField = 0

#define ENDPOINT_TYPE(TransferParam) (TransferParam->Ep.PipeType & 3)
const char *TestDisplayString[] = {"None", "Read", "Write", "Loop", NULL};
const char *EndpointTypeDisplayString[] = {"Control", "Isochronous", "Bulk", "Interrupt", NULL};

LONG WinError(__in_opt DWORD errorCode) {
    LPSTR buffer = NULL;

    errorCode = errorCode ? labs(errorCode) : GetLastError();
    if (!errorCode)
        return errorCode;

    if (FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_ALLOCATE_BUFFER, NULL, errorCode,
                       0, (LPSTR)&buffer, 0, NULL) > 0) {
        SetLastError(0);
    } else {
        LOGERR0("FormatMessage error!\n");
    }

    if (buffer)
        LocalFree(buffer);

    return -labs(errorCode);
}

void SetParamsDefaults(PUVPERF_PARAM TestParams) {
    memset(TestParams, 0, sizeof(*TestParams));

    TestParams->vid = 0x0000;
    TestParams->pid = 0x0000;
    TestParams->intf = -1;
    TestParams->altf = -1;
    TestParams->endpoint = 0x00;
    TestParams->TransferMode = TRANSFER_MODE_SYNC;
    TestParams->TestType = TestTypeIn;
    TestParams->Timer = 0;
    TestParams->timeout = 1000;
    TestParams->fileIO = FALSE;
    TestParams->bufferlength = 1024;
    TestParams->refresh = 1000;
    TestParams->readlenth = TestParams->bufferlength;
    TestParams->writelength = TestParams->bufferlength;
    TestParams->verify = 1;
    TestParams->bufferCount = 1;
    TestParams->ShowTransfer = FALSE;
    TestParams->UseRawIO = 0xFF;
}


void AppendLoopBuffer(PUVPERF_PARAM TestParams, unsigned char *buffer, unsigned int length) {

    if (TestParams->verify && TestParams->TestType == TestTypeLoop) {
        UVPERF_BUFFER *newVerifyBuf = malloc(sizeof(UVPERF_BUFFER) + length);

        memset(newVerifyBuf, 1, sizeof(UVPERF_BUFFER));

        newVerifyBuf->Data = (unsigned char *)newVerifyBuf + sizeof(UVPERF_BUFFER);
        newVerifyBuf->dataLenth = length;
        memcpy(newVerifyBuf->Data, buffer, length);

        VerifyListLock(TestParams);
        DL_APPEND(TestParams->VerifyList, newVerifyBuf);
        VerifyListUnlock(TestParams);
    }
}

BOOL Bench_Open(__in PUVPERF_PARAM TestParams) {
    UCHAR altSetting;
    KUSB_HANDLE associatedHandle;
    UINT transferred;
    KLST_DEVINFO_HANDLE deviceInfo;

    TestParams->SelectedDeviceProfile = NULL;

    LstK_MoveReset(TestParams->DeviceList);

    while (LstK_MoveNext(TestParams->DeviceList, &deviceInfo)) {
        // enabled
        UINT userContext = (UINT)LibK_GetContext(deviceInfo, KLIB_HANDLE_TYPE_LSTINFOK);
        if (userContext != TRUE)
            continue;

        if (!LibK_LoadDriverAPI(&K, deviceInfo->DriverID)) {
            WinError(0);
            LOG_WARNING("can not load driver api %s\n", GetDrvIdString(deviceInfo->DriverID));
            continue;
        }
        if (!TestParams->use_UsbK_Init) {
            TestParams->DeviceHandle =
                CreateFileA(deviceInfo->DevicePath, GENERIC_READ | GENERIC_WRITE,
                            FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, OPEN_EXISTING,
                            FILE_FLAG_OVERLAPPED, NULL);

            if (!TestParams->DeviceHandle || TestParams->DeviceHandle == INVALID_HANDLE_VALUE) {
                WinError(0);
                TestParams->DeviceHandle = NULL;
                LOG_WARNING("can not create device handle\n%s\n", deviceInfo->DevicePath);
                continue;
            }

            if (!K.Initialize(TestParams->DeviceHandle, &TestParams->InterfaceHandle)) {
                WinError(0);
                CloseHandle(TestParams->DeviceHandle);
                TestParams->DeviceHandle = NULL;
                TestParams->InterfaceHandle = NULL;
                LOG_WARNING("can not initialize device\n%s\n", deviceInfo->DevicePath);
                continue;
            }
        } else {
            if (!K.Init(&TestParams->InterfaceHandle, deviceInfo)) {
                WinError(0);
                TestParams->DeviceHandle = NULL;
                TestParams->InterfaceHandle = NULL;
                LOG_WARNING("can not open device\n%s\n", deviceInfo->DevicePath);
                continue;
            }
        }

        if (!K.GetDescriptor(TestParams->InterfaceHandle, USB_DESCRIPTOR_TYPE_DEVICE, 0, 0,
                             (PUCHAR)&TestParams->DeviceDescriptor,
                             sizeof(TestParams->DeviceDescriptor), &transferred)) {
            WinError(0);

            K.Free(TestParams->InterfaceHandle);
            TestParams->InterfaceHandle = NULL;

            if (!TestParams->use_UsbK_Init) {
                CloseHandle(TestParams->DeviceHandle);
                TestParams->DeviceHandle = NULL;
            }

            LOG_WARNING("can not get device descriptor\n%s\n", deviceInfo->DevicePath);
            continue;
        }
        TestParams->vid = (int)TestParams->DeviceDescriptor.idVendor;
        TestParams->pid = (int)TestParams->DeviceDescriptor.idProduct;

    NextInterface:

        // While searching for hardware specifics we are also gathering information and storing it
        // in our test.
        memset(&TestParams->InterfaceDescriptor, 0, sizeof(TestParams->InterfaceDescriptor));
        altSetting = 0;

        while (K.QueryInterfaceSettings(TestParams->InterfaceHandle, altSetting,
                                        &TestParams->InterfaceDescriptor)) {
            // found an interface
            UCHAR pipeIndex = 0;
            int hasIsoEndpoints = 0;
            int hasZeroMaxPacketEndpoints = 0;

            memset(&TestParams->PipeInformation, 0, sizeof(TestParams->PipeInformation));
            while (K.QueryPipeEx(TestParams->InterfaceHandle, altSetting, pipeIndex,
                                 &TestParams->PipeInformation[pipeIndex])) {
                // found a pipe
                if (TestParams->PipeInformation[pipeIndex].PipeType == UsbdPipeTypeIsochronous)
                    hasIsoEndpoints++;

                if (!TestParams->PipeInformation[pipeIndex].MaximumPacketSize)
                    hasZeroMaxPacketEndpoints++;

                pipeIndex++;
            }

            if (pipeIndex > 0) {
                // -1 means the user din't specifiy so we find the most suitable device.
                //
                if (((TestParams->intf == -1) ||
                     (TestParams->intf == TestParams->InterfaceDescriptor.bInterfaceNumber)) &&
                    ((TestParams->altf == -1) ||
                     (TestParams->altf == TestParams->InterfaceDescriptor.bAlternateSetting))) {
                    // if the user actually specifies an alt iso setting with zero
                    // MaxPacketEndpoints we let let them.
                    if (TestParams->altf == -1 && hasIsoEndpoints && hasZeroMaxPacketEndpoints) {
                        // user didn't specfiy and we know we can't tranfer with this alt setting so
                        // skip it.
                        LOG_MSG("skipping interface %02X:%02X. zero-length iso endpoints exist.\n",
                                TestParams->InterfaceDescriptor.bInterfaceNumber,
                                TestParams->InterfaceDescriptor.bAlternateSetting);
                    } else {
                        // this is the one we are looking for.
                        TestParams->intf = TestParams->InterfaceDescriptor.bInterfaceNumber;
                        TestParams->altf = TestParams->InterfaceDescriptor.bAlternateSetting;
                        TestParams->SelectedDeviceProfile = deviceInfo;

                        // some buffering is required for iso.
                        if (hasIsoEndpoints && TestParams->bufferCount == 1)
                            TestParams->bufferCount++;

                        TestParams->DefaultAltSetting = 0;
                        K.GetCurrentAlternateSetting(TestParams->InterfaceHandle,
                                                     &TestParams->DefaultAltSetting);
                        if (!K.SetCurrentAlternateSetting(
                                TestParams->InterfaceHandle,
                                TestParams->InterfaceDescriptor.bAlternateSetting)) {
                            LOG_ERROR("can not find alt interface %02X\n", TestParams->altf);
                            return FALSE;
                        }
                        return TRUE;
                    }
                }
            }

            altSetting++;
            memset(&TestParams->InterfaceDescriptor, 0, sizeof(TestParams->InterfaceDescriptor));
        }
        if (K.GetAssociatedInterface(TestParams->InterfaceHandle, 0, &associatedHandle)) {
            // this device has more interfaces to look at.
            //
            K.Free(TestParams->InterfaceHandle);
            TestParams->InterfaceHandle = associatedHandle;
            goto NextInterface;
        }

        // This one didn't match the test specifics; continue on to the next potential match.
        K.Free(TestParams->InterfaceHandle);
        TestParams->InterfaceHandle = NULL;
    }

    LOG_ERROR("device doesn't have %02X interface and %02X alt interface\n", TestParams->intf,
              TestParams->altf);
    return FALSE;
}

BOOL Bench_Configure(__in KUSB_HANDLE handle, __in UVPERF_DEVICE_COMMAND command, __in UCHAR intf,
                     __inout PUVPERF_DEVICE_TRANSFER_TYPE testType) {
    UCHAR buffer[1];
    UINT transferred = 0;
    WINUSB_SETUP_PACKET Pkt;
    KUSB_SETUP_PACKET *defPkt = (KUSB_SETUP_PACKET *)&Pkt;

    memset(&Pkt, 0, sizeof(Pkt));
    defPkt->BmRequest.Dir = BMREQUEST_DIR_DEVICE_TO_HOST;
    defPkt->BmRequest.Type = BMREQUEST_TYPE_VENDOR;
    defPkt->Request = (UCHAR)command;
    defPkt->Value = (UCHAR)*testType;
    defPkt->Index = intf;
    defPkt->Length = 1;

    if (!handle || handle == INVALID_HANDLE_VALUE) {
        return WinError(ERROR_INVALID_HANDLE);
    }

    if (K.ControlTransfer(handle, Pkt, buffer, 1, &transferred, NULL)) {
        if (transferred)
            return TRUE;
    }

    LOGERR0("can not configure device\n");
    return WinError(0);
}

int VerifyData(PUVPERF_TRANSFER_PARAM transferParam, BYTE *data, INT dataLength) {

    WORD verifyDataSize = transferParam->TestParams->verifyBufferSize;
    BYTE *verifyData = transferParam->TestParams->VerifyBuffer;
    BYTE keyC = 0;
    BOOL seedKey = TRUE;
    INT dataLeft = dataLength;
    INT dataIndex = 0;
    INT packetIndex = 0;
    INT verifyIndex = 0;

    while (dataLeft > 1) {
        verifyDataSize = dataLeft > transferParam->TestParams->verifyBufferSize
                             ? transferParam->TestParams->verifyBufferSize
                             : (WORD)dataLeft;

        if (seedKey)
            keyC = data[dataIndex + 1];
        else {
            if (data[dataIndex + 1] == 0) {
                keyC = 0;
            } else {
                keyC++;
            }
        }
        seedKey = FALSE;
        // Index 0 is always 0.
        // The key is always at index 1
        verifyData[1] = keyC;

        if (memcmp(&data[dataIndex], verifyData, verifyDataSize) != 0) {
            // Packet verification failed.

            // Reset the key byte on the next packet.
            seedKey = TRUE;
            //TODO
            // LOGVDAT("Packet=#%d Data=#%d\n", packetIndex, dataIndex);

            if (transferParam->TestParams->verifyDetails) {
                for (verifyIndex = 0; verifyIndex < verifyDataSize; verifyIndex++) {
                    if (verifyData[verifyIndex] == data[dataIndex + verifyIndex])
                        continue;

                    LOGVDAT("packet-offset=%d expected %02Xh got %02Xh\n", verifyIndex,
                            verifyData[verifyIndex], data[dataIndex + verifyIndex]);
                }
            }
        }

        // Move to the next packet.
        packetIndex++;
        dataLeft -= verifyDataSize;
        dataIndex += verifyDataSize;
    }

    return 0;
}

int TransferSync(PUVPERF_TRANSFER_PARAM transferParam) {
    unsigned int trasnferred;
    BOOL success;

    if (transferParam->Ep.PipeId & USB_ENDPOINT_DIRECTION_MASK) {
        success = K.ReadPipe(transferParam->TestParams->InterfaceHandle,
                             transferParam->Ep.PipeId,
                             transferParam->Buffer,
                             transferParam->TestParams->readlenth,
                             &trasnferred,
                             NULL);
    } else {
        AppendLoopBuffer(transferParam->TestParams,
                         transferParam->Buffer,
                         transferParam->TestParams->writelength);
        success = K.WritePipe(transferParam->TestParams->InterfaceHandle,
                             transferParam->Ep.PipeId,
                              transferParam->Buffer,
                              transferParam->TestParams->writelength,
                              &trasnferred,
                              NULL);
    }

    return success ? (int)trasnferred : -labs(GetLastError());
}

BOOL WINAPI IsoTransferCb(_in unsigned int packetIndex, _ref unsigned int *offset,
                          _ref unsigned int *length, _ref unsigned int *status,
                          _in void *userState) {
    BENCHMARK_ISOCH_RESULTS *isochResults = (BENCHMARK_ISOCH_RESULTS *)userState;

    UNREFERENCED_PARAMETER(packetIndex);
    UNREFERENCED_PARAMETER(offset);

    if (*status)
        isochResults->BadPackets++;
    else {
        if (*length) {
            isochResults->GoodPackets++;
            isochResults->Length += *length;
        }
    }
    isochResults->TotalPackets++;

    return TRUE;
}

int TransferAsync(PUVPERF_TRANSFER_PARAM transferParam, PUVPERF_TRANSFER_HANDLE *handleRef) {
    int ret = 0;
    BOOL success;
    PUVPERF_TRANSFER_HANDLE handle = NULL;
    DWORD transferErrorCode;

    *handleRef = NULL;

    // Submit transfers until the maximum number of outstanding transfer(s) is reached.
    while (transferParam->outstandingTransferCount < transferParam->TestParams->bufferCount) {
        // Get the next available benchmark transfer handle.
        *handleRef = handle =
            &transferParam->TransferHandles[transferParam->transferHandleNextIndex];

        // If a libusb-win32 transfer context hasn't been setup for this benchmark transfer
        // handle, do it now.
        //
        if (!handle->Overlapped.hEvent) {
            handle->Overlapped.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
            // Data buffer(s) are located at the end of the transfer param.
            handle->Data = transferParam->Buffer + (transferParam->transferHandleNextIndex *
                                                    transferParam->TestParams->allocBufferSize);
        } else {
            // re-initialize and re-use the overlapped
            ResetEvent(handle->Overlapped.hEvent);
        }

        if (transferParam->Ep.PipeId & USB_ENDPOINT_DIRECTION_MASK) {
            handle->DataMaxLength = transferParam->TestParams->readlenth;
            if (transferParam->Ep.PipeType == UsbdPipeTypeIsochronous) {
                success = K.IsochReadPipe(handle->IsochHandle, handle->DataMaxLength,
                                          &transferParam->frameNumber, 0, &handle->Overlapped);
            } else {
                success =
                    K.ReadPipe(transferParam->TestParams->InterfaceHandle, transferParam->Ep.PipeId,
                               handle->Data, handle->DataMaxLength, NULL, &handle->Overlapped);
            }
        }

        // Isochronous write pipe -> doesn't need right now
        else {
            AppendLoopBuffer(transferParam->TestParams, handle->Data,
                             transferParam->TestParams->writelength);
            handle->DataMaxLength = transferParam->TestParams->writelength;
            if (transferParam->Ep.PipeType == UsbdPipeTypeIsochronous) {
                success = K.IsochWritePipe(handle->IsochHandle, handle->DataMaxLength,
                                           &transferParam->frameNumber, 0, &handle->Overlapped);
            } else {
                success =
                    K.WritePipe(transferParam->TestParams->InterfaceHandle, transferParam->Ep.PipeId,
                                handle->Data, handle->DataMaxLength, NULL, &handle->Overlapped);
            }
        }

        transferErrorCode = GetLastError();

        if (!success && transferErrorCode == ERROR_IO_PENDING) {
            transferErrorCode = ERROR_SUCCESS;
            success = TRUE;
        }

        // Submit this transfer now.
        handle->ReturnCode = ret = -labs(transferErrorCode);
        if (ret < 0) {
            handle->InUse = FALSE;
            goto Final;
        }

        // Mark this handle has InUse.
        handle->InUse = TRUE;

        // When transfers ir successfully submitted, OutstandingTransferCount goes up; when
        // they are completed it goes down.
        //
        transferParam->outstandingTransferCount++;

        // Move TransferHandleNextIndex to the next available transfer.
        INC_ROLL(transferParam->transferHandleNextIndex, transferParam->TestParams->bufferCount);
    }

    // If the number of outstanding transfers has reached the limit, wait for the
    // oldest outstanding transfer to complete.
    //
    if (transferParam->outstandingTransferCount == transferParam->TestParams->bufferCount) {
        UINT transferred;
        // TransferHandleWaitIndex is the index of the oldest outstanding transfer.
        *handleRef = handle =
            &transferParam->TransferHandles[transferParam->transferHandleWaitIndex];

        // Only wait, cancelling & freeing is handled by the caller.
        if (WaitForSingleObject(handle->Overlapped.hEvent, transferParam->TestParams->timeout) !=
            WAIT_OBJECT_0) {
            if (!transferParam->TestParams->isUserAborted) {
                ret = WinError(0);
            } else
                ret = -labs(GetLastError());

            handle->ReturnCode = ret;
            goto Final;
        }

        if (!K.GetOverlappedResult(transferParam->TestParams->InterfaceHandle, &handle->Overlapped,
                                   &transferred, FALSE)) {
            if (!transferParam->TestParams->isUserAborted) {
                ret = WinError(0);
            } else
                ret = -labs(GetLastError());

            handle->ReturnCode = ret;
            goto Final;
        }

        if (transferParam->Ep.PipeType == UsbdPipeTypeIsochronous &&
            transferParam->Ep.PipeId & 0x80) {
            // iso read pipe
            memset(&handle->IsochResults, 0, sizeof(handle->IsochResults));
            IsochK_EnumPackets(handle->IsochHandle, &IsoTransferCb, 0, &handle->IsochResults);
            transferParam->IsochResults.TotalPackets += handle->IsochResults.TotalPackets;
            transferParam->IsochResults.GoodPackets += handle->IsochResults.GoodPackets;
            transferParam->IsochResults.BadPackets += handle->IsochResults.BadPackets;
            transferParam->IsochResults.Length += handle->IsochResults.Length;
            transferred = handle->IsochResults.Length;
        }

        // Isochronous write pipe -> doesn't need right now
        else if (transferParam->Ep.PipeType == UsbdPipeTypeIsochronous) {
            // iso write pipe
            transferred = handle->DataMaxLength;

            transferParam->IsochResults.TotalPackets += transferParam->numberOFIsoPackets;
            transferParam->IsochResults.GoodPackets += transferParam->numberOFIsoPackets;
        }

        handle->ReturnCode = ret = (DWORD)transferred;

        if (ret < 0)
            goto Final;

        // Mark this handle has no longer InUse.
        handle->InUse = FALSE;

        // When transfers ir successfully submitted, OutstandingTransferCount goes up; when
        // they are completed it goes down.
        //
        transferParam->outstandingTransferCount--;

        // Move TransferHandleWaitIndex to the oldest outstanding transfer.
        INC_ROLL(transferParam->transferHandleWaitIndex, transferParam->TestParams->bufferCount);
    }

Final:
    return ret;
}

// TODO : later for Loop data
void VerifyLoopData() { return; }



void ShowParams(PUVPERF_PARAM TestParams) {
    if (!TestParams)
        return;

    LOG_MSG("\tDriver         :  %s\n", GetDrvIdString(TestParams->SelectedDeviceProfile->DriverID));
    LOG_MSG("\tVID:           :  0x%04X\n", TestParams->vid);
    LOG_MSG("\tPID:           :  0x%04X\n", TestParams->pid);
    LOG_MSG("\tInterface:     :  %d\n", TestParams->intf);
    LOG_MSG("\tAlt Interface: :  %d\n", TestParams->altf);
    LOG_MSG("\tEndpoint:      :  0x%02X\n", TestParams->endpoint);
    LOG_MSG("\tTransfer mode  :  %s\n", TestParams->TransferMode ? "Isochronous" : "Bulk");
    LOG_MSG("\tTimeout:       :  %d\n", TestParams->timeout);
    LOG_MSG("\tRead Length:   :  %d\n", TestParams->readlenth);
    LOG_MSG("\tWrite Length:  :  %d\n", TestParams->writelength);
    LOG_MSG("\tRepeat:        :  %d\n", TestParams->repeat);
    LOG_MSG("\n");
}



DWORD TransferThread(PUVPERF_TRANSFER_PARAM transferParam) {
    int ret, i;
    PUVPERF_TRANSFER_HANDLE handle;
    unsigned char *buffer;

    transferParam->isRunning = TRUE;

    while (!transferParam->TestParams->isCancelled) {
        buffer = NULL;
        handle = NULL;

        if (transferParam->TestParams->TransferMode == TRANSFER_MODE_SYNC) {
            ret = TransferSync(transferParam);
            if (ret >= 0)
                buffer = transferParam->Buffer;
        } else if (transferParam->TestParams->TransferMode == TRANSFER_MODE_ASYNC) {
            ret = TransferAsync(transferParam, &handle);
            if ((handle) && ret >= 0)
                buffer = transferParam->Buffer;
        } else {
            LOG_ERROR("Invalid transfer mode %d\n", transferParam->TestParams->TransferMode);
            goto Final;
        }

        if (transferParam->TestParams->verify && transferParam->TestParams->VerifyList &&
            transferParam->TestParams->TestType == TestTypeLoop &&
            USB_ENDPOINT_DIRECTION_IN(transferParam->Ep.PipeId) && ret > 0) {
            VerifyLoopData(transferParam->TestParams, buffer);
        }

        if (ret < 0) {
            // user pressed 'Q' or 'ctrl+c'
            if (transferParam->TestParams->isUserAborted)
                break;

            // timeout
            if (ret == ERROR_SEM_TIMEOUT || ret == ERROR_OPERATION_ABORTED ||
                ret == ERROR_CANCELLED) {
                transferParam->TotalTimeoutCount++;
                transferParam->RunningTimeoutCount++;
                LOG_ERROR("Timeout #%d %s on EP%02Xh.. \n", transferParam->RunningTimeoutCount,
                          TRANSFER_DISPLAY(transferParam, "reading", "writing"),
                          transferParam->Ep.PipeId);

                if (transferParam->RunningTimeoutCount > transferParam->TestParams->retry)
                    break;
            }

            // other error
            else {
                transferParam->TotalErrorCount++;
                transferParam->RunningErrorCount++;
                LOG_ERROR("failed %s, %d of %d ret=%d, error message : %s\n",
                          TRANSFER_DISPLAY(transferParam, "reading", "writing"),
                          transferParam->RunningErrorCount, transferParam->TestParams->retry + 1,
                          ret, strerror(ret));
                K.ResetPipe(transferParam->TestParams->InterfaceHandle, transferParam->Ep.PipeId);

                if (transferParam->RunningErrorCount > transferParam->TestParams->retry)
                    break;
            }

            ret = 0;
        } else {
            transferParam->RunningTimeoutCount = 0;
            transferParam->RunningErrorCount = 0;
            // log the data to the file
            if (USB_ENDPOINT_DIRECTION_IN(transferParam->Ep.PipeId)) {
                // LOG_MSG("Read %d bytes\n", ret);
                if (transferParam->TestParams->verify &&
                    transferParam->TestParams->TestType != TestTypeLoop) {
                    VerifyData(transferParam, buffer, ret);
                }
            } else {
                // LOG_MSG("Wrote %d bytes\n", ret);
                if (transferParam->TestParams->verify &&
                    transferParam->TestParams->TestType != TestTypeLoop) {
                    // VerifyData(transferParam, buffer, ret);
                }
            }
        }

        EnterCriticalSection(&DisplayCriticalSection);

        if (!transferParam->StartTick.tv_nsec && transferParam->Packets >= 0) {
            clock_gettime(CLOCK_MONOTONIC, &transferParam->StartTick);
            transferParam->LastStartTick = transferParam->StartTick;
            transferParam->LastTick = transferParam->StartTick;

            transferParam->LastTransferred = 0;
            transferParam->TotalTransferred = 0;
            transferParam->Packets = 0;
        } else {
            if (!transferParam->LastStartTick.tv_nsec) {
                transferParam->LastStartTick = transferParam->LastTick;
                transferParam->LastTransferred = 0;
            }
            clock_gettime(CLOCK_MONOTONIC, &transferParam->LastTick);

            transferParam->LastTransferred += ret;
            transferParam->TotalTransferred += ret;
            transferParam->Packets++;
        }

        LeaveCriticalSection(&DisplayCriticalSection);
    }

Final:

    for (i = 0; i < transferParam->TestParams->bufferCount; i++) {
        if (transferParam->TransferHandles[i].Overlapped.hEvent) {
            if (transferParam->TransferHandles[i].InUse) {
                if (!K.AbortPipe(transferParam->TestParams->InterfaceHandle,
                                 transferParam->Ep.PipeId) &&
                    !transferParam->TestParams->isUserAborted) {
                    ret = WinError(0);
                    LOG_ERROR("failed cancelling transfer ret = %d, error message : %s\n", ret,
                              strerror(ret));
                } else {
                    CloseHandle(transferParam->TransferHandles[i].Overlapped.hEvent);
                    transferParam->TransferHandles[i].Overlapped.hEvent = NULL;
                    transferParam->TransferHandles[i].InUse = FALSE;
                }
            }
            Sleep(0);
        }
    }

    for (i = 0; i < transferParam->TestParams->bufferCount; i++) {
        if (transferParam->TransferHandles[i].Overlapped.hEvent) {
            if (transferParam->TransferHandles[i].InUse) {
                WaitForSingleObject(transferParam->TransferHandles[i].Overlapped.hEvent,
                                    transferParam->TestParams->timeout);
            } else {
                WaitForSingleObject(transferParam->TransferHandles[i].Overlapped.hEvent, 0);
            }
            CloseHandle(transferParam->TransferHandles[i].Overlapped.hEvent);
            transferParam->TransferHandles[i].Overlapped.hEvent = NULL;
        }
        transferParam->TransferHandles[i].InUse = FALSE;
    }

    transferParam->isRunning = FALSE;
    return 0;
}

int ParseArgs(PUVPERF_PARAM TestParams, int argc, char **argv) {
    int i;
    int arg;
    char *temp;
    int value;
    int status = 0;

    int c;
    while ((c = getopt(argc, argv, "Vv:p:i:a:e:m:t:fb:l:w:r:SRWL")) != -1) {
        switch (c) {
        case 'V':
            verbose = TRUE;
            break;
        case 'v':
            TestParams->vid = strtol(optarg, NULL, 0);
            break;
        case 'p':
            TestParams->pid = strtol(optarg, NULL, 0);
            break;
        case 'i':
            TestParams->intf = strtol(optarg, NULL, 0);
            break;
        case 'a':
            TestParams->altf = strtol(optarg, NULL, 0);
            break;
        case 'e':
            TestParams->endpoint = strtol(optarg, NULL, 0);
            break;
        case 'm':
            TestParams->TransferMode =
                (strtol(optarg, NULL, 0) ? TRANSFER_MODE_ASYNC : TRANSFER_MODE_SYNC);
            break;
        case 'T':
            TestParams->Timer = strtol(optarg, NULL, 0);
            break;
        case 't':
            TestParams->timeout = strtol(optarg, NULL, 0);
            break;
        case 'f':
            TestParams->fileIO = TRUE;
            break;
        case 'b':
            TestParams->bufferCount = strtol(optarg, NULL, 0);
            if (TestParams->bufferCount > 1) {
                TestParams->TransferMode = TRANSFER_MODE_ASYNC;
            }
            break;
        case 'l':
            TestParams->readlenth = strtol(optarg, NULL, 0);
            break;
        case 'w':
            TestParams->writelength = strtol(optarg, NULL, 0);
            break;
        case 'r':
            TestParams->repeat = strtol(optarg, NULL, 0);
            break;
        case 'S':
            TestParams->ShowTransfer = TRUE;
            break;
        case 'R':
            TestParams->TestType = TestTypeIn;
            break;
        case 'W':
            TestParams->TestType = TestTypeOut;
            break;
        case 'L':
            TestParams->TestType = TestTypeLoop;
            break;
        default:
            LOGERR0("Invalid argument\n");
            status = -1;
            break;
        }
    }

    if (optind < argc) {
        printf("Non-option arguments: ");
        while (optind < argc)
            printf("%s ", argv[optind++]);
        printf("\n");
    }

    return status;
}

int CreateVerifyBuffer(PUVPERF_PARAM TestParam, WORD endpointMaxPacketSize) {
    int i;
    BYTE indexC = 0;
    TestParam->VerifyBuffer = malloc(endpointMaxPacketSize);
    if (!TestParam->VerifyBuffer) {
        LOG_ERROR("memory allocation failure at line %d!\n", __LINE__);
        return -1;
    }

    TestParam->verifyBufferSize = endpointMaxPacketSize;

    for (i = 0; i < endpointMaxPacketSize; i++) {
        TestParam->VerifyBuffer[i] = indexC++;
        if (indexC == 0)
            indexC = 1;
    }

    return 0;
}

void FreeTransferParam(PUVPERF_TRANSFER_PARAM *transferParamRef) {
    PUVPERF_TRANSFER_PARAM pTransferParam;
    int i;
    if ((!transferParamRef) || !*transferParamRef)
        return;
    pTransferParam = *transferParamRef;

    if (pTransferParam->TestParams) {
        for (i = 0; i < pTransferParam->TestParams->bufferCount; i++) {
            if (pTransferParam->TransferHandles[i].IsochHandle) {
                IsochK_Free(pTransferParam->TransferHandles[i].IsochHandle);
            }
        }
    }
    if (pTransferParam->ThreadHandle) {
        CloseHandle(pTransferParam->ThreadHandle);
        pTransferParam->ThreadHandle = NULL;
    }

    free(pTransferParam);

    *transferParamRef = NULL;
}

PUVPERF_TRANSFER_PARAM CreateTransferParam(PUVPERF_PARAM TestParam, int endpointID) {
    PUVPERF_TRANSFER_PARAM transferParam = NULL;
    int pipeIndex, bufferIndex;
    int allocSize;

    PWINUSB_PIPE_INFORMATION_EX pipeInfo = NULL;

    /// Get Pipe Information
    for (pipeIndex = 0; pipeIndex < TestParam->InterfaceDescriptor.bNumEndpoints; pipeIndex++) {
        if (!(endpointID & USB_ENDPOINT_ADDRESS_MASK)) {
            // Use first endpoint that matches the direction
            if ((TestParam->PipeInformation[pipeIndex].PipeId & USB_ENDPOINT_DIRECTION_MASK) ==
                endpointID) {
                pipeInfo = &TestParam->PipeInformation[pipeIndex];
                break;
            }
        } else {
            if ((int)TestParam->PipeInformation[pipeIndex].PipeId == endpointID) {
                pipeInfo = &TestParam->PipeInformation[pipeIndex];
                break;
            }
        }
    }

    if (!pipeInfo) {
        LOG_ERROR("failed locating EP0x%02X\n", endpointID);
        goto Final;
    }

    if (!pipeInfo->MaximumPacketSize) {
        LOG_WARNING("MaximumPacketSize=0 for EP%02Xh. check alternate settings.\n",
                    pipeInfo->PipeId);
    }

    TestParam->bufferlength = max(TestParam->bufferlength, TestParam->readlenth);
    TestParam->bufferlength = max(TestParam->bufferlength, TestParam->writelength);

    allocSize = sizeof(UVPERF_TRANSFER_PARAM) + (TestParam->bufferlength * TestParam->bufferCount);
    transferParam = (PUVPERF_TRANSFER_PARAM)malloc(allocSize);

    if (transferParam) {
        UINT numIsoPackets;
        memset(transferParam, 0, allocSize);
        transferParam->TestParams = TestParam;

        memcpy(&transferParam->Ep, pipeInfo, sizeof(transferParam->Ep));
        transferParam->HasEpCompanionDescriptor = K.GetSuperSpeedPipeCompanionDescriptor(
            TestParam->InterfaceHandle, TestParam->InterfaceDescriptor.bAlternateSetting,
            (UCHAR)pipeIndex, &transferParam->EpCompanionDescriptor);

        if (ENDPOINT_TYPE(transferParam) == USB_ENDPOINT_TYPE_ISOCHRONOUS) {
            transferParam->TestParams->TransferMode = TRANSFER_MODE_ASYNC;

            if (!transferParam->Ep.MaximumBytesPerInterval) {
                LOG_ERROR(
                    "Unable to determine 'MaximumBytesPerInterval' for isochronous pipe %02X\n",
                    transferParam->Ep.PipeId);
                LOGERR0("- Device firmware may be incorrectly configured.");
                FreeTransferParam(&transferParam);
                goto Final;
            }
            numIsoPackets =
                transferParam->TestParams->bufferlength / transferParam->Ep.MaximumBytesPerInterval;
            transferParam->numberOFIsoPackets = numIsoPackets;
            if (numIsoPackets == 0 || ((numIsoPackets % 8)) ||
                transferParam->TestParams->bufferlength %
                    transferParam->Ep.MaximumBytesPerInterval) {
                const UINT minBufferSize = transferParam->Ep.MaximumBytesPerInterval * 8;
                LOG_ERROR("Buffer size is not correct for isochronous pipe 0x%02X\n",
                          transferParam->Ep.PipeId);
                LOG_ERROR("- Buffer size must be an interval of %u\n", minBufferSize);
                FreeTransferParam(&transferParam);
                goto Final;
            }

            for (bufferIndex = 0; bufferIndex < transferParam->TestParams->bufferCount;
                 bufferIndex++) {
                transferParam->TransferHandles[bufferIndex].Overlapped.hEvent =
                    CreateEvent(NULL, TRUE, FALSE, NULL);

                // Data buffer(s) are located at the end of the transfer param.
                transferParam->TransferHandles[bufferIndex].Data =
                    transferParam->Buffer + (bufferIndex * transferParam->TestParams->bufferlength);

                if (!IsochK_Init(&transferParam->TransferHandles[bufferIndex].IsochHandle,
                                 TestParam->InterfaceHandle, transferParam->Ep.PipeId,
                                 numIsoPackets, transferParam->TransferHandles[bufferIndex].Data,
                                 transferParam->TestParams->bufferlength)) {
                    DWORD ec = GetLastError();

                    LOG_ERROR("IsochK_Init failed for isochronous pipe %02X\n",
                              transferParam->Ep.PipeId);
                    LOG_ERROR("- ErrorCode = %u (%s)\n", ec, strerror(ec));
                    FreeTransferParam(&transferParam);
                    goto Final;
                }

                if (!IsochK_SetPacketOffsets(
                        transferParam->TransferHandles[bufferIndex].IsochHandle,
                        transferParam->Ep.MaximumBytesPerInterval)) {
                    DWORD ec = GetLastError();

                    LOG_ERROR("IsochK_SetPacketOffsets failed for isochronous pipe %02X\n",
                              transferParam->Ep.PipeId);
                    LOG_ERROR("- ErrorCode = %u (%s)\n", ec, strerror(ec));
                    FreeTransferParam(&transferParam);
                    goto Final;
                }
            }
        }

        transferParam->ThreadHandle =
            CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)TransferThread, transferParam,
                         CREATE_SUSPENDED, &transferParam->ThreadId);

        if (!transferParam->ThreadHandle) {
            LOGERR0("failed creating thread!\n");
            FreeTransferParam(&transferParam);
            goto Final;
        }

        // If verify mode is on, this is a loop test, and this is a write endpoint, fill
        // the buffers with the same test data sent by a benchmark device when running
        // a read only test.
        if (transferParam->TestParams->TestType == TestTypeLoop &&
            USB_ENDPOINT_DIRECTION_OUT(pipeInfo->PipeId)) {
            // Data Format:
            // [0][KeyByte] 2 3 4 5 ..to.. wMaxPacketSize (if data byte rolls it is incremented to
            // 1) Increment KeyByte and repeat
            //
            BYTE indexC = 0;
            INT bufferIndex = 0;
            WORD dataIndex;
            INT packetIndex;
            INT packetCount = ((transferParam->TestParams->bufferCount * TestParam->readlenth) /
                               pipeInfo->MaximumPacketSize);
            for (packetIndex = 0; packetIndex < packetCount; packetIndex++) {
                indexC = 2;
                for (dataIndex = 0; dataIndex < pipeInfo->MaximumPacketSize; dataIndex++) {
                    if (dataIndex == 0) // Start
                        transferParam->Buffer[bufferIndex] = 0;
                    else if (dataIndex == 1) // Key
                        transferParam->Buffer[bufferIndex] = packetIndex & 0xFF;
                    else // Data
                        transferParam->Buffer[bufferIndex] = indexC++;

                    // if wMaxPacketSize is > 255, indexC resets to 1.
                    if (indexC == 0)
                        indexC = 1;

                    bufferIndex++;
                }
            }
        }
    }

Final:
    if (!transferParam)
        LOGERR0("failed creating transfer param!\n");

    return transferParam;
}

void GetAverageBytesSec(PUVPERF_TRANSFER_PARAM transferParam, DOUBLE *byteps) {
    DWORD elapsedSeconds = 0.0;
    if (!transferParam)
        return;

    if (transferParam->StartTick.tv_nsec &&
        (transferParam->StartTick.tv_sec + transferParam->StartTick.tv_nsec / 1000000000.0) <
            (transferParam->LastTick.tv_sec + transferParam->LastTick.tv_nsec / 1000000000.0)) {

        elapsedSeconds =
            (transferParam->LastTick.tv_sec - transferParam->StartTick.tv_sec) +
            (transferParam->LastTick.tv_nsec - transferParam->StartTick.tv_nsec) / 1000000000.0;

        *byteps = (DOUBLE)transferParam->TotalTransferred / elapsedSeconds;
        if(transferParam->TotalTransferred == 0)
            *byteps = 0;
    } else {
        *byteps = 0;
    }
}
void GetCurrentBytesSec(PUVPERF_TRANSFER_PARAM transferParam, DOUBLE *byteps) {
    DWORD elapsedSeconds;
    if (!transferParam)
        return;

    if (transferParam->LastStartTick.tv_nsec &&
        (transferParam->LastStartTick.tv_sec +
         transferParam->LastStartTick.tv_nsec / 1000000000.0) <
            (transferParam->LastTick.tv_sec + transferParam->LastTick.tv_nsec / 1000000000.0)) {

        elapsedSeconds =
            (transferParam->LastTick.tv_sec - transferParam->LastStartTick.tv_sec) +
            (transferParam->LastTick.tv_nsec - transferParam->LastStartTick.tv_nsec) / 1000000000.0;

        *byteps = (DOUBLE)transferParam->LastTransferred / elapsedSeconds;
    } else {
        *byteps = 0;
    }
}

void ShowRunningStatus(PUVPERF_TRANSFER_PARAM readParam, PUVPERF_TRANSFER_PARAM writeParam) {
    static UVPERF_TRANSFER_PARAM gReadParamTransferParam, gWriteParamTransferParam;
    DOUBLE bpsReadOverall = 0;
    DOUBLE bpsReadLastTransfer = 0;
    DOUBLE bpsWriteOverall = 0;
    DOUBLE bpsWriteLastTransfer = 0;
    UINT zlp = 0;
    UINT totalPackets = 0;
    UINT totalIsoPackets = 0;
    UINT goodIsoPackets = 0;
    UINT badIsoPackets = 0;
    UINT errorCount = 0;

    // LOCK the display critical section
    EnterCriticalSection(&DisplayCriticalSection);

    if (readParam)
        memcpy(&gReadParamTransferParam, readParam, sizeof(UVPERF_TRANSFER_PARAM));

    if (writeParam)
        memcpy(&gWriteParamTransferParam, writeParam, sizeof(UVPERF_TRANSFER_PARAM));

    // UNLOCK the display critical section
    LeaveCriticalSection(&DisplayCriticalSection);

    if (readParam != NULL && (!gReadParamTransferParam.StartTick.tv_nsec ||
                              (gReadParamTransferParam.StartTick.tv_sec +
                               gReadParamTransferParam.StartTick.tv_nsec / 1000000000.0) >
                                  (gReadParamTransferParam.LastTick.tv_sec +
                                   gReadParamTransferParam.LastTick.tv_nsec / 1000000000.0))) {
        LOG_MSG("Synchronizing Read %d..\n", abs(gReadParamTransferParam.Packets));
        errorCount++;
        if (errorCount > 5) {
            LOGERR0("Too many errors, exiting..\n");
            return;
        }
    }

    if (writeParam != NULL && (!gWriteParamTransferParam.StartTick.tv_nsec ||
                               (gWriteParamTransferParam.StartTick.tv_sec +
                                gWriteParamTransferParam.StartTick.tv_nsec / 1000000000.0) >
                                   (gWriteParamTransferParam.LastTick.tv_sec +
                                    gWriteParamTransferParam.LastTick.tv_nsec / 1000000000.0))) {
        LOG_MSG("Synchronizing Write %d..\n", abs(gWriteParamTransferParam.Packets));
        errorCount++;
        if (errorCount > 5) {
            LOGERR0("Too many errors, exiting..\n");
            return;
        }

    } else {
        if (readParam) {
            GetAverageBytesSec(&gReadParamTransferParam, &bpsReadOverall);
            GetCurrentBytesSec(&gReadParamTransferParam, &bpsReadLastTransfer);
            if (gReadParamTransferParam.LastTransferred == 0)
                zlp++;
            readParam->LastStartTick.tv_nsec = 0.0;
            totalPackets += gReadParamTransferParam.Packets;
            totalIsoPackets += gReadParamTransferParam.IsochResults.TotalPackets;
            goodIsoPackets += gReadParamTransferParam.IsochResults.GoodPackets;
            badIsoPackets += gReadParamTransferParam.IsochResults.BadPackets;
        }

        if (writeParam) {
            GetAverageBytesSec(&gWriteParamTransferParam, &bpsWriteOverall);
            GetCurrentBytesSec(&gWriteParamTransferParam, &bpsWriteLastTransfer);

            if (gWriteParamTransferParam.LastTransferred == 0) {
                zlp++;
            }

            writeParam->LastStartTick.tv_nsec = 0.0;
            totalPackets += gWriteParamTransferParam.Packets;
            totalIsoPackets += gWriteParamTransferParam.IsochResults.TotalPackets;
            goodIsoPackets += gWriteParamTransferParam.IsochResults.GoodPackets;
            badIsoPackets += gWriteParamTransferParam.IsochResults.BadPackets;
        }
        if (totalIsoPackets) {
            LOG_MSG("Average %.2f Mbps\n", ((bpsReadOverall + bpsWriteOverall) * 8) / 1000 / 1000);
            LOG_MSG("Total %d Transfer\n", totalPackets);
            LOG_MSG("ISO-Packets (Total/Good/Bad) : %u/%u/%u\n", totalIsoPackets, goodIsoPackets,
                    badIsoPackets);
        } else {
            if (zlp) {
                LOG_MSG("Average %.2f Mbps\n",
                        (bpsReadOverall + bpsWriteOverall) * 8 / 1000 / 1000);
                LOG_MSG("Transfers: %u\n", totalPackets);
                LOG_MSG("Zero-length-transfer(s)\n", zlp);
            } else {
                LOG_MSG("Average %.2f Mbps\n",
                        (bpsReadOverall + bpsWriteOverall) * 8 / 1000 / 1000);
                LOG_MSG("Total %d Transfers\n", totalPackets);
                LOG_MSG("\n");
            }
        }
    }
}

void ShowTransfer(PUVPERF_TRANSFER_PARAM transferParam) {
    DOUBLE BytepsAverage;
    DOUBLE BytepsCurrent;
    DOUBLE elapsedSeconds;

    if (!transferParam)
        return;

    if (transferParam->HasEpCompanionDescriptor) {
        if (transferParam->EpCompanionDescriptor.wBytesPerInterval) {
            if (transferParam->Ep.PipeType == UsbdPipeTypeIsochronous) {
                LOG_MSG(
                    "%s %s from Ep0x%02X Maximum Bytes Per Interval:%lu Max Bursts:%u Multi:%u\n",
                    EndpointTypeDisplayString[ENDPOINT_TYPE(transferParam)],
                    TRANSFER_DISPLAY(transferParam, "Read", "Write"), transferParam->Ep.PipeId,
                    transferParam->Ep.MaximumBytesPerInterval,
                    transferParam->EpCompanionDescriptor.bMaxBurst + 1,
                    transferParam->EpCompanionDescriptor.bmAttributes.Isochronous.Mult + 1);
            } else if (transferParam->Ep.PipeType == UsbdPipeTypeBulk) {
                LOG_MSG("%s %s from Ep0x%02X Maximum Bytes Per Interval:%lu Max Bursts:%u Max "
                        "Streams:%u\n",
                        EndpointTypeDisplayString[ENDPOINT_TYPE(transferParam)],
                        TRANSFER_DISPLAY(transferParam, "Read", "Write"), transferParam->Ep.PipeId,
                        transferParam->Ep.MaximumBytesPerInterval,
                        transferParam->EpCompanionDescriptor.bMaxBurst + 1,
                        transferParam->EpCompanionDescriptor.bmAttributes.Bulk.MaxStreams + 1);
            } else {
                LOG_MSG("%s %s from Ep0x%02X Maximum Bytes Per Interval:%lu\n",
                        EndpointTypeDisplayString[ENDPOINT_TYPE(transferParam)],
                        TRANSFER_DISPLAY(transferParam, "Read", "Write"), transferParam->Ep.PipeId,
                        transferParam->Ep.MaximumBytesPerInterval);
            }
        } else {
            LOG_MSG("%s %s Ep0x%02X Maximum Packet Size:%d\n",
                    EndpointTypeDisplayString[ENDPOINT_TYPE(transferParam)],
                    TRANSFER_DISPLAY(transferParam, "Read", "Write"), transferParam->Ep.PipeId,
                    transferParam->Ep.MaximumPacketSize);
        }
    } else {
        LOG_MSG("%s %s Ep0x%02X Maximum Packet Size: %d\n",
                EndpointTypeDisplayString[ENDPOINT_TYPE(transferParam)],
                TRANSFER_DISPLAY(transferParam, "Read", "Write"), transferParam->Ep.PipeId,
                transferParam->Ep.MaximumPacketSize);
    }

    if (transferParam->StartTick.tv_nsec) {
        GetAverageBytesSec(transferParam, &BytepsAverage);
        GetCurrentBytesSec(transferParam, &BytepsCurrent);
        LOG_MSG("\tTotal %I64d Bytes\n", transferParam->TotalTransferred);
        LOG_MSG("\tTotal %d Transfers\n", transferParam->Packets);

        if (transferParam->shortTrasnferred) {
            LOG_MSG("\tShort %d Transfers\n", transferParam->shortTrasnferred);
        }

        if (transferParam->TotalTimeoutCount) {
            LOG_MSG("\tTimeout %d Errors\n", transferParam->TotalTimeoutCount);
        }

        if (transferParam->TotalErrorCount) {
            LOG_MSG("\tOther %d Errors\n", transferParam->TotalErrorCount);
        }

        LOG_MSG("\tAverage %.2f Mbps/sec\n", (BytepsAverage * 8) / 1000 / 1000);

        if (transferParam->StartTick.tv_nsec &&
            (transferParam->LastStartTick.tv_sec +
             transferParam->LastStartTick.tv_nsec / 1000000000.0) <
                (transferParam->LastTick.tv_sec + transferParam->LastTick.tv_nsec / 1000000000.0)) {
            elapsedSeconds =
                (transferParam->LastTick.tv_sec - transferParam->StartTick.tv_sec) +
                (transferParam->LastTick.tv_nsec - transferParam->StartTick.tv_nsec) / 1000000000.0;
            LOG_MSG("\tElapsed Time %.2f seconds\n", elapsedSeconds);
        }

        LOG_MSG("\n");
    }
}

void ShowUsage() {
    LOG_MSG("Version : V1.0.5\n");
    LOG_MSG("\n");
    LOG_MSG(
        "Usage: uvperf -v VID -p PID -i INTERFACE -a AltInterface -e ENDPOINT -m TRANSFERMODE "
        "-T TIMER -t TIMEOUT -f FileIO -b BUFFERCOUNT-l READLENGTH -w WRITELENGTH -r REPEAT -S \n");
    LOG_MSG("\t-v VID           USB Vendor ID\n");
    LOG_MSG("\t-p PID           USB Product ID\n");
    LOG_MSG("\t-i INTERFACE     USB Interface\n");
    LOG_MSG("\t-a AltInterface  USB Alternate Interface\n");
    LOG_MSG("\t-e ENDPOINT      USB Endpoint\n");
    LOG_MSG("\t-m TRANSFER      0 = isochronous, 1 = bulk\n");
    LOG_MSG("\t-T TIMER         Timer in seconds\n");
    LOG_MSG("\t-t TIMEOUT       USB Transfer Timeout\n");
    LOG_MSG("\t-f FileIO        Use file I/O, default : FALSE\n");
    LOG_MSG("\t-b BUFFERCOUNT   Number of buffers to use\n");
    LOG_MSG("\t-l READLENGTH    Length of read transfers\n");
    LOG_MSG("\t-w WRITELENGTH   Length of write transfers\n");
    LOG_MSG("\t-r REPEAT        Number of transfers to perform\n");
    LOG_MSG("\t-S               Show transfer data, default : FALSE\n");
    LOG_MSG("\n");
    LOG_MSG("Example:\n");
    LOG_MSG("uvperf -v 0x1004 -p 0xa000 -i 0 -a 0 -e 0x81 -m 0 -t 1000 -l 1024 -r 1000 -R\n");
    LOG_MSG("This will perform 1000 bulk transfers of 1024 bytes to endpoint 0x81\n");
    LOG_MSG("on interface 0, alternate setting 0 of a device with VID 0x1004 and PID 0xA000.\n");
    LOG_MSG("The transfers will have a timeout of 1000ms.\n");

    LOG_MSG("\n");
}


BOOL WaitForTestTransfer(PUVPERF_TRANSFER_PARAM transferParam, UINT msToWait) {
    DWORD exitCode;

    while (transferParam) {
        if (!transferParam->isRunning) {
            if (GetExitCodeThread(transferParam->ThreadHandle, &exitCode)) {
                LOG_MSG("stopped Ep0x%02X thread \tExitCode=%d\n", transferParam->Ep.PipeId,
                        exitCode);
                break;
            }

            LOG_ERROR("failed getting Ep0x%02X thread exit code!\n", transferParam->Ep.PipeId);
            break;
        }

        LOG_MSG("waiting for Ep%02Xh thread..\n", transferParam->Ep.PipeId);
        WaitForSingleObject(transferParam->ThreadHandle, 100);
        if (msToWait != INFINITE) {
            if ((msToWait - 100) == 0 || (msToWait - 100) > msToWait)
                return FALSE;
        }
    }

    return TRUE;
}

int GetDeviceParam(PUVPERF_PARAM TestParams) {
    char id[MAX_PATH];
    KLST_DEVINFO_HANDLE deviceInfo = NULL;

    LstK_MoveReset(TestParams->DeviceList);

    while (LstK_MoveNext(TestParams->DeviceList, &deviceInfo)) {
        int vid = -1;
        int pid = -1;
        int mi = -1;
        PCHAR chID;

        // disabled
        LibK_SetContext(deviceInfo, KLIB_HANDLE_TYPE_LSTINFOK, (KLIB_USER_CONTEXT)FALSE);

        memset(id, 0, sizeof(id));
        strcpy_s(id, MAX_PATH - 1, deviceInfo->DeviceID);
        _strlwr_s(id, MAX_PATH);

        if ((chID = strstr(id, "vid_")) != NULL)
            sscanf_s(chID, "vid_%04x", &vid);
        if ((chID = strstr(id, "pid_")) != NULL)
            sscanf_s(chID, "pid_%04x", &pid);
        if ((chID = strstr(id, "mi_")) != NULL)
            sscanf_s(chID, "mi_%02x", &mi);

        if (TestParams->vid == vid && TestParams->pid == pid) {
            // enabled
            LibK_SetContext(deviceInfo, KLIB_HANDLE_TYPE_LSTINFOK, (KLIB_USER_CONTEXT)TRUE);
        }
    }

    return ERROR_SUCCESS;
}


int GetDeviceInfoFromList(PUVPERF_PARAM TestParams) {
    UCHAR selection;
    UCHAR count = 0;
    KLST_DEVINFO_HANDLE deviceInfo = NULL;

    LstK_MoveReset(TestParams->DeviceList);

    if (TestParams->listDevicesOnly) {
        while (LstK_MoveNext(TestParams->DeviceList, &deviceInfo)) {
            count++;
            LOG_MSG("%02u. %s (%s) [%s]\n", count, deviceInfo->DeviceDesc, deviceInfo->DeviceID,
                    GetDrvIdString(deviceInfo->DriverID));
        }

        return ERROR_SUCCESS;
    } else {
        while (LstK_MoveNext(TestParams->DeviceList, &deviceInfo) && count < 9) {
            LOG_MSG("%u. %s (%s) [%s]\n", count + 1, deviceInfo->DeviceDesc, deviceInfo->DeviceID,
                    GetDrvIdString(deviceInfo->DriverID));
            count++;

            // enabled
            LibK_SetContext(deviceInfo, KLIB_HANDLE_TYPE_LSTINFOK, (KLIB_USER_CONTEXT)TRUE);
        }

        if (!count) {
            LOG_ERROR("can not find vid : 0x%04X, pid : 0x%04X device\n", TestParams->vid,
                      TestParams->pid);
            return -1;
        }

        int validSelection = 0;

        do {
            LOG_MSG("Select device (1-%u): ", count);
            while (_kbhit()) {
                _getch();
            }

            selection = (CHAR)_getche() - (UCHAR)'0';
            fprintf(stderr, "\n");
            if (selection == 'q' - '0') {
                return -1;
            }
            if (selection > 0 && selection <= count) {
                count = 0;
                while (LstK_MoveNext(TestParams->DeviceList, &deviceInfo) && ++count != selection) {
                    // disabled
                    LibK_SetContext(deviceInfo, KLIB_HANDLE_TYPE_LSTINFOK,
                                    (KLIB_USER_CONTEXT)FALSE);
                }

                if (!deviceInfo) {
                    LOGERR0("Unknown selection\n");
                    continue;
                }

                TestParams->SelectedDeviceProfile = deviceInfo;
                validSelection = 1;
            } else {
                fprintf(stderr, "Invalid selection. Please select a number between 1 and %u\n",
                        count);
                fprintf(stderr, "Press 'q' to quit\n");
            }
        } while (!validSelection);

        return ERROR_SUCCESS;
    }

    return -1;
}



void FileIOOpen(PUVPERF_PARAM TestParams) {
    time_t now = time(NULL);
    struct tm *t = localtime(&now);

    strftime(TestParams->LogFileName, MAX_PATH - 1, "../log/uvperf_log_%Y%m%d_%H%M%S.txt", t);
    TestParams->LogFileName[MAX_PATH - 1] = '\0';

    if (TestParams->fileIO) {
        TestParams->LogFile =
            CreateFile(TestParams->LogFileName, GENERIC_READ | GENERIC_WRITE,
                       FILE_SHARE_READ | FILE_SHARE_WRITE, NULL,
                       OPEN_ALWAYS, // Open the file if it exists; otherwise, create it
                       FILE_ATTRIBUTE_NORMAL, NULL);

        if (TestParams->LogFile == INVALID_HANDLE_VALUE) {
            LOG_ERROR("failed opening %s\n", TestParams->LogFileName);
            TestParams->fileIO = FALSE;
        }
    }
}

void FileIOLog(PUVPERF_PARAM TestParams) {
    if (!TestParams->fileIO) {
        return;
    }

    freopen(TestParams->LogFileName, "a+", stdout);
    freopen(TestParams->LogFileName, "a+", stderr);

    ShowParams(TestParams);
}

void FileIOClose(PUVPERF_PARAM TestParams) {
    if (TestParams->fileIO) {
        // if (TestParams->BufferFile != INVALID_HANDLE_VALUE) {
        //     CloseHandle(TestParams->BufferFile);
        //     TestParams->BufferFile = INVALID_HANDLE_VALUE;
        // }

        if (TestParams->LogFile != INVALID_HANDLE_VALUE) {
            fclose(stdout);
            CloseHandle(TestParams->LogFile);
            TestParams->LogFile = INVALID_HANDLE_VALUE;
        }
    }
}





void ShowEndpointDescriptor(libusb_device *dev, int interface_index, int endpoint_index) {
    struct libusb_config_descriptor *config;
    const struct libusb_interface_descriptor *interface;
    const struct libusb_endpoint_descriptor *endpoint;
    int r = libusb_get_active_config_descriptor(dev, &config);
    if (r < 0) {
        LOGERR0("Failed to get configuration descriptor\n");
        return;
    }

    if (interface_index >= config->bNumInterfaces) {
        LOGERR0("Invalid interface index\n");
        libusb_free_config_descriptor(config);
        return;
    }

    interface = &config->interface[interface_index].altsetting[0];
    if (endpoint_index >= interface->bNumEndpoints) {
        LOGERR0("Invalid endpoint index\n");
        libusb_free_config_descriptor(config);
        return;
    }

    endpoint = &interface->endpoint[endpoint_index];

    LOG_MSG("Endpoint Descriptor:\n");
    LOG_MSG("  bLength: %d\n", endpoint->bLength);
    LOG_MSG("  bDescriptorType: %d\n", endpoint->bDescriptorType);
    LOG_MSG("  bEndpointAddress: %02X\n", endpoint->bEndpointAddress);
    LOG_MSG("  bmAttributes: %02X\n", endpoint->bmAttributes);
    LOG_MSG("  wMaxPacketSize: %d\n", endpoint->wMaxPacketSize);
    LOG_MSG("  bInterval: %d\n", endpoint->bInterval);

    libusb_free_config_descriptor(config);
}

void ShowInterfaceDescriptor(libusb_device *dev, int interface_index) {
    struct libusb_config_descriptor *config;
    const struct libusb_interface_descriptor *interface;
    int r = libusb_get_active_config_descriptor(dev, &config);
    if (r < 0) {
        LOGERR0("Failed to get configuration descriptor\n");
        return;
    }

    if (interface_index >= config->bNumInterfaces) {
        LOGERR0("Invalid interface index\n");
        libusb_free_config_descriptor(config);
        return;
    }

    interface = &config->interface[interface_index].altsetting[0];

    LOG_MSG("Interface Descriptor:\n");
    LOG_MSG("  bLength: %d\n", interface->bLength);
    LOG_MSG("  bDescriptorType: %d\n", interface->bDescriptorType);
    LOG_MSG("  bInterfaceNumber: %d\n", interface->bInterfaceNumber);
    LOG_MSG("  bAlternateSetting: %d\n", interface->bAlternateSetting);
    LOG_MSG("  bNumEndpoints: %d\n", interface->bNumEndpoints);
    LOG_MSG("  bInterfaceClass: %d\n", interface->bInterfaceClass);
    LOG_MSG("  bInterfaceSubClass: %d\n", interface->bInterfaceSubClass);
    LOG_MSG("  bInterfaceProtocol: %d\n", interface->bInterfaceProtocol);
    LOG_MSG("  iInterface: %d\n", interface->iInterface);

    libusb_free_config_descriptor(config);

}

void ShowConfigurationDescriptor(libusb_device *dev) {
    struct libusb_config_descriptor *config;
    int r = libusb_get_active_config_descriptor(dev, &config);
    if (r < 0) {
        LOGERR0("Failed to get configuration descriptor\n");
        return;
    }

    LOG_MSG("Configuration Descriptor:\n");
    LOG_MSG("  bLength: %d\n", config->bLength);
    LOG_MSG("  bDescriptorType: %d\n", config->bDescriptorType);
    LOG_MSG("  wTotalLength: %d\n", config->wTotalLength);
    LOG_MSG("  bNumInterfaces: %d\n", config->bNumInterfaces);
    LOG_MSG("  bConfigurationValue: %d\n", config->bConfigurationValue);
    LOG_MSG("  iConfiguration: %d\n", config->iConfiguration);
    LOG_MSG("  bmAttributes: %02X\n", config->bmAttributes);
    LOG_MSG("  MaxPower: %d\n", config->MaxPower);

    libusb_free_config_descriptor(config);
}

void ShowDeviceDescriptor(libusb_device *dev) {
    struct libusb_device_descriptor desc;
    int r = libusb_get_device_descriptor(dev, &desc);
    if (r < 0) {
        LOGERR0("Failed to get device descriptor\n");
        return;
    }

    LOG_MSG("Device Descriptor:\n");
    LOG_MSG("  bLength: %d\n", desc.bLength);
    LOG_MSG("  bDescriptorType: %d\n", desc.bDescriptorType);
    LOG_MSG("  bcdUSB: %04X\n", desc.bcdUSB);
    LOG_MSG("  bDeviceClass: %d\n", desc.bDeviceClass);
    LOG_MSG("  bDeviceSubClass: %d\n", desc.bDeviceSubClass);
    LOG_MSG("  bDeviceProtocol: %d\n", desc.bDeviceProtocol);
    LOG_MSG("  bMaxPacketSize0: %d\n", desc.bMaxPacketSize0);
    LOG_MSG("  idVendor: %04X\n", desc.idVendor);
    LOG_MSG("  idProduct: %04X\n", desc.idProduct);
    LOG_MSG("  bcdDevice: %04X\n", desc.bcdDevice);
    LOG_MSG("  iManufacturer: %d\n", desc.iManufacturer);
    LOG_MSG("  iProduct: %d\n", desc.iProduct);
    LOG_MSG("  iSerialNumber: %d\n", desc.iSerialNumber);
    LOG_MSG("  bNumConfigurations: %d\n", desc.bNumConfigurations);
}

void PerformTransfer() {
    LOG_MSG("Performing transfer...\n");
}

void ShowMenu() {
    LOG_MSG("Press e: Endpoint descriptor\n");
    LOG_MSG("Press i: Interface descriptor\n");
    LOG_MSG("Press c: Configuration descriptor\n");
    LOG_MSG("Press d: Device descriptor\n");
    LOG_MSG("Press q: Quit\n");
    LOG_MSG("Press t or Enter: Transfer\n");

}


void ShowDeviceInterfaces(libusb_device *dev) {
    struct libusb_config_descriptor *config;
    int r = libusb_get_active_config_descriptor(dev, &config);
    if (r < 0) {
        LOGERR0("Failed to get configuration descriptor\n");
        return;
    }

    LOG_MSG("\n");
    LOG_MSG("Number of interfaces: %d\n", config->bNumInterfaces);
    for (int i = 0; i < config->bNumInterfaces; i++) {
        const struct libusb_interface *interface = &config->interface[i];
        LOG_MSG("Interface %d has %d alternate settings\n", i, interface->num_altsetting);
        for (int j = 0; j < interface->num_altsetting; j++) {
            const struct libusb_interface_descriptor *altsetting = &interface->altsetting[j];
            LOG_MSG("  Alternate setting %d: Interface number: %d, Number of endpoints: %d\n",
                    altsetting->bAlternateSetting, altsetting->bInterfaceNumber, altsetting->bNumEndpoints);
        }
    }

    libusb_free_config_descriptor(config);
}


int main(int argc, char **argv) {
    UVPERF_PARAM TestParams;
    PUVPERF_TRANSFER_PARAM InTest = NULL;
    PUVPERF_TRANSFER_PARAM OutTest = NULL;
    int key;
    long ec;
    unsigned int count;
    UCHAR bIsoAsap;

//showing descriptors
    libusb_device **devs;
    libusb_device_handle *handle = NULL;
    int r;
    ssize_t cnt;

    libusb_init(NULL);


    cnt = libusb_get_device_list(NULL, &devs);
    if (cnt < 0) {
        LOG_MSG("Error in getting device list\n");
        libusb_exit(NULL);
        return -1;
    }

    handle = libusb_open_device_with_vid_pid(NULL, 0x1004, 0x61A1);
    //handle = libusb_open_device_with_vid_pid(NULL, 0x054C, 0x0E4F);
    if (handle == 0) {
        LOGERR0("Device not found\n");
    }


    if (argc == 1) {
        LOG_VERBOSE("No arguments\n");
        ShowUsage();
        return -1;
    }

    LOG_VERBOSE("SetParamsDefaults\n");
    SetParamsDefaults(&TestParams);

    LOG_VERBOSE("ParseArgs\n");
    if (ParseArgs(&TestParams, argc, argv) < 0)
        return -1;

    FileIOOpen(&TestParams);


    LOG_VERBOSE("InitializeCriticalSection\n");
    InitializeCriticalSection(&DisplayCriticalSection);

    LOG_VERBOSE("LibusbK device List Initialize\n");
    if (!LstK_Init(&TestParams.DeviceList, 0)) {
        ec = GetLastError();
        LOG_ERROR("Failed to initialize device list ec=%08Xh, message %s: \n", ec, strerror(ec));
        goto Final;
    }

    count = 0;
    LstK_Count(TestParams.DeviceList, &count);
    if (count == 0) {
        LOGERR0("No devices found\n");
        goto Final;
    }

    ShowMenu();

    LOG_MSG("Enter your choice: ");
    key = _getch();
    printf("\n");
    int interface_index, endpoint_index;

    if (key != '\r') {  // If not Enter key
        switch (key) {
            case 'e':
                LOG_MSG("Enter interface index: ");
                scanf("%d", &interface_index);
                LOG_MSG("Enter endpoint index: ");
                scanf("%d", &endpoint_index);
                ShowEndpointDescriptor(libusb_get_device(handle), interface_index, endpoint_index);
                break;
            case 'i':
            //interface0 set as bulk transfer
            //interface1 set as isochronous transfe
            //ref find_usb.py 
                LOG_MSG("Enter interface index: ");
                scanf("%d", &interface_index);
                ShowInterfaceDescriptor(libusb_get_device(handle), interface_index);
                break;
            case 'c':
                ShowConfigurationDescriptor(libusb_get_device(handle));
                break;
            case 'd':
                ShowDeviceDescriptor(libusb_get_device(handle));
                break;
            case 't':
                PerformTransfer();
                break;
            case 'q':
                LOG_MSG("User Aborted\n");
                return 0;
            default:
                LOGERR0("Invalid input\n");
                break;
        }
    }

    ShowDeviceInterfaces(libusb_get_device(handle));

    LOG_VERBOSE("GetDeviceInfoFromList\n");
    if (TestParams.intf == -1 || TestParams.altf == -1 || TestParams.endpoint == 0x00) {
        if (GetDeviceInfoFromList(&TestParams) < 0) {
            goto Final;
        }
        KLST_DEVINFO_HANDLE deviceInfo;
        WINUSB_PIPE_INFORMATION_EX pipeInfo[32];
        int userChoice;
        UCHAR pipeIndex;

        int altresult;
        int altSetting_tmp;

        while (1) {
            LOG_MSG("Enter the alternate setting number for the device interface (0-n): ");
            altresult = scanf("%d", &altSetting_tmp);

            if (altresult == 1) {
                break;
            } else {
                LOG_ERROR("Invalid input for alternate setting. Please enter a number: ");
                while (getchar() != '\n');
            }
        }


        int validInput = 0; // Flag to check for valid input
        do {
            while (LstK_MoveNext(TestParams.DeviceList, &deviceInfo)) {
                if (!LibK_LoadDriverAPI(&K, deviceInfo->DriverID)) {
                    WinError(GetLastError());
                    LOG_ERROR("Cannot load driver API for %s\n",
                              GetDrvIdString(deviceInfo->DriverID));
                    continue;
                }

                if (!K.Init(&TestParams.InterfaceHandle, deviceInfo)) {
                    WinError(GetLastError());
                    LOG_ERROR("Cannot initialize device interface for %s\n",
                              deviceInfo->DevicePath);
                    continue;
                }

                UCHAR altSetting=altSetting_tmp;

                
                int attemptCount = 0;
                int maxAttempts = 5;

                LOG_MSG("Device %s initialized successfully.\n", deviceInfo->DevicePath);
                while (K.QueryInterfaceSettings(TestParams.InterfaceHandle, altSetting,
                                                &TestParams.InterfaceDescriptor)) {
                    LOG_MSG("Interface %d: Checking pipes...\n",
                            TestParams.InterfaceDescriptor.bInterfaceNumber);
                    pipeIndex = 0;
                    if (!K.QueryPipeEx(TestParams.InterfaceHandle, altSetting, pipeIndex, &pipeInfo[pipeIndex])) {
                        LOG_ERROR("No pipes available. ErrorCode=0x%08X, message : %s\n",
                            GetLastError(), strerror(GetLastError()));
                        if (++attemptCount >= maxAttempts) {
                            LOG_ERROR("Maximum retry attempts reached. Exiting loop.\n");
                            goto Final;
                        }
                        continue;
                    }

                    while (K.QueryPipeEx(TestParams.InterfaceHandle, altSetting, pipeIndex, &pipeInfo[pipeIndex])) {
                        LOG_MSG("Pipe %d: Type : %11s, %3s, MaximumBytesPerInterval : %4d, "
                                "MaxPacketSize : %4d, MC = %2d\n",
                                pipeIndex + 1,
                                EndpointTypeDisplayString[pipeInfo[pipeIndex].PipeType],
                                (pipeInfo[pipeIndex].PipeId & USB_ENDPOINT_DIRECTION_MASK) ? "in"
                                                                                           : "out",
                                pipeInfo[pipeIndex].MaximumBytesPerInterval,
                                pipeInfo[pipeIndex].MaximumPacketSize,
                                (pipeInfo[pipeIndex].MaximumPacketSize & 0x1800) >> 11);
                        pipeIndex++;
                    }

                    LOG_MSG(
                        "Enter the number of the pipe to use for transfer (1-%d), 'Q' to quit: ",
                        pipeIndex);
                    int ch = _getche();
                    printf("\n");

                    if (ch == 'Q' || ch == 'q') {
                        LOG_MSG("Exiting program.\n");
                        return 0;
                    }

                    userChoice = ch - '0';
                    if (userChoice < 1 || userChoice > pipeIndex) {
                        LOGERR0("Invalid pipe selection.\n");
                        continue;
                    }

                    TestParams.endpoint = (int)(pipeInfo[userChoice - 1].PipeId);
                    TestParams.TestType =
                        (pipeInfo[userChoice - 1].PipeId & USB_ENDPOINT_DIRECTION_MASK)
                            ? TestTypeIn
                            : TestTypeOut;

                    LOG_MSG("Selected pipe 0x%02X\n", pipeInfo[userChoice - 1].PipeId);

                    validInput = 1;
                    break;
                }
            }
        } while (!validInput);
    } else {
        LOG_VERBOSE("GetDeviceParam\n");
        if (GetDeviceParam(&TestParams) < 0) {
            goto Final;
        }
    }

    LOG_VERBOSE("Open Bench\n");
    if (!Bench_Open(&TestParams)) {
        goto Final;
    }

    if (TestParams.TestType & TestTypeIn) {
        LOG_VERBOSE("CreateTransferParam for InTest\n");
        InTest = CreateTransferParam(&TestParams, TestParams.endpoint | USB_ENDPOINT_DIRECTION_MASK);
        if (!InTest){
            LOGERR0("Failed to create transfer param for InTest\n");
            goto Final;
        }
        if (TestParams.UseRawIO != 0xFF) {
            if (!K.SetPipePolicy(TestParams.InterfaceHandle, InTest->Ep.PipeId, RAW_IO, 1,
                                 &TestParams.UseRawIO)) {
                ec = GetLastError();
                LOG_ERROR("SetPipePolicy:RAW_IO failed. ErrorCode=%08Xh message : %s\n", ec,
                          strerror(ec));
                goto Final;
            }
        }
    }

    if (TestParams.TestType & TestTypeOut) {
        LOG_VERBOSE("CreateTransferParam for OutTest\n");
        OutTest = CreateTransferParam(&TestParams, TestParams.endpoint & 0x0F);
        if (!OutTest)
            goto Final;
        if (TestParams.fixedIsoPackets) {
            if (!K.SetPipePolicy(TestParams.InterfaceHandle, OutTest->Ep.PipeId,
                                 ISO_NUM_FIXED_PACKETS, 2, &TestParams.fixedIsoPackets)) {
                ec = GetLastError();
                LOG_ERROR("SetPipePolicy:ISO_NUM_FIXED_PACKETS failed. ErrorCode=0x%08X, "
                          "message : %s\n",
                          ec, strerror(ec));
                goto Final;
            }
        }
        if (TestParams.UseRawIO != 0xFF) {
            if (!K.SetPipePolicy(TestParams.InterfaceHandle, OutTest->Ep.PipeId, RAW_IO, 1,
                                 &TestParams.UseRawIO)) {
                ec = GetLastError();
                LOG_ERROR("SetPipePolicy:RAW_IO failed. ErrorCode=0x%08X\n, gessage : %s", ec,
                          strerror(ec));
                goto Final;
            }
        }
    }

    if (TestParams.verify) {
        if (InTest && OutTest) {
            LOG_VERBOSE("CreateVerifyBuffer for OutTest\n");
            if (CreateVerifyBuffer(&TestParams, OutTest->Ep.MaximumPacketSize) < 0)
                goto Final;
        } else if (InTest) {
            LOG_VERBOSE("CreateVerifyBuffer for InTest\n");
            if (CreateVerifyBuffer(&TestParams, InTest->Ep.MaximumPacketSize) < 0)
                goto Final;
        }
    }

    if ((OutTest && OutTest->Ep.PipeType == UsbdPipeTypeIsochronous) ||
        (InTest && InTest->Ep.PipeType == UsbdPipeTypeIsochronous)) {
        UINT frameNumber;
        LOG_VERBOSE("GetCurrentFrameNumber\n");
        if (!K.GetCurrentFrameNumber(TestParams.InterfaceHandle, &frameNumber)) {
            ec = GetLastError();
            LOG_ERROR("GetCurrentFrameNumber Failed. ErrorCode=%u, message : %s", ec, strerror(ec));
            goto Final;
        }
        frameNumber += TestParams.bufferCount * 2;
        if (OutTest) {
            OutTest->frameNumber = frameNumber;
            frameNumber++;
        }
        if (InTest) {
            InTest->frameNumber = frameNumber;
            frameNumber++;
        }
    }

    LOG_VERBOSE("ShowParams\n");
    ShowParams(&TestParams);
    if (InTest)
        ShowTransfer(InTest);
    if (OutTest)
        ShowTransfer(OutTest);

    bIsoAsap = (UCHAR)TestParams.UseIsoAsap;
    if (InTest)
        K.SetPipePolicy(TestParams.InterfaceHandle, InTest->Ep.PipeId, ISO_ALWAYS_START_ASAP, 1,
                        &bIsoAsap);
    if (OutTest)
        K.SetPipePolicy(TestParams.InterfaceHandle, OutTest->Ep.PipeId, ISO_ALWAYS_START_ASAP, 1,
                        &bIsoAsap);

    if (InTest) {
        LOG_VERBOSE("ResumeThread for InTest\n");
        SetThreadPriority(InTest->ThreadHandle, TestParams.priority);
        ResumeThread(InTest->ThreadHandle);
    }

    if (OutTest) {
        LOG_VERBOSE("ResumeThread for OutTest\n");
        SetThreadPriority(OutTest->ThreadHandle, TestParams.priority);
        ResumeThread(OutTest->ThreadHandle);
    }

    LOGMSG0("Press 'Q' to abort\n");

    FileIOLog(&TestParams);

    while (!TestParams.isCancelled) {

        Sleep(TestParams.refresh);
        if (_kbhit()) {
            key = _getch();
            switch (key) {
            case 'Q':
            case 'q':
                LOG_VERBOSE("User Aborted\n");
                TestParams.isUserAborted = TRUE;
                TestParams.isCancelled = TRUE;
            }

            if ((InTest) && !InTest->isRunning) {
                LOG_VERBOSE("InTest is not running\n");
                TestParams.isCancelled = TRUE;
                break;
            }

            if ((OutTest) && !OutTest->isRunning) {
                LOG_VERBOSE("OutTest is not running\n");
                TestParams.isCancelled = TRUE;
                break;
            }
        }

        if (TestParams.Timer && InTest &&
            InTest->LastTick.tv_sec - InTest->StartTick.tv_sec >= TestParams.Timer) {
            LOG_VERBOSE("Over 60 seconds\n");
            DWORD elapsedSeconds =
                ((InTest->LastTick.tv_sec - InTest->StartTick.tv_sec) +
                 (InTest->LastTick.tv_nsec - InTest->StartTick.tv_nsec) / 100000000.0);
            LOG_MSG("Elapsed Time %.2f  seconds\n", elapsedSeconds);
            TestParams.isUserAborted = TRUE;
            TestParams.isCancelled = TRUE;
        }

        if (TestParams.Timer && OutTest &&
            OutTest->LastTick.tv_sec - OutTest->StartTick.tv_sec >= TestParams.Timer) {
            LOG_VERBOSE("Over 60 seconds\n");
            DWORD elapsedSeconds =
                ((InTest->LastTick.tv_sec - InTest->StartTick.tv_sec) +
                 (InTest->LastTick.tv_nsec - InTest->StartTick.tv_nsec) / 100000000.0);
            LOG_MSG("Elapsed Time %.2f  seconds\n", elapsedSeconds);
            TestParams.isUserAborted = TRUE;
            TestParams.isCancelled = TRUE;
        }

        // if (TestParams.fileIO) {
        //     if (InTest) {
        //         FileIOBuffer(&TestParams, InTest);
        //     }
        //     if (OutTest) {
        //         FileIOBuffer(&TestParams, OutTest);
        //     }
        // }

        LOG_VERBOSE("ShowRunningStatus\n");
        ShowRunningStatus(InTest, OutTest);
        while (_kbhit())
            _getch();
    }

    LOG_VERBOSE("WaitForTestTransfer\n");
    WaitForTestTransfer(InTest, 1000);
    if ((InTest) && InTest->isRunning) {
        LOG_WARNING("Aborting Read Pipe 0x%02X..\n", InTest->Ep.PipeId);
        K.AbortPipe(TestParams.InterfaceHandle, InTest->Ep.PipeId);
    }

    WaitForTestTransfer(OutTest, 1000);
    if ((OutTest) && OutTest->isRunning) {
        LOG_WARNING("Aborting Write Pipe 0x%02X..\n", OutTest->Ep.PipeId);
        K.AbortPipe(TestParams.InterfaceHandle, OutTest->Ep.PipeId);
    }

    if ((InTest) && InTest->isRunning)
        WaitForTestTransfer(InTest, INFINITE);
    if ((OutTest) && OutTest->isRunning)
        WaitForTestTransfer(OutTest, INFINITE);

    LOG_VERBOSE("Show Transfer\n");
    if (InTest)
        ShowTransfer(InTest);
    if (OutTest)
        ShowTransfer(OutTest);

    freopen("CON", "w", stdout);
    freopen("CON", "w", stderr);

Final:
    LOG_VERBOSE("Free TransferParam\n");
    if (TestParams.InterfaceHandle) {
        LOG_VERBOSE("ResetPipe\n");
        K.SetAltInterface(TestParams.InterfaceHandle, TestParams.InterfaceDescriptor.bInterfaceNumber,
                          FALSE, TestParams.defaultAltSetting);
        K.Free(TestParams.InterfaceHandle);

        TestParams.InterfaceHandle = NULL;
    }

    LOG_VERBOSE("Close Handle\n");
    if (!TestParams.use_UsbK_Init) {
        if (TestParams.DeviceHandle) {
            CloseHandle(TestParams.DeviceHandle);
            TestParams.DeviceHandle = NULL;
        }
    }

    LOG_VERBOSE("Close Bench\n");
    if (TestParams.VerifyBuffer) {
        PUVPERF_BUFFER verifyBuffer, verifyListTemp;

        free(TestParams.VerifyBuffer);
        TestParams.VerifyBuffer = NULL;
    }

    LOG_VERBOSE("Free TransferParam\n");
    LstK_Free(TestParams.DeviceList);
    FreeTransferParam(&InTest);
    FreeTransferParam(&OutTest);

    DeleteCriticalSection(&DisplayCriticalSection);

    if (!TestParams.listDevicesOnly) {
        LOGMSG0("Press any key to exit\n");
        _getch();
        LOGMSG0("\n");
    }

    libusb_free_device_list(devs, 1);
    libusb_exit(NULL);

    FileIOClose(&TestParams);

    return 0;
}
