//
//  RFComm.h
//  RFComm
//
//  Created by Michael Dreher on 15.08.11.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//


#import "RFComm-Prefix.h"
//#import <Carbon/Carbon.h>
//#import <IOKit/hid/IOHIDLib.h>
#import "RingBuffer.h"
void myRunLoopObserver (
                        CFRunLoopObserverRef observer,
                        CFRunLoopActivity activity,
                        void *info
                        );

enum connStates
{
    connStateDisconnected = 0,
    connStateConnecting,
    connStateConnected,
};

enum sdpQueryStates
{
    sdpQueryStateNotRunning = 0,
    sdpQueryStateRunning,
    sdpQueryStateFinished,
};

enum connectionNotifyReceivedMask
{
    connectionNotifyReceivedSignalsChanged     = 1 << 0,
    connectionNotifyReceivedFlowControlChanged = 1 << 1,
    connectionNotifyReceivedChannelClosed      = 1 << 2,
    connectionNotifyReceivedWriteError         = 1 << 3,
};

@interface RFCAsyncWriteRequest : NSObject
{
}
@property (assign) uint64_t mWritePosition;
- (id)init:(uint64_t)writePos;

@end

@interface RFComm : NSObject
{
    IOBluetoothHostController *mHc;
    IOBluetoothRFCOMMChannel *mRfCommChannel;    
    NSCondition *mInqLock; // synchronsation for BT inquiry
    //NSCondition *mChannelLock; // synchronsation for channel functions
    IOBluetoothSDPServiceRecord *mServiceRecord;

    RingBuffer *mReceiveRingBuffer;
    IOReturn mDelegateIoReturn;
	uint64_t mWritePosition;
	uint64_t mWritePositionAck;
    enum connStates mConnState;
	enum sdpQueryStates mSdpQueryState;
	enum connectionNotifyReceivedMask mConnNotifyReceived;
}
- (IOReturn)connectToPairedSppDevByMacAddr:(NSString*)macAddressString;
- (IOReturn)connect:(IOBluetoothSDPServiceRecord *)serviceRecord;
- (IOReturn)closeChannel;
- (ssize_t)readSync:(void *)buf  nbyte:(size_t)nbyte;
- (ssize_t)readSync:(void *)buf  nbyte:(size_t)nbyte timeOut:(NSTimeInterval)timeOut;
- (ssize_t)writeSync:(const void *)buf  nbyte:(size_t)nbyte;
- (ssize_t)writeSync:(const void *)buf  nbyte:(size_t)nbyte  timeOut:(NSTimeInterval)timeOut;
- (IOReturn)discover:(NSString*)portName;
- (bool)initRunLoop;
- (void)dispatchWaitingEvents;

// IOBluetoothHostControllerDelegate
- (void)	controllerClassOfDeviceReverted:(id)sender;
- (void)	readRSSIForDeviceComplete:(id)controller device:(IOBluetoothDevice*)device	info:(BluetoothHCIRSSIInfo*)info	error:(IOReturn)error;
- (void)	readLinkQualityForDeviceComplete:(id)controller device:(IOBluetoothDevice*)device	info:(BluetoothHCILinkQualityInfo*)info	error:(IOReturn)error;


// IOBluetoothDeviceInquiry
- (void)	deviceInquiryStarted:(IOBluetoothDeviceInquiry*)sender;
- (void)	deviceInquiryComplete:(IOBluetoothDeviceInquiry*)sender	error:(IOReturn)error	aborted:(BOOL)aborted;


// IOBluetoothRFCOMMChannelDelegate
- (void)rfcommChannelData:(IOBluetoothRFCOMMChannel*)rfcommChannel data:(void *)dataPointer length:(size_t)dataLength;
- (void)rfcommChannelOpenComplete:(IOBluetoothRFCOMMChannel*)rfcommChannel status:(IOReturn)error;
- (void)rfcommChannelClosed:(IOBluetoothRFCOMMChannel*)rfcommChannel;
- (void)rfcommChannelControlSignalsChanged:(IOBluetoothRFCOMMChannel*)rfcommChannel;
- (void)rfcommChannelFlowControlChanged:(IOBluetoothRFCOMMChannel*)rfcommChannel;
//- (void)rfcommChannelWriteComplete:(IOBluetoothRFCOMMChannel*)rfcommChannel refcon:(void*)refcon status:(IOReturn)error;
//- (void)rfcommChannelQueueSpaceAvailable:(IOBluetoothRFCOMMChannel*)rfcommChannel;

- (void)sdpQueryComplete:(IOBluetoothDevice *)device status:(IOReturn)status;
@end



