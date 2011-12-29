//
//  RFComm.m
//  RFComm
//
//  Created by Michael Dreher on 15.08.11.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#import "RFComm.h"

@implementation RFCAsyncWriteRequest
@synthesize mWritePosition;

- (id)init
{
    self = [super init];
    if (self) {
	
    }
    
    return self;
}

- (id)init:(uint64_t)writePos
{
	[self init];
	self.mWritePosition = writePos;
    
    return self;
}

- (void)dealloc
{

    [super dealloc];
}

@end


@implementation RFComm

- (id)init
{
    self = [super init];
    if (self) {
        // Initialization code here.
        mHc = [[IOBluetoothHostController defaultController] retain];
        [mHc setDelegate:self];
        mInqLock = [NSCondition new];
        //mChannelLock = [NSCondition new];
        mReceiveRingBuffer = [[RingBuffer alloc] init];
        mServiceRecord = nil;
        mRfCommChannel = nil;
        mConnState = connStateDisconnected;
        mDelegateIoReturn = kIOReturnError;
		mSdpQueryState = sdpQueryStateNotRunning;
		mConnNotifyReceived = 0;
		mWritePosition = 0;
		mWritePositionAck = 0;
        //[self initRunLoop];
    }
    
    return self;
}

- (void)dealloc
{
    [self closeChannel];
    [mHc setDelegate:nil];
    [mHc release];
    //[mChannelLock release];
    [mInqLock release];
    [super dealloc];
}

- (bool)initRunLoop
{
    bool rc = false;
    NSRunLoop* mMyRunLoop;
    CFRunLoopObserverRef mObserver;
    mMyRunLoop = [NSRunLoop currentRunLoop];
    
    // Create a run loop observer and attach it to the run loop.
    CFRunLoopObserverContext obscontext = {0, self, NULL, NULL, NULL};
    mObserver = CFRunLoopObserverCreate(kCFAllocatorDefault,
        kCFRunLoopAllActivities, YES, 0, &myRunLoopObserver, &obscontext);
    
    if (mObserver)
    {
        CFRunLoopRef cfLoop = [mMyRunLoop getCFRunLoop];
        CFRunLoopAddObserver(cfLoop, mObserver, kCFRunLoopDefaultMode);
    
        // Create and schedule the timer.
        [NSTimer scheduledTimerWithTimeInterval:0.1 target:self
            selector:@selector(doFireTimer:) userInfo:nil repeats:YES];
        
        NSInteger loopCount = 10;
        do
        {
            // Run the run loop 10 times to let the timer fire.
            [mMyRunLoop runUntilDate:[NSDate dateWithTimeIntervalSinceNow:1]];
            loopCount--;
        }
        while (loopCount);
        rc = true;
    }
    
    return rc;
}


- (IOReturn)connectToPairedSppDevByMacAddr:(NSString*)macAddressString
{
    IOReturn rc = kIOReturnNoDevice;
    IOReturn ioret;
    BluetoothDeviceAddress macAddress;
    
    ioret = IOBluetoothNSStringToDeviceAddress(macAddressString, &macAddress);
    if(ioret != kIOReturnSuccess)
    {
        rc = ioret;
        return rc;
    }    
    
    /*
    IOBluetoothDeviceSearchDeviceAttributes dsal[1];
    memset(dsal, 0, sizeof(dsal));
    dsal[0].deviceClassMajor = kBluetoothDeviceClassMajorAny;
    dsal[0].deviceClassMinor = kBluetoothDeviceClassMinorAny;
    dsal[0].serviceClassMajor = kBluetoothServiceClassMajorAny;
    //dsal[0].address =
    //dsal[0].name =

    
    IOBluetoothDeviceSearchAttributes btdsa;
    btdsa.options = kSearchOptionsNone; // IOBluetoothDeviceSearchOptionsBits
    btdsa.maxResults = 1;
    btdsa.attributeList = dsal;
    btdsa.deviceAttributeCount = sizeof(dsal) / sizeof(dsal[0]);
    */

    NSArray *devices = [IOBluetoothDevice pairedDevices];
    IOBluetoothDevice* dev;
    NSEnumerator *enumerator;
    for(enumerator = [devices objectEnumerator]; dev = [enumerator nextObject]; )
    {
        /*NSLog(@"Device nameOrAddress=\"%@\", %X %X %X %X  %d %d %d", [dev nameOrAddress],
              [dev classOfDevice], [dev deviceClassMajor], [dev deviceClassMinor], [dev serviceClassMajor],
              (int)[dev isConnected], (int)[dev RSSI], (int)[dev rawRSSI]
        );*/
        
        const BluetoothDeviceAddress *btAddr = [dev getAddress];
        if(btAddr && memcmp(btAddr, &macAddress, sizeof(macAddress)) == 0)
        {
            // matching device found, now look for first SPP service
            IOBluetoothSDPUUID* sppUUID = [IOBluetoothSDPUUID uuid16:kBluetoothSDPUUID16ServiceClassSerialPort];
			
			// TODO: should  update the service record to make sure the port number of the SPP service is still the same
			// this means call [dev performSDPQuery:uuids:]
			// it might also resolve the connection error to the BCA8 modules which sometimes need a kind of refresher
			NSArray *uuids = [NSArray arrayWithObjects:sppUUID, nil];
			mSdpQueryState = sdpQueryStateRunning;
			ioret = [dev performSDPQuery:self uuids:uuids];

			// wait using a RunLoop
            NSRunLoop* mMyRunLoop = [NSRunLoop currentRunLoop];
            if(mMyRunLoop)   
            {
                NSDate *timeOutDate = [NSDate dateWithTimeIntervalSinceNow:10];
                BOOL hasRun;
                bool timedOut;
                do {
                    hasRun = [mMyRunLoop runMode:NSDefaultRunLoopMode beforeDate:timeOutDate];
                    timedOut = [timeOutDate compare:[NSDate date]] == NSOrderedAscending;
                } while (mSdpQueryState == sdpQueryStateRunning && !timedOut);
            }
			mSdpQueryState = sdpQueryStateNotRunning;
			
			IOBluetoothSDPServiceRecord *serviceRecord = [dev getServiceRecordForUUID:sppUUID]; // 0x1101 is SPP
            if(serviceRecord)
            {
                rc = [self connect:serviceRecord];
            } 
            
            break; // last devices
        }

        /*
        IOServiceGetMatchingServices(<#mach_port_t masterPort#>, <#CFDictionaryRef matching#>, <#io_iterator_t *existing#>)
        IORegistryEntryCreateCFProperties(<#io_registry_entry_t entry#>, <#CFMutableDictionaryRef *properties#>, <#CFAllocatorRef allocator#>, <#IOOptionBits options#>)
        kIOCalloutDeviceKey
        kIOBSDNameKey
         CFSTR("IOBluetoothSerialClient"
         CFSTR("PortDeviceService"
         CFSTR("Bluetooth Serial Console"
        */
    }
    
    return rc;
}


- (IOReturn)connect:(IOBluetoothSDPServiceRecord *)serviceRecord
{
    IOReturn rc = kIOReturnError;
    IOReturn ioret;

    [self closeChannel];
        
    BluetoothRFCOMMChannelID rfcommChannelId = 255;
    ioret = [serviceRecord getRFCOMMChannelID:&rfcommChannelId];            
    if(ioret == kIOReturnSuccess)
    {
        mConnState = connStateConnecting;
		mConnNotifyReceived = 0;
        ioret = [[serviceRecord device] openRFCOMMChannelAsync:&mRfCommChannel withChannelID:rfcommChannelId delegate:self];        
        if(ioret == kIOReturnSuccess)
        {
            //[mRfCommChannel retain]; // TODO: the examples say it is required, the docu says it is already done, who is right?

            // wait using a RunLoop
            NSRunLoop* mMyRunLoop = [NSRunLoop currentRunLoop];
            if(mMyRunLoop)   
            {
                NSDate *timeOutDate = [NSDate dateWithTimeIntervalSinceNow:33];
                BOOL hasRun;
                bool timedOut;
                do {
                    hasRun = [mMyRunLoop runMode:NSDefaultRunLoopMode beforeDate:timeOutDate];
                    timedOut = [timeOutDate compare:[NSDate date]] == NSOrderedAscending;
                } while (mConnState == connStateConnecting && !timedOut);

				// at this moment secondary notifications will be send, e.g. rfcommChannelControlSignalsChanged
				// TODO: workaround for initial hang problem after opening the port
				// without this, asyncRead will hang because the BCA8 ignores the first data packet when
				// it is sent too fast after opening the port
				// is there any notification which can be used instead of delaying by time?
				timeOutDate = [NSDate dateWithTimeIntervalSinceNow:0.150];
                do {
                    hasRun = [mMyRunLoop runMode:NSDefaultRunLoopMode beforeDate:timeOutDate];
                    timedOut = [timeOutDate compare:[NSDate date]] == NSOrderedAscending;
                } while (!timedOut 
						 //&& !(mConnNotifyReceived & connectionNotifyReceivedFlowControlChanged)
						);
			}

            if(mConnState == connStateConnected && [mRfCommChannel isOpen])
            {
                rc = kIOReturnSuccess; // this is the only way to get out here without an error
                // store service record of connected service
                mServiceRecord = [serviceRecord retain];
                ioret = [mRfCommChannel setSerialParameters:115200 dataBits:8 parity:kBluetoothRFCOMMParityTypeNoParity stopBits:1];
            }
            else
            {
                if(mConnState == connStateConnecting)
                {
                    rc = kIOReturnTimeout;
                }
                else if(mConnState == connStateDisconnected)
                {
                    rc = mDelegateIoReturn;
                }
                else
                {
                    rc = kIOReturnError; // should never happen
                }
            }

            
/*        
        NSLog(@" Servicename=\"%@\", rfcomm channelID=%d", [serviceRecord getServiceName], (int)rfcommchannel);
        NSDictionary *dict = [serviceRecord attributes];
        NSNumber *key;
        for(NSEnumerator *enumerator = [dict keyEnumerator]; key = [enumerator nextObject]; )
        {
            IOBluetoothSDPDataElement* elem = [dict objectForKey:key];
            NSLog(@"    Key %@, val=\"%@\"", key, elem);
        }
*/
        }
        else
        {
            rc = ioret;
        }
    }
    
    if(rc == kIOReturnSuccess)
    {
    }
    else
    {
        [self closeChannel];
    }
    
    return rc;
}


- (ssize_t)writeSync:(const void *)buf  nbyte:(size_t)nbyte
{
	return [self writeSync:buf nbyte:nbyte timeOut:(-1.0)];
}

// pass -1.0 for a default timeout
// pass 0.0 for no timeout which means return immediately when no data is available
- (ssize_t)writeSync:(const void *)buf  nbyte:(size_t)nbyte  timeOut:(NSTimeInterval)timeOut
{
    ssize_t rc = 0;
    IOReturn ioret;
	
	if(timeOut < -0.000001)
	{
		timeOut = 10.0 + nbyte / 100.0; // default value
	}
	//[self dispatchWaitingEvents];
    BluetoothRFCOMMMTU mtu = [mRfCommChannel getMTU]; // TODO: often returns 0, might have to do with the incomplete RunLoop handling
    if(mtu < 10)
        mtu = 126;

    NSRunLoop* myRunLoop = [NSRunLoop currentRunLoop];
	NSDate *timeOutDate = [NSDate dateWithTimeIntervalSinceNow:timeOut];
	mDelegateIoReturn = kIOReturnSuccess;
    while (nbyte > 0 && myRunLoop && timeOutDate)
    {
        size_t curLen = nbyte <= mtu ? nbyte : mtu;
		mWritePosition += curLen;
		RFCAsyncWriteRequest *awr = [[RFCAsyncWriteRequest alloc] init:mWritePosition];
		if(!awr)
		{
			break;
		}
			
        ioret = [mRfCommChannel writeAsync:(void*)(buf) length:curLen refcon:awr];
        if(ioret == kIOReturnSuccess)
        {
            rc += curLen;
            buf += curLen;
            nbyte -= curLen;
            
        }
        else
        {
			mWritePosition -= curLen;
			[awr release];
            break;
        }
    }

	bool timedOut;
	if(myRunLoop)   
	{
		do {
			[myRunLoop runMode:NSDefaultRunLoopMode beforeDate:timeOutDate];
			timedOut = [timeOutDate compare:[NSDate date]] == NSOrderedAscending;
		} while (!timedOut && (mWritePositionAck != mWritePosition));
	}

	if(mDelegateIoReturn != kIOReturnSuccess || mWritePositionAck != mWritePosition)
	{
		rc = 0; // actually we don't know how much has been written
	}

    return rc;
}


- (void)dispatchWaitingEvents
{
	 SInt32 reason;
	 do {
		 reason = CFRunLoopRunInMode(kCFRunLoopDefaultMode, 5, true);
	 } while (reason == kCFRunLoopRunHandledSource);
}


- (ssize_t)readSync:(void *)buf  nbyte:(size_t)nbyte
{
	return [self readSync:buf nbyte:nbyte timeOut:(-1.0)];
}

// pass -1.0 for a default timeout
// pass 0.0 for no timeout which means return immediately when no data is available
- (ssize_t)readSync:(void *)buf  nbyte:(size_t)nbyte timeOut:(NSTimeInterval)timeOut
{
    ssize_t rc = 0;
    
	if(timeOut < -0.000001)
	{
		timeOut = 10.0 + nbyte / 100.0; // default value
	}
	//[self dispatchWaitingEvents];

    NSDate *timeOutDate = [NSDate dateWithTimeIntervalSinceNow:timeOut];
    NSRunLoop* mMyRunLoop = [NSRunLoop currentRunLoop];
    bool timedOut = false;
    while(nbyte > 0 && !timedOut) {
        size_t curLen = [mReceiveRingBuffer read:buf nbyte:nbyte];
        nbyte -= curLen;
        rc += curLen;
        buf += curLen;
				
		// wait for new data using a RunLoop
		if(nbyte > 0 && mMyRunLoop)   
		{
			[mMyRunLoop runMode:NSDefaultRunLoopMode beforeDate:timeOutDate];
		}
		timedOut = [timeOutDate compare:[NSDate date]] == NSOrderedAscending;
    }
    
    return rc;
}



- (IOReturn)closeChannel
{
    if(mRfCommChannel)
    {
        [mRfCommChannel closeChannel];
        [mRfCommChannel setDelegate:nil]; // is this needed? It doesn't hurt either...
        [mRfCommChannel release];
        mRfCommChannel = nil;
    }
    
    if(mServiceRecord)
    {
        [mServiceRecord release];
        mServiceRecord = nil;
    }
    
    mConnState = connStateDisconnected;
    mDelegateIoReturn = kIOReturnError;
	mConnNotifyReceived = 0;

    return kIOReturnSuccess;
}

/*
-(void)connect:(IOBluetoothDevice*)device
{
	IOReturn ret;
	NSArray* services = [device getServices];
	
	BluetoothRFCOMMChannelID rfcommChannelID = -1;
	for (IOBluetoothSDPServiceRecord* service in services)
    {
		NSLog(@"  Service: %@", [service getServiceName]);
		ret = [service getRFCOMMChannelID:&rfcommChannelID];
		if (ret == kIOReturnSuccess) {
			NSLog(@"ChannelID %d FOUND", (int)rfcommChannelID);
			//break;
		}
	}
	
	//[device openRFCOMMChannelAsync:&channel withChannelID:rfcommChannelID delegate:self];
}
*/
    

- (IOReturn)discover:(NSString*)portName
{
    IOReturn rc = kIOReturnError;
    IOBluetoothDeviceInquiry *inq = [IOBluetoothDeviceInquiry inquiryWithDelegate:self];
    //[inq setSearchCriteria];
    
    NSDate *inqEndDate = [NSDate dateWithTimeIntervalSinceNow:65];
    //[mChannelLock lock]; TODO: rewrite with RunLoop
    [inq start];
    BOOL signaled = [mInqLock waitUntilDate:inqEndDate];
    [mInqLock unlock];
    if(!signaled)
        [inq stop];
    
    //if(signaled)
    {
        NSArray *devices = [inq foundDevices];
        IOBluetoothDevice* dev;
        NSEnumerator *enumerator;
        for(enumerator = [devices objectEnumerator]; dev = [enumerator nextObject]; )
        {
            NSLog(@"Device nameOrAddress=\"%@\", %X %X %X %X  %d %d %d", [dev nameOrAddress],
                  [dev classOfDevice], [dev deviceClassMajor], [dev deviceClassMinor], [dev serviceClassMajor],
                  (int)[dev isConnected], (int)[dev RSSI], (int)[dev rawRSSI]
                  );
            
            IOBluetoothSDPServiceRecord *sr = [dev getServiceRecordForUUID:[IOBluetoothSDPUUID uuid16:kBluetoothSDPUUID16ServiceClassSerialPort]];
            BluetoothRFCOMMChannelID rfcommchannel = -1;
            [sr getRFCOMMChannelID:&rfcommchannel];
            NSLog(@" Servicename=\"%@\", rfcomm channelID=%d", [sr getServiceName], (int)rfcommchannel);
            
            // doesn't work
            if(![dev isPaired])
            {
                IOBluetoothDevicePair *devpair = [IOBluetoothDevicePair pairWithDevice:dev];
                
                BluetoothPINCode pin;
                pin.data[0] = '1';
                pin.data[1] = '2';
                pin.data[2] = '3';
                pin.data[3] = '4';
                
                [devpair replyPINCode:4 PINCode:&pin];
                [devpair start];
            }
        }
    }
    return rc;
}


// IOBluetoothHostControllerDelegate
- (void) controllerClassOfDeviceReverted:(id)sender
{
}

- (void) readRSSIForDeviceComplete:(id)controller device:(IOBluetoothDevice*)device	info:(BluetoothHCIRSSIInfo*)info	error:(IOReturn)error
{
}

- (void) readLinkQualityForDeviceComplete:(id)controller device:(IOBluetoothDevice*)device	info:(BluetoothHCILinkQualityInfo*)info	error:(IOReturn)error
{
}


// IOBluetoothDeviceInquiry
- (void) deviceInquiryStarted:(IOBluetoothDeviceInquiry*)sender
{
}

- (void) deviceInquiryComplete:(IOBluetoothDeviceInquiry*)sender	error:(IOReturn)error	aborted:(BOOL)aborted
{
    [mInqLock signal];
}


// IOBluetoothRFCOMMChannelDelegate
- (void)rfcommChannelData:(IOBluetoothRFCOMMChannel*)rfcommChannel data:(void *)dataPointer length:(size_t)dataLength
{
    [mReceiveRingBuffer write:dataPointer nbyte:dataLength];
}

- (void)rfcommChannelOpenComplete:(IOBluetoothRFCOMMChannel*)rfcommChannel status:(IOReturn)error
{
    if(mConnState == connStateConnecting) // avoid processing later arriving notifications (after timeout)
    {
        mDelegateIoReturn = error;
        if(error == kIOReturnSuccess)
        {
            mConnState = connStateConnected;
        }
        else
        {
            mConnState = connStateDisconnected;
        }
    }
}

- (void)rfcommChannelControlSignalsChanged:(IOBluetoothRFCOMMChannel*)rfcommChannel
{
	if(mConnState == connStateConnected)
	{
		mConnNotifyReceived |= connectionNotifyReceivedSignalsChanged;
	}
}

- (void)rfcommChannelClosed:(IOBluetoothRFCOMMChannel*)rfcommChannel
{
	if(mConnState == connStateConnected)
	{
		mConnNotifyReceived |= connectionNotifyReceivedChannelClosed;
	}
}

- (void)rfcommChannelFlowControlChanged:(IOBluetoothRFCOMMChannel*)rfcommChannel
{
	if(mConnState == connStateConnected)
	{
		mConnNotifyReceived |= connectionNotifyReceivedFlowControlChanged;
	}
}


- (void)rfcommChannelWriteComplete:(IOBluetoothRFCOMMChannel*)rfcommChannel refcon:(void*)refcon status:(IOReturn)error
{
	RFCAsyncWriteRequest *awr = (RFCAsyncWriteRequest *)refcon;
	
	if(awr.mWritePosition > mWritePositionAck)
	{
		mWritePositionAck = awr.mWritePosition;
	}
	if(error != kIOReturnSuccess)
	{
		mConnNotifyReceived |= connectionNotifyReceivedWriteError;
		mDelegateIoReturn = error;
	}
	[awr release];
}


- (void)rfcommChannelQueueSpaceAvailable:(IOBluetoothRFCOMMChannel*)rfcommChannel
{
    
}

// for performSDPQuery, actually not part of any delegate interface?
- (void)sdpQueryComplete:(IOBluetoothDevice *)device status:(IOReturn)status
{
	if(mSdpQueryState == sdpQueryStateRunning)
	{
		mSdpQueryState = sdpQueryStateFinished;
		mDelegateIoReturn = status;
	}

}

@end


void myRunLoopObserver (
                 CFRunLoopObserverRef observer,
                 CFRunLoopActivity activity,
                 void *info
                 )
{
    //RFComm *self = (RFComm*)info;
    
    
}

