//
//  RingBuffer.h
//  RFComm
//
//  Created by Michael Dreher on 18.08.11.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#import "RFComm-Prefix.h"
//#import <Foundation/Foundation.h>

@interface RingBuffer : NSObject
{
    uint8_t* mBuf;
    size_t mSize;
    size_t mWritePos;
    size_t mReadPos;
    bool mOverflow;
}

- (id)init;
- (id)init:(size_t)size;
- (size_t)write:(const void *)buf  nbyte:(size_t)nbyte;
- (size_t)read:(void *)buf  nbyte:(size_t)nbyte;
- (size_t)fillLevel;
- (void)clear;

@end
