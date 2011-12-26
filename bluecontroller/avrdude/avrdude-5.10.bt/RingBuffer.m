//
//  RingBuffer.m
//  RFComm
//
//  Created by Michael Dreher on 18.08.11.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#import "RingBuffer.h"

@implementation RingBuffer
// TODO: implement thread-safe locking for all methods. Currently this is not needed because
// the rfcommChannelData delegate is executed in the same thread 

// Lesezeiger kann Schreibzeiger nicht überholen: mReadPos <= mWritePos
// Wenn Schreibzeiger Lesezeiger überholt (mWritePos >= mReadPos), dann muß Lesezeiger mitgenommen werden => OVERFLOW
// mReadPos = nächstes zu lesendes Zeichen (START)
// mWritePos = nächstes zu schreibende Zeichen (END)


- (id)init
{
    return [self init:0];
}

- (id)init:(size_t)size
{
    if(size <= 0)
        size = 1024 * 8;
    
    self = [super init];
    if (self) {
        mSize = size + 1; // we need one extra slot to distinguish between an empty and a full buffer so we increase the buffer by one
        mBuf = malloc(mSize);
        if(!mBuf)
        {
            [self release];
            return nil;
        }
    }
    [self clear];
    
    return self;
}

- (void)clear
{
    mWritePos = 0;
    mReadPos = 0;
    mOverflow = NO;
}

- (size_t)fillLevel
{
    return (mWritePos - mReadPos) % mSize; // TODO: check this when mWritePos < mReadPos
}

// when buffer is full, overflow flag is set and contents are overwritten
- (size_t)write:(const void *)buf  nbyte:(size_t)nbyte
{
    if(nbyte > (mSize - 1))
    {
        // buffer overflow: only take the last part and throw the rest away (no backpressing / flow control)
        buf += nbyte - (mSize - 1);
        nbyte = (mSize - 1); // maximum capacity
        mOverflow = YES;
    }
    
    size_t newEnd = mWritePos + nbyte; // unwrapped!
    size_t newFill = [self fillLevel] + nbyte; // could by greater than mSize-1 !
    if(newFill >= mSize)
    {
        mOverflow = YES;
        mReadPos = (newEnd + 1) % mSize; // move read pointer in front of write pointer
    }
    if(newEnd > mSize)
    {
        // must split copy in two parts
        size_t copyLenEnd = mSize - mWritePos;
        size_t copyLenBegin = nbyte - copyLenEnd;
        memcpy(mBuf + mWritePos, buf, copyLenEnd);
        memcpy(mBuf, buf+copyLenEnd, copyLenBegin);
    }
    else
    {
        memcpy(mBuf + mWritePos, buf, nbyte);
    }
    mWritePos = (mWritePos + nbyte) % mSize;
    
    return nbyte;
}

// will not block and can never read more than what's currently in the buffer
- (size_t)read:(void *)buf  nbyte:(size_t)nbyte
{
    if(nbyte > [self fillLevel])
        nbyte = [self fillLevel];
    
    size_t newStart = mReadPos + nbyte; // unwrapped!
    if(newStart > mSize)
    {
        // must split copy in two parts
        size_t copyLenEnd = mSize - mReadPos;
        size_t copyLenBegin = nbyte - copyLenEnd;
        memcpy(buf, mBuf + mReadPos, copyLenEnd);
        memcpy(buf+copyLenEnd, mBuf, copyLenBegin);
    }
    else
    {
        memcpy(buf, mBuf + mReadPos, nbyte);
    }
    mReadPos = (mReadPos + nbyte) % mSize;
    mOverflow = NO; // TODO: should be passed back to the caller before clearing
    
    return nbyte;
}


@end


