#pragma once

#include <inttypes.h>
#include <Stream.h>
#include <RingBuffer.h>
#include <RTL_Stdlib.h>
#include <Streaming.h>


class SafePrint : public Print
{
public:
    //virtual int available(void) { return buffer.available(); };
    //
    //virtual int peek(void) { buffer.peek(); };
    //
    //virtual int read(void) { return buffer.available() ? buffer.read_char() : -1; };
  
    virtual int availableForWrite(void) { return buffer.availableForStore(); };
  
    virtual size_t write(uint8_t c) { if (!buffer.isFull()) { buffer.store_char(c); return 1; } return 0; };
    
    virtual size_t write(const uint8_t *data, size_t size) { auto i = 0; while (i < size && write(data[i++])); return i; };
    
    using Print::write; // pull in write(str) from Print

    virtual void flush(void) 
    {
        while (buffer.available() > 0) ConsoleStream.write(buffer.read_char());
    };

private:
    RingBufferN<256> buffer;
};

