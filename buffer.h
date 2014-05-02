#ifndef BUFFER_H
#define BUFFER_H

#define BUFF_SIZE 512				// buffer size set at compile time

typedef struct readings{
	int current;
	int voltage;
	int DUTstate;
//	uint8_t DUTstate;//It doesnt know what a uint8_t is.. figure out how to include the def
}readings;

typedef struct buffer{
	readings buff[BUFF_SIZE];
	int readIndex;
	int writeIndex;
	_Bool full;
	_Bool halffull;
	_Bool empty;
//	bool full;
//	bool halffull;
//	bool empty;
}buffer;



/********************************\
* void initBuffer(buffer* buffer)
* initializes buffer, this function is VITAL to correct operation. forgetting to call this function WILL result in memory access violations
* @param buffer* buffer
*	pointer to bufer to be initialized
\********************************/
void initBuffer(buffer* buffer);

/********************************\
* void write(buffer* buffer, float value)
* writes value into the buffer
* @param buffer* buffer
*	pointer to buffer to be used
* @param float value
*	valueto be written in buffer
\********************************/
void writebuffer(buffer* buffer,readings value);

/********************************\
* void read(buffer* buffer)
* reads oldest value from buffer
* @param buffer* buffer
*	pointer to buffer to be read from
\********************************/
readings readbuffer(buffer* buffer);

/********************************\
* void readn(buffer* buffer, int Xn)
* reads specified value from buffer
* @param buffer* buffer
*	pointer to buffer to be read from
* @param int Xn
*	specifies the value to be read from buffer counting backwards from the most recently written value
*	i.e. the most recently writen value can be read with readn(buffer, 0), the value written before that with readn(buffer, 1)
\********************************/
readings readn(buffer* buffer, int Xn);

#endif //BUFFER_H
#if 0
Ideas to expand for my use:
add a flag bit for full
add a flab bit for empty
bool_t full;
bool_t empty;
Read()
	if empty, return zero;
	else
	full = zero (cant be full after a read)
	reading = buffer[readpointer];
moduloway	readpointer= (readpointer++) % buffsize;
if way		readpointer++
		if (readpointer >= buffsize) readpointer =0;
	emptyflag =(readpointer == writepointer)
	return reading;
Write()
	empty = zero (cant be empty after a write)
	buffer[writepointer] = reading;
moduloway	writepointer= (writepointer++) % buffsize;
if way		writepointer++
		if(writepointer >= buffsize) writepointer =0;
	if(full)	

Readall()
	numbertoread= (buffsize - readpointer) + writepointer;
	
#endif
