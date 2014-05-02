#include "buffer.h"
/********************************\
* void initBuffer(buffer* buffer)
* initializes buffer, this function is VITAL to correct operation. forgetting to call this function WILL result in memory access violations
* @param buffer* buffer
*	pointer to bufer to be initialized
\********************************/
void initBuffer(buffer* buffer){
	int i;
	readings zeroout;
	zeroout.current=0;
	zeroout.voltage=0;
	zeroout.DUTstate=0;
	buffer->readIndex = 0;
	buffer->writeIndex = 0;
	buffer->empty = 1;
	buffer->full = 0;
	buffer->halffull=0;
	for(i=0;i<BUFF_SIZE;i++){
		buffer->buff[i]=zeroout;
	}
}
/********************************\
* void write(buffer* buffer, float value)
* writes value into the buffer
* @param buffer* buffer
*	pointer to buffer to be used
* @param float value
*	valueto be written in buffer
\********************************/
void writebuffer(buffer* buffer,readings value){
	buffer->buff[buffer->writeIndex]=value;
	buffer->empty = 0; //cant be empty after a write
	buffer->writeIndex++;
	if(buffer->writeIndex>=BUFF_SIZE)
		buffer->writeIndex=0;
	buffer->full = (buffer->readIndex == buffer->writeIndex);//buffer is full if read == write after a write
	if(buffer->full){
		buffer->readIndex++;
		if(buffer->readIndex>=BUFF_SIZE) //wrap the pointer around if its at the end
			buffer->readIndex=0;
	}
}
/********************************\
* void read(buffer* buffer)
* reads oldest value from buffer
* flags added for overall buffer fill states
* 
* @param buffer* buffer
*	pointer to buffer to be read from
\********************************/
readings readbuffer(buffer* buffer){
	readings temp;
	if(buffer->empty){//TODO Make this a more recognizable code
		temp.current=0;
		temp.voltage=0;
		temp.DUTstate=0;
		return temp; //if buffer is empty return a empty reading
	}
	temp=buffer->buff[buffer->readIndex];
	buffer->full = 0; //cant be full after a read
	buffer->readIndex++;
	if(buffer->readIndex>=BUFF_SIZE) //wrap the pointer around if its at the end
		buffer->readIndex=0;
	buffer->empty = (buffer->readIndex == buffer->writeIndex);//buffer is empty if read == write after a read
	return temp;
}
/********************************\
* void readn(buffer* buffer, int Xn)
* reads specified value from buffer
* @param buffer* buffer
*	pointer to buffer to be read from
* @param int Xn
*	specifies the value to be read from buffer counting backwards from the most recently written value
*	i.e. the most recently writen value can be read with readn(buffer, 0), the value written before that with readn(buffer, 1)
\********************************/
readings readn(buffer* buffer, int Xn){
	int tempIndex;
	
	tempIndex=buffer->writeIndex-(Xn+1);
	while(tempIndex<0){
		tempIndex+=BUFF_SIZE;
	}

	return buffer->buff[tempIndex];
}
