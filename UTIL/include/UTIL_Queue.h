#ifndef __UTIL_QUEUE_H__
#define __UTIL_QUEUE_H__

#include "f2838x_Device.h"
//#include "UTIL_1553B_ESICD.h"

#define QUEUE_SIZE              128
#define QUEUE_ARRAY_SIZE        3

#define SINGLE_QUEUE_SIZE       64


typedef struct
{
	Uint16 m_auQueue_buffer[QUEUE_SIZE][QUEUE_ARRAY_SIZE];
	Uint16 m_uFront;
	Uint16 m_uRear;
    Uint16 m_uCount;
    void    (*Push)(); // Pointer to
    Uint16  (*Pop)(); // Pointer to

} ARRAY_QUEUE_def;
typedef ARRAY_QUEUE_def *ARRAY_QUEUE_Handle;


typedef struct
{
	Uint16 m_auQueue_buffer[SINGLE_QUEUE_SIZE];
	Uint16 m_uFront;
	Uint16 m_uRear;
    Uint16 m_uCount;
    void    (*Push)(); // Pointer to
    Uint16  (*Pop)(); // Pointer to

} QUEUE_def;


typedef QUEUE_def *QUEUE_Handle;



/*-----------------------------------------------------------------------------
Default initalizer for the Queue object.
-----------------------------------------------------------------------------*/
#define QUEUE_DEFAULTS { {0,}, \
                            0, \
                            0, \
                            0, \
              			  (void (*)(Uint32))Push_QueueData, \
              			  (Uint16 (*)(Uint32))Pop_QueueData}


#define ARRAY_QUEUE_DEFAULTS { {{0,},}, \
                            0, \
                            0, \
                            0, \
              			  (void (*)(Uint32))Push_QueueArray, \
              			  (Uint16 (*)(Uint32))Pop_QueueArray}



/*------------------------------------------------------------------------------
Prototypes for the functions in Queue.C
------------------------------------------------------------------------------*/
void Push_QueueData(QUEUE_def *i_pstQue_Handle, Uint16 i_uElem);
Uint16 Pop_QueueData(QUEUE_def *i_pstQue_Handle);

void Push_QueueArray(ARRAY_QUEUE_def *i_pstArrQue_Handle, Uint16 *i_uSourceAddr);
void Pop_QueueArray(ARRAY_QUEUE_def *i_pstArrQue_Handle, Uint16 *i_puReturnDataBuff);


#endif
