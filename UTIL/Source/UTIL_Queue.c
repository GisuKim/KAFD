
#include "f2838x_Device.h"     // DSP2833x Headerfile Include File
#include "UTIL_Queue.h"

void Push_QueueArray(ARRAY_QUEUE_def *i_pstArrQue_Handle, Uint16 *i_uSourceAddr)
{
    Uint8 i;

    i_pstArrQue_Handle->m_uRear = i_pstArrQue_Handle->m_uFront + i_pstArrQue_Handle->m_uCount;

    if(i_pstArrQue_Handle->m_uCount < QUEUE_SIZE)
    {
        for(i =0; i < QUEUE_ARRAY_SIZE; i++)
        {
           i_pstArrQue_Handle->m_auQueue_buffer[ (i_pstArrQue_Handle->m_uRear) % QUEUE_SIZE ][i] = *i_uSourceAddr++;
        }
    i_pstArrQue_Handle->m_uCount++;
    }
}

void Pop_QueueArray(ARRAY_QUEUE_def *i_pstArrQue_Handle, Uint16 *i_puReturnDataBuff)
{
    Uint8 i;
    if(i_pstArrQue_Handle->m_uCount > 0)
    {
        for(i =0; i<QUEUE_ARRAY_SIZE; i++)
        {
           *i_puReturnDataBuff++ = i_pstArrQue_Handle->m_auQueue_buffer[i_pstArrQue_Handle->m_uFront][i];
        }

        i_pstArrQue_Handle->m_uCount--;
        i_pstArrQue_Handle->m_uFront++;

        if(i_pstArrQue_Handle->m_uFront == QUEUE_SIZE)
        {
           i_pstArrQue_Handle->m_uFront = 0;
        }
    }
}



void Push_QueueData(QUEUE_def *i_pstQue_Handle, Uint16 i_uElem)
{
    i_pstQue_Handle->m_uRear = i_pstQue_Handle->m_uFront + i_pstQue_Handle->m_uCount;
    if(i_pstQue_Handle->m_uCount < SINGLE_QUEUE_SIZE)
    {
       i_pstQue_Handle->m_auQueue_buffer[(i_pstQue_Handle->m_uRear)%SINGLE_QUEUE_SIZE] = i_uElem;
       i_pstQue_Handle->m_uCount++;
    }
}


Uint16 Pop_QueueData(QUEUE_def *i_pstQue_Handle)
{
    Uint16 Elem;

    if(i_pstQue_Handle->m_uCount > 0)
    {
        Elem =  i_pstQue_Handle->m_auQueue_buffer[i_pstQue_Handle->m_uFront];
        i_pstQue_Handle->m_uCount--;
        i_pstQue_Handle->m_uFront++;

        if(i_pstQue_Handle->m_uFront == SINGLE_QUEUE_SIZE)
        {
           i_pstQue_Handle->m_uFront = 0;
        }
    }
    else
    {
        Elem = 0;
    }
    return Elem;
}




