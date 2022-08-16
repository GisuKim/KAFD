/* ==================================================================================
File name:       F280XBLDCPWM.C

Originator:	Digital Control Systems Group
			Texas Instruments

Description:   This file contains source for the Full Compare BLDC PWM drivers for the F280x

Target: TMS320F280x family

=====================================================================================
History:
-------------------------------------------------------------------------------------
 04-15-2005	Version 3.20: Using DSP280x v. 1.10 or higher
------------------------------------------------------------------------------------*/
#include "f2838x_Device.h"     // DSP2833x Headerfile Include File
#include "UTIL_Queue.h"

/*------------------------------------------------------------------------
DATE : 2015-04-30
AUTHOR : SUNG-JUN PARK
--------------------------------------------------------------------------
���� �Լ� :
    void Push_QueueArray(ARRAY_QUEUE_def *i_pstArrQue_Handle, Uint16 *i_uSourceAddr)
--------------------------------------------------------------------------
Etc.
    - �����ϴ� ������ �������� ��� ó�� �Ұ�����.
      �ּҰ� ������ �ƴ� �迭 ������ ������ �ҽ��ڵ带 ��ó�Ͽ� �����Ͽ� �ҽ��ڵ带 ������ ������
------------------------------------------------------------------------
INPUT :                                                                       INPUT RANGE               INPUT
    1.      i_pstArrQue_Handle->m_uCount                                      [ 0 ~ 2 ]                 [0]
    2.      i_pstArrQue_Handle->m_uFront                                      [ 0 ~ 1 ]                 [0]
    3.      *i_uSourceAddr                                                    [ 0 ~ 65535 ]             [ALL ARRANGE VALUE : 1]
------------------------------------------------------------------------
OUTPUT :                                                                      OUTPUT RANGE              OUTPUT
    1.      i_pstArrQue_Handle->m_uCount                                      [ 0 ~ 2 ]                 [1]
    2.      i_pstArrQue_Handle->m_uRear                                       [ 0 ~ 3 ]                 [1]
    3.      m_auQueue_buffer[0~1][0~59]                                       [ 0 ~ 65535 ]             [ALL ARRANGE VALUE : 1]
------------------------------------------------------------------------*/
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

/*------------------------------------------------------------------------
DATE : 2015-04-30
AUTHOR : SUNG-JUN PARK
--------------------------------------------------------------------------
���� �Լ� :
    void Pop_QueueArray(ARRAY_QUEUE_def *i_pstArrQue_Handle, Uint16 *i_puReturnDataBuff)
--------------------------------------------------------------------------
Etc.
    - �����ϴ� ������ �������� ��� ó�� �Ұ�����.
      �ּҰ� ������ �ƴ� �迭 ������ ������ �ҽ��ڵ带 ��ó�Ͽ� �����Ͽ� �ҽ��ڵ带 ������ ������
------------------------------------------------------------------------
INPUT :                                                                 INPUT RANGE         INPUT
    1.      i_pstArrQue_Handle->m_uCount                                [ 0 ~ 2 ]           [ 1 ]
    2.      i_pstArrQue_Handle->m_uFront                                [ 0 ~ 1 ]           [ 0 ]
    3.      i_pstArrQue_Handle->m_auQueue_buffer[0][0~59]               [ 0 ~ 65535 ]       [ ALL ARRANGE VALUE : 1 ]
------------------------------------------------------------------------
OUTPUT :                                                                OUTPUT RANGE        OUTPUT
    1.      *i_puReturnDataBuff                                         [ 0 ~ 65535 ]       [ ALL ARRANGE VALUE : 1 ]
    2.      i_pstArrQue_Handle->m_uCount                                [ 0 ~ 2 ]           [ 0 ]
    3.      i_pstArrQue_Handle->m_uFront                                [ 0 ~ 1 ]           [ 1 ]
------------------------------------------------------------------------*/
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



/*------------------------------------------------------------------------
DATE : 2015-04-30
AUTHOR : SUNG-JUN PARK
--------------------------------------------------------------------------
���� �Լ� :
    void Push_QueueData(QUEUE_def *i_pstQue_Handle, Uint16 i_uElem)
--------------------------------------------------------------------------
Etc.
    - �����ϴ� ������ �������� ��� ó�� �Ұ�����.
      �ּҰ� ������ �ƴ� �迭 ������ ������ �ҽ��ڵ带 ��ó�Ͽ� �����Ͽ� �ҽ��ڵ带 ������ ������
------------------------------------------------------------------------
INPUT :                                                                                         INPUT RANGE         INPUT
    1.      i_pstArrQue_Handle->m_uCount                                                        [ 0 ~ 2 ]           [ 1 ]
    2.      i_pstArrQue_Handle->m_uFront                                                        [ 0 ~ 65 ]          [ 0 ]
    3.      i_uElem                                                                             [ 0 ~ 65535 ]       [ 1 ]
------------------------------------------------------------------------
OUTPUT :                                                                                        OUTPUT RANGE        OUTPUT
    1.      i_pstQue_Handle->m_auQueue_buffer[(i_pstQue_Handle->m_uRear)%SINGLE_QUEUE_SIZE]     [ 0 ~ 65535 ]       [ ALL ARRANGE VALUE : 1 ]
    2.      i_pstArrQue_Handle->m_uCount                                                        [ 0 ~ 64 ]          [ 2 ]
------------------------------------------------------------------------*/
void Push_QueueData(QUEUE_def *i_pstQue_Handle, Uint16 i_uElem)
{
    i_pstQue_Handle->m_uRear = i_pstQue_Handle->m_uFront + i_pstQue_Handle->m_uCount;
    if(i_pstQue_Handle->m_uCount < SINGLE_QUEUE_SIZE)
    {
       i_pstQue_Handle->m_auQueue_buffer[(i_pstQue_Handle->m_uRear)%SINGLE_QUEUE_SIZE] = i_uElem;
       i_pstQue_Handle->m_uCount++;
    }
}

/*------------------------------------------------------------------------
DATE : 2015-04-30
AUTHOR : SUNG-JUN PARK
--------------------------------------------------------------------------
���� �Լ� :
    Uint16 Pop_QueueData(QUEUE_def *i_pstQue_Handle)
--------------------------------------------------------------------------
Etc.
    - �����ϴ� ������ �������� ��� ó�� �Ұ�����.
      �ּҰ� ������ �ƴ� �迭 ������ ������ �ҽ��ڵ带 ��ó�Ͽ� �����Ͽ� �ҽ��ڵ带 ������ ������
------------------------------------------------------------------------
INPUT :                                                                     INPUT RANGE         INPUT
    1-1.    i_pstQue_Handle->m_uCount                                       [ 0 ~ 2 ]           [ 1 ]
    1-2.    i_pstQue_Handle->m_uCount                                       [ 0 ~ 2 ]           [ 0 ]
    2.      i_pstQue_Handle->m_uFront                                       [ 0 ~ 65 ]          [ 65 ]
    3.      i_pstQue_Handle->m_auQueue_buffer[0]                            [ 0 ~ 65535 ]       [ 0 ]
------------------------------------------------------------------------
OUTPUT :                                                                    OUTPUT RANGE        OUTPUT
    1-1.    uElem                                                           [ 0 ~ 65535 ]       [ 0 ]
    1-2.    uElem                                                           [ 0 ~ 65535 ]       [ 0 ]
    2.      i_pstQue_Handle->m_uCount                                       [ 0 ~ 2 ]           [ 0 ]
    3.      i_pstQue_Handle->m_uFront                                       [ 0 ~ 65 ]          [ 0 ]
------------------------------------------------------------------------*/
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




