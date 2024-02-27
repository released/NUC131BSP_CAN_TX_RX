/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include <string.h>
#include "NUC131.h"

#include "misc_config.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

#define PLL_CLOCK                                       (48000000)

struct flag_32bit flag_PROJ_CTL;
#define FLAG_PROJ_TIMER_PERIOD_1000MS                 	(flag_PROJ_CTL.bit0)
#define FLAG_PROJ_CAN_TX                      			(flag_PROJ_CTL.bit1)
#define FLAG_PROJ_REVERSE2                 				(flag_PROJ_CTL.bit2)
#define FLAG_PROJ_REVERSE3                              (flag_PROJ_CTL.bit3)
#define FLAG_PROJ_REVERSE4                              (flag_PROJ_CTL.bit4)
#define FLAG_PROJ_REVERSE5                              (flag_PROJ_CTL.bit5)
#define FLAG_PROJ_REVERSE6                              (flag_PROJ_CTL.bit6)
#define FLAG_PROJ_REVERSE7                              (flag_PROJ_CTL.bit7)


#define SYS_GPC_MFP_PC15_GPIO                           (0x00000000UL)
#define SYS_GPC_MFP_PC15_Msk                            (1UL<<15)

#define CAN_MASK_MSG_DIR    (0x1ul << 30) /*!< CAN mask direction bit \hideinitializer */
#define CAN_MASK_EXT_ID_BIT (0x1ul << 31) /*!< CAN mask extended id bit \hideinitializer */
/**
 * @brief Specifies the standard identifier mask used for acceptance filtering
 *
 * @param[in] mask_bit The standard id mask bits.
 *
 * @return Mask ID bit.
 *
 *  \hideinitializer
 */
#define CAN_STD_ID_MASK(mask_bit) (mask_bit << 18)

/**
 * @brief Specifies the extended identifier mask used for acceptance filtering
 *
 * @param[in] mask_bit The extended id mask bits.
 *
 * @return Mask ID bit.
 *
*  \hideinitializer
 */
#define CAN_EXT_ID_MASK(mask_bit) (mask_bit)

#define RX_OK_COUNT             10

#define BAUDRATE_MIN            50000
#define BAUDRATE_MAX            1000000
#define BAUDRATE_OFFSET         50000


#define CAN_ON_INT              1
#define CAN_RX_OK_INT           2
#define CAN_TX_OK_INT           3
#define CAN_EWARN_INT           4
#define CAN_BOFF_INT            5

#define CAN_ID_TX                                       (0x520)
#define MSG_ID_TX                                       (MSG(4))

#define CAN_ID_RX1                                      (0x612)
#define MSG_ID_RX1                                      (MSG(5))

#define CAN_ID_RX2                                      (0x707)
#define MSG_ID_RX2                                      (MSG(31))

/*_____ D E F I N I T I O N S ______________________________________________*/

volatile unsigned int counter_systick = 0;
volatile uint32_t counter_tick = 0;

uint32_t RealBaudRate;	

uint8_t canbus_INT_flag = 0x00;
uint8_t BUSrecovery = 0x00;
uint8_t TEC, REC;

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
extern int32_t CAN_SetRxMsgObjAndMsk(CAN_T *tCAN, uint8_t u8MsgObj, uint8_t u8idType, uint32_t u32id, uint32_t u32idmask, uint8_t u8singleOrFifoLast);


unsigned int get_systick(void)
{
	return (counter_systick);
}

void set_systick(unsigned int t)
{
	counter_systick = t;
}

void systick_counter(void)
{
	counter_systick++;
}

void SysTick_Handler(void)
{

    systick_counter();

    if (get_systick() >= 0xFFFFFFFF)
    {
        set_systick(0);      
    }

    // if ((get_systick() % 1000) == 0)
    // {
       
    // }

    #if defined (ENABLE_TICK_EVENT)
    TickCheckTickEvent();
    #endif    
}

void SysTick_delay(unsigned int delay)
{  
    
    unsigned int tickstart = get_systick(); 
    unsigned int wait = delay; 

    while((get_systick() - tickstart) < wait) 
    { 
    } 

}

void SysTick_enable(unsigned int ticks_per_second)
{
    set_systick(0);
    if (SysTick_Config(SystemCoreClock / ticks_per_second))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        printf("Set system tick error!!\n");
        while (1);
    }

    #if defined (ENABLE_TICK_EVENT)
    TickInitTickEvent();
    #endif
}

uint32_t get_tick(void)
{
	return (counter_tick);
}

void set_tick(uint32_t t)
{
	counter_tick = t;
}

void tick_counter(void)
{
	counter_tick++;
    if (get_tick() >= 60000)
    {
        set_tick(0);
    }
}

// void delay_ms(uint16_t ms)
// {
// 	TIMER_Delay(TIMER0, 1000*ms);
// }


void Busoff_Recovery(void)
{
    if(canbus_INT_flag == CAN_BOFF_INT) {
        printf("\nBOFF - Main\n");

        /*   bus-off recovery check*/
        #if 1
        BUSrecovery = 1;
        #else
        if(PA0 == 0) {
            BUSrecovery = 1;
        }
        #endif

        if(BUSrecovery != 0) {
            CAN0->CON &= (~(CAN_CON_INIT_Msk | CAN_CON_CCE_Msk));
            while(CAN0->CON & CAN_CON_INIT_Msk);
            while((CAN0->STATUS & CAN_STATUS_LEC_Msk) == 0x05) {
                if(CAN0->ERR ==0) {
                    CAN_EnableInt(CAN0, CAN_CON_IE_Msk|CAN_CON_SIE_Msk|CAN_CON_EIE_Msk);
                    canbus_INT_flag = 0x00;
                    BUSrecovery = 0;
                    break;
                }
            }
        }
    }    
}

static uint32_t GetFreeIF(CAN_T *tCAN)
{
    /* Check Read/write action has finished */
    if((tCAN->IF[0].CREQ & CAN_IF_CREQ_BUSY_Msk) == 0)
        return 0;
    else if((tCAN->IF[1].CREQ  & CAN_IF_CREQ_BUSY_Msk) == 0)
        return 1;
    else
        return 2;
}

int32_t SetMsgObjMask(CAN_T *tCAN, uint8_t u8MsgObj, STR_CANMASK_T* MaskMsg)
{
    /* Set the Message Buffer Register */
    uint8_t u8MsgIfNum = 0;

    /* Check Free Interface for configure */
    if((u8MsgIfNum = GetFreeIF(tCAN)) == 2)
    {
        return FALSE;
    }

    if(MaskMsg->u8IdType == CAN_STD_ID)
    {
        /* Set the Mask Standard ID(11-bit) for IFn Mask Register is used for acceptance filtering*/
        tCAN->IF[u8MsgIfNum].MASK1 =  0;
        tCAN->IF[u8MsgIfNum].MASK2 = ((MaskMsg->u32Id & 0x7FF) << 2) ;
    }
    else
    {
        /* Set the Mask Extended ID(29-bit) for IFn Mask Register is used for acceptance filtering*/
        tCAN->IF[u8MsgIfNum].MASK1 = (MaskMsg->u32Id) & 0xFFFF;
        tCAN->IF[u8MsgIfNum].MASK2 = ((MaskMsg->u32Id) & 0x1FFF0000) >> 16 ;
    }

    if(MaskMsg->u8Xtd)
        tCAN->IF[u8MsgIfNum].MASK2 |= CAN_IF_MASK2_MXTD_Msk;            /* The extended identifier bit (IDE) is used for acceptance filtering */
    else
        tCAN->IF[u8MsgIfNum].MASK2 &= (~CAN_IF_MASK2_MXTD_Msk);  /* The extended identifier bit (IDE) has no effect on the acceptance filtering */

    if(MaskMsg->u8Dir)
        tCAN->IF[u8MsgIfNum].MASK2 |= CAN_IF_MASK2_MDIR_Msk;     /* The message direction bit (Dir) is used for acceptance filtering */
    else
        tCAN->IF[u8MsgIfNum].MASK2 &= (~CAN_IF_MASK2_MDIR_Msk);  /* The message direction bit (Dir) has no effect on the acceptance filtering */

    tCAN->IF[u8MsgIfNum].MCON |= CAN_IF_MCON_UMASK_Msk;                 /* Use Mask (Msk28-0, MXtd, and MDir) for acceptance filtering */

    /* Update the contents needed for transmission*/
    tCAN->IF[u8MsgIfNum].CMASK = CAN_IF_CMASK_WRRD_Msk      /* Transfer data from the selected Message Buffer Registers to the Message Object addressed */
                                 | CAN_IF_CMASK_MASK_Msk;     /* Transfer Identifier Mask + MDir + MXtd to Message Object  */

    /* Set the Message Object in the Message RAM is selected for data transfer */
    tCAN->IF[u8MsgIfNum].CREQ  = 1 + u8MsgObj;

    return TRUE;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Set Rx message object                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
int32_t SetRxMsgObj(CAN_T  *tCAN, uint8_t u8MsgObj, uint8_t u8idType, uint32_t u32id, uint8_t u8singleOrFifoLast)
{
    uint8_t u8MsgIfNum = 0;

    if((u8MsgIfNum = GetFreeIF(tCAN)) == 2)                         /* Check Free Interface for configure */
    {
        return FALSE;
    }
    /* Command Setting */
    tCAN->IF[u8MsgIfNum].CMASK = CAN_IF_CMASK_WRRD_Msk | CAN_IF_CMASK_MASK_Msk | CAN_IF_CMASK_ARB_Msk |
                                 CAN_IF_CMASK_CONTROL_Msk | CAN_IF_CMASK_DATAA_Msk | CAN_IF_CMASK_DATAB_Msk;

    if(u8idType == CAN_STD_ID)    /* According STD/EXT ID format,Configure Mask and Arbitration register */
    {
        tCAN->IF[u8MsgIfNum].ARB1 = 0;
        tCAN->IF[u8MsgIfNum].ARB2 = CAN_IF_ARB2_MSGVAL_Msk | (u32id & 0x7FF) << 2;
    }
    else
    {
        tCAN->IF[u8MsgIfNum].ARB1 = u32id & 0xFFFF;
        tCAN->IF[u8MsgIfNum].ARB2 = CAN_IF_ARB2_MSGVAL_Msk | CAN_IF_ARB2_XTD_Msk | (u32id & 0x1FFF0000) >> 16;
    }

    tCAN->IF[u8MsgIfNum].MCON |= CAN_IF_MCON_UMASK_Msk | CAN_IF_MCON_RXIE_Msk;
    if(u8singleOrFifoLast)
        tCAN->IF[u8MsgIfNum].MCON |= CAN_IF_MCON_EOB_Msk;
    else
        tCAN->IF[u8MsgIfNum].MCON &= (~CAN_IF_MCON_EOB_Msk);

    tCAN->IF[u8MsgIfNum].DAT_A1  = 0;
    tCAN->IF[u8MsgIfNum].DAT_A2  = 0;
    tCAN->IF[u8MsgIfNum].DAT_B1  = 0;
    tCAN->IF[u8MsgIfNum].DAT_B2  = 0;

    tCAN->IF[u8MsgIfNum].CREQ = 1 + u8MsgObj;

    return TRUE;
}


/**
  * @brief The function is used to configure a receive message object.
  *
  * @param[in] tCAN The pointer to CAN module base address.
  * @param[in] u32MsgNum Specifies the Message object number, from 0 to 31.
  * @param[in] u32IDType Specifies the identifier type of the frames that will be transmitted. Valid values are:
  *                      - \ref CAN_STD_ID The 11-bit identifier.
  *                      - \ref CAN_EXT_ID The 29-bit identifier.
  * @param[in] u32ID Specifies the identifier used for acceptance filtering.
  * @param[in] u32IDMask Specifies the identifier mask used for acceptance filtering.
  *
  * @retval FALSE No useful interface.
  * @retval TRUE Configure a receive message object success.
  *
  * @details If the RxIE bit (CAN_IFn_MCON[10]) is set, the IntPnd bit (CAN_IFn_MCON[13])
  *          will be set when a received Data Frame is accepted and stored in the Message Object.
  */
int32_t CAN_SetRxMsgAndMsk(CAN_T *tCAN, uint32_t u32MsgNum, uint32_t u32IDType, uint32_t u32ID, uint32_t u32IDMask)
{
    int32_t  rev = (int32_t)TRUE;
    uint32_t u32TimeOutCount = 0ul;
    uint32_t RETRY_COUNTS = 0x10000000ul;

    while(CAN_SetRxMsgObjAndMsk(tCAN, (uint8_t)u32MsgNum, (uint8_t)u32IDType, u32ID, u32IDMask, (uint8_t)TRUE) == (int32_t)FALSE)
    {
        if(++u32TimeOutCount >= RETRY_COUNTS)
        {
            rev = (int32_t)FALSE;
            break;
        }
    }

    return rev;
}

void CAN_ShowMsg(STR_CANMSG_T* Msg)
{
    uint8_t i;

    printf("(aa)Read ID=0x%08X, Type=%s, DLC=%d,Data=",Msg->Id,Msg->IdType?"EXT":"STD",Msg->DLC);

    for(i=0; i<Msg->DLC; i++)
        printf("0x%02X,",Msg->Data[i]);
    printf("\r\n");
}

void CAN_MsgInterrupt(CAN_T *tCAN, uint32_t u32IIDR)
{
    uint8_t msgID = u32IIDR-1;
    STR_CANMSG_T rrMsg;
    static uint32_t cnt = 0;

    printf("msgID : %d(0x%02X,0x%08X),cnt=%d\r\n",msgID ,msgID ,u32IIDR , cnt++);
    if ((msgID == 0 ) |
        (msgID == 5 ) | 
        (msgID == 31) )
    {
        printf("Msg-%d INT and Callback\r\n" , msgID);
        CAN_Receive(tCAN, MSG(msgID), &rrMsg);
        CAN_ShowMsg(&rrMsg);

        printf("\r\n");
    }
}

void CAN0_IRQHandler(void)
{
    uint32_t u8IIDRstatus;

    u8IIDRstatus = CAN0->IIDR;

    if(u8IIDRstatus == 0x00008000)        /* Check Status Interrupt Flag (Error status Int and Status change Int) */
    {
        /**************************/
        /* Status Change interrupt*/
        /**************************/
        if(CAN0->STATUS & CAN_STATUS_RXOK_Msk)
        {
            CAN0->STATUS &= ~CAN_STATUS_RXOK_Msk;   /* Clear RxOK status*/

            // printf("RX OK INT(0x%08X)\n",u8IIDRstatus) ;
            printf("RX OK INT\n") ;
            while(!UART_IS_TX_EMPTY(UART0));

            canbus_INT_flag = CAN_RX_OK_INT;
        }

        if(CAN0->STATUS & CAN_STATUS_TXOK_Msk)
        {
            CAN0->STATUS &= ~CAN_STATUS_TXOK_Msk;    /* Clear TxOK status*/

            printf("TX OK INT\n") ;

            canbus_INT_flag = CAN_TX_OK_INT;
        }

        /**************************/
        /* Error Status interrupt */
        /**************************/
        if(CAN0->STATUS & CAN_STATUS_BOFF_Msk)
        {
            CAN0->STATUS &= ~CAN_STATUS_BOFF_Msk;

            printf("BOFF INT\n") ;

            canbus_INT_flag = CAN_BOFF_INT;
        }
        else if(CAN0->STATUS & CAN_STATUS_EWARN_Msk)
        {
            CAN0->STATUS &= ~CAN_STATUS_EWARN_Msk;

            printf("EWARN INT\n") ;
            
            TEC = (uint8_t)(CAN0->ERR &0xFF);
            REC = (uint8_t)(((CAN0->ERR)>>8) &0x7F);
            printf("TEC = %d, REC = %d\n", TEC,REC);

            canbus_INT_flag = CAN_EWARN_INT;
        }
        else if((CAN0->ERR & CAN_ERR_TEC_Msk) != 0)
        {
            printf("Transmit error!\n") ;
        }
        else if((CAN0->ERR & CAN_ERR_REC_Msk) != 0)
        {
            printf("Receive error!\n") ;
        }
    }
    else if((u8IIDRstatus >= 0x1) || (u8IIDRstatus <= 0x20))
    {
        CAN_MsgInterrupt(CAN0, u8IIDRstatus);

        CAN_CLR_INT_PENDING_BIT(CAN0, (u8IIDRstatus - 1)); /* Clear Interrupt Pending */
    }
    else if(CAN0->WU_STATUS == 1)
    {
        printf("Wake up\n");

        CAN0->WU_STATUS = 0;    /* Write '0' to clear */
    }

}

void CAN_RS_PIN_Init(void)  // CAN transceiver RS pin (#8) PULL LOW
{
    PC15 = 0;
    SYS->GPC_MFP &= ~(SYS_GPC_MFP_PC15_Msk);
    SYS->GPC_MFP |= (SYS_GPC_MFP_PC15_GPIO);
	
    GPIO_SetMode(PC, BIT15, GPIO_PMD_OUTPUT);	

    PC->DOUT &= ~0x8000 ;                                  // Enable CAN transceiver

}

void CAN_SetRXFilter(void)
{    
    /* Declare a CAN message structures */
    STR_CANMASK_T MaskMsg;

    CAN_SetRxMsg(CAN0, MSG_ID_RX1, CAN_STD_ID, CAN_ID_RX1);
    CAN_SetRxMsg(CAN0, MSG_ID_RX2, CAN_STD_ID, CAN_ID_RX2);

    /*
        if ID bit0/bit1/... is zero , will ignore , and receive the ID data

        0x7FF
        8 4 2 1     8 4 2 1     8 4 2 1
        0 1 1 1     1 1 1 1     1 1 1 1 

        0x7FE , means allow to receive 0x7FE , 0x7FF
        0 1 1 1     1 1 1 1     1 1 1 0

        0x7FC , means allow to receive 0x7FC ,0x7FD , 0x7FE , 0x7FF
        0 1 1 1     1 1 1 1     1 1 0 0
    */
    // CAN_SetRxMsgAndMsk(CAN0, MSG_ID_RX2, CAN_STD_ID, 0x7FF , 0x7F0 | CAN_MASK_MSG_DIR);   
    // CAN_SetRxMsgAndMsk(CAN0, MSG_ID_RX2, CAN_STD_ID, 0x7FF , 0x7FC | CAN_MASK_MSG_DIR);  
    

    #if 0
    /*
        Mask Extended Identifier 
        0 = The extended identifier bit (IDE) has no effect on the acceptance filtering. 
        1 = The extended identifier bit (IDE) is used for acceptance filtering. 
    */
    MaskMsg.u8Xtd    = 1;                           /* 1: 29-bit Extended Identifier or 0:11-bit Standard Identifier */
    /*
        Mask Message Direction 
        0 = The message direction bit (Dir (CAN_IFn_ARB2[13])) has no effect on the acceptance 
        filtering.             
        1 = The message direction bit (Dir) is used for acceptance filtering. 
    */
    MaskMsg.u8Dir    = 1;                           /* 1:Direction is transmit or 0:Direction is receive*/
    MaskMsg.u8IdType = CAN_STD_ID;                  /* 1: 29-bit Extended Identifier or 0:11-bit Standard Identifier */
    MaskMsg.u32Id    = CAN_ID_RX2;                  /* Set the Message Identifier  */

    /* Configures Mask as the message object */
    SetMsgObjMask(CAN0, MSG_ID_RX2, &MaskMsg);

    /* Set Rx message object */
    SetRxMsgObj(CAN0, MSG(0), CAN_STD_ID, 0x7FF , TRUE);

    #endif
}

void CAN_Init(void)
{    
    SYS_ResetModule(CAN0_RST);		// Reset CAN	
	RealBaudRate = CAN_Open(CAN0, 500000, CAN_NORMAL_MODE);
    printf("CAN baud rate:%4d\r\n",RealBaudRate);

	// CAN_EnableInt(CAN0, CAN_CON_IE_Msk|CAN_CON_SIE_Msk|CAN_CON_EIE_Msk);      /* Enable CAN interrupt and corresponding NVIC of CAN */
    CAN_EnableInt(CAN0, CAN_CON_IE_Msk|CAN_CON_SIE_Msk);      /* Enable CAN interrupt and corresponding NVIC of CAN */  
    NVIC_SetPriority(CAN0_IRQn, (1 << __NVIC_PRIO_BITS) - 2);   /* Install CAN call back functions */
    NVIC_EnableIRQ(CAN0_IRQn);
	
	CAN_SetRXFilter();
}

void CAN_Sender(uint32_t can_id , uint8_t* can_data , uint8_t dlc)
{
    STR_CANMSG_T msg1;
    uint8_t i = 0;
 
    msg1.FrameType  = CAN_DATA_FRAME;
    msg1.IdType     = CAN_STD_ID;
    msg1.Id         = can_id;
    msg1.DLC        = dlc;

    for(i = 0 ; i < dlc; i++)
    {
        msg1.Data[i] = can_data[i];
    }
    
    if (CAN_Transmit(CAN0, MSG_ID_TX , &msg1) == FALSE)
    {
        printf("set tx msg failed\r\n");
        return;
    }
}


uint8_t CAN_data_packing(void)
{
    uint8_t can_data[8] = {0};
    uint8_t i = 0;

    __IO static uint8_t cnt = 0;

    can_data[0] = 0x5A;
    can_data[1] = 0x5A;

    can_data[2] = cnt;

    can_data[6] = 0xA5;
    can_data[7] = 0xA5;

    printf("%s start\r\n",__FUNCTION__);

    for ( i = 0 ; i < 8 ; i++)
    {
        printf("0x%02X,",can_data[i]);
    }
    printf("\r\n");

    CAN_Sender(CAN_ID_TX,can_data,8);

    cnt++;

    while(!UART_IS_TX_EMPTY(UART0));
    printf("%s end\r\n",__FUNCTION__);

    return 1;
}


void TMR1_IRQHandler(void)
{
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
		tick_counter();

		if ((get_tick() % 1000) == 0)
		{
            FLAG_PROJ_TIMER_PERIOD_1000MS = 1;//set_flag(flag_timer_period_1000ms ,ENABLE);
		}

		if ((get_tick() % 50) == 0)
		{

		}	
    }
}

void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void loop(void)
{
	// static uint32_t LOG1 = 0;
	// static uint32_t LOG2 = 0;

    if ((get_systick() % 1000) == 0)
    {
        // printf("%s(systick) : %4d\r\n",__FUNCTION__,LOG2++);    
    }

    if (FLAG_PROJ_TIMER_PERIOD_1000MS)//(is_flag_set(flag_timer_period_1000ms))
    {
        FLAG_PROJ_TIMER_PERIOD_1000MS = 0;//set_flag(flag_timer_period_1000ms ,DISABLE);

        // printf("%s(timer) : %4d\r\n",__FUNCTION__,LOG1++);
        PB14 ^= 1;        
    }

    if (FLAG_PROJ_CAN_TX)
    {
        FLAG_PROJ_CAN_TX = 0;
        CAN_data_packing();
    }

    Busoff_Recovery();
}

void UARTx_Process(void)
{
	uint8_t res = 0;
	res = UART_READ(UART0);

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
		printf("press : %c\r\n" , res);
		switch(res)
		{
			case '1':
                FLAG_PROJ_CAN_TX = 1;
				break;

			case 'X':
			case 'x':
			case 'Z':
			case 'z':
                SYS_UnlockReg();
				// NVIC_SystemReset();	// Reset I/O and peripherals , only check BS(FMC_ISPCTL[1])
                // SYS_ResetCPU();     // Not reset I/O and peripherals
                SYS_ResetChip();    // Reset I/O and peripherals ,  BS(FMC_ISPCTL[1]) reload from CONFIG setting (CBS)	
				break;
		}
	}
}

void UART02_IRQHandler(void)
{
    if(UART_GET_INT_FLAG(UART0, UART_ISR_RDA_INT_Msk | UART_ISR_TOUT_IF_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
			UARTx_Process();
        }
    }

    if(UART0->FSR & (UART_FSR_BIF_Msk | UART_FSR_FEF_Msk | UART_FSR_PEF_Msk | UART_FSR_RX_OVER_IF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_ISR_RLS_INT_Msk| UART_ISR_BUF_ERR_INT_Msk));
    }	
}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
    UART_EnableInt(UART0, UART_IER_RDA_IEN_Msk | UART_IER_TOUT_IEN_Msk);
    NVIC_EnableIRQ(UART02_IRQn);
	
	#if (_debug_log_UART_ == 1)	//debug
	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHCLKFreq : %8d\r\n",CLK_GetHCLKFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetPCLKFreq : %8d\r\n",CLK_GetPCLKFreq());	
	#endif
	
    #if 0
    printf("FLAG_PROJ_TIMER_PERIOD_1000MS : 0x%2X\r\n",FLAG_PROJ_TIMER_PERIOD_1000MS);
    printf("FLAG_PROJ_REVERSE1 : 0x%2X\r\n",FLAG_PROJ_REVERSE1);
    printf("FLAG_PROJ_REVERSE2 : 0x%2X\r\n",FLAG_PROJ_REVERSE2);
    printf("FLAG_PROJ_REVERSE3 : 0x%2X\r\n",FLAG_PROJ_REVERSE3);
    printf("FLAG_PROJ_REVERSE4 : 0x%2X\r\n",FLAG_PROJ_REVERSE4);
    printf("FLAG_PROJ_REVERSE5 : 0x%2X\r\n",FLAG_PROJ_REVERSE5);
    printf("FLAG_PROJ_REVERSE6 : 0x%2X\r\n",FLAG_PROJ_REVERSE6);
    printf("FLAG_PROJ_REVERSE7 : 0x%2X\r\n",FLAG_PROJ_REVERSE7);
    #endif

}

void GPIO_Init (void)
{
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB14_Msk | SYS_GPB_MFP_PB14_Msk);
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB14_GPIO | SYS_GPB_MFP_PB15_GPIO);
	
    GPIO_SetMode(PB, BIT14, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PB, BIT15, GPIO_PMD_OUTPUT);	
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);


    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    // CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    // CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);
    // CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);
    CLK_SetCoreClock(PLL_CLOCK);
    CLK_WaitClockReady(CLK_CLKSTATUS_PLL_STB_Msk);

    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HIRC, CLK_CLKDIV_UART(1));

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD);

    CLK_EnableModuleClock(TMR0_MODULE);
  	CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HIRC, 0);

    CLK_EnableModuleClock(TMR1_MODULE);
  	CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1_S_HIRC, 0);

    CLK_EnableModuleClock(CAN0_MODULE);
    /* Set PD multi-function pins for CANTX0 and CANRX0 */
    SYS->GPD_MFP &= ~(SYS_GPD_MFP_PD6_Msk | SYS_GPD_MFP_PD7_Msk);
    SYS->GPD_MFP |= SYS_GPD_MFP_PD6_CAN0_RXD | SYS_GPD_MFP_PD7_CAN0_TXD;

   /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

int main()
{
    SYS_Init();

	GPIO_Init();
	UART0_Init();
	TIMER1_Init();

    SysTick_enable(1000);
    #if defined (ENABLE_TICK_EVENT)
    TickSetTickEvent(1000, TickCallback_processA);  // 1000 ms
    TickSetTickEvent(5000, TickCallback_processB);  // 5000 ms
    #endif

    CAN_RS_PIN_Init();  
    CAN_Init();

    /* Got no where to go, just loop forever */
    while(1)
    {
        loop();

    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
