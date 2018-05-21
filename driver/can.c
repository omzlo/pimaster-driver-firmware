#include <stm32f0xx.h>
#include "stm32f0_helpers.h"
#include "can.h"
#include "nocan.h"
#include "gpio.h"

/***/


int can_filter_set(uint32_t filter_id, const can_filter_t *finfo)
{
    // Init mode
    CAN->FMR |= CAN_FMR_FINIT;

    // Deactivate filter
    CAN->FA1R &= ~(1<<filter_id);

    // 32 bitr scale for filter
    CAN->FS1R |= (1<<filter_id);

    CAN->sFilterRegister[filter_id].FR1 = finfo->filter;
    CAN->sFilterRegister[filter_id].FR2 = finfo->mask;
    
    // Id/Mask mode (0)
    CAN->FM1R &= ~(1<<filter_id);

    // set fifo
    if (finfo->fifo==FIFO_0)
        CAN->FFA1R &= ~(1<<filter_id);
    if (finfo->fifo==FIFO_1)
        CAN->FFA1R |= (1<<filter_id);

    if (finfo->active)
    {
        // activate
        CAN->FA1R |= (1<<filter_id);
    }
    // else: leave deactivated
    
    // Leave filter init mode
    CAN->FMR &= ~CAN_FMR_FINIT;

    return 0;
}

int can_filter_get(uint32_t filter_id, can_filter_t *finfo)
{
    finfo->active = ((CAN->FA1R >> filter_id)&1);
    finfo->filter = CAN->sFilterRegister[filter_id].FR1;
    finfo->mask   = CAN->sFilterRegister[filter_id].FR2;
    finfo->fifo   = ((CAN->FFA1R >> filter_id)&1);
    return 0;
}

/********************/

uint8_t can_tx_buffer[16];

int can_transmit_commit_buffer(void) 
{
    uint32_t eid;
    #define transmit_mailbox 0

    if (can_tx_buffer[0]!=13)
    {
        gpio_set_tx_int();
        return -1;
    }

    NOCAN_STATUS_SET(NOCAN_STATUS_TX_PENDING);

    eid = ((uint32_t)can_tx_buffer[1]<<24)
        | ((uint32_t)can_tx_buffer[2]<<16)
        | ((uint32_t)can_tx_buffer[3]<<8)
        | ((uint32_t)can_tx_buffer[4])
        ;

    /*** KEEP ***
    if ((CANx->TSR&CAN_TSR_TME0) == CAN_TSR_TME0)
    {
        transmit_mailbox = 0;
    }
    else if ((CANx->TSR&CAN_TSR_TME1) == CAN_TSR_TME1)
    {
        transmit_mailbox = 1;
    }
    else if ((CANx->TSR&CAN_TSR_TME2) == CAN_TSR_TME2)
    {
        transmit_mailbox = 2;
    }
    else
    {
        return -1;
    }
    *** KEEP ***/

    // check if transmit mailbox 0 is avaiable
    if ((CAN->TSR&CAN_TSR_TME0) != CAN_TSR_TME0)
        return -1;

    CAN->sTxMailBox[transmit_mailbox].TIR = (eid<<3) | (1<<2); 
        // (1<<2) means extended id, eid is shifted left by 3

    CAN->sTxMailBox[transmit_mailbox].TDTR &= (uint32_t)0xFFFFFFF0;
    CAN->sTxMailBox[transmit_mailbox].TDTR |= can_tx_buffer[5];

    CAN->sTxMailBox[transmit_mailbox].TDLR = ((uint32_t)can_tx_buffer[9] << 24) 
                                           | ((uint32_t)can_tx_buffer[8] << 16) 
                                           | ((uint32_t)can_tx_buffer[7] << 8) 
                                           | ((uint32_t)can_tx_buffer[6])
                                           ;

    CAN->sTxMailBox[transmit_mailbox].TDHR = ((uint32_t)can_tx_buffer[13] << 24) 
                                           | ((uint32_t)can_tx_buffer[12] << 16) 
                                           | ((uint32_t)can_tx_buffer[11] << 8) 
                                           | ((uint32_t)can_tx_buffer[10])
                                           ;

    CAN->sTxMailBox[transmit_mailbox].TIR |= 1; // (1<<0) is TXRQ flag (transmit request)

    return 0;
}

/********************/
#define CAN_RX_QUEUE_LENGTH 15
uint8_t can_rx_queue[16*(CAN_RX_QUEUE_LENGTH+1)];
uint32_t can_rx_head;
uint32_t can_rx_tail;
uint8_t *can_rx_buffer;

int can_receive_shift_buffer(void)
{
    if (can_rx_tail == can_rx_head)
        return -1;

    can_rx_head = (can_rx_head+1) & CAN_RX_QUEUE_LENGTH;

    if (can_rx_tail == can_rx_head) // empty now?
    {
        NOCAN_STATUS_CLEAR(NOCAN_STATUS_RX_PENDING);
        gpio_set_rx_int();
    }
    else
    {
        NOCAN_STATUS_SET(NOCAN_STATUS_RX_PENDING);
        gpio_clear_rx_int();
        can_rx_buffer = can_rx_queue+(16*can_rx_head);
    }
    return 0;    
}

static inline uint8_t *_can_receive_tail_buffer(void)
{
    /*
    uint32_t next = (can_rx_tail+1) & CAN_RX_QUEUE_LENGTH;

    if (next == can_rx_head)    // full ?
    {
        return (uint8_t *)0;
    }
    */
    return can_rx_queue+(16*can_rx_tail);
}

static int _can_receive_push_buffer(void)
{
    uint32_t next = (can_rx_tail+1) & CAN_RX_QUEUE_LENGTH;

    if (next == can_rx_head)
    {
        return -1;
    }

    if (can_rx_head == can_rx_tail)
        can_rx_buffer = can_rx_queue+(16*can_rx_head);

    can_rx_tail = next;

    NOCAN_STATUS_SET(NOCAN_STATUS_RX_PENDING);
    gpio_clear_rx_int();
    return 0; 
}

/********************/

int load_frame_to_receive_queue(uint8_t fifo_number)
{
   uint8_t *buffer = _can_receive_tail_buffer();

   /*
   if (buffer == 0)
       return -1;
       */

   uint32_t eid = (CAN->sFIFOMailBox[fifo_number].RIR>>3) | ((CAN->sFIFOMailBox[fifo_number].RIR & 0x4) << 29);

   buffer[0] = 13;
   buffer[1] = (uint8_t)(eid >> 24);
   buffer[2] = (uint8_t)(eid >> 16);
   buffer[3] = (uint8_t)(eid >> 8);
   buffer[4] = (uint8_t)(eid);
   buffer[5] = (uint8_t)CAN->sFIFOMailBox[fifo_number].RDTR & 0xF;
   buffer[6] = (uint8_t)CAN->sFIFOMailBox[fifo_number].RDLR & 0xFF;
   buffer[7] = (uint8_t)(CAN->sFIFOMailBox[fifo_number].RDLR >>8)& 0xFF;
   buffer[8] = (uint8_t)(CAN->sFIFOMailBox[fifo_number].RDLR >>16)& 0xFF;
   buffer[9] = (uint8_t)(CAN->sFIFOMailBox[fifo_number].RDLR >>24)& 0xFF;
   buffer[10] = (uint8_t)CAN->sFIFOMailBox[fifo_number].RDHR & 0xFF;
   buffer[11] = (uint8_t)(CAN->sFIFOMailBox[fifo_number].RDHR >> 8) & 0xFF;
   buffer[12] = (uint8_t)(CAN->sFIFOMailBox[fifo_number].RDHR >> 16)& 0xFF;
   buffer[13] = (uint8_t)(CAN->sFIFOMailBox[fifo_number].RDHR >> 24)& 0xFF; 

   return _can_receive_push_buffer();
}

/********************/
void CEC_CAN_IRQHandler(void)
{
    if((CAN->RF0R & CAN_RF0R_FMP0)!=0)/* check if a packet is filtered and received by FIFO 0 */
    {  
        load_frame_to_receive_queue(0);
        // TODO: signal error if overflow
        CAN->RF0R |= CAN_RF0R_RFOM0; // release the fifo
    }
    
    if((CAN->RF1R & CAN_RF1R_FMP1)!=0)/* check if a packet is filtered and received by FIFO 1 */
    {   
        load_frame_to_receive_queue(1);
        // TODO: signal error if overflow
        CAN->RF1R |= CAN_RF1R_RFOM1; // release the fifo
    }

    if((CAN->TSR & CAN_TSR_RQCP0)!=0) /* check if request completed for mailbox 0 */
    {
        NOCAN_STATUS_CLEAR(NOCAN_STATUS_TX_PENDING);
        gpio_set_tx_int();
        CAN->TSR |= CAN_TSR_RQCP0; // release flag
    } 
}

void can_reset(void)
{
    NVIC_DisableIRQ(CEC_CAN_IRQn);
    
    // enable CAN1 RESET State
    RCC->APB1RSTR |= RCC_APB1RSTR_CANRST;
    // leave CAN1 RESET State
    RCC->APB1RSTR &= ~RCC_APB1RSTR_CANRST;

    can_init();
}


int can_init(void)
{
    can_filter_t default_can_filter;

    can_rx_head = 0;
    can_rx_tail = 0;
    can_rx_buffer = can_rx_queue;

    /* REMAP PINS, Only for STM32F042 TSSOP20 */
    // RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
    // SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;

    /* CAN GPIOs configuration **************************************************/

    /* Enable GPIO clock */
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    /* Connect CAN pins to AF4 */
    GPIO_CONFIGURE_ALTERNATE_FUNCTION(GPIOB, 8, GPIO_AF_4);
    GPIO_CONFIGURE_ALTERNATE_FUNCTION(GPIOB, 9, GPIO_AF_4);


    /* CAN SETUP ****************************************************************/

    /* Enable CAN clock */
    RCC->APB1ENR &= ~RCC_APB1ENR_CANEN;
    RCC->APB1ENR |= RCC_APB1ENR_CANEN;

    /* CAN cell init */
    CAN->MCR &=~ CAN_MCR_SLEEP;  // Exit sleep mode
   
    /* CAN register init */
    CAN->MCR |= CAN_MCR_INRQ; /* (1) */
    while((CAN->MSR & CAN_MSR_INAK)!=CAN_MSR_INAK) {}

    
    /* Automatic bus off management: 
     * The Bus-Off state is left automatically by hardware once 128 occurrences of 11 recessive
     * bits have been monitored.
     */
    CAN->MCR |= CAN_MCR_ABOM;

#ifdef USE_FASTER_CAN

#define CAN_SJW_1tq 0
#define CAN_BS1_11tq 10
#define CAN_BS2_4tq 3
#define CAN_PRESCALE 12

    /* CAN Baudrate = 250K (CAN clocked at 48 MHz) */
    // (48000000/250000)/12 => 16 => (1+SJW)+(1+BS1)+(1+BS2) where (1+SJW)+(1+BS1)/8==75%

    CAN->BTR = (0<<30) // normal mode 
        | (CAN_SJW_1tq << 24)
        | (CAN_BS1_11tq << 16)
        | (CAN_BS2_4tq << 20)
        | (CAN_PRESCALE-1)               // prescaler
        ;

#else

#define CAN_SJW_1tq 0
#define CAN_BS1_11tq 10
#define CAN_BS2_4tq 3 
#define CAN_PRESCALE 24

    /* CAN Baudrate = 125k (CAN clocked at 48 MHz) */
    // (48000000/125000)/24 => 16 => (1+SJW)+(1+BS1)+(1+BS2) where (1+SJW)+(1+BS1)/16==75%
    CAN->BTR = (0 << 30) // normal mode
        | (CAN_SJW_1tq << 24)
        | (CAN_BS1_11tq << 16)
        | (CAN_BS2_4tq << 20)
        | (CAN_PRESCALE-1)              // prescaler
        ;

#endif

    /* Leave Init mode */
    CAN->MCR &= ~CAN_MCR_INRQ;
    while((CAN->MSR & CAN_MSR_INAK) == CAN_MSR_INAK) {}

   
    /* Enable FIFO 0 and FIFO 1 message pending Interrupts 
     * Enable transmit mailbox empty Interrupt is set in can_tx_send()
     */
    CAN->IER = CAN_IER_FMPIE0
        | CAN_IER_FMPIE1
        | CAN_IER_TMEIE
        ;

    /* CAN filter init */
    default_can_filter.active  = 1;
    default_can_filter.filter  = 0;
    default_can_filter.mask    = 0;
    default_can_filter.fifo    = 0;
    can_filter_set(0,&default_can_filter);

    /* Set default pin state */
    gpio_set_rx_int();
    gpio_set_tx_int();

    /* NVIC configuration *******************************************************/
    NVIC_SetPriority(CEC_CAN_IRQn, 1); 
    NVIC_EnableIRQ(CEC_CAN_IRQn);
    return 0;
}


