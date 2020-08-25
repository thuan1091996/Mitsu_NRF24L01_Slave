/* --------0.Project information--------------------
 * Mitsu FX PLC Communicatin Slave
 * Protocol communication: SPI (SSI0), UART1
 * Debug through LEDs
 * Author : TRAN MINH THUAN
---------------------------------------------------*/

/* ------------------------------1.System requirement-------------------------------
 * 1.Create a simple SSI communication system send and receive 1 byte(SPI.c)
 * 2.Delay function using SYSTICK timer
 * 3.Monitor the system through LEDs
 * 4.Create a protocol to send data through a Master NRF24L01 to NRF24L01 slave
     using Enhanced ShockBurst with IRQ pin Falling edge interrupt
   5.System operating
     * Master - TX - Click Button->Send 0x01 then monitor IRQ Flags(TX_DS or RT_MAX)
     * Slave  - RX - Monitor IRQ Flag (RX_DR)
 * 6.Debugging Notes
6.1- Can't read the TX_DR and MAX_RT flags in GPIO (IRQ) interrupt
Solution: Change nrf24l01_CONFIG_DEFAULT_VAL 0x08 - 0x28 to disable TX_DR interrupt
6.2- When read multiple from FIFO, the data collected represent just 1st byte
Solution: Read data in RX interrupt
7. Backup history
7.1: 15-12-2018: System work but quite slow and function declaration is not correct
7.2: 20-12-2018: System work, data transfer reach 5% rate (error_slave+error_recv/complete).
7.4: 15-1-2019 : FINAL VERSION 1.0 - Complete implementation of SIM800A, the system now can communication with 1 Slave and then
upload data to sever with optimization timing and detect and display error (~30s)
-----------------------------------------------------------------------------------*/

/* -----------------2.Pre-processor Directives Section-------------------*/
#include "include.h"
//-----------------Debugging LEDs---------------------
#define LEDs_DATA_R    (*((volatile unsigned long *)0x40025038)) //LEDs Data Register Addressing
#define LEDs_PORT      SYSCTL_PERIPH_GPIOF
#define LEDs_BASE      GPIO_PORTF_BASE
#define RED_PIN        GPIO_PIN_1
#define BLUE_PIN       GPIO_PIN_2
#define GREEN_PIN      GPIO_PIN_3
//Color from mixing RBG LEDs
#define RED            0x02
#define GREEN          0x08
#define YELLOW         0x0A
#define BLUE           0x04
#define WHITE          0x0E
#define SKY_BLUE       0x0C
#define PINK           0x06
//-----------------Buttons-----------------------
#define BUTTON_PORT    SYSCTL_PERIPH_GPIOF
#define BUTTON_BASE    GPIO_PORTF_BASE
#define BUTTON_PIN     GPIO_PIN_4
//-----------------System States----------------------
#define RX_STATE       0x00
#define TX_STATE       0x01
//-----------------NRF24L01---------------------------
//                  CSN (Chip Not Select- active LOW)
#define CSN_PORT            SYSCTL_PERIPH_GPIOA
#define CSN_BASE            GPIO_PORTA_BASE
#define CSN_PIN             GPIO_PIN_3
//                  CSN (Chip Enable)
#define CE_PORT             SYSCTL_PERIPH_GPIOA
#define CE_BASE             GPIO_PORTA_BASE
#define CE_PIN              GPIO_PIN_6
//                  IRQ (Interrupt Request)
#define IRQ_PORT            SYSCTL_PERIPH_GPIOA
#define IRQ_BASE            GPIO_PORTA_BASE
#define IRQ_PIN             GPIO_PIN_7
#define IRQ_MASK            GPIO_INT_PIN_7
//                  Slave definition
#define SLAVE_ADDR  01
/*-------------------------------------------------------------------------------*/


/* -------------3. Global Declarations Section---------------------*/

//-----------------------Variable for monitor system----------------//
typedef enum System_State System_State; enum System_State {State_Normal, State_Error_nRF24L01, State_Error_PLC};
unsigned short  State_disp=0;               //System state display
unsigned short  State=0;                    //System state nrf24l01
unsigned short  Error_PLC=0;                //Cannot communication with PLC
unsigned short  Complete=0;                 //Finish transmission with Master
unsigned short  Error_Trans_2Master=0;      //Receive correct message but can't send data to Master
unsigned short  Error_Recv_Req=0;           //Haven't receive message after a long time
//------------------------Variable for nRF24L01--------------------//
unsigned char   TX_complete=0;              //nRF24L01 transmission flag
unsigned char   RX_complete=0;              //nRF24L01 transmission flag
unsigned char   Packet[50]={0};             //contain data message to send to Master
unsigned char   Data_NRF[50];               //Debugging NRF24L01 Registers
unsigned char   ui8Data_RECV_MASTER[12]={0};//Correct data receive from Master
unsigned char   ui8Data_RECV_temp[12]={0};  //Data receive at nRF24L01 pipe0 RX
//-------------------------Variable for data PLC-------------------//
         short  ui16DataPLC_D[100]={0};     //String store value of D memory PLC
unsigned char   ui8Master_Request_Funct=0;  //Function that Master request from message (Apply when communication with PLC)
unsigned char   ui8Master_Request_Size=0;   //Size of data that Master request from message (Apply when communication with PLC)
unsigned short  ui16Master_Request_Addr=0;  //The data address that Mater ask from message (Apply when communication with PLC)
unsigned short  ui16_Data2Master_Length=0;  //The length of data to send to Master, MUST detect to build data frame to send through NRF24L01

//Function declaration
void Monitor_Init(void);
void Monitor(void);
void NRF24L01_Init();
void Processing_Request(unsigned char *ui8Pack_Location);
void StrCopy(unsigned char *Dest,
             unsigned char *Source,
             unsigned short length);
void Data_Packet( unsigned char *ui8Location,
                  unsigned short  ui16Slave_Addr,
                  unsigned char ui8Funct,
                  unsigned short  ui16Data_PLC);
bool Timeout(unsigned long Timeout_100ms, unsigned char Number);
bool CheckPacket(unsigned char *Location);
bool Master_Request(unsigned long Limited_time);
bool Data_2Master(unsigned long Limit_time);
/*------------------------------------------------------------------*/

/* ---------------------------4. Subroutines Section--------------------------*/
void main(void)
   {
    //-----------------Initialize System------------------------
    SysCtlClockSet(SYSCTL_SYSDIV_2_5| SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //80MHz
    Monitor_Init();                             //LEDs from port F  (PF1,2,3)
    Systick_Init();                             //1us interrupt
    SSI0_Init();                                //Master,8bit,2Mhz,Mode0
    UART_Init();                                //PLC UART communication
//    NRF24L01_Init();                            //nRF24L01 initialization
//    Watchdog_Init(90);
    //----------------Interrupt Enable----------------------------
    IntMasterEnable();
    //----------------Infinite Loop-------------------------------
    while(true)
    {
        //
        while(1)
        {
            uint16_t dataD[10]={0};
            Read_D(dataD, 1, 2);
        }

        //
        if(Master_Request(5)==1)    //If receive request from Master
        {
                Error_Recv_Req=0;
                if (Data_2Master(3)==0) Error_Trans_2Master++; //change to Data_2master(3) if system slow
                else
                {
                    Error_Trans_2Master=0;
                    Complete++;
                    Watchdog_Feed(90);
                    delay_us(1000);
                }
        }
        else
        {
            Error_Recv_Req++;                       //If doesn't receive message after a long time
            Read_D(&ui16DataPLC_D[0],0,10);
            delay_us(1000);
        }
    }
}

/* NRF ISR Handler
 * Caused by NRF24L01 PIN PF7 (FALLING EDGE)
 * Interrupt when IRQ PIN CHANGE FROM HIGH  -> LOW (Falling Edge)
                     +MASTER: TX_DS     - Transmit complete
                              MAX_RT    - Fail to transmit
                     +SLAVE:  RX_DR     - Receive complete
 * Usage: Use to acknowledge the NRF24L01 transmission status. When transmission complete
 then clear all data in FIFO, interrupt flag and then acknowledge interrupt
*/
void NRF_ISRHandler()
{
    //Slave sent data to Master complete
    if      ((State==TX_STATE) && nrf24l01_irq_tx_ds_active())
            {
                    TX_complete=1;
            }
//    //Sent data to Master fail
//    else if ((State==TX_STATE) && nrf24l01_irq_max_rt_active() )
//            {
//            }
    //Slave received request of Master
    else if ((State==RX_STATE) && nrf24l01_irq_rx_dr_active() )
            {
                    nrf24l01_read_rx_payload(&ui8Data_RECV_temp[0],12);
                    RX_complete=1;
            }
    nrf24l01_clear_flush();
    GPIOIntClear(IRQ_BASE,IRQ_MASK);    //Acknowledge interrupt
}

/* NRF24L01 Initialization
 * Function:
         1. Settings for Input pin (IRQ) and Output pins (CE,CSN)
         2. Setup for IRQ interrupt
         3. Setup NRF24L01 Registers
 * Input: No
 * Output: No
 * Change:
         - Change the IRQ Interrupt handler
         - Change another configure for NRF registers
*/
void NRF24L01_Init(){
    //-------1. Setup for GPIO (CSN,CE,IRQ Pins)-----------
    //------------------CE Pin-----------------------
    if(!SysCtlPeripheralReady(CE_PORT))           //Check Clock peripheral
    {
        SysCtlPeripheralEnable(CE_PORT);            //Enable if not ready (busy or disable)
        while (!SysCtlPeripheralReady(CE_PORT));  //Wait for peripheral ready
    }
    GPIOPinTypeGPIOOutput(CE_BASE, CE_PIN);         //CE OUTPUT
    //------------------CSN Pin-----------------------
    if(!SysCtlPeripheralReady(CSN_PORT))          //Check Clock peripheral
    {
        SysCtlPeripheralEnable(CSN_PORT);           //Enable if not ready (busy or disable)
        while (!SysCtlPeripheralReady(CSN_PORT)); //Wait for peripheral ready
    }
    GPIOPinTypeGPIOOutput(CSN_BASE, CSN_PIN);       //CSN OUTPUT
    //------------------IRQ Pin-----------------------
    if(!SysCtlPeripheralReady(IRQ_PORT))          //Check Clock peripheral
    {
        SysCtlPeripheralEnable(IRQ_PORT);           //Enable if not ready (busy or disable)
        while (!SysCtlPeripheralReady(IRQ_PORT)); //Wait for peripheral ready
    }
    GPIOPinTypeGPIOInput(IRQ_BASE, IRQ_PIN);        //IRQ INPUT
    //----------------2. Setup IRQ Interrupt --------------
    GPIOIntTypeSet(IRQ_BASE, IRQ_PIN, GPIO_LOW_LEVEL); // NRF24L01 ISR Active "LOW"
    GPIOIntRegister(IRQ_BASE, NRF_ISRHandler);            // Assign interrupt handler
    GPIOIntEnable(IRQ_BASE, IRQ_MASK);                    // Enable interrupt
    //---------------3.Setup NRF Register----------------------
    nrf24l01_initialize_debug(true, 12, true);  //RX,12 Byte, Enhanced ShockBurst
    State=RX_STATE;
//    LEDs_DATA_R=0;
    nrf24l01_irq_clear_all();
    nrf24l01_flush_rx();                       //Clear TX FIFO
    nrf24l01_flush_tx();                       //Clear RX FIFO
}

/* Monitor Debugging System Initialization
 * Function: Initialize I/O for LED pins
 * Input: No
 * Output: No
*/
void Monitor_Init(void)
{
        if(!SysCtlPeripheralReady(LEDs_PORT))        //Check Clock peripheral
        {
            SysCtlPeripheralEnable(LEDs_PORT);       //Enable if not ready (busy or disable)
            while(!SysCtlPeripheralReady(LEDs_PORT));//Wait for peripheral ready
        }
        GPIOPinTypeGPIOOutput(LEDs_BASE, RED_PIN|BLUE_PIN|GREEN_PIN);
}

/* Monitor Debugging System
 * Function: Show the status of the system
               Flash Green:       OK
               Flash Yellow:      Warning
               Flash Red:         Dangerous
               Red:               System stop
 * Input: No
 * Output: No
 * Change:
          - Change the color of LEDs
          - Add more system states
          - Change monitor time
 * Affect global variable:
          1. "State" - system state variable
          2. "Tick_monitor"
 */
void Monitor(void)
{
    //---------------------------------Change state---------------------------------
        if((Error_Recv_Req>=50) || (Error_Trans_2Master>=50))    State_disp=State_Error_nRF24L01;
        if(Error_PLC>=20)           State_disp=State_Error_PLC;
        if((Error_Recv_Req<50) && (Error_Trans_2Master<50) &&(Error_PLC<20) ) State_disp=State_Normal;

        //---------------------------------update display colors---------------------------------
        if(State_disp==State_Normal)
        {
            LEDs_DATA_R=0;
            LEDs_DATA_R=BLUE;
        }
        if(State_disp==State_Error_nRF24L01)
        {
            LEDs_DATA_R=0;
            LEDs_DATA_R=RED;
        }
        if(State_disp==State_Error_PLC)
        {
            LEDs_DATA_R=0;
            LEDs_DATA_R=WHITE;

        }
        Tick_monitor=0;
}

/* void Change_State_RX(void)
 * Function: Change to RX State include changes State variable and LEDs status
 * Input:  No
 * Output: No
 * Affect global variable: State, LEDs_DATA_R
 */
void Change_State_RX(void)
{
    nrf24l01_set_as_rx(true);     //Change NRF24L01 State
    delay_us(200);                //Wait for state changes
    State=RX_STATE;               //Change state for indicator
//    LEDs_DATA_R=0;
}

/* void Change_State_TX(void)
 * Function: Change to TX State include changes in State variable and LEDs status
 * Input:  No
 * Output: No
 * Affect global variable: State, LEDs_DATA_R
 */
void Change_State_TX(void)
{
    nrf24l01_set_as_tx();         //Change NRF24L01 State
    delay_us(200);                //Wait for state changes
    State=TX_STATE;               //Change state for indicator
//    LEDs_DATA_R=0;
}

/* StrCopy(unsigned char *Dest, unsigned char *Source, unsigned short length)
 * Function: Copy "length" byte data in "Source" to "Dest"
 * Input: Dest      - Destination string
 *        Source    - Source string
 *        Length    - String length
 * Output: No
*/
void StrCopy(unsigned char *Dest, unsigned char *Source, unsigned short length)
{
    uint8_t count=0;
    for(count=0;count<length;count++)
    Dest[count]=Source[count];
}

/*void Processing_Request(unsigned char *ui8Pack_Location)
 * Function: - Collect information from Master's request
 *           + Start addres
 *           + Fuction
 *           + Size
 *           + Length
 * Input: unsigned char *Pack_Location  - Location of packet
 * Output: No
 * Affect global variables:
    - ui8Master_Request_Funct - Function to communicate with PLC for message
    - ui8Master_Request_Size  - Number of unit to control
    - ui16Master_Request_Addr - Start address of PLC memory
    - ui16_Data2Master_Length - Length of message data to PLC
 */
void Processing_Request(unsigned char *ui8Pack_Location)
{
   ui8Master_Request_Funct=    ui8Pack_Location[3];
   ui8Master_Request_Size =    ui8Pack_Location[6];
   ui16Master_Request_Addr=   (ui8Pack_Location[4]<<8)|ui8Pack_Location[5];
   ui16_Data2Master_Length=    6+ui8Master_Request_Size*2+3;
}

/*void Data_Packet( unsigned char   *ui8Location,
                  unsigned short  ui16Slave_Addr,
                  unsigned char   ui8Funct,
                  unsigned short  ui16Start_Addr)
 * Function: Packet the information to send to Master
 * Input:
             . *ui8Location    - Pointer to the variable location that Packet will be saved
             . ui16Slave_Addr  - Slave address (Define in top off main.c)
             . ui8Funct        - Define what the slave need to do
                                     + M_SET        -> Set   M memory
                                     + M_RESET      -> Reset M Memory
                                     + D_MEM_READ   -> Read  D Memory
             . ui16Start_Addr  - The memory location to communication
 * Output: No
 * Use the global variable "ui16_Data2Master_Length" to detect the length -> detect "ETX" location
*/
void Data_Packet( unsigned char   *ui8Location,
                  unsigned short  ui16Slave_Addr,
                  unsigned char   ui8Funct,
                  unsigned short  ui16Start_Addr)
{
    unsigned long crc_sum=0;
    unsigned char crc_sum_b0,crc_sum_b1;
    unsigned char slave_addr_b0,slave_addr_b1;
    unsigned char start_addr_b0,start_addr_b1;
    unsigned char i;
    unsigned short etx_location=ui16_Data2Master_Length-3;
    unsigned char count_temp=0;
    unsigned short data_byte1,data_byte0;
    unsigned short start_addr;
    unsigned char  next_location=6;
    slave_addr_b0=SLAVE_ADDR&0xFF;
    slave_addr_b1=(SLAVE_ADDR>>8)&0xFF;
    start_addr=ui16Start_Addr;
    start_addr_b0=start_addr&0xFF;
    start_addr_b1=(start_addr>>8)&0xFF;
    ui8Location[0]=STX;                     //Start Signal
    ui8Location[1]=slave_addr_b1;           //Slave address
    ui8Location[2]=slave_addr_b0;           //Slave address
    ui8Location[3]=ui8Funct;
    ui8Location[4]=start_addr_b1;           //Start address 1
    ui8Location[5]=start_addr_b0;           //Start address 0
    if(ui8Funct==D_MEM_READ)
    {
        for(count_temp=0;count_temp<ui8Master_Request_Size;count_temp++)
        {
            data_byte0=ui16DataPLC_D[ui16Start_Addr]&0xFF;
            data_byte1=(ui16DataPLC_D[ui16Start_Addr]>>8)&0xFF;
            ui8Location[next_location]=data_byte1;
            ui8Location[next_location+1]=data_byte0;
            next_location+=2;
            ui16Start_Addr++;
        }
    }
    ui8Location[etx_location]=ETX;        //End signal
    for(i=0;i<=etx_location;i++)
    crc_sum+=ui8Location[i];
    crc_sum_b0=crc_sum&0xFF;        crc_sum=crc_sum>>8;
    crc_sum_b1=crc_sum&0xFF;
    ui8Location[etx_location+1]=crc_sum_b1;
    ui8Location[etx_location+2]=crc_sum_b0;
}

/* bool CheckPacket(unsigned char *Location)
 * Function: Check the Packet just arrived in RX FIFO to see if it has the right "Frame Data" by
 *           checking the  "STX,SLAVE ADDR, ETX, CRC1,CRC0"
 * Input: * Location - Pointer to array location
 * Output: True if data correct (correct data frame) and false if doesn't
*/
bool CheckPacket(unsigned char *Location)
{
    unsigned short  slave_addr=0;
    unsigned long   crc_sum=0;
    unsigned char   crc_sum_b0,crc_sum_b1;
    unsigned char   i;
    //calculate addr from data received
    slave_addr= (Location[1]<<8)|Location[2];
    //Sum calculate
    for(i=0;i<10;i++)
    crc_sum+=Location[i];
    crc_sum_b0=crc_sum&0xFF;    crc_sum=crc_sum>>8;
    crc_sum_b1=crc_sum&0xFF;
    if(     (slave_addr       == SLAVE_ADDR)     &&
            (Location[0]      == STX)            &&
            (Location[9]      == ETX)            &&
            (Location[10]     == crc_sum_b1)     &&
            (Location[11]     == crc_sum_b0)
      )
        return 1;
    else
        return 0;
}

/* bool Master_Request(unsigned long Limited_time)
 * Function: Check to see if the Master sent request or not
 * Input: unsigned long Limited_time - Maximum time to wait to see if Master send request, if
don't return 0
 * Output: True if receive request (correct data frame) and False if it doesn't
 * Affect global variable:
        + State - between RX and TX mode
        + ui8Data_RECV_MASTER: Collect data from Master's message
*/
bool Master_Request(unsigned long Limited_time)
{
    bool recv_request=0; //Receive flag
    //If not reach the limited time wait for Master request
    while((State==RX_STATE) && (Timeout(Limited_time,0)==0)) //Timeout module 0
    {
        if(RX_complete==1) //Receive data from RX FIFO (could be wrong data)
        {
            GPIOIntDisable(IRQ_BASE, IRQ_MASK);          //Disarm until store new data complete
            StrCopy(&ui8Data_RECV_MASTER[0], &ui8Data_RECV_temp[0], 12);
            if(CheckPacket(&ui8Data_RECV_MASTER[0])==1)    //If packet has correct data frame and slave address
            {
                //IF packet is correct, store data and check data frame
                Processing_Request(&ui8Data_RECV_MASTER[0]);   //Collect data from Master message
                recv_request=1; //Active flag if receive complete
                nrf24l01_set_rx_pw(ui16_Data2Master_Length, 0);
                delay_us(50);
                Change_State_TX();
                delay_us(1000);
            }
            RX_complete=0;
            nrf24l01_clear_flush();
            GPIOIntEnable(IRQ_BASE,IRQ_MASK);
        }
    }
    //If reach the limited time and still not receive anything from Master, Reset all value of nrf24l01
    if( (recv_request==0) && (State==RX_STATE) )
    {
        RX_complete=0;
        TX_complete=0;
        nrf24l01_clear_flush();
        GPIOIntEnable(IRQ_BASE,IRQ_MASK);
    }
    if (recv_request==1) return 1;
    else                 return 0;
}

/* bool Data_2Master(unsigned long Limit_time)
 * Function: Send the data PACKET that is processed  to Master after receive request message
 * Input: unsigned long Limited_time - Maximum time to transmit data to Master
 * Output: True if completed to send data to Master
 *         False if timeout and still not be able to send to master
*/
bool Data_2Master(unsigned long Limit_time)
{
    uint8_t send_complete=0;
    Data_Packet(&Packet[0],SLAVE_ADDR,ui8Master_Request_Funct,ui16Master_Request_Addr);
    while ( (State==TX_STATE) && (Timeout(Limit_time, 1)==0) )
        {
            nrf24l01_write_tx_payload(&Packet[0], ui16_Data2Master_Length, true);
            delay_us(5000);
            if(TX_complete==1)
            {
                send_complete=1;
                // // // // // // // // // // // // // // // // // // // // //
                delay_us(1000);
                // // // // // // // // // // // // // // // // // // // // //
                break;
            }
        }
        nrf24l01_set_rx_pw(12,0);
        delay_us(50);
        Change_State_RX();
        TX_complete=0;
        RX_complete=0;
        nrf24l01_clear_flush();
    if(send_complete==1) return 1;
    else                 return 0;
}
/*-------------------------------------------------------------------------------*/

