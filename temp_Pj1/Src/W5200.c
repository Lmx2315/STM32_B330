
#include "main.h"
#include "w5200.h"

#define bool u8

/**
 * The volatile 32-bit unsigned data type.
 */
//typedef volatile unsigned long vuint32;


typedef uint8			u_char;		/**< 8-bit value */
typedef uint8 			SOCKET;
typedef uint16			u_short;	/**< 16-bit value */
typedef uint16			u_int;		/**< 16-bit value */
typedef uint32			u_long;		/**< 32-bit value */


extern volatile  unsigned char tick_us;

// START ADDRESS REGISTERs
#define Ip_address        0x000F
#define Gateway_address   0x0001
#define Mask_address      0x0006
#define Hard_address      0x0010
#define Sn_MR_address     0x4000  //Socket n-th-th Mode Register [R/W]
#define Sn_CR_address     0x4001  //Socket n-th Command Register [R/W] [0x4001
#define Sn_IR_address     0x4002  //Socket n-th Interrupt Register [R] [0x4002+0x0n00]
#define Sn_SR_address     0x4003  //Socket n-th Status Register [R] [0x4003+0x0n00]
#define Sn_PORT_address   0x4004  //Socket  n-th Source  Port  Register [R/W]  [0x4004
#define Sn_DHAR_address   0x4006  //Socket n-th Destination Hardware Address Register [R/W]
#define Sn_DIPR_address   0x400C  //Socket  n-th Destination  IP  Address  Register R/W][0x400C
#define Sn_DPORT_address  0x4010  //Socket  n-th Destination  Port  Register[R/W][0x4010
#define Sn_MSS_address    0x4012  //Socket  n-th Maximum  Segment  Size  Register [R/W][0x4012
#define Sn_TOS_address    0x4015  //Socket n-th IP Type Of Service Register) [R/W] [0x4015
#define Sn_TTL_address    0x4016  //Socket n-th IP Time To Live Register) [R/W] [0x4016
#define Sn_RXMEM_SIZE_address 0x401E //Socket n-th RX Memory Size Register) [R/W] [0x401E
#define Sn_TXMEM_SIZE_address 0x401F //Socket n-th TX Memory size Register) [R/W][0x401F
#define Sn_TX_FSR_address 0x4020  //Socket  n-th TX  Free  Size  Register)  [R]  [0x4020
#define Sn_TX_RD_address  0x4022  //Socket  n-th TX  Read  Pointer  Register)  [R]  [0x4022
#define Sn_TX_WR_address  0x4024  //Socket  n-th TX  Write  Pointer  Register)  [R/W]  [0x4024
#define Sn_RX_RSR_address 0x4026  //SOCKET  n-th Received  Size  Register)  [R]  [0x4026
#define Sn_RX_RD_address  0x4028  //Socket n-th RX Read Pointer Register) [R/W] [0x4028
#define Sn_RX_WR_address  0x402A //Socket n-th RX Write Pointer Register)[R/W][(0x402A
#define Sn_IMR_address    0x402C  //Socket n-th Interrupt Mask Register)[R/W][0x402C
#define Sn_FRAG_address   0x402D  //Socket n-th Fragment Register)[R/W][0x402D
#define Sn_MR_UPD_aadress 0x4000  //This register sets up socket option or protocol type for each socket
#define Socet_receive     0xC000  // Base adress Socket 0 receive
#define Socket_transmit   0x8000  // Base adress Socket 0 transmit  
#define Sn_CR_Open        0x01
#define Sn_CR_Close       0x10
#define Sn_CR_Send        0x20
#define Sn_CR_Send_Mac    0x21
#define Sn_CR_Recv        0x40
#define Sn_CR_Open        0x01
#define WRITE_1Byte 0x8001
#define Read_1Byte  0x0001

 
u8  spi4send8 (u8 );   // SPI
 
 //************************************************************************************************************************************
/**
@brief	 __DEF_IINCHIP_MAP_xxx__ : define memory map for iinchip 
*/
#define __DEF_IINCHIP_MAP_BASE__ 0x0000
#define COMMON_BASE 0x0000
#define __DEF_IINCHIP_MAP_TXBUF__ (COMMON_BASE + 0x8000) /* Internal Tx buffer address of the iinchip */
#define __DEF_IINCHIP_MAP_RXBUF__ (COMMON_BASE + 0xC000) /* Internal Rx buffer address of the iinchip */
#define __DEF_IINCHIP_PPP

#define IINCHIP_ISR_DISABLE()
#define IINCHIP_ISR_ENABLE()	

//#define NULL		((void *) 0)

#define WINDOWFULL_FLAG_ON 1
#define WINDOWFULL_FLAG_OFF 0 
#define WINDOWFULL_MAX_RETRY_NUM 3
#define WINDOWFULL_WAIT_TIME 1000


//typedef enum { false, true } bool;

#ifndef _SIZE_T
#define _SIZE_T
typedef unsigned int size_t;
#endif

  char  strng1[64];// 


#define PPP_OPTION_BUF_LEN 64	  
	
#define SOCK_CONFIG		2	// UDP
#define SOCK_DNS		2	// UDP
#define SOCK_DHCP		3	// UDP


#define KEEP_ALIVE_TIME	30	// 30sec

#define ON	1
#define OFF	0

#define HIGH		  1
#define LOW		      0
#define	MAX_SOCK_NUM  2	/**< Maxmium number of socket  */
#define	uint8  u8
#define	uint16 u16
#define	uint32 u32
#define int16  short

//************************************************************************************************************************************

  unsigned char flag_RX_UDP=0;
  int16 RSR_len_TST;
  extern unsigned int lenght;
  u16 sch_tx_packet=0;

//************************************************************************************************************************

#define ApplicationAddress 	0x08004000

#define tick_second 1

uint8 ch_status[MAX_SOCK_NUM];
uint32_t presentTime;
uint16 any_port = 1000;


u8 ch_status[MAX_SOCK_NUM] = { 0,0 };	/** 0:close, 1:ready, 2:connected */

// SRAM address range is 0x2000 0000 ~ 0x2000 4FFF (20KB)

#define RX_MAX_BUF_SIZE	2048
#define TX_MAX_BUF_SIZE 2048

uint8 TX_BUF[TX_MAX_BUF_SIZE];
u32   TX_BUF_LENGTH=0;
uint8 RX_BUF[RX_MAX_BUF_SIZE];

unsigned int lsr_razmer=0;

#define MR				(COMMON_BASE + 0x0000) 
/**
 @brief Gateway IP Register address
 */
#define GAR0			        (COMMON_BASE + 0x0001)
/**
 @brief Subnet mask Register address
 */
#define SUBR0			        (COMMON_BASE + 0x0005)
/**
 @brief Source MAC Register address
 */
#define SHAR0				(COMMON_BASE + 0x0009)
/**
 @brief Source IP Register address
 */
#define SIPR0				(COMMON_BASE + 0x000F)
/**
 @brief Interrupt Register
 */
#define IR				(COMMON_BASE + 0x0015)
/**
 @brief Socket Interrupt Register
 */
#define IR2				(COMMON_BASE + 0x0034) 
/**
 @brief PHY Status Register
 */
#define PHY				(COMMON_BASE + 0x0035)
/**
 @brief Interrupt mask register
 */
#define IMR				(COMMON_BASE + 0x0016)  // #define IMR				(COMMON_BASE + 0x0036)
/**
 @brief Socket Interrupt Mask Register
 */
#define IMR2				(COMMON_BASE + 0x0036)  //#define IMR2				(COMMON_BASE + 0x0016)
/**
 @brief Timeout register address( 1 is 100us )
 */
#define RTR				(COMMON_BASE + 0x0017)
/**
 @brief Retry count reigster
 */
#define RCR				(COMMON_BASE + 0x0019)
/**
 @brief Authentication type register address in PPPoE mode
 */
#define PATR0			        (COMMON_BASE + 0x001C)
#define PPPALGO                         (COMMON_BASE + 0x001E)
/**
 @briefPPP LCP Request Timer register  in PPPoE mode
 */
#define PTIMER 		                (COMMON_BASE + 0x0028)
/**
 @brief PPP LCP Magic number register  in PPPoE mode
 */
#define PMAGIC 		                (COMMON_BASE + 0x0029)
/**
 @brief chip version register address
 */
#define VERSIONR			(COMMON_BASE + 0x001F)   
/**
 @brief Unreachable IP register address in UDP mode
 */
#define UIPR0				(COMMON_BASE + 0x002A)
/**
 @brief Unreachable Port register address in UDP mode
 */
#define UPORT0			        (COMMON_BASE + 0x002E)
/**
 @brief set Interrupt low level timer register address
 */
#define INTLEVEL0			(COMMON_BASE + 0x0030)
#define INTLEVEL1			(COMMON_BASE + 0x0031)
/**
 @brief socket register
*/
#define CH_BASE                         (COMMON_BASE + 0x4000)
/**
 @brief	size of each channel register map
 */
#define CH_SIZE		                0x0100
/**
 @brief socket Mode register
 */
#define Sn_MR(ch)		        (CH_BASE + ch * CH_SIZE + 0x0000)
/**
 @brief channel Sn_CR register
 */
#define Sn_CR(ch)			(CH_BASE + ch * CH_SIZE + 0x0001)
/**
 @brief channel interrupt register
 */
#define Sn_IR(ch)			(CH_BASE + ch * CH_SIZE + 0x0002)
/**
 @brief channel status register
 */
#define Sn_SR(ch)			(CH_BASE + ch * CH_SIZE + 0x0003)
/**
 @brief source port register
 */
#define Sn_PORT0(ch)			(CH_BASE + ch * CH_SIZE + 0x0004)
/**
 @brief Peer MAC register address
 */
#define Sn_DHAR0(ch)			(CH_BASE + ch * CH_SIZE + 0x0006)
/**
 @brief Peer IP register address
 */
#define Sn_DIPR0(ch)			(CH_BASE + ch * CH_SIZE + 0x000C)
/**
 @brief Peer port register address
 */
#define Sn_DPORT0(ch)			(CH_BASE + ch * CH_SIZE + 0x0010)
/**
 @brief Maximum Segment Size(Sn_MSSR0) register address
 */
#define Sn_MSSR0(ch)			(CH_BASE + ch * CH_SIZE + 0x0012)
/**
 @brief Protocol of IP Header field register in IP raw mode
 */
#define Sn_PROTO(ch)			(CH_BASE + ch * CH_SIZE + 0x0014)

/** 
 @brief IP Type of Service(TOS) Register 
 */
#define Sn_TOS(ch)			(CH_BASE + ch * CH_SIZE + 0x0015)
/**
 @brief IP Time to live(TTL) Register 
 */
#define Sn_TTL(ch)			(CH_BASE + ch * CH_SIZE + 0x0016)
/**
 @brief Receive memory size reigster
 */
#define Sn_RXMEM_SIZE(ch)	        (CH_BASE + ch * CH_SIZE + 0x001E)
/**
 @brief Transmit memory size reigster
 */
#define Sn_TXMEM_SIZE(ch)	        (CH_BASE + ch * CH_SIZE + 0x001F)
/**
 @brief Transmit free memory size register
 */
#define Sn_TX_FSR0(ch)		        (CH_BASE + ch * CH_SIZE + 0x0020)
/**
 @brief Transmit memory read pointer register address
 */
#define Sn_TX_RD0(ch)			(CH_BASE + ch * CH_SIZE + 0x0022)
/**
 @brief Transmit memory write pointer register address
 */
#define Sn_TX_WR0(ch)			(CH_BASE + ch * CH_SIZE + 0x0024)
/**
 @brief Received data size register
 */
#define Sn_RX_RSR0(ch)		        (CH_BASE + ch * CH_SIZE + 0x0026)
/**
 @brief Read point of Receive memory
 */
#define Sn_RX_RD0(ch)			(CH_BASE + ch * CH_SIZE + 0x0028)
/**
 @brief Write point of Receive memory
 */
#define Sn_RX_WR0(ch)			(CH_BASE + ch * CH_SIZE + 0x002A)
/**
 @brief socket interrupt mask register
 */
#define Sn_IMR(ch)			(CH_BASE + ch * CH_SIZE + 0x002C)
/**
 @brief frag field value in IP header register
 */
#define Sn_FRAG(ch)			(CH_BASE + ch * CH_SIZE + 0x002D)
/**
 @brief Keep Timer register
 */
#define Sn_KEEP_TIMER(ch)		(CH_BASE + ch * CH_SIZE + 0x002F)

/* MODE register values */
#define MR_RST			0x80 /**< reset */
#define MR_WOL			0x20 /**< Wake on Lan */
#define MR_PB			0x10 /**< ping block */
#define MR_PPPOE		0x08 /**< enable pppoe */
#define MR_LB  	        0x04 /**< little or big endian selector in indirect mode */
#define MR_AI			0x02 /**< auto-increment in indirect mode */
#define MR_IND			0x01 /**< enable indirect mode */

/* IR register values */
#define IR_CONFLICT	        0x80 /**< check ip confict */
#define IR_UNREACH	        0x40 /**< get the destination unreachable message in UDP sending */
#define IR_PPPoE    		0x20 /**< get the PPPoE close message */
#define IR_MAGIC	    	0x10 /**< get the magic packet interrupt */
#define IR_SOCK(ch)	       (0x01 << ch) /**< check socket interrupt */

/* Sn_MR values */
#define Sn_MR_CLOSE			0x00		/**< unused socket */
#define Sn_MR_TCP			0x01		/**< TCP */
#define Sn_MR_UDP			0x02		/**< UDP */
#define Sn_MR_IPRAW	        0x03		/**< IP LAYER RAW SOCK */
#define Sn_MR_MACRAW	    0x04		/**< MAC LAYER RAW SOCK */
#define Sn_MR_PPPOE			0x05		/**< PPPoE */
#define Sn_MR_ND			0x20		/**< No Delayed Ack(TCP) flag */
#define Sn_MR_MULTI			0x80		/**< support multicating */

/* Sn_CR values */
#define Sn_CR_OPEN			0x01		/**< initialize or open socket */
#define Sn_CR_LISTEN		0x02		/**< wait connection request in tcp mode(Server mode) */
#define Sn_CR_CONNECT	    0x04		/**< send connection request in tcp mode(Client mode) */
#define Sn_CR_DISCON		0x08		/**< send closing reqeuset in tcp mode */
#define Sn_CR_CLOSE			0x10		/**< close socket */
#define Sn_CR_SEND			0x20		/**< update txbuf pointer, send data */
#define Sn_CR_SEND_MAC	    0x21		/**< send data with MAC address, so without ARP process */
#define Sn_CR_SEND_KEEP     0x22		/**<  send keep alive message */
#define Sn_CR_RECV			0x40		/**< update rxbuf pointer, recv data */


	#define Sn_CR_PCON		0x23		 
	#define Sn_CR_PDISCON	0x24		 
	#define Sn_CR_PCR		0x25		 
	#define Sn_CR_PCN		0x26		
	#define Sn_CR_PCJ		0x27		


/* Sn_IR values */

	#define Sn_IR_PRECV		0x80		
	#define Sn_IR_PFAIL		0x40		
	#define Sn_IR_PNEXT		0x20		

#define Sn_IR_SEND_OK		0x10		/**< complete sending */
#define Sn_IR_TIMEOUT		0x08		/**< assert timeout */
#define Sn_IR_RECV			0x04		/**< receiving data */
#define Sn_IR_DISCON		0x02		/**< closed socket */
#define Sn_IR_CON			0x01		/**< established connection */

/* Sn_SR values */
#define SOCK_CLOSED				0x00		/**< closed */
#define SOCK_INIT 				0x13		/**< init state */
#define SOCK_LISTEN				0x14		/**< listen state */
#define SOCK_SYNSENT	   		0x15		/**< connection state */
#define SOCK_SYNRECV		   	0x16		/**< connection state */
#define SOCK_ESTABLISHED		0x17		/**< success to connect */
#define SOCK_FIN_WAIT			0x18		/**< closing state */
#define SOCK_CLOSING		   	0x1A		/**< closing state */
#define SOCK_TIME_WAIT			0x1B		/**< closing state */
#define SOCK_CLOSE_WAIT			0x1C		/**< closing state */
#define SOCK_LAST_ACK			0x1D		/**< closing state */
#define SOCK_UDP				0x22		/**< udp socket */
#define SOCK_IPRAW				0x32		/**< ip raw mode socket */
#define SOCK_MACRAW				0x42		/**< mac raw mode socket */
#define SOCK_PPPOE				0x5F		/**< pppoe socket */

/* IP PROTOCOL */
#define IPPROTO_IP              0           /**< Dummy for IP */
#define IPPROTO_ICMP            1           /**< Control message protocol */
#define IPPROTO_IGMP            2           /**< Internet group management protocol */
#define IPPROTO_GGP             3           /**< Gateway^2 (deprecated) */
#define IPPROTO_TCP             6           /**< TCP */
#define IPPROTO_PUP             12          /**< PUP */
#define IPPROTO_UDP             17          /**< UDP */
#define IPPROTO_IDP             22          /**< XNS idp */
#define IPPROTO_ND              77          /**< UNOFFICIAL net disk protocol */
#define IPPROTO_RAW             255         /**< Raw IP packet */

/*********************************************************
* iinchip access function
*********************************************************/
uint8 IINCHIP_READ(uint16 addr); 
uint8 IINCHIP_WRITE(uint16 addr,uint8 data);
uint32_t presentTime;
uint32_t my_time;
 
uint8 Enable_DHCP = OFF;

#define         NB 0//!!!!!!!номер в цепи резервировния!!!!
u8 NUMBER_BLOK =NB; 

uint8 MAC[6]     ={0x64, 0xA2, 0x32, 0x01, 0x03,NB};//MAC Address
uint8 IP [4]     ={1, 3,  3,NB+60};//IP Address  
uint8 GateWay[4] ={1, 3,  3,   1 };// Gateway Address
uint8 SubNet [4] ={255, 255,255,0};//SubnetMask Address

u16 SERVER_DEST_PORT=49000;//порт удалённого сервера куда шлём UDP

extern u32 IP_my;
extern u8 PORT_my;

/* 
uint8 Enable_DHCP = ON;
uint8 MAC[6]    ={0x64, 0xA2, 0x32, 0x01, 0x02, 0x03};//MAC Address
uint8 IP[4]     ={192 , 168 , 10  , 15};//IP Address
uint8 GateWay[4]={0   , 0   , 0   , 0 };//Gateway Address
uint8 SubNet[4] ={255 , 255 , 255 , 0 };//SubnetMask Address
*/

uint8_t 	 Hard_reg [6]={0xe0,0x69,0x95,0x04,0xdf,0x96};     //SHAR Source Hardware Address Register
char		 Receive_DATA[16];
uint8_t		 Receive_IP[4]= {1, 3, 1, 151};//{192, 168, 10, 15};//IP Address;
uint8_t 	 Receive_Port_destination[2]={0, 23};
uint8_t 	 Receive_byte_size[2];
uint8_t 	 Rec_Flag;
uint16_t 	 size;  


   
//FOR TCP Client
//Configuration Network Information of TEST PC
u8  Dest_IP[4] = {1, 3, 1, 1}; //DST_IP Address 
u16 Dest_PORT = 23; //DST_IP port

	
typedef struct 
{
	uint8 Mac[6];
	uint8 Lip[4];
	uint8 Sub[4];
	uint8 Gw[4];
	uint8 DNS_Server_IP[4];	
	uint8  DHCP;
}
CONFIG_MSG;

typedef struct 
{
	uint16 port;
	uint8 destip[4];
}CHCONFIG_TYPE_DEF;
	
	
typedef struct {
        uint32 state[4];    /**< state (ABCD)                            */
        uint32 count[2];    /**< number of bits, modulo 2^64 (lsb first) */
        uint8  buffer[64];  /**< input buffer                            */
      } md5_ctx;
	  
 CONFIG_MSG Config_Msg;
 
 CHCONFIG_TYPE_DEF Chconfig_Type_Def; 

//TX MEM SIZE- SOCKET 0:8KB, SOCKET 1:2KB, SOCKET2-7:1KB
//RX MEM SIZE- SOCKET 0:8KB, SOCKET 1:2KB, SOCKET2-7:1KB
/*
uint8 txsize[MAX_SOCK_NUM] = {2,2,2,2,2,2,2,2};
uint8 rxsize[MAX_SOCK_NUM] = {2,2,2,2,2,2,2,2};
*/
uint8 txsize[MAX_SOCK_NUM] = {16,16};//размер буфера 8кб 
uint8 rxsize[MAX_SOCK_NUM] = {16,16};

unsigned int  sch_packet_UDP_reciv;
extern volatile unsigned int rx_counter0; 

void iinchip_init(void); // reset iinchip
void sysinit(uint8 * tx_size, uint8 * rx_size); // setting tx/rx buf size
uint8 getISR(uint8 s);
void putISR(uint8 s, uint8 val);
uint16 getIINCHIP_RxMAX(uint8 s);
uint16 getIINCHIP_TxMAX(uint8 s);
uint16 getIINCHIP_RxMASK(uint8 s);
uint16 getIINCHIP_TxMASK(uint8 s);
uint16 getIINCHIP_RxBASE(uint8 s);
uint16 getIINCHIP_TxBASE(uint8 s);
void setMR(uint8 val);
void setRTR(uint16 timeout); // set retry duration for data transmission, connection, closing ...
void setRCR(uint8 retry); // set retry count (above the value, assert timeout interrupt)
void setIMR(uint8 mask); // set interrupt mask. 
uint8 getIR( void );
void setSn_MSS(SOCKET s, uint16 Sn_MSSR0); // set maximum segment size
void setSn_PROTO(SOCKET s, uint8 proto); // set IP Protocol value using IP-Raw mode
uint8 getSn_IR(SOCKET s); // get socket interrupt status
uint8 getSn_SR(SOCKET s); // get socket status
uint16 getSn_TX_FSR(SOCKET s); // get socket TX free buf size
uint16 getSn_RX_RSR(SOCKET s); // get socket RX recv buf size
uint8 getSn_SR(SOCKET s);
void setSn_TTL(SOCKET s, uint8 ttl);
void send_data_processing(SOCKET s, uint8 *wizdata, uint16 len);
void recv_data_processing(SOCKET s, uint8 *wizdata, uint16 len);
void write_data(SOCKET s, vuint8 * src, vuint8 * dst, uint16 len);
void read_data(SOCKET s, vuint8 * src, vuint8 * dst, uint16 len);

void setGAR(uint8 * addr); // set gateway address
void saveSUBR(uint8 * addr);	
void setSUBR(void); // set subnet mask address
void clearSUBR(void);												
void setSHAR(uint8 * addr); // set local MAC address
void setSIPR(uint8 * addr); // set local IP address
void getGAR(uint8 * addr);
void getSUBR(uint8 * addr);
void getSHAR(uint8 * addr);
void getSIPR(uint8 * addr);
void init_telopt(SOCKET );
void close(SOCKET );
void sendIAC(SOCKET , uint8 , uint8 ); 
unsigned int  Event_Synhron_func(void); //процедура проверки наличия важных евентов синхронизатора
void spi_cs_SPI(u8 ); 
 u8 SPI(u8 );
void Synhron_section (void);
char TCP_txchar(void);
void Transf(const char *);
void u_out (char s[],u32 a);
void UART_control (void);
void UART_DMA_TX  (void);
void UART_DMA_TX2  (void);
void SYNC_obmen (void);
void loopback_udp(SOCKET , uint16 ,unsigned char );
void Set_network(void);
void RECEIVE_udp(SOCKET s, uint16 port,unsigned char event);
void SEND_udp(SOCKET s, uint16 port,uint32 dest_IP,uint16 dest_PORT);
//******************************************

uint8 pppinit(uint8 *id, uint8 idlen, uint8 *passwd, uint8 passwdlen);
uint8 pppterm(uint8 *mac,uint8 *sessionid);


uint8 incr_windowfull_retry_cnt(uint8 s);
void init_windowfull_retry_cnt(uint8 s);

static uint8 I_STATUS[MAX_SOCK_NUM];
static uint16 SMASK[MAX_SOCK_NUM]; /**< Variable for Tx buffer MASK in each channel */
static uint16 RMASK[MAX_SOCK_NUM]; /**< Variable for Rx buffer MASK in each channel */
static uint16 SSIZE[MAX_SOCK_NUM]; /**< Max Tx buffer size by each channel */
static uint16 RSIZE[MAX_SOCK_NUM]; /**< Max Rx buffer size by each channel */
static uint16 SBUFBASEADDRESS[MAX_SOCK_NUM]; /**< Tx buffer base address by each channel */
static uint16 RBUFBASEADDRESS[MAX_SOCK_NUM]; /**< Rx buffer base address by each channel */
static uint8 SUBNET[4];


// Constants for Transform routine.
#define S11    7
#define S12   12
#define S13   17
#define S14   22
#define S21    5
#define S22    9
#define S23   14
#define S24   20
#define S31    4
#define S32   11
#define S33   16
#define S34   23
#define S41    6
#define S42   10
#define S43   15
#define S44   21


static uint16 local_port;
extern uint16 sent_ptr;



void Delay_us( u8 time_us )
{

}

/*******************************************************************************
* Function Name  : Delay_ms
* Description    : Delay per mili second.
* Input          : time_ms
* Output         : None
* Return         : None
*******************************************************************************/

void Delay_ms( u16 time_ms )
{
  register u16 i;
  for( i=0;i<time_ms;i++ )
  {
    Delay_us(250);
    Delay_us(250);
    Delay_us(250);
    Delay_us(250);
  }
}

void wait_10ms( u16 time_ms )
{
  register u16 i;
  for( i=0;i<time_ms;i++ )
  {
    Delay_ms(1);
   }
}

//-------------------------------------------------------------------
//--------------------------------------------------------------------

uint8 windowfull_retry_cnt[MAX_SOCK_NUM];

uint8 incr_windowfull_retry_cnt(uint8 s)
{
  return windowfull_retry_cnt[s]++;
}

void init_windowfull_retry_cnt(uint8 s)
{
  windowfull_retry_cnt[s] = 0;
}

uint16 pre_sent_ptr, sent_ptr;

uint8 getISR(uint8 s)
{
	return I_STATUS[s];
}
void putISR(uint8 s, uint8 val)
{
   I_STATUS[s] = val;
}
uint16 getIINCHIP_RxMAX(uint8 s)
{
   return RSIZE[s];
}
uint16 getIINCHIP_TxMAX(uint8 s)
{
   return SSIZE[s];
}
uint16 getIINCHIP_RxMASK(uint8 s)
{
   return RMASK[s];
}
uint16 getIINCHIP_TxMASK(uint8 s)
{
   return SMASK[s];
}
uint16 getIINCHIP_RxBASE(uint8 s)
{
   return RBUFBASEADDRESS[s];
}
uint16 getIINCHIP_TxBASE(uint8 s)
{
   return SBUFBASEADDRESS[s];
}
void IINCHIP_CSoff(void)
{
	NSS_4(0);
}
void IINCHIP_CSon(void)
{
	NSS_4(1);
}
u8  IINCHIP_SpiSendData(uint8 dat)
{
	return(spi4send8 (dat)); //8 бит
}


uint8 IINCHIP_WRITE(uint16 addr,uint8 data)
{
 //       IINCHIP_ISR_DISABLE();                      // Interrupt Service Routine Disable

	//SPI MODE I/F
	IINCHIP_CSoff();                            // CS=0, SPI start

	IINCHIP_SpiSendData((addr & 0xFF00) >> 8);  // Address byte 1
	IINCHIP_SpiSendData(addr & 0x00FF);         // Address byte 2
	IINCHIP_SpiSendData(0x80);                  // Data write command and Write data length 1
	IINCHIP_SpiSendData(0x01);                  // Write data length 2
	IINCHIP_SpiSendData(data);                  // Data write (write 1byte data)

	IINCHIP_CSon();                             // CS=1,  SPI end

//  IINCHIP_ISR_ENABLE();                       // Interrupt Service Routine Enable
	return 1;
}


uint8 socket(SOCKET s, uint8 protocol, uint16 port, uint8 flag)
{
	uint8 ret;
    
    Transf("\r\n");
	Transf("socket()\r\n");

	if ((protocol == Sn_MR_TCP) || (protocol == Sn_MR_UDP) || (protocol == Sn_MR_IPRAW) || (protocol == Sn_MR_MACRAW) || (protocol == Sn_MR_PPPOE))
	{
	close(s);
		IINCHIP_WRITE(Sn_MR(s),protocol | flag);
		if (port != 0) {
			IINCHIP_WRITE(Sn_PORT0(s),(uint8)((port & 0xff00) >> 8));
			IINCHIP_WRITE((Sn_PORT0(s) + 1),(uint8)(port & 0x00ff));
		} else {
			local_port++; // if don't set the source port, set local_port number.
			IINCHIP_WRITE(Sn_PORT0(s),(uint8)((local_port & 0xff00) >> 8));
			IINCHIP_WRITE((Sn_PORT0(s) + 1),(uint8)(local_port & 0x00ff));
		}
		IINCHIP_WRITE(Sn_CR(s),Sn_CR_OPEN); // run sockinit Sn_CR

		/* wait to process the command... */
		while( IINCHIP_READ(Sn_CR(s)) ) 
			;
		/* ------- */
		ret = 1;
	}
	else
	{
		ret = 0;
	}
	/*
	itoa(IINCHIP_READ(Sn_SR(s)),strng1);
	Transf("Sn_SR =");
	Transf(strng1);
	Transf(";");
    itoa(IINCHIP_READ(Sn_MR(s)),strng1);
	Transf(" Protocol =");
	Transf(strng1);
	Transf("\r\n");
	*/
	return ret;
}


/**
@brief	This function close the socket and parameter is "s" which represent the socket number
*/ 
void close(SOCKET s)
{

    Transf("\r");
    Transf("close()");
    Transf("\r");
	
	 
	IINCHIP_WRITE(Sn_CR(s),Sn_CR_CLOSE);

	/* wait to process the command... */
	while( IINCHIP_READ(Sn_CR(s)) ) 
		;
	/* ------- */
                /* all clear */
	IINCHIP_WRITE(Sn_IR(s), 0xFF);
}


/**
@brief	This function established  the connection for the channel in passive (server) mode. This function waits for the request from the peer.
@return	1 for success else 0.
*/ 


uint8 listen(SOCKET s)
{
	uint8 ret;

	Transf("\r");
    Transf("listen()\r\n");
	

	if (IINCHIP_READ(Sn_SR(s)) == SOCK_INIT)
	{
		IINCHIP_WRITE(Sn_CR(s),Sn_CR_LISTEN);
		// wait to process the command... 
		while( IINCHIP_READ(Sn_CR(s)) ) 
			;
		//-----------------
		ret = 1;
	}
	else
	{
		ret = 0;

	Transf("Fail[invalid ip,port]\r\n");	

	}
	return ret;
}




/**
@brief	This function established  the connection for the channel in Active (client) mode. 
		This function waits for the untill the connection is established.
		
@return	1 for success else 0.
*/ 
uint8 connect(SOCKET s, uint8 * addr, uint16 port)
{
	uint8 ret;

Transf("\r\n");
	Transf("connect()\r\n");

	if 
		(
			((addr[0] == 0xFF) && (addr[1] == 0xFF) && (addr[2] == 0xFF) && (addr[3] == 0xFF)) ||
		 	((addr[0] == 0x00) && (addr[1] == 0x00) && (addr[2] == 0x00) && (addr[3] == 0x00)) ||
		 	(port == 0x00) 
		) 
 	{
 		ret = 0;
Transf("Fail[invalid ip,port]\r\n");	
	}
	else
	{
		ret = 1;
		// set destination IP
		IINCHIP_WRITE(Sn_DIPR0(s),addr[0]);
		IINCHIP_WRITE((Sn_DIPR0(s) + 1),addr[1]);
		IINCHIP_WRITE((Sn_DIPR0(s) + 2),addr[2]);
		IINCHIP_WRITE((Sn_DIPR0(s) + 3),addr[3]);
		IINCHIP_WRITE(Sn_DPORT0(s),(uint8)((port & 0xff00) >> 8));
		IINCHIP_WRITE((Sn_DPORT0(s) + 1),(uint8)(port & 0x00ff));
		setSUBR();	 // set the subnet mask register
		IINCHIP_WRITE(Sn_CR(s),Sn_CR_CONNECT);
                /* wait for completion */
		while ( IINCHIP_READ(Sn_CR(s)) ) ;
		while ( IINCHIP_READ(Sn_SR(s)) != SOCK_SYNSENT )
		{
			if(IINCHIP_READ(Sn_SR(s)) == SOCK_ESTABLISHED)
			{
				break;
			}
			if (getSn_IR(s) & Sn_IR_TIMEOUT)
			{
				IINCHIP_WRITE(Sn_IR(s), (Sn_IR_TIMEOUT));  // clear TIMEOUT Interrupt
				ret = 0;
				break;
			}
		}
		clearSUBR();   // clear the subnet mask again and keep it because of the ARP errata of W5100

	}
	
	return ret;
}



/**
@brief	This function used for disconnect the socket and parameter is "s" which represent the socket number
@return	1 for success else 0.
*/ 
void disconnect(SOCKET s)
{
Transf("disconnect()\r\n");
	IINCHIP_WRITE(Sn_CR(s),Sn_CR_DISCON);

	/* wait to process the command... */
	while( IINCHIP_READ(Sn_CR(s)) ) 
		;
	/* ------- */
}


/**
@brief	This function used to send the data in TCP mode
@return	1 for success else 0.
*/ 
uint16 send(SOCKET s, const uint8 * buf, uint16 len, bool retry)
{
	uint8 status=0;
	uint16 ret=0;
	uint16 freesize=0;
	uint16 txrd, txrd_before_send;
/*
     Transf("\r\n");
     Transf("send()\r\n");
*/
	if(retry) ;
	else {     
		if (len > getIINCHIP_TxMAX(s)) ret = getIINCHIP_TxMAX(s); // check size not to exceed MAX size.
		else ret = len;

		// if freebuf is available, start.
		do 
		{
			freesize = getSn_TX_FSR(s);
			status = IINCHIP_READ(Sn_SR(s));
			if ((status != SOCK_ESTABLISHED) && (status != SOCK_CLOSE_WAIT))
			{
				ret = 0; 
				break;
			}
 /*
    itoa(s,strng1);
   
	Transf("socket =");
	Transf(strng1);
	Transf("\r");
	
    itoa(freesize,strng1);
    
	Transf("freesize(");
	Transf(strng1);
	Transf(")\r\n");
*/

		} while (freesize < ret);

		// copy data
		send_data_processing(s, (uint8 *)buf, ret);
	}


//	if(ret != 0) // error code
// 2013-07-30 wiznet fix the code to add condition "retry"
	if(retry || ret != 0)
	{
		txrd_before_send = IINCHIP_READ(Sn_TX_RD0(s));
		txrd_before_send = (txrd_before_send << 8) + IINCHIP_READ(Sn_TX_RD0(s) + 1);
  
		IINCHIP_WRITE(Sn_CR(s),Sn_CR_SEND);
	
		/* wait to process the command... */
		while( IINCHIP_READ(Sn_CR(s)) );

		while ( (IINCHIP_READ(Sn_IR(s)) & Sn_IR_SEND_OK) != Sn_IR_SEND_OK )
                {
			if(IINCHIP_READ(Sn_IR(s)) == SOCK_CLOSED)
			{
               	Transf("SOCK_CLOSED.\r\n");  
				close(s);
				return 0;
			}
		}
		IINCHIP_WRITE(Sn_IR(s), Sn_IR_SEND_OK);

		txrd = IINCHIP_READ(Sn_TX_RD0(s));
		txrd = (txrd << 8) + IINCHIP_READ(Sn_TX_RD0(s) + 1);

		if(txrd > txrd_before_send) {
			ret = txrd - txrd_before_send;
		} else {
			ret = (0xffff - txrd_before_send) + txrd + 1;
		}
	}
                
	return ret;
}


/**
@brief	This function is an application I/F function which is used to receive the data in TCP mode.
		It continues to wait for data as much as the application wants to receive.
		
@return	received data size for success else -1.
*/ 
uint16 recv(SOCKET s, uint8 * buf, uint16 len)
{
	uint16 ret=0;

//Transf("recv()\r\n");

	if ( len > 0u )
	{
		recv_data_processing(s, buf, len);
		IINCHIP_WRITE(Sn_CR(s),Sn_CR_RECV);

		/* wait to process the command... */
		while( IINCHIP_READ(Sn_CR(s)));
		/* ------- */
	
		ret = len;
	}
	return ret;
}


/**
@brief	This function is an application I/F function which is used to send the data for other then TCP mode. 
		Unlike TCP transmission, The peer's destination address and the port is needed.
		
@return	This function return send data size for success else -1.
*/ 
uint16 sendto(SOCKET s, const uint8 * buf, uint16 len, uint8 * addr, uint16 port)
{
	uint16 ret=0;
	

   if (len > getIINCHIP_TxMAX(s)) ret = getIINCHIP_TxMAX(s); // check size not to exceed MAX size.
   else ret = len;

	if
		(
		 	((addr[0] == 0x00) && (addr[1] == 0x00) && (addr[2] == 0x00) && (addr[3] == 0x00)) ||
		 	((port == 0x00)) ||(ret == 0)
		) 
 	{
 	   /* added return value */
 	   ret = 0;

	}
	else
	{
		IINCHIP_WRITE(Sn_DIPR0(s),addr[0]);
		IINCHIP_WRITE((Sn_DIPR0(s) + 1),addr[1]);
		IINCHIP_WRITE((Sn_DIPR0(s) + 2),addr[2]);
		IINCHIP_WRITE((Sn_DIPR0(s) + 3),addr[3]);
		IINCHIP_WRITE(Sn_DPORT0(s),(uint8)((port & 0xff00) >> 8));
		IINCHIP_WRITE((Sn_DPORT0(s) + 1),(uint8)(port & 0x00ff));     			
  		// copy data
 		send_data_processing(s, (uint8 *)buf, ret);
		setSUBR();    // set the subnet mask register
		IINCHIP_WRITE(Sn_CR(s),Sn_CR_SEND);
		/* wait to process the command... */
		while( IINCHIP_READ(Sn_CR(s)) ) 
			;
		/* ------- */

	   while ( (IINCHIP_READ(Sn_IR(s)) & Sn_IR_SEND_OK) != Sn_IR_SEND_OK ) 
	   {
    	        if (IINCHIP_READ(Sn_IR(s)) & Sn_IR_TIMEOUT)
		{

                /* clear interrupt */
           	IINCHIP_WRITE(Sn_IR(s), (Sn_IR_SEND_OK | Sn_IR_TIMEOUT)); /* clear SEND_OK & TIMEOUT */
		return 0;
		}
	   }

	   clearSUBR();	   // clear the subnet mask again and keep it because of the ARP errata of W5100
	   IINCHIP_WRITE(Sn_IR(s), Sn_IR_SEND_OK);
	}
	return ret;
}


/**
@brief	This function is an application I/F function which is used to receive the data in other then
	TCP mode. This function is used to receive UDP, IP_RAW and MAC_RAW mode, and handle the header as well. 
	
@return	This function return received data size for success else -1.
*/ 
uint16 recvfrom(SOCKET s, uint8 * buf, uint16 len, uint8 * addr, uint16 *port)
{
	uint8 head[8];
	uint16 data_len=0;
	uint16 ptr=0;

	if ( len > 0u )
	{
   	ptr = IINCHIP_READ(Sn_RX_RD0(s));
   	ptr = ((ptr & 0x00ff) << 8) + IINCHIP_READ(Sn_RX_RD0(s) + 1);

   	switch (IINCHIP_READ(Sn_MR(s)) & 0x07)
   	{
   	case Sn_MR_UDP :
   			read_data(s, (uint8 *)ptr, head, 0x08);
   			ptr += 8;
   			// read peer's IP address, port number.
    		addr[0] = head[0];
   			addr[1] = head[1];
   			addr[2] = head[2];
   			addr[3] = head[3];
   			*port = head[4];
   			*port = (*port << 8) + head[5];
   			data_len = head[6];
   			data_len = (data_len << 8) + head[7];

			read_data(s, (uint8 *)ptr, buf, data_len); // data copy.
			ptr += data_len;

			IINCHIP_WRITE(Sn_RX_RD0(s),(uint8)((ptr & 0xff00) >> 8));
			IINCHIP_WRITE((Sn_RX_RD0(s) + 1),(uint8)(ptr & 0x00ff));
   			break;
   
   	case Sn_MR_IPRAW :
   			read_data(s, (uint8 *)ptr, head, 0x06);
   			ptr += 6;
   
   			addr[0] = head[0];
   			addr[1] = head[1];
   			addr[2] = head[2];
   			addr[3] = head[3];
   			data_len = head[4];
   			data_len = (data_len << 8) + head[5];
   	
			read_data(s, (uint8 *)ptr, buf, data_len); // data copy.
			ptr += data_len;

			IINCHIP_WRITE(Sn_RX_RD0(s),(uint8)((ptr & 0xff00) >> 8));
			IINCHIP_WRITE((Sn_RX_RD0(s) + 1),(uint8)(ptr & 0x00ff));
   			break;
   	case Sn_MR_MACRAW :
   			read_data(s,(uint8*)ptr,head,2);
   			ptr+=2;
   			data_len = head[0];
   			data_len = (data_len<<8) + head[1] - 2;
   			if(data_len > 1514u) 
   			{
   			//	printf("data_len over 1514\r\n");
   				while(1);
   			}

   			read_data(s,(uint8*) ptr,buf,data_len);
   			ptr += data_len;
   			IINCHIP_WRITE(Sn_RX_RD0(s),(uint8)((ptr & 0xff00) >> 8));
   			IINCHIP_WRITE((Sn_RX_RD0(s) + 1),(uint8)(ptr & 0x00ff));
   			
	
			break;

   	default :
   			break;
   	}
		IINCHIP_WRITE(Sn_CR(s),Sn_CR_RECV);

		/* wait to process the command... */
		while( IINCHIP_READ(Sn_CR(s)) ) ;
		/* ------- */
	}

 	return data_len;
}





/**
@brief	This function reads the value from W5200 registers.
*/
uint8 IINCHIP_READ(uint16 addr)
{
	uint8 data;
        
//	IINCHIP_ISR_DISABLE();                       // Interrupt Service Routine Disable
	
	IINCHIP_CSoff();                             // CS=0, SPI start
	
    IINCHIP_SpiSendData((addr & 0xFF00) >> 8);   // Address byte 1
	IINCHIP_SpiSendData(addr & 0x00FF);          // Address byte 2
	IINCHIP_SpiSendData(0x00);                   // Data read command and Read data length 1
	IINCHIP_SpiSendData(0x01);                   // Read data length 2   	
	data = IINCHIP_SpiSendData(0x00);            // Data read (read 1byte data)
	
    IINCHIP_CSon();                              // CS=1,  SPI end
	
// IINCHIP_ISR_ENABLE();                        // Interrupt Service Routine Enable
	return data;
}

/**
@brief	This function writes into W5200 memory(Buffer)
*/ 
uint16 wiz_write_buf(uint16 addr,uint8* buf,uint16 len)
{
	uint16 idx = 0;
	
//	IINCHIP_ISR_DISABLE();
        
        if(len == 0)
        {
          //printf("len is '0'\r\n");
          return 0;
        }
     
	//SPI MODE I/F
	IINCHIP_CSoff();                                        // CS=0, SPI start 
	
        IINCHIP_SpiSendData(((addr+idx) & 0xFF00) >> 8);        // Address byte 1
	IINCHIP_SpiSendData((addr+idx) & 0x00FF);               // Address byte 2
	IINCHIP_SpiSendData((0x80 | ((len & 0x7F00) >> 8)));    // Data write command and Write data length 1
	IINCHIP_SpiSendData((len & 0x00FF));                    // Write data length 2
	for(idx = 0; idx < len; idx++)                          // Write data in loop
	{		
		IINCHIP_SpiSendData(buf[idx]);
	}
	
       IINCHIP_CSon();                                         // CS=1, SPI end 
        
//     IINCHIP_ISR_ENABLE();                                   // Interrupt Service Routine Enable        
	return len;
}


/**
@brief	This function reads into W5200 memory(Buffer)
*/ 
uint16 wiz_read_buf(uint16 addr, uint8* buf,uint16 len)
{
	uint16 idx = 0;
        
//	IINCHIP_ISR_DISABLE();                                  // Interrupt Service Routine Disable
        
	IINCHIP_CSoff();                                        // CS=0, SPI start 
        
	IINCHIP_SpiSendData(((addr+idx) & 0xFF00) >> 8);        // Address byte 1
	IINCHIP_SpiSendData((addr+idx) & 0x00FF);               // Address byte 2
	IINCHIP_SpiSendData((0x00 | ((len & 0x7F00) >> 8)));    // Data read command
	IINCHIP_SpiSendData((len & 0x00FF));		        

	for(idx = 0; idx < len; idx++)                          // Read data in loop
	{
	 	buf[idx] = IINCHIP_SpiSendData(0x00);
		
	}
        
	IINCHIP_CSon();                                         // CS=1, SPI end 	   	
        
//	IINCHIP_ISR_ENABLE();                                   // Interrupt Service Routine Enable
	return len;
}


/**
@brief	This function is for resetting of the iinchip. Initializes the iinchip to work in whether DIRECT or INDIRECT mode
*/ 
void iinchip_init(void)
{	
	Transf("MR_RST\r\n");
	setMR( MR_RST );	
}


/**
@brief	This function set the transmit & receive buffer size as per the channels is used
Note for TMSR and RMSR bits are as follows\n
bit 1-0 : memory size of channel #0 \n
bit 3-2 : memory size of channel #1 \n
bit 5-4 : memory size of channel #2 \n
bit 7-6 : memory size of channel #3 \n
bit 9-8 : memory size of channel #4 \n
bit 11-10 : memory size of channel #5 \n
bit 12-12 : memory size of channel #6 \n
bit 15-14 : memory size of channel #7 \n
Maximum memory size for Tx, Rx in the W5200 is 16K Bytes,\n
In the range of 16KBytes, the memory size could be allocated dynamically by each channel.\n
Be attentive to sum of memory size shouldn't exceed 8Kbytes\n
and to data transmission and receiption from non-allocated channel may cause some problems.\n
If the 16KBytes memory is already  assigned to centain channel, \n
other 3 channels couldn't be used, for there's no available memory.\n
If two 4KBytes memory are assigned to two each channels, \n
other 2 channels couldn't be used, for there's no available memory.\n
*/ 
void sysinit( uint8 * tx_size, uint8 * rx_size	)
{
	int16 i;
	int16 ssum,rsum;


	ssum = 0;
	rsum = 0;
	
	SBUFBASEADDRESS[0] = (uint16)(__DEF_IINCHIP_MAP_TXBUF__);		/* Set base address of Tx memory for channel #0 */
	RBUFBASEADDRESS[0] = (uint16)(__DEF_IINCHIP_MAP_RXBUF__);		/* Set base address of Rx memory for channel #0 */

  for (i = 0 ; i < MAX_SOCK_NUM; i++)       // Set the size, masking and base address of Tx & Rx memory by each channel
	{
    IINCHIP_WRITE((Sn_TXMEM_SIZE(i)),tx_size[i]);
    IINCHIP_WRITE((Sn_RXMEM_SIZE(i)),rx_size[i]);
    /*
    Transf("Sn_TXMEM_SIZE = ");
    itoa(IINCHIP_READ(Sn_TXMEM_SIZE(i)),strng1);
 	Transf(strng1);
	Transf("\r\n");
    Transf("Sn_RXMEM_SIZE = ");
    itoa(IINCHIP_READ(Sn_RXMEM_SIZE(i)),strng1);
 	Transf(strng1);
	Transf("\r\n");
	*/

		SSIZE[i] = (int16)(0);
		RSIZE[i] = (int16)(0);

		if (ssum <= 16384)
		{
         switch( tx_size[i] )
			{
			case 1:
				SSIZE[i] = (int16)(1024);
				SMASK[i] = (uint16)(0x03FF);
				break;
			case 2:
				SSIZE[i] = (int16)(2048);
				SMASK[i] = (uint16)(0x07FF);
				break;
			case 4:
				SSIZE[i] = (int16)(4096);
				SMASK[i] = (uint16)(0x0FFF);
				break;
			case 8:
				SSIZE[i] = (int16)(8192);
				SMASK[i] = (uint16)(0x1FFF);
				break;
			case 16:
				SSIZE[i] = (int16)(16384);
				SMASK[i] = (uint16)(0x3FFF);
			break;
			}
		}

		if (rsum <= 16384)
		{
         switch( rx_size[i] )
			{
			case 1:
				RSIZE[i] = (int16)(1024);
				RMASK[i] = (uint16)(0x03FF);
				break;
			case 2:
				RSIZE[i] = (int16)(2048);
				RMASK[i] = (uint16)(0x07FF);
				break;
			case 4:
				RSIZE[i] = (int16)(4096);
				RMASK[i] = (uint16)(0x0FFF);
				break;
			case 8:
				RSIZE[i] = (int16)(8192);
				RMASK[i] = (uint16)(0x1FFF);
				break;
			case 16:
				RSIZE[i] = (int16)(16384);
				RMASK[i] = (uint16)(0x3FFF);
				break;
			}
		}
		ssum += SSIZE[i];
		rsum += RSIZE[i];

        if (i != 0)             // Sets base address of Tx and Rx memory for channel #1,#2,#3
		{
			SBUFBASEADDRESS[i] = SBUFBASEADDRESS[i-1] + SSIZE[i-1];
			RBUFBASEADDRESS[i] = RBUFBASEADDRESS[i-1] + RSIZE[i-1];
		}
/*
		Transf("ch =");
		itoa(i,strng1);
 	    Transf(strng1);
	    Transf("\r\n");
		Transf("SBUFBASEADDRESS =");
		itoa((uint16)SBUFBASEADDRESS[i],strng1);
 	    Transf(strng1);
	    Transf("\r\n");
		Transf("RBUFBASEADDRESS =");
		itoa((uint16)RBUFBASEADDRESS[i],strng1);
 	    Transf(strng1);
	    Transf("\r\n");
		Transf("SSIZE =");
		itoa(SSIZE[i],strng1);
 	    Transf(strng1);
	    Transf("\r\n");
		Transf("RSIZE =");	
		itoa(RSIZE[i],strng1);
 	    Transf(strng1);
	    Transf("\r\n");
*/
	}
}

// added

/**
@brief	This function sets up gateway IP address.
*/ 
void setGAR(
	uint8 * addr	/**< a pointer to a 4 -byte array responsible to set the Gateway IP address. */
	)
{
	IINCHIP_WRITE((GAR0 + 0),addr[0]);
	IINCHIP_WRITE((GAR0 + 1),addr[1]);
	IINCHIP_WRITE((GAR0 + 2),addr[2]);
	IINCHIP_WRITE((GAR0 + 3),addr[3]);
}

/*
void getGWIP(uint8 * addr)
{
	addr[0] = IINCHIP_READ((GAR0 + 0));
	addr[1] = IINCHIP_READ((GAR0 + 1));
	addr[2] = IINCHIP_READ((GAR0 + 2));
	addr[3] = IINCHIP_READ((GAR0 + 3));
}
*/

/**
@brief	It sets up SubnetMask address
*/ 
void saveSUBR(
	uint8 * addr	/**< a pointer to a 4 -byte array responsible to set the SubnetMask address */
	)
{
	SUBNET[0] = addr[0];
	SUBNET[1] = addr[1];
	SUBNET[2] = addr[2];
	SUBNET[3] = addr[3];
}

void setSUBR(void)
{   
	IINCHIP_WRITE((SUBR0 + 0), SUBNET[0]);
	IINCHIP_WRITE((SUBR0 + 1), SUBNET[1]);
	IINCHIP_WRITE((SUBR0 + 2), SUBNET[2]);
	IINCHIP_WRITE((SUBR0 + 3), SUBNET[3]);
}

void clearSUBR(void)
{
	IINCHIP_WRITE((SUBR0 + 0), 0);
	IINCHIP_WRITE((SUBR0 + 1), 0);
	IINCHIP_WRITE((SUBR0 + 2), 0);
	IINCHIP_WRITE((SUBR0 + 3), 0);
}


/**
@brief	This function sets up MAC address.
*/ 
void setSHAR(
	uint8 * addr	/**< a pointer to a 6 -byte array responsible to set the MAC address. */
	)
{
	IINCHIP_WRITE((SHAR0 + 0),addr[0]);
	IINCHIP_WRITE((SHAR0 + 1),addr[1]);
	IINCHIP_WRITE((SHAR0 + 2),addr[2]);
	IINCHIP_WRITE((SHAR0 + 3),addr[3]);
	IINCHIP_WRITE((SHAR0 + 4),addr[4]);
	IINCHIP_WRITE((SHAR0 + 5),addr[5]);
}

/**
@brief	This function sets up Source IP address.
*/
void setSIPR(
	uint8 * addr	/**< a pointer to a 4 -byte array responsible to set the Source IP address. */
	)
{
	IINCHIP_WRITE((SIPR0 + 0),addr[0]);
	IINCHIP_WRITE((SIPR0 + 1),addr[1]);
	IINCHIP_WRITE((SIPR0 + 2),addr[2]);
	IINCHIP_WRITE((SIPR0 + 3),addr[3]);
}

/**
@brief	This function sets up Source IP address.
*/
void getGAR(uint8 * addr)
{
	addr[0] = IINCHIP_READ(GAR0);
	addr[1] = IINCHIP_READ(GAR0+1);
	addr[2] = IINCHIP_READ(GAR0+2);
	addr[3] = IINCHIP_READ(GAR0+3);
}
void getSUBR(uint8 * addr)
{
	addr[0] = SUBNET[0];
	addr[1] = SUBNET[1];
	addr[2] = SUBNET[2];
	addr[3] = SUBNET[3];
}
void getSHAR(uint8 * addr)
{
	addr[0] = IINCHIP_READ(SHAR0);
	addr[1] = IINCHIP_READ(SHAR0+1);
	addr[2] = IINCHIP_READ(SHAR0+2);
	addr[3] = IINCHIP_READ(SHAR0+3);
	addr[4] = IINCHIP_READ(SHAR0+4);
	addr[5] = IINCHIP_READ(SHAR0+5);
}
void getSIPR(uint8 * addr)
{
	addr[0] = IINCHIP_READ(SIPR0);
	addr[1] = IINCHIP_READ(SIPR0+1);
	addr[2] = IINCHIP_READ(SIPR0+2);
	addr[3] = IINCHIP_READ(SIPR0+3);
}

void setMR(uint8 val)
{
	Transf("setMR\r\n");
	IINCHIP_WRITE(MR,val);
}

/**
@brief	This function gets Interrupt register in common register.
 */
uint8 getIR( void )
{
   return IINCHIP_READ(IR);
}


/**
 Retransmittion 
 **/
 
/**
@brief	This function sets up Retransmission time.

If there is no response from the peer or delay in response then retransmission 
will be there as per RTR (Retry Time-value Register)setting
*/
void setRTR(uint16 timeout)
{
	IINCHIP_WRITE(RTR,(uint8)((timeout & 0xff00) >> 8));
	IINCHIP_WRITE((RTR + 1),(uint8)(timeout & 0x00ff));
}

/**
@brief	This function set the number of Retransmission.

If there is no response from the peer or delay in response then recorded time 
as per RTR & RCR register seeting then time out will occur.
*/
void setRCR(uint8 retry)
{
	IINCHIP_WRITE(RCR,retry);
}




/**
@brief	This function set the interrupt mask Enable/Disable appropriate Interrupt. ('1' : interrupt enable)

If any bit in IMR is set as '0' then there is not interrupt signal though the bit is
set in IR register.
*/
void setIMR(uint8 mask)
{
	IINCHIP_WRITE(IMR,mask); // must be setted 0x10.
}

/**
@brief	This sets the maximum segment size of TCP in Active Mode), while in Passive Mode this is set by peer
*/
void setSn_MSS(SOCKET s, uint16 Sn_MSSR0)
{
	IINCHIP_WRITE(Sn_MSSR0(s),(uint8)((Sn_MSSR0 & 0xff00) >> 8));
	IINCHIP_WRITE((Sn_MSSR0(s) + 1),(uint8)(Sn_MSSR0 & 0x00ff));
}

void setSn_TTL(SOCKET s, uint8 ttl)
{
   IINCHIP_WRITE(Sn_TTL(s), ttl);
}


/**
@brief	These below function is used to setup the Protocol Field of IP Header when
		executing the IP Layer RAW mode.
*/
void setSn_PROTO(SOCKET s, uint8 proto)
{
	IINCHIP_WRITE(Sn_PROTO(s),proto);
}


/**
@brief	get socket interrupt status

These below functions are used to read the Interrupt & Soket Status register
*/
uint8 getSn_IR(SOCKET s)
{
   return IINCHIP_READ(Sn_IR(s));
}


/**
@brief	 get socket status
*/
uint8 getSn_SR(SOCKET s)
{
   return IINCHIP_READ(Sn_SR(s));
}


/**
@brief	get socket TX free buf size

This gives free buffer size of transmit buffer. This is the data size that user can transmit.
User shuold check this value first and control the size of transmitting data
*/
uint16 getSn_TX_FSR(SOCKET s)
{
	uint16 val=0,val1=0;
	do
	{
		val1 = IINCHIP_READ(Sn_TX_FSR0(s));
		val1 = (val1 << 8) + IINCHIP_READ(Sn_TX_FSR0(s) + 1);
      if (val1 != 0)
		{
   			val = IINCHIP_READ(Sn_TX_FSR0(s));
   			val = (val << 8) + IINCHIP_READ(Sn_TX_FSR0(s) + 1);
		}
	} while (val != val1);
   return val;
}


/**
@brief	 get socket RX recv buf size

This gives size of received data in receive buffer. 
*/
uint16 getSn_RX_RSR(SOCKET s)
{
	uint16 val=0,val1=0;
	do
	{
		val1 = IINCHIP_READ(Sn_RX_RSR0(s));
		val1 = (val1 << 8) + IINCHIP_READ(Sn_RX_RSR0(s) + 1);
      if(val1 != 0)
		{
   			val = IINCHIP_READ(Sn_RX_RSR0(s));
   			val = (val << 8) + IINCHIP_READ(Sn_RX_RSR0(s) + 1);
		}
	} while (val != val1);
   return val;
}


/**
@brief	 This function is being called by send() and sendto() function also. 

This function read the Tx write pointer register and after copy the data in buffer update the Tx write pointer
register. User should read upper byte first and lower byte later to get proper value.
*/
void send_data_processing(SOCKET s, uint8 *data, uint16 len)
{
	
	uint16 ptr;
	ptr = IINCHIP_READ(Sn_TX_WR0(s));
	ptr = (ptr << 8) + IINCHIP_READ(Sn_TX_WR0(s) + 1);
	write_data(s, data, (uint8 *)(ptr), len);
	ptr += len;

	IINCHIP_WRITE(Sn_TX_WR0(s),(uint8)((ptr & 0xff00) >> 8));
	IINCHIP_WRITE((Sn_TX_WR0(s) + 1),(uint8)(ptr & 0x00ff));
	
}


/**
@brief	This function is being called by recv() also.

This function read the Rx read pointer register
and after copy the data from receive buffer update the Rx write pointer register.
User should read upper byte first and lower byte later to get proper value.
*/
void recv_data_processing(SOCKET s, uint8 *data, uint16 len)
{
	uint16 ptr;
	ptr = IINCHIP_READ(Sn_RX_RD0(s));
	ptr = ((ptr & 0x00ff) << 8) + IINCHIP_READ(Sn_RX_RD0(s) + 1);
/*
	Transf(" ISR_RX: rd_ptr :");
    itoa(ptr,strng1);
    Transf(strng1);
   	Transf("\r\n");
*/
	read_data(s, (uint8 *)ptr, data, len); // read data
	ptr += len;
	IINCHIP_WRITE(Sn_RX_RD0(s),(uint8)((ptr & 0xff00) >> 8));
	IINCHIP_WRITE((Sn_RX_RD0(s) + 1),(uint8)(ptr & 0x00ff));
}


/**
@brief	for copy the data form application buffer to Transmite buffer of the chip.

This function is being used for copy the data form application buffer to Transmite
buffer of the chip. It calculate the actual physical address where one has to write
the data in transmite buffer. Here also take care of the condition while it exceed
the Tx memory uper-bound of socket.
*/
void write_data(SOCKET s, vuint8 * src, vuint8 * dst, uint16 len)
{
	uint16 size;
	uint16 dst_mask;
	uint8 * dst_ptr;

	dst_mask = (uint32)dst & getIINCHIP_TxMASK(s);
	dst_ptr = (uint8 *)(getIINCHIP_TxBASE(s) + dst_mask);
	
	if (((uint16)(dst_mask + len)) > getIINCHIP_TxMAX(s)) 
	{
		size = getIINCHIP_TxMAX(s) - dst_mask;
		wiz_write_buf((uint32)dst_ptr, (uint8*)src, size);
		src += size;
		size = len - size;
		dst_ptr = (uint8 *)(getIINCHIP_TxBASE(s));
		wiz_write_buf((uint32)dst_ptr, (uint8*)src, size);
	} 
	else
	{
		wiz_write_buf((uint32)dst_ptr, (uint8*)src, len);
	}
}


/**
@brief	This function is being used for copy the data form Receive buffer of the chip to application buffer.

It calculate the actual physical address where one has to read
the data from Receive buffer. Here also take care of the condition while it exceed
the Rx memory uper-bound of socket.
*/
void read_data(SOCKET s, vuint8 * src, vuint8 * dst, uint16 len)
{
	uint16 size;
	uint16 src_mask;
	uint8 * src_ptr;

	src_mask = (uint32)src & getIINCHIP_RxMASK(s);
	src_ptr = (uint8 *)(getIINCHIP_RxBASE(s) + src_mask);
	
	if( ((uint16)(src_mask + len)) > getIINCHIP_RxMAX(s) ) 
	{
		size = getIINCHIP_RxMAX(s) - src_mask;
		wiz_read_buf((uint32)src_ptr, (uint8*)dst,size);
		
		dst += size;
		size = len - size;
		src_ptr = (uint8 *)(getIINCHIP_RxBASE(s));
		wiz_read_buf((uint32)src_ptr, (uint8*) dst,size);
	} 
	else
	{
		wiz_read_buf((uint32)src_ptr, (uint8*) dst,len);
		
	}
}




//*************************************************************************************************************************************

uint32 destip_UDP    =0; //IP корреспондента UDP
uint16 destport_UDP  =0;  //port корреспондента UDP
uint16 FLAG_UDP_EVENT=0u;
u8 FLAG_UDP_INSTALL	 =0u;

void RECEIVE_udp(SOCKET s, uint16 port,unsigned char event)
{
	 int16 RSR_len;
    uint16 received_len;
	uint32 lsr=0;
	PORT_my=port;
   
 	IINCHIP_WRITE(Sn_IR(s), 0xff);  //сброс прерываний, если были	
	
	switch (getSn_SR(s))
	{
	case SOCK_UDP:
				
				if ((RSR_len = getSn_RX_RSR(s)) > 0) 														/* check Rx data */
					{
						if (RSR_len > RX_MAX_BUF_SIZE) RSR_len = RX_MAX_BUF_SIZE;							/* if Rx data size is lager than RX_MAX_BUF_SIZE */
																												/* the data size to read is MAX_BUF_SIZE. */
						received_len = recvfrom(s, RX_BUF, RSR_len, (uint8*)&destip_UDP, &destport_UDP);				/* read the received data */
						//******************************************************************************************
						   	      lsr=(uint32) received_len;
								  lsr_razmer=lsr;
							      flag_RX_UDP=1;
							      FLAG_UDP_INSTALL=1;
							      sch_packet_UDP_reciv=sch_packet_UDP_reciv+1; //счётчик пакетов
							/* 	  Transf("\r\n");
								  Transf("Принял пакет!\r\n");
								  u_out("port:",destport_UDP);
								  u_out("IP  :",destip_UDP);
								  u_out("len :",received_len);
								  Transf("\r\n"); */
								  FRAME_DECODE (RX_BUF,received_len);
			   		    //******************************************************************************************
						FLAG_UDP_EVENT = 1; //поднимаем флаг вновь установленного UDP соеденения
					}	
		break;
	case SOCK_CLOSED:	
	//FLAG_UDP_EVENT = 0; //снимаем флаг вновь установленного UDP соеденения
	Transf("\r\n");
	u_out("",s);                                          /* if a socket is closed */
    Transf(": Loop-Back UDP Started. port :");
	u_out("",port); 
                             
    if(socket(s,Sn_MR_UDP,port,0x00)== 0)   {Transf("\a : Fail to create socket.");		 /* reinitialize the socket */
	u_out("",s);
    Transf("\r\n");};
		break;
	}
}

void SEND_udp(SOCKET s, uint16 port,uint32 dest_IP,uint16 dest_PORT)
{
	 int16 RSR_len;
    uint16 received_len;
	uint32 lsr=0;
   
 	IINCHIP_WRITE(Sn_IR(s), 0xff);  //сброс прерываний, если были	
	
	switch (getSn_SR(s))
	{
	case SOCK_UDP:
				if (FLAG_UDP_INSTALL==1u) //означает то что ранее была принята посылка по UDP и получены адреса отправителя
					{
						sch_tx_packet++;//счётчик отправленных пакетов
						
						if(sendto(s, TX_BUF,TX_BUF_LENGTH,(uint8*)&dest_IP, dest_PORT) == 0)	 // send the message data 
										{
											Transf("\a\a\a : System Fatal Error.");
											u_out("",s);
										}
					}
		break;
	case SOCK_CLOSED:	
	//FLAG_UDP_EVENT = 0; //снимаем флаг вновь установленного UDP соеденения
	Transf("\r\n");
	u_out("",s);                                          /* if a socket is closed */
    Transf(": Loop-Back UDP Started. port :");
	u_out("",port); 
                             
    if(socket(s,Sn_MR_UDP,port,0x00)== 0)   {Transf("\a : Fail to create socket.");		 /* reinitialize the socket */
	u_out("",s);
    Transf("\r\n");};
		break;
	}
}


extern  Frame INVOICE[quantity_SENDER];	//структура квитанций о состоянии дел, по числу потенциальный адресатов
extern  Frame FRM1   [quantity_SENDER];	//принятый фрейм, по числу потенциальный адресатов
extern  SERVER SERV1;					//структура "Хранилище"
extern  ID_SERVER ID_SERV1;				//структура указатель для "Хранилище"
extern  CMD_RUN CMD1;
extern  ADR_SENDER ADDR_SNDR;			//структура хранит массив адресов ОТПРАВИТЕЛЕЙ 
extern  u64 TIME_SYS;					//переменная хранить системное время в милисекундах
extern  u32 ID_CMD;						//переменная хранить текущий ID наших квитанций 

u32 ADR_FINDER (u64 adr,ADR_SENDER *a) //ищем адрес отправителя в структуре адресов (его порядковый номер)
{
	u32 i=0;
	u32 n=0;

	n=a->IDX;
	//ищем среди уже записаных адресов
	while (i<quantity_SENDER)
	{
		if (adr==a->A[i]) break;
		i++;
	}

	if ((i==quantity_SENDER)&&(n==quantity_SENDER)) //проверяем на переполнение списка отправителей
	{
		Transf("Превышено число ОТПРАВИТЕЛЕЙ!\r\n");//записываем отправителя в нулевой адрес структуры
		i=0;
		a->A[0]=adr;
	} else 
		if (i==quantity_SENDER)  //если не нашёл такой адрес в памяти
	{		
		Transf("Новый адрес!\r\n");
		a->A[a->IDX]=adr;	
		   i=a->IDX;
		   	 a->IDX++;
	} 
		
//	u_out("n:",n);	
//	u_out("a->IDX:",a->IDX);
//	u_out("   adr:",adr);
	
	return i;
}

void FRAME_DECODE (uint8 *M,u32 len)
{
	u64 i=0;
	u32 j=0;
	u32 k=0;
	u32 offset=0;
	u32 INDEX_archiv=0;
	u32 ADR=0;//индекс отправителя в структуре адресов отправителей
	u64 SENDER=0;
	
//----------------------это шапка  Фрейма---------------------------------------------	
	SENDER						=((u64)M[ 8]<<56)|((u64)M[ 9]<<48)|((u64)M[10]<<40)|((u64)M[11]<<32)|(M[12]<<24)|(M[13]<<16)|(M[14]<< 8)|(M[15]<<0);//получаем адрес отправителя
	
	ADR=ADR_FINDER (SENDER,&ADDR_SNDR);//ищем порядковый номер отправителя в структуре отправителей, если его там нет  - то заносим туда
//	u_out("ADR:",ADR);
  	FRM1[ADR].Frame_size		=     (M[ 0]<< 8)|     (M[ 1]<<0);
	FRM1[ADR].Frame_number		=     (M[ 2]<< 8)|     (M[ 3]<<0);//где-то среди этих байтов есть STOP_BIT
	FRM1[ADR].Msg_uniq_id		=     (M[ 4]<<24)|     (M[ 5]<<16)|     (M[ 6]<< 8)|     (M[ 7]<<0);
	FRM1[ADR].Sender_id			=((u64)M[ 8]<<56)|((u64)M[ 9]<<48)|((u64)M[10]<<40)|((u64)M[11]<<32)|(M[12]<<24)|(M[13]<<16)|(M[14]<< 8)|(M[15]<<0);
	FRM1[ADR].Receiver_id		=((u64)M[16]<<56)|((u64)M[17]<<48)|((u64)M[18]<<40)|((u64)M[19]<<32)|(M[20]<<24)|(M[21]<<16)|(M[22]<< 8)|(M[23]<<0);
	
	FRM1[ADR].MSG.Msg_size		=     (M[24]<<24)|     (M[25]<<16)|     (M[26]<< 8)|     (M[27]<<0);
	FRM1[ADR].MSG.Msg_type		=     (M[28]<<24)|     (M[29]<<16)|     (M[30]<< 8)|     (M[31]<<0);
	FRM1[ADR].MSG.Num_cmd_in_msg=((u64)M[32]<<56)|((u64)M[33]<<48)|((u64)M[34]<<40)|((u64)M[35]<<32)|(M[36]<<24)|(M[37]<<16)|(M[38]<< 8)|(M[39]<<0);  
//------------------------------------------------------------------------------------	 
	
	offset=40;
	for (i=0;i<FRM1[ADR].MSG.Num_cmd_in_msg;i++)
	{			
		FRM1[ADR].MSG.CMD[i].Cmd_size	=     (M[offset+ 0]<<24)|     (M[offset+ 1]<<16)|     (M[offset+ 2]<< 8)|     (M[offset+ 3]<<0);		
		FRM1[ADR].MSG.CMD[i].Cmd_type	=	  (M[offset+ 4]<<24)|     (M[offset+ 5]<<16)|     (M[offset+ 6]<< 8)|     (M[offset+ 7]<<0);	
		FRM1[ADR].MSG.CMD[i].Cmd_id		=((u64)M[offset+ 8]<<56)|((u64)M[offset+ 9]<<48)|((u64)M[offset+10]<<40)|((u64)M[offset+11]<<32)|(M[offset+12]<<24)|(M[offset+13]<<16)|(M[offset+14]<< 8)|(M[offset+15]<<0);
		FRM1[ADR].MSG.CMD[i].Cmd_time	=((u64)M[offset+16]<<56)|((u64)M[offset+17]<<48)|((u64)M[offset+18]<<40)|((u64)M[offset+19]<<32)|(M[offset+20]<<24)|(M[offset+21]<<16)|(M[offset+22]<< 8)|(M[offset+23]<<0);
		
		INDEX_archiv	=SERV1.INDEX;							//запоминаем текущий индекс "хранилища" в прошлый
		SERV1.INDEX_LAST=SERV1.INDEX;
		
		SERV1.x1=SERV1.INDEX-1;									//запоминаем начало интервала удалённых индексов для реестра
		
	 for (k=0;k<24;k++)	SERV_WR (M[offset+ k],&SERV1);			//переписываем пришедшие пакеты в "Хранилище"
		
	 for (j=0;j<FRM1[ADR].MSG.CMD[i].Cmd_size;j++)
		{
			FRM1[ADR].MSG.CMD[i].Cmd_data[j]=M[offset+24+j];
			SERV_WR (M[offset+24+j],&SERV1);					//переписываем пришедшие пакеты в "Хранилище"
		} 
		
		SERV1.x2=SERV1.INDEX+1;									//запоминаем конец интервала удалённых индексов для реестра
		
		offset=offset+24+j;
/**/		SERV_ID_WR (  &INVOICE[ADR],					//указатель на структуру квитанции
							     &SERV1,					//указатель на Хранилище   
						      &ID_SERV1,					//указатель на реестр
						   INDEX_archiv, 					//заполняем массив ID
					    FRM1[ADR].MSG.CMD[i].Cmd_type,
						FRM1[ADR].Sender_id,
						FRM1[ADR].MSG.CMD[i].Cmd_time,
						FRM1[ADR].MSG.CMD[i].Cmd_id
						);

	}	
	
//-------------------отладка--------------------------	
/*
 	Transf("\r\n");
	u_out("Frame_size    :",FRM1[ADR].Frame_size);
	u_out("Frame_number  :",FRM1[ADR].Frame_number);
	u_out("Msg_uniq_id   :",FRM1[ADR].Msg_uniq_id);
	u_out("Sender_id     :",FRM1[ADR].Sender_id);
	u_out("Receiver_id   :",FRM1[ADR].Receiver_id);
	u_out("Msg_size      :",FRM1[ADR].MSG.Msg_size);
	u_out("Msg_type      :",FRM1[ADR].MSG.Msg_type);
	u_out("Num_cmd_in_msg:",FRM1[ADR].MSG.Num_cmd_in_msg);
	*/
	
	/*  for (i=0;i<FRM1[ADR].MSG.Num_cmd_in_msg;i++)
	{
		Transf("\r\n");
		u_out("Cmd_size:",FRM1[ADR].MSG.CMD[i].Cmd_size);
		u_out("Cmd_type:",FRM1[ADR].MSG.CMD[i].Cmd_type);
		u_out("Cmd_id  :",FRM1[ADR].MSG.CMD[i].Cmd_id);
		u_out("Cmd_time:",FRM1[ADR].MSG.CMD[i].Cmd_time);
		Transf("\r\n");

		for (j=0;j<FRM1[ADR].MSG.CMD[i].Cmd_size;j++)
		{
			x_out("Cmd_data:0x",FRM1[ADR].MSG.CMD[i].Cmd_data[j]);
		}	 
	} 	 */	
//		PRINT_SERV    ();//выводим "Хранилище"
//		PRINT_SERV_ID ();//выводим ID
//		STATUS_ID (&ID_SERV1); 
}

void INIT_SERV_ARCHIV (SERVER *s,ID_SERVER *idx,ADR_SENDER *adr)
{
	u32 i=0;
//--------------------------------------------
//     инициализируем счётчик отправителей 	
	for (i=0;i<quantity_SENDER;i++)
	{
		adr->A[i]=0xffffffff;
	}
	adr->IDX=0;
//--------------------------------------------
//      инициализируем реестр	
	for (i=0;i<SIZE_ID;i++)
	{
		idx->INDEX   [i]=0xffffffff;//очищаем массив индексов
		idx->CMD_TYPE[i]=0xffffffffffffffff;
		idx->TIME    [i]=0xffffffffffffffff;
		idx->FLAG_REAL_TIME[i]=0;
	}
	idx->N_sch=0;
//--------------------------------------------
//      инициализируем ХРАНИЛИЩЕ	
	for (i=0;i<SIZE_SERVER;i++)
	{
		s->MeM[i]=0x00;//очищаем массив индексов
	}	
	s->INDEX		=0;
	s->INDEX_LAST	=0;
	s->CMD_ID		=0;
	s->SENDER_ID	=0;
	s->TIME			=0;
}

void SERV_WR (u8 a,SERVER *s)
{
	u32 i=s->INDEX;	
		  s->MeM[i]=a;
	  if (s->INDEX==(SIZE_SERVER-1)) s->INDEX=0; else  s->INDEX= s->INDEX+1;	  
}

void MSG_SHOW (void)
{
	u32 i=0;
	u32 j=0;
	Transf("----------\r\n");
	
	for (i=0;i<quantity_SENDER;i++)
	{
		u_out("SENDER:",i);
		u_out("Число квитанций:",INVOICE[i].MSG.Num_cmd_in_msg);
		for (j=0;j<INVOICE[i].MSG.Num_cmd_in_msg;j++)
		{
		   u_out("номер квитанции:",j);
		   u_out("CMD:",INVOICE[i].MSG.CMD[j].Cmd_type);
		}
	}
	
}

u32 SERV_ID_WR  //функция заполнения реестра команд
(
Frame  *m,		 //указатель на структуру квитанции
SERVER *srv,     //указатель на структуру "Хранилище"
ID_SERVER *s,	 //указатель на структуру "реестр"
u32 a,			 //индекс в Хранилище
u32 b,			 //CMD_TYPE
u32 c,			 //SENDER_ID
u64 d,			 //TIME
u64 e			 //CMD_ID
) //при записи увеличиваем счётчик записи, при очистке - сбрасываем.
{
	u32 i=0;
	u32 j=0;
	u32 error=0;
	u32 ERROR_ID=0;
	//------ищем затёртые пачки в хранилище и выкидываем их из реестра

		for (j=0;j<SIZE_ID;j++)
		{
			if (s->INDEX[j]!=0xffffffff)//проверяем что индекс действительный (0xffffffff - пустое место)
			{
				if (srv->x1<srv->x2)
				{//------это вариант когда пакет нормально в буфере расположен-----
					if((s->INDEX[j]>srv->x1)&&(s->INDEX[j]<srv->x2)) //нашли индекс команды в реестре, которая была затёрта в кольцевом буфере 
					{
					//	Transf("v1\r\n");
						ERROR_ID = MSG_ERROR_CMD_BUF;				//команда не выполненна потому что была затёрта в буфере
						Transf("квитанция о затёртых пакетах!\r\n");
						ERROR_CMD_MSG(s,m,j,ERROR_ID,0,0);	//заполняем квитанцию о событии
						s->INDEX	[j] =0xffffffff;	
						s->CMD_TYPE	[j] =0;
						s->SENDER_ID[j]	=0;
						s->TIME		[j]	=0;
						s->N_sch--;
					}
				} else
				{//----это вариант когда пол пакета в конце буфера и пол пакета в начале----
					if((s->INDEX[j]>srv->x1)||(s->INDEX[j]<srv->x2)) //нашли индекс команды в реестре, которая была затёрта в кольцевом буфере 
					{
					//	Transf("v2\r\n");
					//	u_out("s->INDEX[j]:",s->INDEX[j]);
					//	u_out("    srv->x1:",srv->x1);
					//	u_out("    srv->x2:",srv->x2);
						
						ERROR_ID = MSG_ERROR_CMD_BUF;				//команда не выполненна потому что была затёрта в буфере
						Transf("квитанция о затёртых пакетах!\r\n");
						u_out("s->CMD_TYPE	[j]:",s->CMD_TYPE	[j]);
						ERROR_CMD_MSG(s,m,j,ERROR_ID,0,0);	//заполняем квитанцию о событии
						s->INDEX	[j] =0xffffffff;	
						s->CMD_TYPE	[j] =0;
						s->SENDER_ID[j]	=0;
						s->TIME		[j]	=0;
						s->N_sch--;
					}
				}				
			}			
		}	
	
	//-----Записываем в реестр новую команду на свободное место
	i=0;
	if (s->N_sch<SIZE_ID)
	{
		while (s->INDEX[i]!=0xffffffff) {i++;};//ищем не занятую строчку в реестре
	
		if (b==CMD_TIME_SETUP) s->FLAG_REAL_TIME[i]=1;//команда реального времени , тут 001 - установка времени, должна производиться с привязкой к секундной метке
		else 	  			   s->FLAG_REAL_TIME[i]=0;
		s->INDEX	[i]	=a;
		s->CMD_TYPE	[i] =b;
		s->CMD_ID   [i] =e;
		s->SENDER_ID[i]	=c;
		s->TIME		[i]	=d;
		s->N_sch++;
	} else error=1;
			
return error; //возвращаем сигнал аварии если буфер переполнен
}

void SERV_ID_DEL (ID_SERVER *id,u32 idx) //процедура удаления команды из реестра
{
	id->INDEX	 [idx] =0xffffffff;
	id->CMD_TYPE [idx] =0xffffffff;
	id->CMD_ID   [idx] =0xffffffff;
	id->SENDER_ID[idx] =0xffffffff;
	id->TIME	 [idx] =0xffffffff;
	id->N_sch--;
//	u_out("удаляем команду из реестра:",idx);
}

void STATUS_ID (ID_SERVER *id)
{
	Transf("--------\r\n");
	u32 i=0;
	for (i=0;i<SIZE_ID;i++)
	{
		un_out("    INDEX[",i);Transf("]:");u_out("",id->INDEX    [i]);
		un_out(" CMD_TYPE[",i);Transf("]:");x_out("",id->CMD_TYPE [i]);
		un_out("SENDER_ID[",i);Transf("]:");x_out("",id->SENDER_ID[i]);
		un_out("     TIME[",i);Transf("]:");u_out("",id->TIME     [i]);
		u_out("N:",id->N_sch);
		Transf("\r\n");
	}
}

//эта функция обслуживает заполнение квитанций
u32 ERROR_CMD_MSG(
ID_SERVER *s,//реестр
Frame *invc, //структура квитанций	
u32 INDEX,	 //индекс в реестре
u32 MSG_TYPE,//тип сообщения
u32 DATA_MSG,//данные сообщения
u32 time	 //время составления квитанции
)
{
//	Transf("Заполняем квитанцию!\r\n");
	u32 STATUS=0;//успешно выполнили заполнение квитанции
	u32 n=0;

		invc->Receiver_id =s->SENDER_ID[INDEX];//устанавливаем адрес получателя квитанции 
	
		n=invc->MSG.Num_cmd_in_msg;		//считываем число уже заполненных квитанций

	if (n<quantity_CMD)					//проверяем что есть место под очередную квитанцию
	{
		invc->MSG.Num_cmd_in_msg++;			  		    //увеличиваем счётчик заполненных квитанций
		invc->MSG.CMD[n].Cmd_time   =time;				//записываем текущее системное время
		invc->MSG.CMD[n].Cmd_type   =MSG_TYPE;			//тип сообщения 
		invc->MSG.CMD[n].Cmd_id	    =s->CMD_ID[INDEX];	//ID сообщения, указываем команду CMD к которой привязано сообщение.
		invc->MSG.CMD[n].Cmd_size   =1;					//размер данный квитанции
		invc->MSG.CMD[n].Cmd_data[0]=DATA_MSG;			//данные сообщения
		
		invc->MSG.Msg_size=(invc->MSG.Msg_size+
											16+			//размер Message header в байтах
											24+			//размер Command header в байтах
											1			//размер Cmd data
											);
		
//		u_out("Cmd_time:",time);
//		u_out("Cmd_type:",MSG_TYPE);
//		u_out("Cmd_id  :",s->CMD_ID[INDEX]);
//		u_out("Cmd_data:",DATA_MSG);
	
	} else 
	{
		STATUS=1;//возвращаем ошибку заполнения квитанции из-за отсутствия места
		Transf("нет места для квитанций!\r\n");
	}

//	u_out("число заполненных квитанций:",n);
	return STATUS;
}

//эта функция обслуживает создание квитанций состояния
u32 SYS_CMD_MSG(
ID_SERVER *s,//реестр
Frame *invc, //структура квитанций	
u32 INDEX,	 //индекс в реестре
u32 MSG_TYPE,//тип сообщения
u32 N,		 //объём данных сообщения в байтах
u8 *DATA_MSG,//данные сообщения - массив данных
u32 time	  //время составления квитанции
)
{
//	Transf("Заполняем квитанцию!\r\n");
	u32 STATUS=0;//успешно выполнили заполнение квитанции
	u32 n=0;
	u32 i=0;

		invc->Receiver_id =s->SENDER_ID[INDEX];//устанавливаем адрес получателя квитанции 
	
		n=invc->MSG.Num_cmd_in_msg;		//считываем число уже заполненных квитанций

	if (n<quantity_CMD)					//проверяем что есть место под очередную квитанцию
	{
		invc->MSG.Num_cmd_in_msg++;			  		    //увеличиваем счётчик заполненных квитанций
		invc->MSG.CMD[n].Cmd_time   =time;				//записываем текущее системное время
		invc->MSG.CMD[n].Cmd_type   =MSG_TYPE;			//тип сообщения 
		invc->MSG.CMD[n].Cmd_id	    =ID_CMD;			//ID сообщения, указываем команду CMD к которой привязано сообщение.
		invc->MSG.CMD[n].Cmd_size   =N;					//размер данный квитанции
		for (i=0;i<N;i++)
		{
			invc->MSG.CMD[n].Cmd_data[i]=DATA_MSG[i];   //данные сообщения
		}		
		
		invc->MSG.Msg_size=(invc->MSG.Msg_size+
											16+			//размер Message header в байтах
											24+			//размер Command header в байтах
											N			//размер Cmd data
											);
		
		ID_CMD++;
//		u_out("Cmd_time:",time);
//		u_out("Cmd_type:",MSG_TYPE);
//		u_out("Cmd_id  :",s->CMD_ID[INDEX]);
//		u_out("Cmd_data:",DATA_MSG);
	
	} else 
	{
		STATUS=1;//возвращаем ошибку заполнения квитанции из-за отсутствия места
		Transf("нет места для квитанций!\r\n");
	}

//	u_out("число заполненных квитанций:",n);
	return STATUS;
}

u32 SEND_UDP_MSG (void)
{
	u32 error=0;
	u32 i=0;
	for (i=0;i<quantity_SENDER;i++)//проверяем по адресатам нет ли квитанций кому?
	{
	  if (INVOICE[i].MSG.Num_cmd_in_msg>0) 
	  {
		 error=TX_MSG_BUFF (&INVOICE[i],TX_BUF,TX_MAX_BUF_SIZE);//заполняем транспортный массив
		 SEND_udp(0, 3001,destip_UDP,SERVER_DEST_PORT);//отправляем квитанцию по UDP	
		 //x_out("IP_dest  <revers byte`s!>:",destip_UDP);		
	  }
	}
}

u32 Msg_uniq_id=0;//уникальный ID отправленных квитанций

u32 TX_MSG_BUFF (Frame *invc,uint8 *buf,u32 buf_size )
{
	u32 i=0;
	u32 j=0;
	u32 offset=0;
    u32 step=0;
	u32 error=0; 	
	
	Msg_uniq_id++;
	invc->Frame_number=0;
	invc->Msg_uniq_id =Msg_uniq_id;
	invc->Receiver_id =0;
	invc->Frame_size  =invc->MSG.Msg_size + 24;	
	
	buf[0]=(invc->Frame_size)>>8;
	buf[1]=(invc->Frame_size)   ;
	
	buf[2]=(invc->Frame_number)>>8;
	buf[3]=(invc->Frame_number|0x01);//установлен STOP BIT
	
	buf[4]=(invc->Msg_uniq_id)>>24;
	buf[5]=(invc->Msg_uniq_id)>>16;
	buf[6]=(invc->Msg_uniq_id)>> 8;
	buf[7]=(invc->Msg_uniq_id);
	
	buf[ 8]=(invc->Sender_id)>>56;
	buf[ 9]=(invc->Sender_id)>>48;
	buf[10]=(invc->Sender_id)>>40;
	buf[11]=(invc->Sender_id)>>32;
	buf[12]=(invc->Sender_id)>>24;
	buf[13]=(invc->Sender_id)>>16;
	buf[14]=(invc->Sender_id)>> 8;
	buf[15]=(invc->Sender_id);
	
	buf[16]=(invc->Receiver_id)>>56;
	buf[17]=(invc->Receiver_id)>>48;
	buf[18]=(invc->Receiver_id)>>40;
	buf[19]=(invc->Receiver_id)>>32;
	buf[20]=(invc->Receiver_id)>>24;
	buf[21]=(invc->Receiver_id)>>16;
	buf[22]=(invc->Receiver_id)>> 8;
	buf[23]=(invc->Receiver_id);
	
	buf[24]=(invc->MSG.Msg_size)>>24;
	buf[25]=(invc->MSG.Msg_size)>>16;
	buf[26]=(invc->MSG.Msg_size)>> 8;
	buf[27]=(invc->MSG.Msg_size);
	
	buf[28]=(invc->MSG.Msg_type)>>24;
	buf[29]=(invc->MSG.Msg_type)>>16;
	buf[30]=(invc->MSG.Msg_type)>> 8;
	buf[31]=(invc->MSG.Msg_type);
	
	buf[32]=(invc->MSG.Num_cmd_in_msg)>>56;
	buf[33]=(invc->MSG.Num_cmd_in_msg)>>48;
	buf[34]=(invc->MSG.Num_cmd_in_msg)>>40;
	buf[35]=(invc->MSG.Num_cmd_in_msg)>>32;
	buf[36]=(invc->MSG.Num_cmd_in_msg)>>24;
	buf[37]=(invc->MSG.Num_cmd_in_msg)>>16;
	buf[38]=(invc->MSG.Num_cmd_in_msg)>> 8;
	buf[39]=(invc->MSG.Num_cmd_in_msg);
	
	offset=40;
	for (i=0;i<invc->MSG.Num_cmd_in_msg;i++)
	{
		buf[offset+0]=(invc->MSG.CMD[i].Cmd_size)>>24;
		buf[offset+1]=(invc->MSG.CMD[i].Cmd_size)>>16;
		buf[offset+2]=(invc->MSG.CMD[i].Cmd_size)>> 8;
		buf[offset+3]=(invc->MSG.CMD[i].Cmd_size);
		
		buf[offset+4]=(invc->MSG.CMD[i].Cmd_type)>>24;
		buf[offset+5]=(invc->MSG.CMD[i].Cmd_type)>>16;
		buf[offset+6]=(invc->MSG.CMD[i].Cmd_type)>> 8;
		buf[offset+7]=(invc->MSG.CMD[i].Cmd_type);
		
		buf[offset+ 8]=(invc->MSG.CMD[i].Cmd_id)>>56;
		buf[offset+ 9]=(invc->MSG.CMD[i].Cmd_id)>>48;
		buf[offset+10]=(invc->MSG.CMD[i].Cmd_id)>>40;
		buf[offset+11]=(invc->MSG.CMD[i].Cmd_id)>>32;
		buf[offset+12]=(invc->MSG.CMD[i].Cmd_id)>>24;
		buf[offset+13]=(invc->MSG.CMD[i].Cmd_id)>>16;
		buf[offset+14]=(invc->MSG.CMD[i].Cmd_id)>> 8;
		buf[offset+15]=(invc->MSG.CMD[i].Cmd_id);
		
		buf[offset+16]=(invc->MSG.CMD[i].Cmd_time)>>56;
		buf[offset+17]=(invc->MSG.CMD[i].Cmd_time)>>48;
		buf[offset+18]=(invc->MSG.CMD[i].Cmd_time)>>40;
		buf[offset+19]=(invc->MSG.CMD[i].Cmd_time)>>32;
		buf[offset+20]=(invc->MSG.CMD[i].Cmd_time)>>24;
		buf[offset+21]=(invc->MSG.CMD[i].Cmd_time)>>16;
		buf[offset+22]=(invc->MSG.CMD[i].Cmd_time)>> 8;
		buf[offset+23]=(invc->MSG.CMD[i].Cmd_time);	

		for (j=0;j<invc->MSG.CMD[i].Cmd_size;j++) 
		{
			if (step<(buf_size-1)) step=offset+24+j; 
			else 	  
			{
				u_out("buf_size     :",buf_size);
				step=offset+24+j; 
				u_out("step         :",step);
				error=1;
				break;
			}//проверка переполнения массива
			buf[step]=invc->MSG.CMD[i].Cmd_data[j];
		}
		    if (offset<(buf_size-1-24-j)) offset=offset+24+j; 
			else
			{
				u_out("buf_size     :",buf_size);
				offset=offset+24+j;
				u_out("offset       :",offset);
				error=1;
				break;
			}//проверка переполнения массива
	}
	
	invc->MSG.Num_cmd_in_msg=0;//сбрасываем счётчик числа сообщений в структуре так как готовы её отослать
	
	TX_BUF_LENGTH=invc->Frame_size;//размер отправляемого массива
//	u_out("TX_BUF_LENGTH:",TX_BUF_LENGTH);
	
	invc->Frame_size=0;
	invc->MSG.Msg_size=0;
	invc->MSG.Num_cmd_in_msg=0;
	
	if (error>0) Transf ("переполнение транспортного буфера!\r\n");
	return error;
}

uint32_t time_return(void) 
{
 // extern uint32_t my_time; 
  return my_time;
}


void Set_network(void)
{
        uint8 tmp_array[6];       
        uint8 i;
		Transf("Set_network\r\n");		

        iinchip_init();
	
        wait_10ms(100);

        IP_my=(IP[0]<<24)|(IP[1]<<16)|(IP[2]<<8)|(IP[3]<<0);//это для справок!!!
          
        // MAC ADDRESS
        for (i = 0u ; i < 6u; i++) Config_Msg.Mac[i] = MAC[i];
        // Local IP ADDRESS
        Config_Msg.Lip[0] = IP[0]; Config_Msg.Lip[1] = IP[1]; Config_Msg.Lip[2] = IP[2]; Config_Msg.Lip[3] = IP[3];
        // GateWay ADDRESS
        Config_Msg.Gw[0] = GateWay[0]; Config_Msg.Gw[1] = GateWay[1]; Config_Msg.Gw[2] = GateWay[2]; Config_Msg.Gw[3] = GateWay[3];
        // Subnet Mask ADDRESS
        Config_Msg.Sub[0] = SubNet[0]; Config_Msg.Sub[1] = SubNet[1]; Config_Msg.Sub[2] = SubNet[2]; Config_Msg.Sub[3] = SubNet[3];
        
        setSHAR(Config_Msg.Mac);
      //  setSUBR(Config_Msg.Sub);
        saveSUBR(Config_Msg.Sub);
        setGAR(Config_Msg.Gw);
        setSIPR(Config_Msg.Lip);

        // Set DHCP
        Config_Msg.DHCP = Enable_DHCP;    
        //Destination IP address for TCP Client
        Chconfig_Type_Def.destip[0] = Dest_IP[0]; Chconfig_Type_Def.destip[1] = Dest_IP[1];
        Chconfig_Type_Def.destip[2] = Dest_IP[2]; Chconfig_Type_Def.destip[3] = Dest_IP[3];
        Chconfig_Type_Def.port = Dest_PORT;

        //Set PTR and RCR register	
        setRTR(6000);
        setRCR(3);

        //Init. TX & RX Memory size
        sysinit(txsize, rxsize); 
        
        Transf("\r\n----------------------------------------- \r\n");         		
        Transf("W5200E01-M3                       \r\n");        
        Transf("Network Configuration Information \r\n");        
        Transf("----------------------------------------- \r");  
        Transf("\r\nMAC : ");
        
        getSIPR (tmp_array);
        Transf("\r\nIP : ");

        un_out ("",tmp_array[0]);
        Transf(".");
        un_out ("",tmp_array[1]);
        Transf(".");
        un_out ("",tmp_array[2]);
        Transf(".");
        un_out ("",tmp_array[3]);
    	          
        getSUBR(tmp_array);
        Transf("\r\nSN : ");

        un_out ("",tmp_array[0]);
        Transf(".");
        un_out ("",tmp_array[1]);
        Transf(".");
        un_out ("",tmp_array[2]);
        Transf(".");
        un_out ("",tmp_array[3]);	   
        
        getGAR(tmp_array);
        Transf("\r\nGW : ");

        un_out ("",tmp_array[0]);
        Transf(".");
        un_out ("",tmp_array[1]);
        Transf(".");
        un_out ("",tmp_array[2]);
        Transf(".");
        un_out ("",tmp_array[3]);
        
        Transf("\r"); 
        Transf("PORT: 3001\r"); 
		
	//	Enable all IRQ for all Sockets

 	IINCHIP_WRITE(IMR, 0x01);
  
	IINCHIP_WRITE(Sn_IMR(0),0x4);   // 

	IINCHIP_WRITE(Sn_IR(0), 0xff);  //сброс прерываний, если были
	  
}

#define close_state 0
#define ready_state 1
#define connected_state 2

#define IPPORT_TELNET 23            // Telnet port
#define LINELEN 400
#define DATA_BUF_SIZE 100

#define USERNAME 1
#define	PASSWORD 2
#define	LOGIN 3
#define	LOGOUT 4

/* Telnet Commands */
#define	IAC 0xFF                    // Interpret as command
#define WILL 251
#define WONT 252
#define DO 253
#define DONT 254 
/* Telnet Commands end */

/* Telnet options */
#define NOPTIONS 6

#define TN_TRANSMIT_BINARY 0	    // TelNet transmit binary
#define TN_ECHO 1
#define TN_SUPPRESS_GA 3            // Supress Go Ahead
#define TN_STATUS 5
#define TN_TIMING_MARK 6
#define EXOPL 0xff	            // EXtended OPtion List
#define TN_TERMINAL_TYPE 0x18
#define TN_NE_WIN_SIZE 0x1f         // TelNet Negotiate Window SIZE
#define TN_ENVIRONMENT 0x24
#define TN_NEW_ENVIRONMENT 0x27
/* Telnet options end */

#define TX_RX_MAX_BUF_SIZE 64

uint8 user_state;

char text_buffer_TCP[64];
u16  text_lengh_TCP=0;


void init_telopt(SOCKET s)
{
  sendIAC(s, DO, TN_ENVIRONMENT);
  sendIAC(s, WILL, TN_ECHO);      // Negotiate ECHO option 
}

void sendIAC(SOCKET s, uint8 r1, uint8 r2) 
{

  switch(r1) {
    case WILL :
      Transf("sent : will\r");     // WILL command 
      break;
      
    case WONT :
      Transf("sent : wont\r");      // WONT command 
      break;
      
    case DO :
      Transf("sent : do\r");      //DO command 
      break;
      
    case DONT :
      Transf("sent : dont\r");      // DONT command 
      break;  
      
    case IAC :
      break;
  }
 

}     /* End init_telopt function */

