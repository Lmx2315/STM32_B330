#include "main.h"


uint8_t  MAC[6] =     {0x64, 0xA2, 0x32, 0x01, 0x02, 0x03}; //MAC Address
uint8_t  IP[4] =      {192, 168, 3, 250};                  //SIPR Source IP Address Register
uint8_t  GateWay[4] = {192, 168, 3, 1};                    //Gateway Address
uint8_t  Mask[4] =    {255, 255, 255, 0};                        //SubnetMask Address
uint8_t  Hard_reg [6]={0x00,0x08,0xDC,0x01,0x02,0x03};     //SHAR Source Hardware Address Register
char Receive_DATA[16];
uint8_t Receive_IP[4];
uint8_t Receive_Port_destination[2];
uint8_t  Receive_byte_size[2];
uint8_t Rec_Flag;
uint16_t size;  
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

struct Config_Msg{
        uint8_t Mac[6];
        uint8_t Ip[4];
        uint8_t Gaw[4];
        uint8_t Mask[4];
        uint8_t Hard_reg[6];
}; struct Config_Msg Config_Msg;

struct UDR_data{
  uint8_t Receive[32];
        uint8_t Transtir[32];
        uint8_t amount_bit_Receive;
        uint8_t amount_bit_Transiver;
        uint8_t Ip_of_the_parcel[4];
}; struct UDR_data UDR_data;


uint8_t Receive_UDR(void);
uint8_t Transmit_UDR(char *DATA_TX, uint8_t size);
void Initialization_wiznet( void );
void Set_Hard_reg(uint8_t  *addr);
void _delay_us (int long  delay);
void Set_GateWay(uint8_t *addr);
void EXTI10_enabled(uint8_t on);
void Write_Word(uint16_t data);
void Write_Byte(uint8_t data);
void Set_Mask(uint8_t *addr);
void Set_IP(uint8_t *addr);
void clear_data_UDR(void);
void Socket_Open  (void);
uint8_t Read_Byte(void);
void close(uint8_t s);


void Initialization_wiznet(void)
{
		Transf("Initialization_wiznet!!!\r\n");
		Transf("----------------------\r\n");
/*
        uint8_t i;

        // Mac Address
        for (i = 0 ; i < 6; i++) {Config_Msg.Mac[i] = MAC[i];}
        // IP address
        Config_Msg.Ip[0] = IP[0]; Config_Msg.Ip[1] = IP[1]; Config_Msg.Ip[2] = IP[2]; Config_Msg.Ip[3] = IP[3];
        // GateWay address
        Config_Msg.Gaw[0] = GateWay[0]; Config_Msg.Gaw[1] = GateWay[1]; Config_Msg.Gaw[2] = GateWay[2]; Config_Msg.Gaw[3] = GateWay[3];
        // Subnet Mask address
        Config_Msg.Hard_reg[0] = Hard_reg[0]; Config_Msg.Hard_reg[1] = Hard_reg[1]; Config_Msg.Hard_reg[2] = Hard_reg[2]; Config_Msg.Hard_reg[3] = Hard_reg[3];
        
        Set_GateWay(Config_Msg.Gaw); 			Transf("Set_GateWay\r\n");
        Set_Hard_reg(Config_Msg.Hard_reg);		Transf("Set_Hard_reg\r\n");
        Set_IP(Config_Msg.Mac);					Transf("Set_IP\r\n");
        Set_Mask(Config_Msg.Mask);				Transf("Set_Mask\r\n");

          NSS_4(0); 
                    Write_Word(0x0034);
                    Write_Word(WRITE_1Byte);  
                    Write_Byte(0x01);  
          NSS_4(1);
                 
          NSS_4(0); 
                    Write_Word(0x0016);
                    Write_Word(WRITE_1Byte);  
                    Write_Byte(0x01);  
          NSS_4(1); 
*/		  
}


void Set_IP(uint8_t *addr){
      uint8_t i;
            uint8_t add_ip = Ip_address;
        
        for(i=0;i<4;i++){
      NSS_4(0);
      Write_Word(add_ip);
      add_ip++; 
                  Write_Word(WRITE_1Byte);  
                  Write_Byte(Config_Msg.Ip[i]); 
                  NSS_4(1);
   }
}

void Set_GateWay(uint8_t *addr){
           uint8_t i;
           uint8_t add_Ga = Gateway_address;
        
        for(i=0;i<4;i++){
      NSS_4(0); Write_Word(add_Ga);  
                  add_Ga++;
                  Write_Word(WRITE_1Byte);  
                  Write_Byte(Config_Msg.Gaw[i]); 
                  NSS_4(1);
  }
}

void Set_Mask(uint8_t *addr){
           uint8_t i;
           uint8_t add_Mask = Mask_address;
        
        for(i=0;i<4;i++){
      NSS_4(0); Write_Word(add_Mask);
      add_Mask++;               
                Write_Word(WRITE_1Byte);  
                  Write_Byte(Config_Msg.Mask[i]); 
                  NSS_4(1);
  }
}
void Set_Hard_reg(uint8_t *addr){
           uint8_t i;
           uint8_t add_Hard = Hard_address;
        
        for(i=0;i<6;i++){
      NSS_4(0); Write_Word(add_Hard);
      add_Hard++;               
                Write_Word(WRITE_1Byte);  
                  Write_Byte(Config_Msg.Hard_reg[i]); 
                  NSS_4(1);
  }
}

void Write_Byte(uint8_t data){
  spi4send8(data);                           
}

uint8_t Read_Byte(void){
        unsigned char spidata = 0;
 		spidata=spi4send8(0x00);
        return spidata;
}


void Write_Word(uint16_t data){
      {
        Write_Byte ((data & 0xFF00) >> 8);                      
        Write_Byte ((data & 0x00FF));   
      }
}

void Socket_Open(void){
     uint8_t i;
           
           NSS_4(0); Write_Word(Sn_RXMEM_SIZE_address);
                 Write_Word(WRITE_1Byte);  
                 Write_Byte(0x01);
                 NSS_4(1);///////////////////////////////////// RXmem size
        
           NSS_4(0); Write_Word(Sn_TXMEM_SIZE_address);
                 Write_Word(WRITE_1Byte);  
                 Write_Byte(0x01);
                 NSS_4(1);  /////////////////////////////////// TXmem size
         
           NSS_4(0); Write_Word(Sn_MR_address);
                 Write_Word(WRITE_1Byte);  
                 Write_Byte(0x02); // UDP socket
                 NSS_4(1);
           
           NSS_4(0); Write_Word(Sn_PORT_address);
                 Write_Word(WRITE_1Byte);  
                 Write_Byte(0xAA); // Socket used PORT 0xAAAA
                 NSS_4(1);
           // 
           NSS_4(0); Write_Word(Sn_PORT_address + 1);
                 Write_Word(WRITE_1Byte);  
                 Write_Byte(0xAA); // Socket used PORT 0xAAAA
                 NSS_4(1);
           
           NSS_4(0); Write_Word(Sn_CR_address);
                 Write_Word(WRITE_1Byte);  
                 Write_Byte(Sn_CR_Open); // Socket - Open
                 NSS_4(1);
        
read:
           NSS_4(0); Write_Word(Sn_SR_address);
                 Write_Word(Read_1Byte);  
                 i = Read_Byte(); // Socket - Open
                 NSS_4(1);
 //       if(i != 0x22){goto read;} // Bit - not UDR protocol  0x22

                 NSS_4(0); Write_Word(Sn_IMR_address);
                 Write_Word(WRITE_1Byte);  
                 Write_Byte(0x04); // RX interrupt
                 NSS_4(1);

                 NSS_4(0); Write_Word(Sn_IMR_address);
                 Write_Word(Read_1Byte);  
                 Read_Byte(); // RX interrupt
                 NSS_4(1);
}
