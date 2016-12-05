/*
 * dxl_constants.h
 *
 *  Created on: 2012. 12. 17.
 *      Author: ROBOTIS(jason@robotis.com)
 */

#ifndef DXL_CONSTANTS_H_
#define DXL_CONSTANTS_H_


#define BROADCAST_ID		(254) /* 0xFE */

/*
 * Instruction command
 * */

/*
#define INST_PING           0x01
#define INST_READ           0x02
#define INST_WRITE          0x03
#define INST_REG_WRITE      0x04
#define INST_ACTION         0x05
#define INST_RESET          0x06
#define INST_DIGITAL_RESET  0x07
#define INST_SYSTEM_READ    0x0C
#define INST_SYSTEM_WRITE   0x0D
#define INST_SYNC_WRITE     0x83
#define INST_SYNC_REG_WRITE 0x84
*/
enum DXL_INSTRUCTION{  //2014-04-02 ROBOTIS DXL Protocol 2.0
	INST_PING           = 1,
	INST_READ           = 2,
	INST_WRITE          = 3,
	INST_REG_WRITE      = 4,
	INST_ACTION         = 5,
	INST_FACTORY_RESET  = 6,
	INST_REBOOT         = 8,
	INST_SYSTEM_WRITE   = 13,   // 0x0D
	INST_STATUS         = 85,   // 0x55
	INST_SYNC_READ      = 130,  // 0x82
	INST_SYNC_WRITE     = 131,  // 0x83
	INST_BULK_READ      = 146,  // 0x92
	INST_BULK_WRITE     = 147   // 0x93
};


/*
 * defines error message for protocol 1.0
 * */
#define ERRBIT_VOLTAGE		(1)
#define ERRBIT_ANGLE		(2)
#define ERRBIT_OVERHEAT		(4)
#define ERRBIT_RANGE		(8)
#define ERRBIT_CHECKSUM		(16)
#define ERRBIT_OVERLOAD		(32)
#define ERRBIT_INSTRUCTION	(64)

/*
 * defines error message for protocol 2.0
 * */
#define ERRBIT_RESULT_FAIL	(1)
#define ERRBIT_INST_ERROR	(2)
#define ERRBIT_CRC			(4)
#define ERRBIT_DATA_RANGE	(8)
#define ERRBIT_DATA_LENGTH	(16)
#define ERRBIT_DATA_LIMIT	(32)
#define ERRBIT_ACCESS		(64)

/*
 * defines message of communication
 * */
#define	COMM_TXSUCCESS		(0)
#define COMM_RXSUCCESS		(1)
#define COMM_TXFAIL			(2)
#define COMM_RXFAIL			(3)
#define COMM_TXERROR		(4)
#define COMM_RXWAITING		(5)
#define COMM_RXTIMEOUT		(6)
#define COMM_RXCORRUPT		(7)

/* timing defines */
#define RX_TIMEOUT_COUNT2		(1600L) //(1000L) //porting
#define NANO_TIME_DELAY			(12000) //ydh added 20111228 -> 20120210 edited ydh
//#define RX_TIMEOUT_COUNT1  	(RX_TIMEOUT_COUNT2*90L)// -> 110msec  before Ver 1.11e
#define RX_TIMEOUT_COUNT1  		(RX_TIMEOUT_COUNT2*128L)//  -> 156msec for ID 110 safe access after Ver 1.11f //porting ydh


/*
 * Dynamixel common control table -> see ROBOTIS support page(support.robotis.com)
 * 
 * 
 * */
#define P_MODEL_NUMBER_L      0
#define P_MODOEL_NUMBER_H     1
#define P_VERSION             2
#define P_ID                  3
#define P_BAUD_RATE           4
#define P_RETURN_DELAY_TIME   5
#define P_CW_ANGLE_LIMIT_L    6
#define P_CW_ANGLE_LIMIT_H    7
#define P_CCW_ANGLE_LIMIT_L   8
#define P_CCW_ANGLE_LIMIT_H   9
#define P_SYSTEM_DATA2        10
#define P_LIMIT_TEMPERATURE   11
#define P_DOWN_LIMIT_VOLTAGE  12
#define P_UP_LIMIT_VOLTAGE    13
#define P_MAX_TORQUE_L        14
#define P_MAX_TORQUE_H        15
#define P_RETURN_LEVEL        16
#define P_ALARM_LED           17
#define P_ALARM_SHUTDOWN      18
#define P_OPERATING_MODE      19
#define P_DOWN_CALIBRATION_L  20
#define P_DOWN_CALIBRATION_H  21
#define P_UP_CALIBRATION_L    22
#define P_UP_CALIBRATION_H    23

#define P_TORQUE_ENABLE         (24)
#define P_LED                   (25)

#define P_CW_COMPLIANCE_MARGIN  (26)
#define P_CCW_COMPLIANCE_MARGIN (27)
#define P_CW_COMPLIANCE_SLOPE   (28)
#define P_CCW_COMPLIANCE_SLOPE  (29)

#define P_P_GAIN  			(26)
#define P_I_GAIN 			(27)
#define P_D_GAIN   			(28)
#define P_RESERVED  		(29)


#define P_GOAL_POSITION_L       (30)
#define P_GOAL_POSITION_H       (31)
#define P_GOAL_SPEED_L          (32)
#define P_GOAL_SPEED_H          (33)
#define P_TORQUE_LIMIT_L        (34)
#define P_TORQUE_LIMIT_H        (35)
#define P_PRESENT_POSITION_L    (36)
#define P_PRESENT_POSITION_H    (37)
#define P_PRESENT_SPEED_L       (38)
#define P_PRESENT_SPEED_H       (39)
#define P_PRESENT_LOAD_L        (40)
#define P_PRESENT_LOAD_H        (41)
#define P_PRESENT_VOLTAGE       (42)
#define P_PRESENT_TEMPERATURE   (43)
#define P_REGISTERED_INSTRUCTION (44)
#define P_PAUSE_TIME            (45)
//#define P_FINAL_GOAL_POSITION_L (46)
#define P_MOVING (46)

/*Half Duflex UART ENABLE PIN ----------------------------------------------------------------------------------------------*/
#ifdef BOARD_CM900
#define PORT_ENABLE_TXD			GPIOB
#define PORT_ENABLE_RXD			GPIOB

#define PIN_ENABLE_TXD			4
#define PIN_ENABLE_RXD			5


//set to TXD Enable
#define	DXL_TXD	{ gpio_write_bit(PORT_ENABLE_TXD, PIN_ENABLE_TXD, 1 ); gpio_write_bit(PORT_ENABLE_RXD, PIN_ENABLE_RXD, 0 );}
//set to RXD Enable
#define	DXL_RXD	{ gpio_write_bit(PORT_ENABLE_TXD, PIN_ENABLE_TXD, 0 ); gpio_write_bit(PORT_ENABLE_RXD, PIN_ENABLE_RXD, 1 );}


#elif BOARD_CM900_REV10 || BOARD_CM904

#define PORT_TXRX_DIRECTION		GPIOB
#define PIN_TXRX_DIRECTION		5


#define	DXL_TXD	{ gpio_write_bit(PORT_TXRX_DIRECTION, PIN_TXRX_DIRECTION, 1); } //set to HIGH
#define	DXL_RXD	{ gpio_write_bit(PORT_TXRX_DIRECTION, PIN_TXRX_DIRECTION, 0); } //set to LOW


#endif

#define PORT_DXL_TXD			GPIOB
#define PORT_DXL_RXD			GPIOB

#define PIN_DXL_TXD				6
#define PIN_DXL_RXD				7

#define PORT_DXL_TXD2                 GPIOA   //TX2
#define PORT_DXL_RXD2                 GPIOA   //RX2

#define PIN_DXL_TXD2                         2
#define PIN_DXL_RXD2                         3
/*
 * For olny OpenCM9.04 485 SHD Board that uses USART3 as DXL3 Bus
 * */

#define PORT_TXRX_DIRECTION3	GPIOC
#define PIN_TXRX_DIRECTION3		14

#define PORT_DXL_TXD3			GPIOB
#define PORT_DXL_RXD3			GPIOB

#define PIN_DXL_TXD3			10
#define PIN_DXL_RXD3			11


#define	DXL_TXD3	{ gpio_write_bit(PORT_TXRX_DIRECTION3, PIN_TXRX_DIRECTION3, 1); } //set to HIGH
#define	DXL_RXD3	{ gpio_write_bit(PORT_TXRX_DIRECTION3, PIN_TXRX_DIRECTION3, 0); } //set to LOW


///////////////// utility for value ///////////////////////////
#define DXL_MAKEWORD(a, b)      ((unsigned short)(((unsigned char)(((unsigned long)(a)) & 0xff)) | ((unsigned short)((unsigned char)(((unsigned long)(b)) & 0xff))) << 8))
#define DXL_MAKEDWORD(a, b)     ((unsigned int)(((unsigned short)(((unsigned long)(a)) & 0xffff)) | ((unsigned int)((unsigned short)(((unsigned long)(b)) & 0xffff))) << 16))
#define DXL_LOWORD(l)           ((unsigned short)(((unsigned long)(l)) & 0xffff))
#define DXL_HIWORD(l)           ((unsigned short)((((unsigned long)(l)) >> 16) & 0xffff))
#define DXL_LOBYTE(w)           ((unsigned char)(((unsigned long)(w)) & 0xff))
#define DXL_HIBYTE(w)           ((unsigned char)((((unsigned long)(w)) >> 8) & 0xff))

#define DXL_PACKET_TYPE1	1 //2014-04-08 sm6787@robotis.com ->  DXL protocol 1.0 packet type
#define DXL_PACKET_TYPE2	2 //2014-04-08 sm6787@robotis.com ->  DXL protocol 1.0 packet type



#endif /* DXL_CONSTANTS_H_ */
