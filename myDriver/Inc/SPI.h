/*
 * SPI.h
 *
 *  Created on: 2 May 2023
 *      Author: baris
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_
#include "stm32f303xx.h"


/*
 * @def_group SPI_Baudrate
 *
 * */


#define SPI_BAUDRATE_DIV2     (0x00U)
#define SPI_BAUDRATE_DIV4     (0x01U)
#define SPI_BAUDRATE_DIV8     (0x02U)
#define SPI_BAUDRATE_DIV16    (0x03U)
#define SPI_BAUDRATE_DIV32    (0x04U)
#define SPI_BAUDRATE_DIV64    (0x05U)
#define SPI_BAUDRATE_DIV128   (0x06U)
#define SPI_BAUDRATE_DIV256   (0x07U)


/*
 * @def_group SPI_CPHA
 *
 * */


#define SPI_CPHA_FIRST       (0x00U)
#define SPI_CPHA_SECOND      (0x01U)


/*
 * @def_group SPI_CPOL
 *
 * */


#define  SPI_CPOL_LOW        (0x00U)
#define  SPI_CPOL_HIGH       (0x01U)

/*
 * @def_group SPI_CRC_EN
 *
 * */


#define SPI_CRC_DISABLE        (0x00U)
#define SPI_CRC_ENABLE         (0x01U)


/*
 * @def_group SPI_CRC_EN
 *
 * */

#define SPI_CRC_TXBFF          (0x00U)
#define SPI_CRC_TXCRC          (0x01U)


/*
 * @def_group SPI_CRC_Length
 *
 * */


#define SPI_CRC_8BIT           (0x00U)
#define SPI_CRC_16BIT          (0x01U)


/*
 * @def_group SPI_MODE
 *
 * */


#define SPI_MODE_SLAVE         (0x00U)
#define SPI_MODE_MASTER        (0x01U)


/*
 * @def_group SPI_SSM
 *
 * */


#define SPI_SSM_DISABLED       (0x00U)
#define SPI_SSM_ENABLED        (0x03U)


/*
 * @def_group SPI_FF
 *
 * */


#define SPI_FF_MSB       (0x00U)
#define SPI_FF_LSB       (0x01U)


/*
 * @def_group SPI_BusConfig
 *
 * */


#define SPI_BUS_FULLDUPLEX    (uint32_t)(0x00000000)
#define SPI_BUS_RECIVEONLY    (uint32_t)(0x00000400)
#define SPI_BUS_HALFDUPLEX_T  (uint32_t)(0x0000C000)
#define SPI_BUS_HALFDUPLEX_R  (uint32_t)(0x00008000)


/*
 * @def_group SPI_DataSize
 *
 * */


#define SPI_DATASIZE_4BIT     (0x00000300U)
#define SPI_DATASIZE_5BIT     (0x00000400U)
#define SPI_DATASIZE_6BIT     (0x00000500U)
#define SPI_DATASIZE_7BIT     (0x00000600U)
#define SPI_DATASIZE_8BIT     (0x00000700U)
#define SPI_DATASIZE_9BIT     (0x00000800U)
#define SPI_DATASIZE_10BIT    (0x00000900U)
#define SPI_DATASIZE_11BIT    (0x00000A00U)
#define SPI_DATASIZE_12BIT    (0x00000B00U)
#define SPI_DATASIZE_13BIT    (0x00000C00U)
#define SPI_DATASIZE_14BIT    (0x00000D00U)
#define SPI_DATASIZE_15BIT    (0x00000E00U)
#define SPI_DATASIZE_16BIT    (0x00000F00U)


/*
 * @def_group SPI_FRXTH
 *
 * */


#define SPI_FRXTH_16BIT   (0x00U)
#define SPI_FRXTH_8BIT    (0x01U)


/*
 * @def_group SPI_FRF
 *
 * */


#define SPI_FRF_MOTOROLA      (0x00U)
#define SPI_FRF_TI            (0x01U)


/*
 * @def_group SPI_NSSP
 *
 * */

#define SPI_NSSP_NOPULSE      (0x00U)
#define SPI_NSSP_PULSE        (0x01U)


/*
 * @def_group SPI_SSOE
 *
 * */

#define SPI_SSOE_DISABLED     (0x00U)
#define SPI_SSOE_ENABLED      (0x01U)



typedef enum{


	SPI_FLAG_RESET=0x0U,
	SPI_FLAG_SET=!SPI_FLAG_RESET


}SPI_FlagStatus_t;




typedef struct{


	uint32_t Mode;           /* Mode Selection for SPI          @def_group SPI_MODE      */
	uint32_t CPHA;           /* CPHA Selection for SPI          @def_group SPI_CPHA      */
	uint32_t CPOL;           /* CPOL Selection for SPI          @def_group SPI_CPOL      */
	uint32_t Baudrate;       /* Baudrate values for SPI         @def_group SPI_Baudrate  */
	uint32_t SSM_Cmd;        /* SSM Selection for SPI           @def_group SPI_SSM       */
	uint32_t Frame_Format;   /* Frame Format Selection for SPI  @def_group SPI_FF        */
	uint32_t CRC_EN;         /* CRC ENABLE for SPI              @def_group SPI_CRC_En    */
	uint32_t CRC_Next;       /* CRC Next for SPI                @def_group SPI_CRC_Next  */
	uint32_t CRC_Cmd;        /* CRC Selection for SPI           @def_group SPI_CRC_Length*/
	uint32_t BusConfig;      /* Bus Mode Selection for SPI      @def_group SPI_BusConfig */
    uint32_t Data_Size;      /* Data Size Selection for SPI     @def_group SPI_DataSize  */
    uint32_t FRXTH;          /* FIFO Level Selection for SPI    @def_group SPI_FRXTH     */
    uint32_t FRF;            /* Frame Format Selection for SPI  @def_group SPI_FRF       */
    uint32_t NSSP;           /* Pulse Mode Selection for SPI    @def_group SPI_NSSP      */
    uint32_t SSOE;           /* SS Output Selection for SPI     @def_group SPI_SSOE      */

}SPI_InitTypedef_t;


typedef struct __SPI_HandleTypedef_t{

    SPI_Typedef_t *Instance;
    SPI_InitTypedef_t Init;
    uint8_t *TxDataAddr;
    uint16_t SizeOfData;
    uint8_t SPI_State_TX;
    void(*TxISRfunc)(struct __SPI_HandleTypedef_t *SPI_Handle);
    uint8_t SPI_State_RX;
    uint16_t SizeOfDataRX;
    uint8_t *RxDataAddr;
    void(*RxISRfunc)(struct __SPI_HandleTypedef_t *SPI_Handle);



}SPI_HandleTypedef_t;

typedef enum{

	SPI_BUS_FREE=0x0U,
	SPI_BUS_TX_BUSY=0x1U,
	SPI_BUS_RX_BUSY=0x2U,

}SPI_BUS_STATUS;

void SPI_Init(SPI_HandleTypedef_t* SPI_Handle);
void SPI_PeriphEnable(SPI_HandleTypedef_t*SPI_Handle, FunctionalState_t state);
void SPI_TransmitData(SPI_HandleTypedef_t*SPI_Handle, uint8_t *pdata,uint16_t SizeofData,uint32_t delay);
void SPI_RecieveData(SPI_HandleTypedef_t*SPI_Handle, uint8_t *buffer,uint16_t SizeofData);
void SPI_TransmitData_IT(SPI_HandleTypedef_t *SPI_Handle, uint8_t *pdata,uint16_t SizeofData);
void SPI_RecieveData_IT(SPI_HandleTypedef_t*SPI_Handle, uint8_t *buffer,uint16_t SizeofData);
void SPI_InterruptHandler(SPI_HandleTypedef_t *SPI_Handle);
SPI_FlagStatus_t SPI_GetFlagStatus(SPI_HandleTypedef_t*SPI_Handle, uint16_t flag);


#endif /* INC_SPI_H_ */
