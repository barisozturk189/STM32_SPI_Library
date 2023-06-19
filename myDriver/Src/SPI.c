#include "SPI.h"
#include "stm32f303xx.h"



/*
 * @brief SPI_RecieveHelper8BIT, Read the data register.
 *
 * @param SPI_Hanlde = User Config Structure
 *
 * @retval void
 * */


static void SPI_RecieveHelper8BIT(SPI_HandleTypedef_t *SPI_Handle){


	*(SPI_Handle->RxDataAddr)= *((_IO uint8_t*)(&SPI_Handle->Instance->DR));
	SPI_Handle->RxDataAddr+=sizeof(uint8_t);
	SPI_Handle->SizeOfDataRX--;

	if(SPI_Handle->SizeOfDataRX==0){


		SPI_Handle->Instance->CR2 &= ~(0x1u<<SPI_CR2_RXEIE);
		SPI_Handle->TxDataAddr=NULL;
     	SPI_Handle->SizeOfDataRX=0;
		SPI_Handle->SPI_State_RX=SPI_BUS_FREE;


	}



}


/*
 * @brief SPI_RecieveHelper16BIT, Read the data register.
 *
 * @param SPI_Hanlde = User Config Structure
 *
 * @retval void
 * */


static void SPI_RecieveHelper16BIT(SPI_HandleTypedef_t *SPI_Handle){


	*((uint16_t*)SPI_Handle->RxDataAddr)=*((_IO uint16_t*)(&SPI_Handle->Instance->DR));
	SPI_Handle->RxDataAddr+=sizeof(uint16_t);
	SPI_Handle->SizeOfDataRX-=2u;

	if(SPI_Handle->SizeOfDataRX==0){


		SPI_Handle->Instance->CR2 &= ~(0x1u<<SPI_CR2_RXEIE);
		SPI_Handle->TxDataAddr=NULL;
		SPI_Handle->SizeOfDataRX=0;
		SPI_Handle->SPI_State_RX=SPI_BUS_FREE;

	}



}


/*
 * @brief SPI_RecieveHelper8BIT, Stores the data register.
 *
 * @param SPI_Hanlde = User Config Structure
 *
 * @retval void
 * */


static void SPI_TransmitHelper8BIT(SPI_HandleTypedef_t *SPI_Handle){

	SPI_Handle->Instance->DR=*((uint8_t*)SPI_Handle->TxDataAddr);
	SPI_Handle->TxDataAddr+=sizeof(uint8_t);
	SPI_Handle->SizeOfData--;

	if(SPI_Handle->SizeOfData==0){


		SPI_Handle->Instance->CR2 &= ~(0x1u<<SPI_CR2_TXEIE);
		SPI_Handle->TxDataAddr=NULL;
		SPI_Handle->SizeOfData=0;
		SPI_Handle->SPI_State_TX=SPI_BUS_FREE;

	}



}


/*
 * @brief SPI_RecieveHelper16BIT, Stores the data register.
 *
 * @param SPI_Hanlde = User Config Structure
 *
 * @retval void
 * */


static void SPI_TransmitHelper16BIT(SPI_HandleTypedef_t *SPI_Handle){


	SPI_Handle->Instance->DR=*((uint16_t*)SPI_Handle->TxDataAddr);
	SPI_Handle->TxDataAddr+=sizeof(uint16_t);
	SPI_Handle->SizeOfData-=2u;

	if(SPI_Handle->SizeOfData==0){


		SPI_Handle->Instance->CR2 &= ~(0x1u<<SPI_CR2_TXEIE);
		SPI_Handle->TxDataAddr=NULL;
		SPI_Handle->SizeOfData=0;
		SPI_Handle->SPI_State_TX=SPI_BUS_FREE;

	}


}


/*
 * @brief SPI_Init, Configures to PI Peripheral
 *
 * @param SPI_Hanlde = User Config Structure
 *
 * @retval void
 * */


void SPI_Init(SPI_HandleTypedef_t *SPI_Handle){

	uint32_t tempvalue=0;
	tempvalue=SPI_Handle->Instance->CR1;
	tempvalue |= (SPI_Handle->Init.BusConfig) | (SPI_Handle->Init.Baudrate<<0x03U) | (SPI_Handle->Init.CPHA<<0x00U) | (SPI_Handle->Init.CPOL<<0x01U) |           \
			     (SPI_Handle->Init.CRC_EN<<0x0DU)| (SPI_Handle->Init.CRC_Cmd<<0x0BU) | (SPI_Handle->Init.Frame_Format<<0x07U) | (SPI_Handle->Init.Mode<<0x02U) |  \
				 (SPI_Handle->Init.CRC_Next<<0x0CU)|(SPI_Handle->Init.SSM_Cmd<<0x08U);
	SPI_Handle->Instance->CR1=tempvalue;
	tempvalue=0;
	tempvalue=SPI_Handle->Instance->CR2;
	tempvalue&=(0x00000000U);
	tempvalue |= (SPI_Handle->Init.Data_Size) | (SPI_Handle->Init.FRXTH<<0x0CU) | (SPI_Handle->Init.FRF<<0x04U) | (SPI_Handle->Init.NSSP<<0x03U) | \
			    (SPI_Handle->Init.SSOE<<0x02U);
	SPI_Handle->Instance->CR2=tempvalue;



}

/*
 * @brief SPI_PeriphEnable, Enable to PI Peripheral
 *
 * @param SPI_Handle = User Config Structure
 *
 * @param State= ENABLE or DISABLE
 *
 * @retval void
 * */


void SPI_PeriphEnable(SPI_HandleTypedef_t*SPI_Handle, FunctionalState_t state){


   if(state==ENABLE)
	    SPI_Handle->Instance->CR1 |= (0x1U<<SPI_CR1_EN);

   else
	   SPI_Handle->Instance->CR1 &= ~(0x1U<<SPI_CR1_EN);



}

/*
 * @brief SPI_TransmitData, Transmit data
 *
 * @param SPI_Handle = User Config Structure
 *
 * @param pdata = data
 *
 * @param SizeofData = Size of Data
 *
 * @retval void
 * */


void SPI_TransmitData(SPI_HandleTypedef_t*SPI_Handle, uint8_t *pdata,uint16_t SizeofData,uint32_t delay){


	if(SPI_Handle->Init.Data_Size > SPI_DATASIZE_8BIT){

	while(SizeofData>0){


		if(SPI_GetFlagStatus(SPI_Handle, SPI_TXE_FLAG)){

					SPI_Handle->Instance->DR= (*(uint16_t*)pdata);

					pdata+=sizeof(uint16_t);
					SizeofData-=2u;
				}

		else{

			while(delay){

				delay--;
			}

		}


	}

	}
	else{


		while(SizeofData>0){


			if(SPI_GetFlagStatus(SPI_Handle,SPI_TXE_FLAG)){

				if(SizeofData>1U){

				    SPI_Handle->Instance->DR= *((uint8_t*)pdata);
						pdata+=sizeof(uint8_t);
						SizeofData--;

				}
				}




		}

	}
		while(SPI_GetFlagStatus(SPI_Handle, SPI_BSY_FLAG));

}


/*
 * @brief SPI_RecieveData, Recieve data
 *
 * @param SPI_Handle = User Config Structure
 *
 * @param pdata = data
 *
 * @param buffer = Size of Data
 *
 * @retval void
 * */


void SPI_RecieveData(SPI_HandleTypedef_t*SPI_Handle, uint8_t *buffer,uint16_t SizeofData){


	if(SPI_Handle->Init.Data_Size>SPI_DATASIZE_8BIT){


		while(SizeofData>0){


			if(SPI_GetFlagStatus(SPI_Handle,SPI_RXNE_FLAG)){

			  *((uint16_t*)buffer)=(uint16_t)SPI_Handle->Instance->DR;
			  buffer+=sizeof(uint16_t);
			  SizeofData--;

			}


		}


	}
	else
	{

		while(SizeofData>0){


			if(SPI_GetFlagStatus(SPI_Handle,SPI_RXNE_FLAG)){

				(* (uint8_t*)buffer )=*(_IO uint8_t*)&SPI_Handle->Instance->DR;
			    buffer+=sizeof(uint8_t);
				SizeofData--;

				}




		}


	}






}



void SPI_TransmitData_IT(SPI_HandleTypedef_t *SPI_Handle, uint8_t *pdata,uint16_t SizeofData){


	 SPI_BUS_STATUS statusflag=SPI_Handle->SPI_State_TX;

     if(statusflag != SPI_BUS_TX_BUSY){

         SPI_Handle->TxDataAddr=(uint8_t*)pdata;
         SPI_Handle->SizeOfData=SizeofData;
         SPI_Handle->SPI_State_TX=SPI_BUS_TX_BUSY;

         if(SPI_Handle->Init.Data_Size>SPI_DATASIZE_8BIT){
        	 SPI_Handle->TxISRfunc=SPI_TransmitHelper16BIT;



         }
         else{

        	 SPI_Handle->TxISRfunc=SPI_TransmitHelper8BIT;


         }


         SPI_Handle->Instance->CR2|=(0x1U<<SPI_CR2_TXEIE);


     }






}


void SPI_RecieveData_IT(SPI_HandleTypedef_t*SPI_Handle, uint8_t *buffer,uint16_t SizeofData){


	SPI_BUS_STATUS statusflag=SPI_Handle->SPI_State_RX;

	if(statusflag != SPI_BUS_RX_BUSY){


		SPI_Handle->RxDataAddr= (uint8_t*) buffer;
		SPI_Handle->SizeOfDataRX=SizeofData;
		SPI_Handle->SPI_State_RX=SPI_BUS_RX_BUSY;


		if(SPI_Handle->Init.Data_Size>SPI_DATASIZE_8BIT){


			SPI_Handle->RxISRfunc=SPI_RecieveHelper16BIT;

		}
		else{


			SPI_Handle->RxISRfunc=SPI_RecieveHelper8BIT;


		}

		SPI_Handle->Instance->CR2|=(0x1U<<SPI_CR2_RXEIE);



	}


}



void SPI_InterruptHandler(SPI_HandleTypedef_t *SPI_Handle){


	uint8_t ints=0;
	uint8_t intf=0;

	ints=SPI_Handle->Instance->CR2 & (0x1u<<SPI_CR2_TXEIE);
	intf=SPI_Handle->Instance->SR & (0x1u<<SPI_TXE_FLAG);

	if((ints !=0) && (intf !=0)){



      SPI_Handle->TxISRfunc(SPI_Handle);



	}
	ints=SPI_Handle->Instance->CR2 & (0x1u<<SPI_CR2_RXEIE);
	intf=SPI_Handle->Instance->SR & (0x1u<<SPI_RXNE_FLAG);

	if((ints !=0) && (intf !=0)){



      SPI_Handle->RxISRfunc(SPI_Handle);



	}



}














/*
 * @brief SPI_GetFlagStatus, return flag of SR register
 *
 * @param SPI_Handle = User Config Structure
 *
 * @param flag= flag name of SR register
 *
 * @retval SPI_Flag_Status_t
 * */



SPI_FlagStatus_t SPI_GetFlagStatus(SPI_HandleTypedef_t*SPI_Handle, uint16_t flag){

	return (SPI_Handle->Instance->SR & flag) ? SPI_FLAG_SET:SPI_FLAG_RESET;


}

