/*
 * spi.h
 *
 * Created: 4/1/2015 2:07:48 PM
 *  Author: Emon
 */ 


#ifndef SPI_H_
#define SPI_H_

void spi_master_initialize()
{
	//set !SS,MOSI,SCK pin as output pins
	DDRB |= (1<<PINB2) | (1<<PINB3) | (1<<PINB5);
	//set MISO as input pin
	DDRB &= ~(1<<PINB4);
	//enable SPI and set as master
	SPCR |= (1<<SPE) | (1<<MSTR);

	
}

void spi_master_transmit_byte(uint8_t data){
	//fill SPDR with data to write
	SPDR = data;
	//wait for transmission to complete
	while(!(SPSR&(1<<SPIF)));
}

void spi_master_receive_byte(uint8_t *data){
	//transmit dummy byte
	spi_master_transmit_byte(0x00);
	*data = SPDR;
}





void spi_master_transmit_byte_ref(uint8_t *data){
	//fill SPDR with data to write
	SPDR = *data;
	//wait for transmission to complete
	while(!(SPSR&(1<<SPIF)));
	
}

uint8_t spi_master_transmit_byte_val(uint8_t data){
	
	//fill SPDR with data to write
	SPDR = data;
	//wait for transmission to complete
	while(!(SPSR&(1<<SPIF)));
	return SPDR;
}

void spi_master_receive_byte_ref(uint8_t *data){
	
	//transmit dummy byte
	spi_master_transmit_byte(0x00);
	*data = SPDR;
}

void spi_master_byte_trade(uint8_t datain, uint8_t *dataout)
{
	spi_master_transmit_byte(datain);
	*dataout = SPDR;
}



#endif /* SPI_H_ */