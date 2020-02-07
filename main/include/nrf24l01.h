int nrf24_receive_pkt ( uint8_t *data, int timeout) {
    //setup nrf24l01 transmitter
    spiWriteByte (hspi, 0x20 | 0x00, 0x01); //no crc, rx mode
    spiWriteByte (hspi, 0x20 | 0x01, 0x00); //no auto ack
    spiWriteByte (hspi, 0x20 | 0x02, 0x01); //pipe0
    spiWriteByte (hspi, 0x20 | 0x03, 0x03); 
    spiWriteByte (hspi, 0x20 | 0x04, 0x00); 
    spiWriteByte (hspi, 0x20 | 0x05, 0x05); //freq channel 5
    spiWriteByte (hspi, 0x20 | 0x06, 0x06); //low power, 1MB/sec
    spiWriteByte (hspi, 0x20 | 0x11, 0x20); //use all 32 bytes

    //turn on and flush fifo
    spiWriteByte (hspi, 0x20 | 0x00, 0x03); //turn on
    spiWriteByte (hspi, 0xe1, 0x00); //flush tx fifo
    spiWriteByte (hspi, 0x20 | 0x07, 0x70); 
    gpio_set_level (NRF24L01_CE, 1);
    ets_delay_us(100);                          //busy-wait

    int waitcnt = 0;
    while(1){
        spiReadByte (hspi, 0x07, data);
        //ets_delay_us(1000);                          //busy-wait
        vTaskDelay(1);
        if( (data[0] & 0x40) > 1 || waitcnt > timeout) break;
        ++waitcnt;
    }
    //if (waitcnt > timeout) printf("wait timed out\n");

    //read packet from nrf24l01 transmitter
    spiReadBytes ( hspi, 0x61, 32+1, data);

    //set ce = 0
    gpio_set_level (NRF24L01_CE, 0);

    return(waitcnt);
}

int nrf24_transmit_pkt ( uint8_t *data, int length) {
    //setup nrf24l01 transmitter
    spiWriteByte (hspi, 0x20 | 0x00, 0x00); //no crc, tx mode
    spiWriteByte (hspi, 0x20 | 0x01, 0x00); //no auto ack
    spiWriteByte (hspi, 0x20 | 0x02, 0x01); //pipe0
    spiWriteByte (hspi, 0x20 | 0x03, 0x03); 
    spiWriteByte (hspi, 0x20 | 0x04, 0x04); 
    spiWriteByte (hspi, 0x20 | 0x05, 0x05); //freq channel 5
    spiWriteByte (hspi, 0x20 | 0x06, 0x06); //high power, 1MB/sec
    spiWriteByte (hspi, 0x20 | 0x11, 0x20); //use all 32 bytes

    //turn on and flush fifo
    spiWriteByte (hspi, 0x20 | 0x00, 0x02); //turn on
    spiWriteByte (hspi, 0xe1, 0x00);        //flush tx fifo
    spiWriteByte (hspi, 0x20 | 0x07, 0x70); 
    ets_delay_us(100);                          //busy-wait

    //send packet to nrf24l01 transmitter
    spiWriteBytes (hspi, 0xa0, length, data);

    //ce chip radio enable - bradboard need to really slow this up (100us)
    gpio_set_level (NRF24L01_CE, 1);
    ets_delay_us(500);                          //busy-wait
    gpio_set_level (NRF24L01_CE, 0);

    spiWriteByte (hspi, 0x20 | 0x00, 0x00); //turn off

    return(0);
}

void nrf24_gpio_init() {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << NRF24L01_CE);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config ( &io_conf );
}

