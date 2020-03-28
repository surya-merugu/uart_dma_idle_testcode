DMA_rx_Buffer must be greater than 10bytes and uart_rx_buffer can be as big as user wants (preferrably uart_rx_buffer is must be 
atleast some 2times greater than dma_rx_buffer).
write a user defined callback function in usart_irqhandler after idle line detection for data not to be lost in the dma_rx_buffer because
there after updation of data into uart_rx_buffer new data may be received in dma_rx_buffer which makes harder to process the data in 
uart_rx_buffer.
