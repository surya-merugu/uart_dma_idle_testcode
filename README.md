# PROBLEMS AND SOLUTIONS


#define DMA_RX_BUFFER_SIZE          64
uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];

The DMA buffer size must be greater than 10 or else the buffer will be filled with some junk data sometimes and it hard to process the buffer.


#define DMA_RX_BUFFER_SIZE            256
uint8_t UART_Buffer[UART_BUFFER_SIZE];

The UART buffer size can vary according to the user requirement but must be greater than twice of the DMA buffer is preferrable.


# USER DEFINED FILES AND CALLBACKS

There are 2 user defined irqhandlers which must be replaced with the hal usart and dma irqhandlers

>>void USART_IrqHandler (UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma);

the above irq handler is called whenever there is the data in DMA_RX_Buffer and idle line is detected where we can write a user defined callback the data processing.

>>hdma->Instance->CR &= ~DMA_SxCR_EN;       /* idle line detection */

after the above statement user can write his own callback.

>>void DMA_IrqHandler (DMA_HandleTypeDef *hdma)

the above irqhandler called when the data is received and this user defined irq also copies data from  DMA_RX_Buffer to UART_Buffer 

Note:- User defined callback is better to have in USART_IrqHandler(); because idle line is detected in this function and data is readily available in DMA_RX_Buffer and it is bit complicate to search the data in UART_Buffer

