void USART1_IRQHandler(void) {     

    /* Test status flags to determine condition(s) that triggered interrupt 
     * Only the "receive register not empty" event is shown in this example
     */
    if( USART1->ISR & USART_ISR_RXNE) {
        /* Clear the appropriate status flag in the USART
         * Technically this isn't necessary because the RXNE bit is 
         * automatically cleared by reading the RDR register.
         */
        USART1->RQR |= USART_RQR_RXFRQ;   // Clears the RXNE status bit
        
        rxbuf_push( USART1->RDR );        // Save the data in the RX register
    }
    
    /* Additional status flag checks can follow, can operate on multiple
     * events in a single interrupt as long as the total execution
     * time of the handler is short.
     */
}