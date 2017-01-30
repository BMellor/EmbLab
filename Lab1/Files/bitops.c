// Sets the 3rd bit in the GPIOC_MODER register
GPIOC->MODER |= (1 << 3);   
        
// Sets the 3rd and 5th bits in the GPIOC_MODER register		
GPIOC->MODER |= (1 << 3) | (1 << 5);   
        
// Clears the 3rd and 5th bits in the GPIOC_MODER register		
GPIOC->MODER &= ~((1 << 3) | (1 << 5));  