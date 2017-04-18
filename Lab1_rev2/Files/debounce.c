uint32_t debouncer = 0;
while(1) {
    debouncer = (debouncer << 1); // Always shift every loop iteration
    
    if (input_signal) {     // If input signal is set/high
        debouncer |= 0x01;  // Set lowest bit of bit-vector
    }
    
    if (debouncer == 0xFFFFFFFF) {
        // This code triggers repeatedly when button is steady high!
    }
    if (debouncer == 0x00000000) {
        // This code triggers repeatedly when button is steady low!
    }
    if (debouncer == 0x7FFFFFFF) {
        // This code triggers only once when transitioning to steady high!
    }
    // When button is bouncing the bit-vector value is random since bits are set when the button is high and not when it bounces low.
}