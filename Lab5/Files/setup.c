/* Clear the NBYTES and SADD bit fields
 * The NBYTES field begins at bit 16
 * The SADD field begins at bit 0
 */
I2C1->CR2 &= ~((0x7F << 16) | (0x3FF << 0));

/* Set the NBYTES and SADD bit fields
 * Can use hex or decimal values directly as bitmasks. 
 * Sets NBYTES = 42, and SADD = 0x14
 * Remember that for 7-bit addresses, the lowest SADD bit
 * 	 is not used and the mask must be shifted by one.
 */
I2C1->CR2 &= ~((42 << 16) | (0x14 << 1));