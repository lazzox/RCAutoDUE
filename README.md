# RCAutoDUE
RC Autic Android DUE projekat


receivedData bytes meaning: 
 * [0] - S (beginning of transmision)
 
 * [1] - commands [1/2]
 
 *      [bit 4] - movement [0 - OFF; 1 - ON]  
 *      [bit 0] - way [0 - FORWARD; 1 - BACKWARD]
 
 * [2] - commands [2/2]
 *      [bit 4] - stearing [0 - NO; 1 - YES]  
 *      [bit 0] - side [0 - LEFT; 1 - RIGHT]
 
 * [3] - T (termination of transmision)
  
  Example of moving forward and stearing right:
 
 * [0] - 'S'
 * [1] - 0x10 (hexadecimal)
 * [2] - 0x11 (hexadecimal)
 * [3] - 'T'
 **/
 
 
 
