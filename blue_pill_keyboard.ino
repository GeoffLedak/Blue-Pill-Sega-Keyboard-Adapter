/*
bit0    PB12
bit1    PB13
bit2    PB14
bit3    PB15

TH      PA8     controlled by sega
TR      PA9     controlled by sega
TL      PA10    controlled by adapter
*/

#define KEYBOARD_DATA_PIN   PB11
#define KEYBOARD_CLOCK_PIN  PB10

#define KEYBOARD_CLOCK_PIN_BIT  0b0000010000000000

#define TH_BIT  0b0000000100000000      // PA8
#define TR_BIT  0b0000001000000000      // PA9
#define TL_BIT  0b0000010000000000      // PA10

// Scancode buffer
#define BUFFER_SIZE 128
volatile uint8_t buffer[BUFFER_SIZE];
volatile uint8_t head, tail;

// Control data (to send to keyboard, Reset, Caps LED, etc) buffer
volatile uint8_t sendBuffer[BUFFER_SIZE];
volatile uint8_t sendHead, sendTail;

volatile uint8_t lastByteSentToKeyboard = 0;
volatile char waitingForAck = 0;

volatile uint8_t daTimer = 0;

volatile char PS2busy = 0;
volatile char WriteToPS2keyboard = 0;

volatile uint8_t bitcount = 0;
volatile uint8_t incoming = 0;
volatile uint8_t outgoing = 0;
volatile uint32_t prev_ms = 0;

volatile uint8_t _parity = 0;

volatile char hasParityError = 0;


void setup()
{
    Serial.begin(9600);
    
    // Setup AT keyboard communication
    
    // set KEYBOARD_DATA_PIN (PB11) to input floating
    GPIOB->regs->CRH = (GPIOB->regs->CRH & 0xFFFF0FFF) | 0x00004000;    // CNF = 01 MODE = 00
    
    // set KEYBOARD_CLOCK_PIN (PB10) to input floating
    GPIOB->regs->CRH = (GPIOB->regs->CRH & 0xFFFFF0FF) | 0x00000400;    // CNF = 01 MODE = 00
    
    head = 0;
    tail = 0;
    sendHead = 0;
    sendTail = 0;
    
    attachInterrupt(KEYBOARD_CLOCK_PIN, ps2interrupt, FALLING);
    
    
    // Setup XBAND communication
    
    /*
    CRH is used to set type/and or speed of pins 8-15 of the port
    CRL is used to set type/and or speed of pins 0-7 of the port
    
    Out of these 4 bits, the low 2 bits are MODE, and high 2 bits are CNF.
    */
    
    // set TH (PA8) to input floating
    GPIOA->regs->CRH = (GPIOA->regs->CRH & 0xFFFFFFF0) | 0x00000004;    // CNF = 01 MODE = 00
    
    // set TR (PA9) to input floating
    GPIOA->regs->CRH = (GPIOA->regs->CRH & 0xFFFFFF0F) | 0x00000040;    // CNF = 01 MODE = 00
    
    // set TL (PA10) to output open drain
    GPIOA->regs->CRH = (GPIOA->regs->CRH & 0xFFFFF0FF) | 0x00000500;    // CNF = 01 MODE = 01
    
    initPins();
}

void loop()
{
    // do nothing while we wait for TH to go high again (transaction complete)
    do{ }
    while( (GPIOA->regs->IDR & TH_BIT) != TH_BIT );

    // wait for TH to go low
    do{ }
    while( (GPIOA->regs->IDR & TH_BIT) != 0 );
    
    Talk_To_Sega();
    
    

    
    if( ( sendHead != sendTail ) && !waitingForAck )
    {
        daTimer = 0;
        sendNow();
    }
    
    

   if( ( sendHead != sendTail ) && waitingForAck )
   {
       daTimer++;
       
       if( daTimer >= 30 )
       {
           if(sendTail == 0)
           {
               sendTail = (BUFFER_SIZE - 1);
           }
           else
           {
               sendTail--;
           }
           
           daTimer = 0;
           
           sendNow();
       }
   }

   
}



void initPins()
{
    delayMicroseconds(5);
    
    // make the data port an output, open drain // CNF = 01 MODE = 01
    GPIOB->regs->CRH = (GPIOB->regs->CRH & 0x0000FFFF) | 0x55550000;

    // present 1st nybble of ID 0xC  0b 1100                       3210
    GPIOB->regs->ODR = (GPIOB->regs->ODR & 0b0000111111111111) | 0b1100000000000000;
    
    delayMicroseconds(1);
    
    // Raise TL (key ACK) (PA10)
    GPIOA->regs->ODR = (GPIOA->regs->ODR & 0b1111101111111111) | 0b0000010000000000;    
}


short waitForPin(short pin, short value)
{
    short numLoops = 5670;

    while(numLoops != 0)
    {
        __asm__("nop\n\t");
        
        if( (GPIOA->regs->IDR & TH_BIT) != 0 )      // TH went high, transaction aborted
            return 0;
            
        if( value == LOW && (GPIOA->regs->IDR & pin) == value )     // We found the value we want, return 1
            return 1;
        
        if( value == HIGH && (GPIOA->regs->IDR & pin) == pin )      // We found the value we want, return 1
            return 1;
          
        numLoops--;
    }
    
    return 0;                           // We timed out, return 0
}


void endWait()
{
    short numLoops = 5670;               // this number of loops will give us a timeout of
                                         // about 1.19 milliseconds which is what the
    while(numLoops != 0)                 // original keyboard has
    {
        __asm__("nop\n\t");
        
        if( (GPIOA->regs->IDR & TH_BIT) != 0 )      // TH went high, transaction aborted
            return;
            
        numLoops--;
    }
    
    return;                             // We timed out
}



void Talk_To_Sega()
{
    // gen TH = select
    // gen TR = request
    // key TL = acknowledge
    
    // --------------------------------------------------------------------------------
    // Give Sega the 3 remaining nibbles of our 4 nibble Catapult Keyboard ID - $C369
    // --------------------------------------------------------------------------------
    
    delayMicroseconds(9);
    
    // present 2nd nybble of ID 0x3  0b 0011                       3210
    GPIOB->regs->ODR = (GPIOB->regs->ODR & 0b0000111111111111) | 0b0011000000000000;
    
    if( !waitForPin(TR_BIT, LOW) ) {        // wait for TR (REQ) to go LOW
        initPins();
        return;
    }
    
    
    delayMicroseconds(7);
    
    // present 3rd nybble of ID 0x6  0b 0110                       3210
    GPIOB->regs->ODR = (GPIOB->regs->ODR & 0b0000111111111111) | 0b0110000000000000;
    
    delayMicroseconds(1);
    
    // Lower TL (key ACK) (PA10)
    GPIOA->regs->ODR = (GPIOA->regs->ODR & 0b1111101111111111) | 0b0000000000000000;
    
    if( !waitForPin(TR_BIT, HIGH) ) {       // wait for TR (gen REQ) to go HIGH
        initPins();
        return;
    }
    
    delayMicroseconds(5);
    
    // turn the data port around (make it an input), is this a write?
    // make the data port an input, floating // CNF = 01 MODE = 00
    GPIOB->regs->CRH = (GPIOB->regs->CRH & 0x0000FFFF) | 0x44440000;
    
    delayMicroseconds(7);
    
    unsigned short value = GPIOB->regs->IDR & 0b1111000000000000;
    
    // does the Sega want to send us data?
    if(value == 0)
    {
        Listen_To_Sega();
        return;
    }
    
    // otherwise
    // make data port an output
    GPIOB->regs->CRH = (GPIOB->regs->CRH & 0x0000FFFF) | 0x55550000;
    
    delayMicroseconds(1);
    
    // present 4th nybble of ID 0x9  0b 1001                       3210
    GPIOB->regs->ODR = (GPIOB->regs->ODR & 0b0000111111111111) | 0b1001000000000000;
    
    delayMicroseconds(2);
    
    // Raise TL (key ACK) (PA10)
    GPIOA->regs->ODR = (GPIOA->regs->ODR & 0b1111101111111111) | 0b0000010000000000;
    
    if( !waitForPin(TR_BIT, LOW) ) {        // wait for TR (REQ) to go LOW
        delayMicroseconds(4);               // if TH went high here, this is a find
        initPins();
        return;
    }
    
    // --------------------------------------------------------------------------------
    // TALK to Sega
    // --------------------------------------------------------------------------------
    
    
    // is there stuff in the buffer?
    
    uint8_t s = get_scan_code();
    
    if(s == 0)                                  // buffer is empty
    {
        delayMicroseconds(7);
        
        // zero bytes to send                                          0000
        GPIOB->regs->ODR = (GPIOB->regs->ODR & 0b0000111111111111) | 0b0000000000000000;
        
        delayMicroseconds(1);
        
        // Lower TL (key ACK) (PA10)
        GPIOA->regs->ODR = (GPIOA->regs->ODR & 0b1111101111111111) | 0b0000000000000000;
    
    
        endWait();                              // wait for start to go up
        initPins();                             // We're all done
        return;
    }
    else                                        // buffer has stuff in it
    {
        uint16_t length = 0;
        uint8_t scancodes[15];
        uint8_t *index = scancodes;
        
        *index = 0;                             // put a type byte of 0 to send first
        index++;
        length++;
        
        while( s != 0)                          // now add the actual scancodes
        {
            *index = s;
            index++;
            s = get_scan_code();
            length++;
            
            if( length > 14 )
                break;
        }
        
        
        // now send stuff here
        
        delayMicroseconds(4);
        
        // how many bytes we want to send
        GPIOB->regs->ODR = (GPIOB->regs->ODR & 0b0000111111111111) | ( length << 12 );
    
        delayMicroseconds(1);
        
        // Lower TL (key ACK) (PA10)
        GPIOA->regs->ODR = (GPIOA->regs->ODR & 0b1111101111111111) | 0b0000000000000000;
        
        
          
        index = scancodes; 
        
        while( length > 0 )                                 // then send the actual scancodes
        {
            
            if( !waitForPin(TR_BIT, HIGH) ) {               // wait for TR (gen REQ) to go HIGH
                initPins();
                return;
            }
            
            delayMicroseconds(3);
            
            // write high nibble
            GPIOB->regs->ODR = (GPIOB->regs->ODR & 0b0000111111111111) | ( (*index >> 4) << 12 );
            
            delayMicroseconds(6);
            
            // Raise TL (key ACK) (PA10)
            GPIOA->regs->ODR = (GPIOA->regs->ODR & 0b1111101111111111) | 0b0000010000000000;
        
        
            // --------------
        
        
            if( !waitForPin(TR_BIT, LOW) ) {                // wait for TR (gen REQ) to go LOW
                initPins();
                return;
            }
            
            delayMicroseconds(4);
            
            // write low nibble
            GPIOB->regs->ODR = (GPIOB->regs->ODR & 0b0000111111111111) | ( (*index & 0xF) << 12 );
            
            delayMicroseconds(5);
            
            // Lower TL (key ACK) (PA10)
            GPIOA->regs->ODR = (GPIOA->regs->ODR & 0b1111101111111111) | 0b0000000000000000;
            
            index++;
            length--;
        }
    
    
        endWait();                              // wait for start to go up
        initPins();                             // We're all done
        return;   
    }
    
}


void Listen_To_Sega()
{
    // Raise TL (key ACK) (PA10)
    GPIOA->regs->ODR = (GPIOA->regs->ODR & 0b1111101111111111) | 0b0000010000000000;
    
    if( !waitForPin(TR_BIT, LOW) ) {        // wait for TR (REQ) to go LOW
        initPins();
        return;
    }
    
    delayMicroseconds(2);
    
    // get the BYTE count (includes TYPE, which we don't care about)
    short byteCount = (GPIOB->regs->IDR & 0b1111000000000000) >> 12;
    
    delayMicroseconds(3);
    
    // Lower TL (key ACK) (PA10)
    GPIOA->regs->ODR = (GPIOA->regs->ODR & 0b1111101111111111) | 0b0000000000000000;
    
    
    
    
    
    
    if( !waitForPin(TR_BIT, HIGH) ) {       // wait for TR (gen REQ) to go HIGH
        initPins();
        return;
    }
    
    delayMicroseconds(2);
    
    short type1 = (GPIOB->regs->IDR & 0b1111000000000000) >> 12;
    
    delayMicroseconds(6);
    
    // Raise TL (key ACK) (PA10)
    GPIOA->regs->ODR = (GPIOA->regs->ODR & 0b1111101111111111) | 0b0000010000000000;
    
    
    
    
    
    if( !waitForPin(TR_BIT, LOW) ) {        // wait for TR (REQ) to go LOW
        initPins();
        return;
    }
    
    delayMicroseconds(2);
    
    short type2 = (GPIOB->regs->IDR & 0b1111000000000000) >> 12;
    
    delayMicroseconds(2);
    
    // Lower TL (key ACK) (PA10)
    GPIOA->regs->ODR = (GPIOA->regs->ODR & 0b1111101111111111) | 0b0000000000000000;
    
    byteCount--;
    
    
    
    
    static volatile uint8_t incomingValue = 0;
    
    
    while(byteCount > 0)
    {
        
        if( !waitForPin(TR_BIT, HIGH) ) {       // wait for TR (gen REQ) to go HIGH
            initPins();
            return;
        }
        
        delayMicroseconds(2);
        
        incomingValue = (GPIOB->regs->IDR & 0b1111000000000000) >> 12;
        incomingValue <<= 4;
                    
        delayMicroseconds(7);
        
        // Raise TL (key ACK) (PA10)
        GPIOA->regs->ODR = (GPIOA->regs->ODR & 0b1111101111111111) | 0b0000010000000000;
        
        
        
        
        if( !waitForPin(TR_BIT, LOW) ) {        // wait for TR (REQ) to go LOW
            initPins();
            return;
        }
        
        delayMicroseconds(2);
        
        incomingValue |= ((GPIOB->regs->IDR & 0b1111000000000000) >> 12);
        
        delayMicroseconds(4);
        
        // Lower TL (key ACK) (PA10)
        GPIOA->regs->ODR = (GPIOA->regs->ODR & 0b1111101111111111) | 0b0000000000000000;
        
        
        // add incomingValue to buffer
        uint8_t i = sendHead + 1;

        if (i >= BUFFER_SIZE) i = 0;

        if (i != sendTail)
        {
            sendBuffer[i] = incomingValue;
            sendHead = i;
        }
        

        byteCount--;   
    }
    
    
    Serial.println(incomingValue, HEX);
    
    
    endWait();                              // wait for start to go up
    initPins();                             // We're all done
    return;
}




void sendNow()
{
    outgoing = get_byte_to_send_to_keyboard();
    lastByteSentToKeyboard = outgoing;
    
    
    // Spin here until PS2busy == 0;
    // and keyboard clock pin is high
    // ADD A TIMEOUT FOR THIS
    do { }
    while(PS2busy != 0 && (GPIOB->regs->IDR & KEYBOARD_CLOCK_PIN_BIT) != KEYBOARD_CLOCK_PIN_BIT );
    
   
    PS2busy = 1;
    WriteToPS2keyboard = 1;
    
    waitingForAck = 1;
    
    
    _parity = 0;
    bitcount = 0;
    
    
    // set pins to outputs and high
    
    // set KEYBOARD_DATA_PIN (PB11) high
    GPIOB->regs->ODR = (GPIOB->regs->ODR & 0b1111011111111111) | 0b0000100000000000;
    
    // set KEYBOARD_DATA_PIN (PB11) output open drain
    GPIOB->regs->CRH = (GPIOB->regs->CRH & 0xFFFF0FFF) | 0x00005000;    // CNF = 01 MODE = 01
    

    // set KEYBOARD_CLOCK_PIN (PB10) high
    GPIOB->regs->ODR = (GPIOB->regs->ODR & 0b1111101111111111) | 0b0000010000000000;
    
    // set KEYBOARD_CLOCK_PIN (PB10) output open drain
    GPIOB->regs->CRH = (GPIOB->regs->CRH & 0xFFFFF0FF) | 0x00000500;    // CNF = 01 MODE = 01
    
    
    
    delayMicroseconds( 10 );
    
    
    // set KEYBOARD_CLOCK_PIN (PB10) low
    GPIOB->regs->ODR = (GPIOB->regs->ODR & 0b1111101111111111) | 0b0000000000000000;
    
    // set clock low for 60us
    delayMicroseconds( 60 );
    
    
    // set KEYBOARD_DATA_PIN (PB11) low
    GPIOB->regs->ODR = (GPIOB->regs->ODR & 0b1111011111111111) | 0b0000000000000000;
    

    // set KEYBOARD_CLOCK_PIN (PB10) to input floating
    GPIOB->regs->CRH = (GPIOB->regs->CRH & 0xFFFFF0FF) | 0x00000400;    // CNF = 01 MODE = 00  
}




void sendSpecificByte(uint8_t byteValue)
{
    outgoing = byteValue;

    // Spin here until PS2busy == 0;
    // and keyboard clock pin is high
    // ADD A TIMEOUT FOR THIS
    do { }
    while(PS2busy != 0 && (GPIOB->regs->IDR & KEYBOARD_CLOCK_PIN_BIT) != KEYBOARD_CLOCK_PIN_BIT );
    

    PS2busy = 1;
    WriteToPS2keyboard = 1;
    
    waitingForAck = 1;
    
    
    _parity = 0;
    bitcount = 0;
    
    
    // set pins to outputs and high
    
    // set KEYBOARD_DATA_PIN (PB11) high
    GPIOB->regs->ODR = (GPIOB->regs->ODR & 0b1111011111111111) | 0b0000100000000000;
    
    // set KEYBOARD_DATA_PIN (PB11) output open drain
    GPIOB->regs->CRH = (GPIOB->regs->CRH & 0xFFFF0FFF) | 0x00005000;    // CNF = 01 MODE = 01
    

    // set KEYBOARD_CLOCK_PIN (PB10) high
    GPIOB->regs->ODR = (GPIOB->regs->ODR & 0b1111101111111111) | 0b0000010000000000;
    
    // set KEYBOARD_CLOCK_PIN (PB10) output open drain
    GPIOB->regs->CRH = (GPIOB->regs->CRH & 0xFFFFF0FF) | 0x00000500;    // CNF = 01 MODE = 01
    
    
    
    delayMicroseconds( 10 );
    
    
    // set KEYBOARD_CLOCK_PIN (PB10) low
    GPIOB->regs->ODR = (GPIOB->regs->ODR & 0b1111101111111111) | 0b0000000000000000;
    
    // set clock low for 60us
    delayMicroseconds( 60 );
    
    
    // set KEYBOARD_DATA_PIN (PB11) low
    GPIOB->regs->ODR = (GPIOB->regs->ODR & 0b1111011111111111) | 0b0000000000000000;
    

    // set KEYBOARD_CLOCK_PIN (PB10) to input floating
    GPIOB->regs->CRH = (GPIOB->regs->CRH & 0xFFFFF0FF) | 0x00000400;    // CNF = 01 MODE = 00  
}





void ps2interrupt( void )
{
    if( ( GPIOB->regs->IDR & 0b0000010000000000 ) != 0 )
        return;
    
    
    if( WriteToPS2keyboard )
        send_bit();
    else
    {
      uint32_t now_ms;
      uint8_t val, i;


      // make sure we read in the middle of the low clock
      delayMicroseconds(15);

      // Read value of KEYBOARD_DATA_PIN
      val = ( GPIOB->regs->IDR & 0b0000100000000000 ) >> 11;
      
      now_ms = millis();
      
      if( now_ms - prev_ms > 250 )
      {
        bitcount = 0;
        incoming = 0;
        PS2busy = 0;
      }
      
      prev_ms = now_ms;
      bitcount++;
      
      switch( bitcount )
      {
        case 1: // Start bit
                _parity = 0;
                PS2busy = 1;
                break;
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9: // Data bits
                _parity += val;          // another one received ?
                incoming |= (val << (bitcount - 2) );
                break;
        case 10: // Parity check
        
        
                _parity &= 1;            // Get LSB if 1 = odd number of 1's so parity bit should be 0
                
                if( _parity == val )     // Both same parity error
                {
                    hasParityError = 1;
                    Serial.println("P.E ====");
                } 
        
        
        
                break;
        case 11: // Stop bit lots of spare time now
        
        
                // IF HAS PARITY ERROR, DONT ADD IT TO THE BUFFER
                // PULL CLOCK LOW
        
        
                Serial.println(incoming, HEX);
        
        
                if( hasParityError && incoming != 0xFF )
                {
                    sendSpecificByte(lastByteSentToKeyboard);
                }
                else if( incoming == 0xFF )
                {
                    Serial.println("ASS ****");
                    waitingForAck = 0;
                    sendSpecificByte(0xF4);
                }
                else
                {
                    if( incoming == 0xFA || incoming == 0xFE )
                    {
                        waitingForAck = 0;
                        
                        if( incoming == 0xFE )
                        {
                            if(sendTail == 0)
                            {
                                sendTail = (BUFFER_SIZE - 1);
                            }
                            else
                            {
                                sendTail--;
                            }

                        }
                    }
            
            
                    i = head + 1;

                    if (i >= BUFFER_SIZE) i = 0;

                    if (i != tail)
                    {
                        buffer[i] = incoming;
                        head = i;
                    }

                }


                bitcount = 0;
                incoming = 0;
                
                PS2busy = 0;
        default:
                bitcount = 0;
                PS2busy = 0;
        }
    }
}


void send_bit()
{
    uint8_t val;

    bitcount++;               // Now point to next bit
    
    switch( bitcount )
    {
      case 1:
              // set KEYBOARD_DATA_PIN low (it should be low already)
              GPIOB->regs->ODR = (GPIOB->regs->ODR & 0b1111011111111111);
              
              
              break;
      case 2:
      case 3:
      case 4:
      case 5:
      case 6:
      case 7:
      case 8:
      case 9:
              // Data bits
              val = outgoing & 0x01;   // get LSB
              
              // send bit. KEYBOARD_DATA_PIN = PB11
              GPIOB->regs->ODR = (GPIOB->regs->ODR & 0b1111011111111111) | (val << 11);
              
              _parity += val;            // another one received ?
              outgoing >>= 1;          // right _SHIFT one place for next bit
              break;
      case 10:
              // Parity - Send LSB if 1 = odd number of 1's so parity should be 0
              GPIOB->regs->ODR = (GPIOB->regs->ODR & 0b1111011111111111) | (( ~_parity & 1 ) << 11);         
              
              break;
      case 11: // Stop bit write change to input for high stop bit
              // set KEYBOARD_DATA_PIN (PB11) to input floating
              GPIOB->regs->CRH = (GPIOB->regs->CRH & 0xFFFF0FFF) | 0x00004000;    // CNF = 01 MODE = 00
    
              
              
              break;
      case 12: // Acknowledge bit low we cannot do anything if high instead of low
                
               bitcount = 0;
               PS2busy = 0;
               outgoing = 0;
               WriteToPS2keyboard = 0;
      
         
     
              
              
      default: // in case of weird error and end of byte reception re-sync
              bitcount = 0;
    }
    
}



static inline uint8_t get_scan_code(void)
{
    uint8_t c, i;

    i = tail;
    if (i == head) return 0;
    i++;
    if (i >= BUFFER_SIZE) i = 0;
    c = buffer[i];
    tail = i;
    return c;
}


static inline uint8_t get_byte_to_send_to_keyboard(void)
{
    uint8_t c, i;

    i = sendTail;
    if (i == sendHead) return 0;
    i++;
    if (i >= BUFFER_SIZE) i = 0;
    c = sendBuffer[i];
    sendTail = i;
    return c;
}
