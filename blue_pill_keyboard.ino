/*
Data Pins:

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

#define TH_BIT  0b0000000100000000      // PA8
#define TR_BIT  0b0000001000000000      // PA9
#define TL_BIT  0b0000010000000000      // PA10


// Scancode buffer

#define BUFFER_SIZE 128
static volatile uint8_t buffer[BUFFER_SIZE];
static volatile uint8_t head, tail;

char PS2busy = 0;
char WriteToPS2keyboard = 0;

uint8_t bitcount = 0;
uint8_t incoming = 0;
uint8_t outgoing = 0;
uint32_t prev_ms = 0;

void setup() {

    delay(1000);
    Serial.begin(2000000);

    // Setup AT keyboard communication
    
    pinMode(PB11, INPUT_PULLUP);
    pinMode(KEYBOARD_CLOCK_PIN, INPUT_PULLUP);
    
    head = 0;
    tail = 0;
    
    attachInterrupt(KEYBOARD_CLOCK_PIN, ps2interrupt, FALLING);
    
    
    // Setup XBAND communication
    
    /*
    CRH is used to set type/and or speed of pins 8-15 of the port
    CRL is used to set type/and or speed of pins 0-7 of the port
    
    Out of these 4 bits, the low 2 bits are MODE, and high 2 bits are CNF.
    
    CNF     MODE
    
    01      00
    
    01      01
    
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
    
    
    // Serial.println("a");
    
    Talk_To_Sega();

}



void initPins()
{
//    delayMicroseconds(8);
    
//    DDRB = (DDRB & B110000) + B001111;      // make the data port an output
//    PORTB = (PORTB & B110000) + 0xC;        // present 1st nybble of ID
    
//    PORTD |= TL_BIT;                        // Raise TL (key ACK)



    delayMicroseconds(8);
    
    /*
    bit0	PB12
    bit1	PB13
    bit2	PB14
    bit3	PB15
    */
    
    // make the data port an output, open drain // CNF = 01 MODE = 01
    GPIOB->regs->CRH = (GPIOB->regs->CRH & 0x0000FFFF) | 0x55550000;

    // present 1st nybble of ID 0xC  0b 1100                       3210
    GPIOB->regs->ODR = (GPIOB->regs->ODR & 0b0000111111111111) | 0b1100000000000000;
    
    // Raise TL (key ACK) (PA10)
    GPIOA->regs->ODR = (GPIOA->regs->ODR & 0b1111101111111111) | 0b0000010000000000;
      
}


short waitForPin(short pin, short value)
{
    short numLoops = 2690;               // this number of loops will give us a timeout of
                                         // about 1.19 milliseconds which is what the
    while(numLoops != 0)                 // original keyboard has
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
    
    Serial.println("fail 111");
    return 0;                           // We timed out, return 0
}


void Talk_To_Sega()
{
    // gen TH = select
    // gen TR = request
    // key TL = acknowledge
    
    // --------------------------------------------------------------------------------
    // Give Sega the 3 remaining nibbles of our 4 nibble Catapult Keyboard ID - $C369
    // --------------------------------------------------------------------------------
    
    delayMicroseconds(8);
    
    // present 2nd nybble of ID 0x3  0b 0011                       3210
    GPIOB->regs->ODR = (GPIOB->regs->ODR & 0b0000111111111111) | 0b0011000000000000;
    
    if( !waitForPin(TR_BIT, LOW) ) {        // wait for TR (REQ) to go LOW
        initPins();
        Serial.println("fail 333");
        return;
    }
    
    
    
    delayMicroseconds(8);
    
    // present 3rd nybble of ID 0x6  0b 0110                       3210
    GPIOB->regs->ODR = (GPIOB->regs->ODR & 0b0000111111111111) | 0b0110000000000000;
    
    delayMicroseconds(1);
    
    // Lower TL (key ACK) (PA10)
    GPIOA->regs->ODR = (GPIOA->regs->ODR & 0b1111101111111111) | 0b0000000000000000;
    
    if( !waitForPin(TR_BIT, HIGH) ) {       // wait for TR (gen REQ) to go HIGH
        initPins();
        Serial.println("fail 444");
        return;
    }
    
    
    
    // !!  TURN THE DATA PORT  !!
    // !! AROUND AND SHIT HERE !!
    
    
    
    
    delayMicroseconds(8);
    
    
    
    
    // present 4th nybble of ID 0x9  0b 1001                       3210
    GPIOB->regs->ODR = (GPIOB->regs->ODR & 0b0000111111111111) | 0b1001000000000000;
    
    delayMicroseconds(1);
    
    // Raise TL (key ACK) (PA10)
    GPIOA->regs->ODR = (GPIOA->regs->ODR & 0b1111101111111111) | 0b0000010000000000;
    
    if( !waitForPin(TR_BIT, LOW) ) {        // wait for TR (REQ) to go LOW
        initPins();
        Serial.println("fail 555");
        return;
    }
    
    
    
    
    
    
    
    initPins();
    return;
}




void ps2interrupt( void )
{
//    if( WriteToPS2keyboard )
//        send_bit();
//    else
//    {
      uint32_t now_ms;
      uint8_t val, i;

      val = digitalRead( KEYBOARD_DATA_PIN );
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
                incoming |= (val << (bitcount - 2) );
                break;
        case 10: // Parity check
                break;
        case 11: // Stop bit lots of spare time now
        
        
                if(incoming == 0xFA || incoming == 0xAA || incoming == 0xFC || incoming == 0xFD || incoming == 0xFE)
                {
                    bitcount = 0;
                    incoming = 0;
                    PS2busy = 0;
                   break; 
                }
                    
        
                i = head + 1;

                if (i >= BUFFER_SIZE) i = 0;

                if (i != tail)
                {
                    buffer[i] = incoming;
                    head = i;
                }

                bitcount = 0;
                incoming = 0;
                
                PS2busy = 0;
        default:
                bitcount = 0;
                PS2busy = 0;
        }
 //     }
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
