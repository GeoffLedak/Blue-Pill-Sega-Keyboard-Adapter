#define KEYBOARD_DATA_PIN   PB11
#define KEYBOARD_CLOCK_PIN  PB10

#define TH_BIT  0b0000000100000000      // PA8


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
    
    // pinMode(TL, OUTPUT);
    
    /*
    CRH is used to set type/and or speed of pins 8-15 of the port
    CRL is used to set type/and or speed of pins 0-7 of the port
    */
    
    
    // set TH (PA8) to input floating
    GPIOA->regs->CRH = (GPIOA->regs->CRH & 0xFFFFFFF0) | 0x00000004;
    
    // pinMode(TR, INPUT);
    
    
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
    
    
    Serial.println("a"); 

}



void initPins()
{
//    delayMicroseconds(8);
    
//    DDRB = (DDRB & B110000) + B001111;      // make the data port an output
//    PORTB = (PORTB & B110000) + 0xC;        // present 1st nybble of ID
    
//    PORTD |= TL_BIT;                        // Raise TL (key ACK)
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
