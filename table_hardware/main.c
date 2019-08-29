// import all the relevant informations from the project configuration, such as devices
#include <project.h>
#include <stdbool.h>
#include <stdio.h>

// We only need one bit right now but it will be stored on a byte when sent over the wire.
// That allows us to reserve some space in case we want to add new features in the future,
// and I believe it is better not to mix bytes of header and of data together anyway.
enum UARTMessageType {
    EOL = 0,
    Measurement = 1,
    Validation = 2
};

enum State {
    Base,
    Prepare_moving,
    Moving
    
};

enum move_pins {
    FW1 = (1<<7),
    FW2 = (1<<5),
    FW3 = (1<<3),
    FWALL = (1<<1),
    REV1 = (1<<6),
    REV2 = (1<<4),
    REV3 = (1<<2),
    REVALL = (1<<0)
};

enum clutch_pins {
    BLK1_CLUTCH_FORCE = (1<<0),
    BLK2_CLUTCH_FORCE = (1<<1),
    BLK3_CLUTCH_FORCE = (1<<2),
    MAIN_CLUTCH_FORCE = (1<<3)
};

static enum State state;
static bool usb_enabled;

// This buffer contains the informations that will be sent to the user via USB.
static char uart_data[24];

// this function will send the data currently in the 'uart_data' buffer over the USB-UART "wire"
void uart_submit_data(void) {
    // wait for the previous transfer to complete before sending the bytes
    while (!USBUART_CDCIsReady());
    USBUART_PutData((unsigned char*)uart_data, strlen(uart_data)+1);
}

// retrieve the number of the view currently selected (from 1 to 3, but return 0 if none or multiple views are selected simultaneously)
int8 get_camera_number(void) {
    int8 lamps = ~LAMPS_INPUT_Read();
    switch (lamps) {
        case 1<<0:
            return 1;
        case 1<<1:
            return 2;
        case 1<<2:
            return 3;
        default:
            return 0;
    };
}

// retrieve the current encoded position and send it over the wire
CY_ISR(update_rotary_encoders) {
    // read the counters from the rotary encoders and send then over the USB-UART connection
    snprintf((char*)uart_data, 24, "%i,%ld,%ld\r\n", get_camera_number(), QuadDec_1_GetCounter(), QuadDec_2_GetCounter());
    uart_data[23] = 0;
    
    uart_submit_data();

    // reset counters
    QuadDec_1_SetCounter(0);
    QuadDec_2_SetCounter(0);
}

// retrieve the number or the current view (as indicated by the two-digit selector on the table)
// TODO: add support with the MCP23016 I/O expanders
int8 get_view_number(void) {
    return 0;
}

// inform the computer this sequence of measurements are finished
CY_ISR(validate_measurements) {
    snprintf(uart_data, 24, "validate: %i\r\n", get_view_number());
    uart_data[23] = 0;
    
    uart_submit_data();
    
    // reset counters
    QuadDec_1_SetCounter(0);
    QuadDec_2_SetCounter(0);
} 

void reset_motor(void) {
    MOTOR_REVERSE_Write(0);
    MOTOR_ENABLE_Write(0);
}

// disable the relays driving the clutches of each block as well as the main one
void reset_clutches(void) {
    CLUTCHES_FORCE_Write(0);
}


// disable all the relays (in case of emergency stop for example)
void disable_relays(void) {
    // I wish there was a better way of enumerating all the relays, but the only one I can think about
    // isn't very great either.
    reset_motor();
    reset_clutches();
    
    BLK1_MAGNET_Write(0);
    BLK2_MAGNET_Write(0);
    BLK3_MAGNET_Write(0);
}

CY_ISR(max_speed_exceeded) {
    // Notify the user via a blinking LED
    STATUS_LED_PWM_Start();
    if (usb_enabled) {
        snprintf(uart_data, 24, "Slow down !\r\n");
        uart_data[23] = 0;
        
        uart_submit_data();
    }
}

// Warn the user of an emergency stop
CY_ISR(kill_switch_warn) {
    // stop what we can, this will protect us in case there is a bad contact in the kill switch or seomething
    disable_relays();
    INPUT_CLOCK_Stop();

    if (usb_enabled) {
        snprintf(uart_data, 24, "Emergency shutdown !\r\n");
        uart_data[23] = 0;
        
        uart_submit_data();
    }

    // make the status LED blink to warn the user we are aware there is an issue
    STATUS_LED_PWM_Start();

    while (KS_Read() == 0) {
        CyDelay(150);
    }
    
    // We cannot afford to return where we were before the interrupt, as we don't want to end up in a random state
    // (which is clearly possible as we could return just before an instruction like 'state = FW_1').
    CySoftwareReset();
}

int main() {
    CyGlobalIntEnable;
    
    // Initialize the rotary encoders
    QUAD_DECODER_CLOCK_Start();
    Comp_IA1_1_Start();
    Comp_IA1_2_Start();
    Comp_IA2_1_Start();
    Comp_IA2_2_Start();
    QuadDec_1_Start();
    QuadDec_2_Start();
    
    // Initialize the USB device, then the UART module on top of it
    // we wait 2 seconds and disable USB if it doesn't work
    USBUART_Start(0u, USBUART_5V_OPERATION);

    int count_iter = 0;
    do {
        CyDelay(100);
        count_iter++;
    } while(USBUART_GetConfiguration() == 0 && count_iter < 20);
    if (count_iter < 20) {
        usb_enabled = true;
    }
    
    if (usb_enabled) {
        USBUART_CDC_Init();
    }
    


    if (usb_enabled) {
        // Initialize the push buttons (handled via interrupts)
        Clock_debouncers_Start();
        measurement_int_StartEx(update_rotary_encoders);
        validation_int_StartEx(validate_measurements);
    }
    quad_dec_1_speed_int_StartEx(max_speed_exceeded);
    quad_dec_2_speed_int_StartEx(max_speed_exceeded);

    // Enable the other inputs
    INPUT_CLOCK_Start();
    
    // Enable the kill switch interrupt
    STATUS_LED_PWM_CLOCK_Start();
    
    ks_int_StartEx(kill_switch_warn);
    // initial state
    state = Base;
    
    for (;;) {
    
        int8 move_cmd = ~MOVE_FILMS_Read();
         // do we want to roll/unroll some films ?
        bool move_all = (move_cmd & FWALL) || (move_cmd & REVALL);
        bool move_1 =  (move_cmd & FW1) || (move_cmd & REV1) || move_all;
        bool move_2 = (move_cmd & FW2) || (move_cmd & REV2) || move_all;
        bool move_3 = (move_cmd & FW3) || (move_cmd & REV3) || move_all;
        bool move_fw = (move_cmd & FW1) || (move_cmd & FW2) || (move_cmd & FW3) || (move_cmd & FWALL);
        bool move_rev = (move_cmd & REV1) || (move_cmd & REV2) || (move_cmd & REV3) || (move_cmd & REVALL);
        
        switch (state) {
            case Base:
            {
                // Do nothing if the user want to forward AND reverse at the same time. That way, we have the guarantee
                // that all the films will move in the same direction.
                if ((move_fw || move_rev) && !(move_fw & move_rev)) {
                    // activate the clutches of the parts we do not want to move
                    int8 clutches = 0;
                    if (!move_1)
                        clutches |= BLK1_CLUTCH_FORCE;
                    if (!move_2)
                        clutches |= BLK2_CLUTCH_FORCE;
                    if (!move_3)
                        clutches |= BLK3_CLUTCH_FORCE;
                    
                    // disconnect the control panel handle while the motor is on
                    clutches |= MAIN_CLUTCH_FORCE;

                    CLUTCHES_FORCE_Write(clutches);
                    
                    // activate the reverse relay if we want to roll back the film
                    MOTOR_REVERSE_Write(move_rev);
                    
                    state = Prepare_moving;
                }
            }
            break;
            case Prepare_moving:
            {
                uint8 em1 = EM_SW_1_Read();
                uint8 em2 = EM_SW_2_Read();
                uint8 em3 = EM_SW_3_Read();
                // we decided against moving in the meantime ? Let's revert !
                 if (!move_cmd) {
                    MOTOR_REVERSE_Write(0);
                    reset_clutches();
                    state = Base;
                    break;
                }
                // "block" (aka. do not change state) until the various electromagnet switches are all in a correct state
                // note to self: !EM_SW_x <=> Clutch x is on
                // so each switch is in a valid state if clutch on (!EM) & !move or if clutch off (EM) & move
                else if (!ECO_Read() && !(EM_SW_1_Read()^move_1) && !(EM_SW_2_Read()^move_2) && !(EM_SW_3_Read()^move_3)) {
                    // activate the motor
                    MOTOR_ENABLE_Write(1);                    

                    state = Moving;
                    
                }
            }
            break;
            case Moving:
            {
                // we stopped moving or we decided to reverse
                if (!move_cmd || (!MOTOR_REVERSE_Read() && move_rev)) {
                    reset_motor();
                    state = Prepare_moving;   
                }
             }
            break;

        }
        
        // TODO: use a timer
        CyDelay((state != Base) ? 50 : 250);
    }

}