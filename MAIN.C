//#define DEBUG
//#define BOARDV1
//#define TEST

#define servo_pin PIN_B1  //Setting servo out pin to be hardware pin b1
#define brake_pin PIN_B4
#define ADC_DELAY delay_us(20)
#define Acaps_pin PIN_A1
#define Acaps_channel 1
#define Athrottle_pin PIN_A0  //Voltage goes from 1.5 (306)to 4.1(836)
#define Athrottle_Min 316
#define Athrottle_Max 860
#define Athrottle_3quarter 520
#define Athrottle_Full Athrottle_Max-Athrottle_Min
#define Athrottle_channel 0
#ifdef BOARDV1
#define Electric_Controller_SWITCH PIN_B0
#define Contactor_SWITCH PIN_B2
#ELSE
#define Contactor_SWITCH PIN_B5
#define Controller_Power_SWITCH PIN_B2
#define Electric_Controller_SWITCH PIN_B0 //this is the switch from acceleration/breaking with driving high as breaking
#define ALGORITHM_INPUT_SWITCH PIN_A2
#endif
#define A_CAPS_MAX 725//893
#define A_CAPS_MIN 400//335
#define A_CAPS_MID_LOW (A_CAPS_MIN + 100) //This is the low end of the middle ACaps range. The range in which discharge and charge are allowed. 
#define A_CAPS_MID_HIGH (A_CAPS_MAX - 100) //This is the low end of the middle ACaps range. The range in which discharge and charge are allowed. 
#define V_SPEED_REGEN_MIN 250 //Why is there a minimum speed? Because below this no regenerative action is possible with electric motor
#define INSUFFICIENT_BRAKING_RUNNAWAY_ERROR 50000
//PID Values
#define K_P 1.50
#define K_I 0.00
#define K_D 0.20
struct PID_DATA pidData;
#define TIME_INTERVAL 157 //TODO replace
#define left_position 2500//4450
#define right_position 4600
#define servo_difference  right_position-left_position
#define servo_difference_div 5200
#define ELEC_CONTROLLER_OFFSET 900
//const FLOAT Athrottle_servo_factor = ((float) servo_difference)/((float) Athrottle_FULL);
#define servo_period   65356-50000


unsigned INT16 number_of_timer0_interupts_since_reset =0;
unsigned INT16 vSpeed= 0;
signed INT16 ELECthrottle = 0;
unsigned INT16 ICEthrottle = 0;
unsigned INT16 Athrottle = 0;
unsigned INT16 Acaps = 0;
int1 ICE_ON = 0;
int1 CURRENTLY_CHARGING = 0;
int1 RUNNAWAY_CHECK = 0;
signed INT16 returnedValue =0;
enum
{
   EVERYTHING_OFF,
   SPEED_TO_LOW_ICE_DIRECT,
   CHARGING_ALLOWED,
   DISCHARGING_ALLOWED,
   CHARGING_AND_DISCHARING_ALLOWED,
   INSUFFICIENT_BRAKING_RUNAWAY,
   USER_INPUT_OFF
} CHARGING_STATE;

/*
The #INT_timer0 interupt is triggered on each timer0 8bit interupt
the function simply increments a overflow counter to be used by the ccp2
interupt to calculate speed
NUMBER OF OPERATIONS = 
*/
#INT_timer0
void timer0_isr()
{
   
   number_of_timer0_interupts_since_reset = number_of_timer0_interupts_since_reset + 256;
   //current_servo_position = current_servo_position + 1;
   
   
   IF (number_of_timer0_interupts_since_reset >= 1024)
   {
      vSpeed = 0;
      number_of_timer0_interupts_since_reset = 1024;
      
   }
}

/*
#INT_timer1 is used by the ICEservo throttle to regulate the timing pulses. The 
timer is setup FOR a pulsetrain of 20ms period. This is done as follows,
Period is 1/(CLOCK/4opsperclock)*(startingPostionOfClock) = 20 ms
for this setup 1/(20000000/4)*(50000) = 20 ms
The timer1 starts at 0 and counts up. So we set the beggining of the clock at
servo_period which is 65356-50000 so that the total time is 20ms.
*/
#INT_timer1
void isr()
{
   //Make sure that the position is within the left and right positions of the servo
   IF (current_servo_position < left_position)
   {
      current_servo_position = left_position;
   }

   else IF (current_servo_position > right_position)
   {
      current_servo_position = right_position;
   }

   //printf("Current servo position %ld",current_servo_position);
   IF (SERVO_PIN_TO_BE_SET_HIGH_ON_NEXT_TIMER)
   {
      output_high (servo_pin);  //Set the servo control pin to high
      SERVO_PIN_TO_BE_SET_HIGH_ON_NEXT_TIMER = 0;
      set_timer1 (65356 - current_servo_position); //Set timer FOR the position high pulse
   }

   ELSE
   {
      output_low (servo_pin);     // Set the servo control pin to low
      SERVO_PIN_TO_BE_SET_HIGH_ON_NEXT_TIMER = 1;
      set_timer1 (servo_period + current_servo_position);  //Set timer FOR the low position the length is the difference between
      //the total INT16 lenght - high pulse length
   }
}

/*
#INT_ccp2 is called on the falling edge of the encoder pulse. We calculate the time
between pulses. 
TODO will have to put some kind of smoothing mechanism
*/
#ifdef BOARDV1
#INT_ccp2
#ELSE
#INT_ccp1
#endif
void isr2()
{
   IF (number_of_timer0_interupts_since_reset > 10)
   {
      vSpeed  = 1280 - (number_of_timer0_interupts_since_reset+get_timer0());
      //vSpeed  = -(get_timer0());
      set_timer0(0);
      number_of_timer0_interupts_since_reset = 0;
   }
}

void trickBreaking()
{
   write_dac((UNSIGNED int16) 400+ELEC_CONTROLLER_OFFSET);
   delay_ms(500);
}

void printfLogf(CHAR string)
{
   #ifdef DEBUG
   printf ("IN PrintfLog") ;
   printf (" %c", string) ;
   #ELSE
   //delay_ms (250) ;
   #endif
}

#ifdef TEST
/*
TEST SUITE
*/
void createHeartbeat()
{
   //This creates a heartbeat on pin B1
   WHILE (1)
   {
      output_high (servo_pin);  //Set the servo control pin to high
      delay_ms (1000) ;
      output_low (servo_pin) ;
      delay_ms (1000) ;
   }
}

void wiperAnalogVoltage()
{
   //SIGNED int16 test = 0;
   UNSIGNED int16 wiperValue = 0;
   WHILE (1)
   {
      printf("Wiper Value is now %ld",wiperValue);
      
      IF (wiperValue > (4095))
      {
         wiperValue = 0;
         printf ("Wiper Value is now % ld", wiperValue);
      }

      wiperValue = wiperValue + 10;
      write_dac (wiperValue) ;
      delay_ms (10) ;
   }
}

void heartbeatElectricControllerPower()
{
   WHILE (1)
   {
      output_high (Contactor_SWITCH);  //Set the servo control pin to high
      delay_ms (1000) ;
      output_low (Contactor_SWITCH) ;
      delay_ms (1000) ;
   }
}

void wiperServo()
{
   UNSIGNED int32 wiperValue = 0;
   current_servo_position = left_position;
   WHILE (1)
   {
      printf ("Servo Value % ld", current_servo_position);
      IF ((current_servo_position + 10) > right_position)
      {
         current_servo_position = left_position; //servo_difference;
      }

      current_servo_position = current_servo_position + 10;
      //delay_ms (1) ;
   }
}

void printAnalogThrottleInput()
{
   set_adc_channel (Athrottle_channel) ;
   ADC_DELAY;
   Athrottle = read_adc ();
   printf ("Analog Throttle is % ld", Athrottle);
}

void main()
{
   setup_adc_ports (NO_ANALOGS) ;
   setup_adc (ADC_OFF) ;
   setup_spi (FALSE) ;
   setup_counters (RTCC_INTERNAL, RTCC_DIV_2) ;
   setup_timer_1 (T1_DISABLED) ;
   setup_timer_2 (T2_DISABLED, 0, 1) ;
   setup_port_a (ALL_ANALOG) ;
   setup_adc (ADC_CLOCK_INTERNAL) ;
   init_dac () ;
   setup_timer_1 (T1_DIV_BY_2|T1_INTERNAL) ;
   setup_timer_0 (RTCC_INTERNAL|RTCC_DIV_128) ;
   // Configure CCP2 to capture fall
   #ifdef BOARDV1
   setup_ccp2 (CCP_CAPTURE_RE) ;
   enable_interrupts (INT_CCP2); // Setup interrupt on falling edge
   #ELSE
   setup_ccp1 (CCP_CAPTURE_RE) ;
   enable_interrupts (INT_CCP1) ;
   #endif
   enable_interrupts (INT_TIMER0) ;
   enable_interrupts (INT_TIMER1); // Setup interrupt on falling edge
   enable_interrupts (GLOBAL) ;
   CHARGING_STATE = EVERYTHING_OFF;
   output_high (Electric_Controller_SWITCH) ;
   output_high (Controller_Power_SWITCH) ;
   write_dac (0) ;
   ICE_ON = TRUE;
   output_high (Contactor_SWITCH) ;
   output_high (brake_pin) ;
   pid_Init (K_P * SCALING_FACTOR, K_I * SCALING_FACTOR, K_D * SCALING_FACTOR,&pidData) ;
   delay_ms (3000) ;
   current_servo_position = right_position - 1000;
   delay_ms (3000) ;
   current_servo_position = right_position;
   //write_dac (1000) ;
   //delay_ms (10000) ;
   //output_high (Contactor_SWITCH) ;
   //output_high (brake_pin) ;
   //output_high (Electric_Controller_SWITCH) ;
   WHILE (TRUE)
   {
      
      //GET INPUTS
      //Vspeedhappens in interrupts
      //set_adc_channel (Acaps_channel) ;
      //ADC_DELAY;
      //Acaps = read_adc ();
      
      //set_adc_channel (Athrottle_channel) ;
      //ADC_DELAY;
      //Athrottle = read_adc ();
      //current_servo_position = right_position - (Athrottle - Athrottle_Min) * 4;
      //#ifdef DEBUG
      printf ("State: Weak HybridTEZT \n");
      // #ELSE
      // delay_ms (250);
      /// #endif
      //IF ((Athrottle > Athrottle_3quarter)&& (Acaps > A_CAPS_MIN))
      {
         //CURRENTLY_CHARGING = 1;
         
         output_low (Electric_Controller_SWITCH) ;
         write_dac (0) ;
         output_low (brake_pin) ;
         //set electric motor to drive
         //#ifdef DEBUG
         // printf ("drive \n");
         //#ELSE
         // delay_ms (250) ;
         //#endif
         
         
         //
      }
   }
}

#ELSE
void main()
{
   
   
   setup_adc_ports (NO_ANALOGS) ;
   setup_adc (ADC_OFF) ;
   setup_spi (FALSE) ;
   
   setup_counters (RTCC_INTERNAL, RTCC_DIV_2) ;
   setup_timer_1 (T1_DISABLED) ;
   setup_timer_2 (T2_DISABLED, 0, 1) ;
   setup_port_a (AN0_AN1_AN3) ;
   setup_adc (ADC_CLOCK_INTERNAL) ;
   
   init_dac () ;
   setup_timer_1 (T1_DIV_BY_2|T1_INTERNAL);
   setup_timer_0 (RTCC_INTERNAL|RTCC_DIV_128) ;
   #ifdef BOARDV1
   setup_ccp2 (CCP_CAPTURE_RE) ;
   enable_interrupts (INT_CCP2); // Setup interrupt on falling edge
   #ELSE
   setup_ccp1 (CCP_CAPTURE_RE) ;
   enable_interrupts (INT_CCP1) ;
   #endif
   enable_interrupts (INT_TIMER0) ;
   enable_interrupts (INT_TIMER1); // Setup interrupt on falling edge
   enable_interrupts (GLOBAL) ;
   
   
   
   
   CHARGING_STATE = EVERYTHING_OFF;
   output_high (Electric_Controller_SWITCH) ;
   output_high (Controller_Power_SWITCH) ;
   write_dac (0) ;
   ICE_ON = TRUE;
   output_high (Contactor_SWITCH) ;
   output_high (brake_pin) ;
   pid_Init (K_P * SCALING_FACTOR, K_I * SCALING_FACTOR, K_D * SCALING_FACTOR,&pidData) ;
   delay_ms (3000) ;
   current_servo_position = right_position - 1000;
   delay_ms (3000) ;
   current_servo_position = right_position;
   //write_dac (1000) ;
   //delay_ms (10000) ;
   //output_high (Contactor_SWITCH) ;
   //output_high (brake_pin) ;
   //output_high (Electric_Controller_SWITCH) ;
   WHILE (TRUE)
   {
      
      //GET INPUTS
      //Vspeedhappens in interrupts
      set_adc_channel (Acaps_channel) ;
      ADC_DELAY;
      Acaps = read_adc ();
      
      set_adc_channel (Athrottle_channel) ;
      ADC_DELAY;
      Athrottle = read_adc ();
      if (!input (ALGORITHM_INPUT_SWITCH))
      {
         //delay_ms (100) ;
         current_servo_position = right_position - (Athrottle - Athrottle_Min) * 4;
         #ifdef DEBUG
         printf ("State: Weak Hybrid %ld\n", vSpeed);
         #ELSE
         delay_ms (250) ;
         #endif
         IF (Acaps > A_CAPS_MAX&& (vSpeed > V_SPEED_REGEN_MIN))
         {
            output_low (Electric_Controller_SWITCH) ;
            write_dac (0) ;
            output_low (brake_pin) ;
            //Voltage is high and we are going fast
            #ifdef DEBUG
            printf ("electric system voltage is getting high \n");
            #ELSE
            delay_ms (250) ;
            #endif
            }else IF ((Athrottle > Athrottle_3quarter)&& (Acaps > A_CAPS_MIN)){
            //CURRENTLY_CHARGING = 1;
            output_low (Electric_Controller_SWITCH) ;
            write_dac (2000) ;
            output_low (brake_pin) ;
            //set electric motor to drive
            #ifdef DEBUG
            printf ("drivex \n");
            #ELSE
            delay_ms (250) ;
            #endif
            
            
            }else IF ((Acaps < A_CAPS_MAX)&& (vSpeed > V_SPEED_REGEN_MIN)){
            // set electric motor to charge
            #ifdef DEBUG
            printf ("breakingy \n");
            #ELSE
            delay_ms (250) ;
            #endif
            IF (CURRENTLY_CHARGING == 1)
            {
               //  trickBreaking ();
            }

            CURRENTLY_CHARGING = 0;
            
            output_high (Electric_Controller_SWITCH) ;
            write_dac (1200 + ELEC_CONTROLLER_OFFSET) ;
            output_high (brake_pin) ;
            }ELSE {
            //set electric motor to zero
            #ifdef DEBUG
            printf ("turn off motor \n");
            #ELSE
            delay_ms (250) ;
            #endif
            CURRENTLY_CHARGING = 1;
            write_dac (0) ;
         }

         }ELSE{
         //Now Algorithm 2
         IF (Athrottle < Athrottle_Min)
         {
            Athrottle = Athrottle_Min;
         }

         
         returnedValue = pid_Controller ( (Athrottle - AThrottle_Min), vSpeed,&pidData);
        
         ELECthrottle = ELECthrottle + returnedValue;
         //printf ("Throttle % ld and electhrottle % ld \n", Athrottle, ELECthrottle);
         IF (Athrottle == Athrottle_Min)
         {
            ELECthrottle = 0;
         } 
         
         IF (ELECthrottle > 2500)
         {
            ELECthrottle = 2500;
         }

         else IF (ELECthrottle < - 2500)
         {
            ELECthrottle = -2500;
         }

         
         //delay_ms (100) ;
         current_servo_position = right_position - (Athrottle - Athrottle_Min) * 4;
         #ifdef DEBUG
         printf ("Weak Hybrid with PID %ld\n", returnedValue);
         #ELSE
         delay_ms (250) ;
         #endif
         IF (Acaps > A_CAPS_MAX&& (vSpeed > V_SPEED_REGEN_MIN))
         {
            output_low (Electric_Controller_SWITCH) ;
            write_dac (0) ;
            output_low (brake_pin) ;
            //Voltage is high and we are going fast
            #ifdef DEBUG
            printf ("electric system voltage is getting high \n");
            #ELSE
            delay_ms (250) ;
            #endif
         }ELSE{
            IF ((ELECthrottle > 0)&& (Acaps > A_CAPS_MIN))
            {
               output_low (Electric_Controller_SWITCH) ;
               write_dac (abs (ELECthrottle) + ELEC_CONTROLLER_OFFSET);
               output_low (brake_pin) ;
               //set electric motor to drive
               #ifdef DEBUG
               printf ("drivex \n");
               #ELSE
               delay_ms (250) ;
               #endif
            }else IF ((ELECthrottle < 0)&& (Acaps < A_CAPS_MAX)){
               // set electric motor to charge
               #ifdef DEBUG
               printf ("breakingy \n");
               #ELSE
               delay_ms (250) ;
               #endif
               output_high (Electric_Controller_SWITCH) ;
               write_dac (abs (ELECthrottle) + ELEC_CONTROLLER_OFFSET);
               output_high (brake_pin) ;
            }ELSE {
               //set electric motor to zero
               #ifdef DEBUG
               printf ("turn off motor \n");
               #ELSE
               delay_ms (250) ;
               #endif
               write_dac (0) ;
            }
         }
      }
   }
}

#endif
boolean checkRunnaway(STRUCT PID_DATA *pid)
{
   //RETURN (pid->sumError > INSUFFICIENT_BRAKING_RUNNAWAY_ERROR);
   RETURN FALSE;
}

/*! \brief Initialisation of PID controller parameters.
 *
 * Initialise the variables used by the PID algorithm.
 *
 * \param p_factor Proportional term.
 * \param i_factor Integral term.
 * \param d_factor Derivate term.
 * \param pid STRUCT with PID status.
 */
void pid_Init(int16 p_factor, int16 i_factor, int16 d_factor, STRUCT PID_DATA *pid)
// Set up PID controller parameters
{
   // Start values FOR PID controller
   pid -> sumError =  0;
   pid -> lastProcessValue =  0;
   // Tuning constants FOR PID loop
   pid -> P_Factor = p_factor;
   pid -> I_Factor = i_factor;
   pid -> D_Factor = d_factor;
   // Limits to avoid overflow
   pid -> maxError = MAX_INT / (pid ->P_Factor +  1);
   ////printf ("Max % ld factor % ld and pid % ld", MAX_INT, pid - > I_Factor,pid - > maxError);
   pid -> maxSumError = MAX_I_TERM / (pid ->I_Factor +  1);
}

/*! \brief PID control algorithm.
 *
 * Calculates output from setpoint, process value and PID status.
 *
 * \param setPoint Desired value.
 * \param processValue Measured value.
 * \param pid_st PID status STRUCT.
 */
int16 pid_Controller(int16 setPoint, int16 processValue, STRUCT PID_DATA *pid_st)
{
   SIGNED int16 error, p_term, d_term;
   SIGNED int32 i_term, ret, temp;
   processValue = processValue / 4;
   ////printf ("input % ld speed % ld ", setPoint, processValue);
   error = setPoint - processValue;
   
   // Calculate Pterm and limit error overflow
   
   if (error > (SIGNED int16) pid_st -> maxError)
   {
      p_term = MAX_INT;
      ////printf ("p greater error % ld a % ld p % ld", error, pid_st - > maxError,p_term);
   }

   else if (error < (SIGNED int16) - pid_st -> maxError)
   {
      p_term = -MAX_INT;
      ////printf ("p less error % ld a % ld p % ld", error, - pid_st- > maxError, p_term);
   }

   ELSE
   {
      p_term = (SIGNED int16) (pid_st -> P_Factor * (float) error);
      ////printf ("error % ld a % ld p % ld", error, pid_st - > maxError,p_term);
   }

   
   // Calculate Iterm and limit integral runaway
   temp = pid_st -> sumError + error;
   if (temp > (SIGNED int32) pid_st -> maxSumError)
   {
      i_term = MAX_I_TERM;
      pid_st -> sumError = pid_st -> maxSumError;
      ////printf ("\n greater temp % ld a % ld sum % ld", temp, pid_st - > maxSumError, pid_st -  > sumError);
   }

   else if (temp < (SIGNED int32) - pid_st -> maxSumError)
   {
      i_term = -MAX_I_TERM;
      pid_st -> sumError = -pid_st -> maxSumError;
      ////printf ("\n less temp % ld a % ld sum % ld", temp, pid_st - > maxSumError, pid_st -  > sumError);
   }

   ELSE
   {
      pid_st -> sumError = temp;
      i_term = pid_st -> I_Factor * pid_st -> sumError;
      ////printf ("\n eror temp % ld i_term % ld sum % ld error % ld ",temp, i_term, pid_st - > sumError, error);
   }

   // Calculate Dterm
   d_term = pid_st -> D_Factor * (pid_st-> lastProcessValue - processValue);
   ////printf("\n p_term %ld d_term %ld i_term %ld",p_term,d_term,i_term);
   pid_st -> lastProcessValue = processValue;
   //ret = (p_term + i_term + d_term) / SCALING_FACTOR;
   ret = (p_term + d_term + (SIGNED int16) i_term) / SCALING_FACTOR;
   if (ret > MAX_INT)
   {
      ret = MAX_INT;
   }

   else if (ret < -MAX_INT)
   {
      ret = -MAX_INT;
   }

   return ( (SIGNED int16) ret) ;
}

/*! \brief Resets the integrator.
 *
 * Calling this function will reset the integrator in the PID regulator.
 */
void pid_Reset_Integrator(pidData_t *pid_st)
{
   pid_st -> sumError =  0;
}


