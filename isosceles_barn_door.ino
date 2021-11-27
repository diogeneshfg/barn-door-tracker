#include <FiniteStateMachine.h>
#include <AccelStepper.h>
#include <math.h>

//#define DEBUG

const int pinOutStep = 3;      // Arduino digital pin connected to Driver step
const int pinOutDirection = 2; // Arduino digital pin connected to Driver direction

#define PI 3.1415926

#define MS1 4                  // Microstepping mode pin 1
#define MS2 5                  // Microstepping mode pin 2
#define MS3 6                  // Microstepping mode pin 3

//CONSTANTS

const float SIDERAL_TIME_EARTH_REVOLUTION = 86164.1; //seconds
const float MAX_THETA = 30.0; //degrees

const float DISTANCE_ROD_TO_HINGE = 830.0; //milimeters
const float INITIAL_APERTURE = 27.0; //milimeters
const float STEPS_DEGREES = 1.8;
const float MICRO_STEPS_PER_STEPS = 16;
const float STEPS_PER_REVOLUTION = 360.0 / (STEPS_DEGREES / MICRO_STEPS_PER_STEPS);
const float THREAD_ROD_PITCH = 2; //milimeters


static State state_sideral = State(state_sideral_enter, state_sideral_run, state_sideral_exit);
static State state_off = State(state_off_enter, state_off_update, state_off_exit);
static FSM barndoor = FSM(state_off);


//GLOBAL VARIABLES
float sum_steps = 0.0;
float actual_theta = 0.0;
float total_time = (SIDERAL_TIME_EARTH_REVOLUTION/360.0) * MAX_THETA;
float real_theta = 0.0;

float initial_angle = 0.0;
float initial_time = 0.0;    
float distance_given = 0.0;    
float steps_given = 0.0;


static AccelStepper motor(AccelStepper::DRIVER,
                          pinOutStep,
                          pinOutDirection);


void setup()
{
    pinMode(MS1, OUTPUT);
    pinMode(MS2, OUTPUT);
    pinMode(MS3, OUTPUT);

    motor.setPinsInverted(true, false, false);
    motor.setMaxSpeed(3000);

    //Set motor microstepping to 1/16 half steps
    digitalWrite(MS1,HIGH);
    digitalWrite(MS2,HIGH);
    digitalWrite(MS3,HIGH); //LOW for 8 microstepping
    
#ifdef DEBUG
    Serial.begin(9600);      
#endif

    set_up_offset_tracker();
}


void loop() {  
  barndoor.transitionTo(state_sideral);
  barndoor.update();
}



float find_initial_angle(float aperture)
{
    if (aperture > 0)
    {
        float angle_rad = asin(aperture/DISTANCE_ROD_TO_HINGE);
        float angle = (angle_rad * 180.0 / PI);
        return angle;
    }
    else
    {
        return 0.0;
    }
}


float find_time_spent(float initial_angle)
{
    if (initial_angle > 0)
    {
        float time_spent = (SIDERAL_TIME_EARTH_REVOLUTION/360.0) * initial_angle;
        return time_spent;
    }
    else
    {
        return 0.0;
    }
}


float find_travel_distance(float time_delta)
{
    if (time_delta > 0)
    {
        float theta = real_theta - ((real_theta/total_time)*(total_time-time_delta));
        float theta_rad = radians(theta);
        return (sin(theta_rad) * DISTANCE_ROD_TO_HINGE);
    }
    else
    {
        return 0.0;
    }
}



float find_number_steps(float distance)
{
    if(distance > 0)
    {
        return (STEPS_PER_REVOLUTION * distance) / THREAD_ROD_PITCH;
    }
    else
    {
        return 0.0;
    }
}


void set_up_offset_tracker(void)
{
    initial_angle = find_initial_angle(INITIAL_APERTURE);
    real_theta = MAX_THETA - initial_angle;
    actual_theta = initial_angle;
    initial_time = find_time_spent(initial_angle);
    total_time = total_time - initial_time;
    distance_given = find_travel_distance(initial_time);
    steps_given = find_number_steps(distance_given);
    sum_steps = steps_given;
    
#ifdef DEBUG      
    Serial.print("\n");
    Serial.print("Real Theta: ");
    Serial.print(real_theta,8);
    Serial.print("\n");
    Serial.print("Total Time: ");
    Serial.print(total_time,8);
    Serial.print("\n");
    Serial.print("Sum Steps ");
    Serial.print(sum_steps,8);
    Serial.print("\n");
    Serial.print("Initial Time ");
    Serial.print(initial_time,8);
    Serial.print("\n");
    Serial.print("Distance Given ");
    Serial.print(distance_given,8);
    Serial.print("\n");
    Serial.print("Steps Given ");
    Serial.print(steps_given,8);
    Serial.print("\n");
#endif
}



void execute_dso_barn_door(void)
{      
    float start = millis()/1000;
    float end = 0;
    float time_delta = 0;
    float later_distance = 0;
    float next_distance = 0 ;
    
    while(actual_theta <= real_theta)
    {  
        end =  millis()/1000;
        time_delta = (end-start);
        
        next_distance = find_travel_distance(time_delta);
        float distance_delta = (next_distance - later_distance);
        later_distance = next_distance;
        
        float steps = round(find_number_steps(distance_delta));
        sum_steps += steps;
        
        actual_theta = asin(((sum_steps/STEPS_PER_REVOLUTION) * THREAD_ROD_PITCH)/DISTANCE_ROD_TO_HINGE);
        actual_theta = actual_theta * 180 / PI;
        float steps_to_perform = motor.currentPosition() + steps;        
        
        while(motor.currentPosition() < steps_to_perform)
        {    
            motor.setSpeed(steps);      
            motor.runSpeed();
        }

#ifdef DEBUG                
        Serial.print(round(time_delta));
        Serial.print(" - ");
        Serial.print(steps);
        Serial.print(" - ");
        Serial.print(actual_theta, 8);
        Serial.print("\n");
#endif
    }   
    motor.stop();    
}

############################## Finite State Machine Cases ######################################

void state_sideral_enter(void){
  
}


void state_sideral_run(void){
  execute_dso_barn_door();  
}


void state_sideral_exit(void){
  
}


void state_off_enter(void)
{
#ifdef DEBUG
  Serial.print("Enter off\n");
#endif
  motor.stop();
}

void state_off_update(void)
{
    
}

void state_off_exit(void)
{
    
}
