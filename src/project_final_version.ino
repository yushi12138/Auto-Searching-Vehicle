//add detect_hitting() function in update_m_t_table(),which means detect hitting every listen(),and add judgement in rotate_sample,main loop,go_straight
#include "arduinoFFT.h"

// Common Variables
#define NUM_OF_BEACONS 10               //last one is 9500
#define FINAL_BEACON_INDEX 9
#define DISTRIBUTION_INDEX 9
#define FORWARD_DUTY_CYCLE 2500
#define BACKWARD_DUTY_CYCLE 2000
#define near_mag_threshold 60          // threshold magnitude for FFT (far/near)
#define far_mag_threshold 35

#define DISTANCE_AVOID_THRESHOLD 15
#define DISTANCE_FINAL_THRESHOLD 8
#define DISTANCE_HIT_THRESHOLD 12
#define MODIFY_DIR_DELAY_TIME 270
// Create FFT object
arduinoFFT FFT = arduinoFFT();

// Values for FFT
#define CHANNEL A0
const uint16_t samples = 128;           // Power of 2
double samplingFrequency = 32000;       // Hz
unsigned int sampling_period_us;
unsigned long microseconds;
// beacon_freq_index[ ] includes the index values for necessary frequencies
int beacon_freq_index[NUM_OF_BEACONS] = {20, 22, 24, 26, 28, 30, 32, 34, 36, 38};
//int smooth_rotation_delay[7] = {0, 600, 1150, 1700, 2080, 1500, 3240};
int smooth_rotation_delay[12] = {0, 600, 1150, 1700, 2080, 2500, 3240, 3750, 4200, 4750, 5090, 5500};
int mag_baseline[NUM_OF_BEACONS] = {100, 60, 50, 50, 50, 35, 35, 30, 30, 20};

double vReal[samples];
double vImag[samples];
//double vReal_former[NUM_OF_BEACONS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int t_table[NUM_OF_BEACONS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int m_table[NUM_OF_BEACONS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,};
int m_table_copy[NUM_OF_BEACONS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// If visited -> 1, otherwise 0
int v_table[NUM_OF_BEACONS - 1] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
int former_closest_beacon_index = 15; // invalid number
int closest_beacon_index = 0;

int counter = 0;
int counter_1 = 0;
int counter_2 = 0;
int counter_3 = 0;
int counter_4 = 0;
int counter_5 = 0;
int counter_6 = 0;
int counter_7 = 0;
int counter_8 = 0;
int counter_9 = 0;
unsigned long time;

// Used for Averaging FFT values
double average_mag = 0;

// LED Pin Number
int led = 13;

// Ultrasonic Sensor pin numbers
const int trigPin = 15;
const int echoPin = 16;
const int trigPin1 = 0;
const int echoPin1 = 1;//left
const int trigPin2 = 11;
const int echoPin2 = 12;

// Extra variables
long duration;
int distance;
long duration1;
int distance1;
long duration2;
int distance2;

//hit_warning flag,if magnitude is too large and the distance is also large
bool hit_warning = false;



void setup()
{
  //---------------------------------FFT Section------------------------------
  sampling_period_us = round(1000000 * (1.0 / samplingFrequency));
  Serial.begin(115200);

  //---------------------------------PWM Section------------------------------
  PORTD_PCR4 |= (1U << 10);
  PORTD_PCR7 |= (1U << 10);

  // 50% duty cycle ~ 1.6v with 8 BIT RESOLUTION
  // Bit 3 and bit 4 of status control, select system clock
  FTM0_SC |= _BV(3);
  FTM0_SC &= ~(_BV(4));

  //DISABLE WRITE PROTECTION
  FTM0_MODE |= (1U << 2);     // Mode register
  FTM0_MODE |= _BV(1);        // Enables FTM
  FTM0_SC |= 0x4;

  // Set MSB and ELSB, bit 5 and bit 3, which sets edge aligned PWM mode
  FTM0_C4SC |= (1U << 3) | (1U << 5);
  FTM0_C7SC |= (1U << 3) | (1U << 5);
  // Requirements for edge aligned up counter pwm is done,
  // MSnB 1 (Channel mode select B), QUADEN 0 (Quadrature Decoder),
  // DECAPEN 0 (Dual edge capture), COMBINE 0 (Pair dual channel),
  // CPWMS 0 (center aligned disabled and set as upcounting mode)

  FTM0_CNTIN = 0;         // Initial value is 0 for PWM counter
  FTM0_MOD = 29999;       // Counts up to MOD

  //---------------------------------LED Section------------------------------
  pinMode(led, OUTPUT);

  //---------------------------------USS Section------------------------------
  //---------------------------------USS Section------------------------------
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  pinMode(trigPin1, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin1, INPUT); // Sets the echoPin as an Input
  pinMode(trigPin2, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin2, INPUT); // Sets the echoPin as an Input
}

//---------------------------------Main Section---------------------------------
void loop() {
  hit_warning = false;
  //  delay(1000);
  listen(0);
  if (!hit_warning) {
    initialize_values();

    rotate_sample(0, 1, FORWARD_DUTY_CYCLE, 550);   // Adjusts the front wheel
    rotate_sample(1, 12, FORWARD_DUTY_CYCLE, 520);  // Rotate and sample
    print_results();
    print_counter();

    //stop_forever();
  }
  else go_backward();

  if (!hit_warning) {
    int found_final = find_target_beacon();
    if (!found_final) visit_closest_unvisited_beacon();
  }
}
void initialize_values() {
  counter_1 = 0;
  counter_2 = 0;
  counter_3 = 0;
  counter_4 = 0;
  counter_5 = 0;
  counter_6 = 0;
  counter_7 = 0;
  counter_8 = 0;
  counter_9 = 0;
  // Refresh the table after each loop
  for (int i = 0; i < NUM_OF_BEACONS; i++) {
    t_table[i] = 0;
    m_table[i] = 0;
    //    Serial.println(m_table[i]);
    //    Serial.println(t_table[i]);
  }
  // Refresh the average_mag
  average_mag = 0;


}
//-------------------------------Find Target Beacon-----------------------------
// Check if you found the final beacon frequency
int find_target_beacon() {
  detect_distance();

  if (m_table[FINAL_BEACON_INDEX] < far_mag_threshold) {
    return 0;
  }
  else if ((m_table[FINAL_BEACON_INDEX] >= near_mag_threshold) && (distance < DISTANCE_FINAL_THRESHOLD)) stop_forever();
  else if (m_table[FINAL_BEACON_INDEX] >= far_mag_threshold) {
    // Rotate in direct of final beacon
    light_led(3);
    rotate_sample(0, t_table[FINAL_BEACON_INDEX], FORWARD_DUTY_CYCLE, 550);
    modify_direction(2500, 2250, MODIFY_DIR_DELAY_TIME);
    delay(300);
    go_straight(true, 2);
    return 1;
  }
}


//-------------------------------find_closest_unvisited_twice_beacon_------------------
void visit_closest_unvisited_beacon() {

  find_closest_beacon_index_roughly();
  bool can_hear = whether_can_hear(0);
  if (can_hear) {
    light_led(1);
    rotate_sample(0, t_table[closest_beacon_index], FORWARD_DUTY_CYCLE, 550); //just rotate to certain position don't sample
    modify_direction(2500, 2250, MODIFY_DIR_DELAY_TIME); //turn the front wheel to straight direction
    //    delay(100);
    go_straight(false, 4); // breaks once beacon is too close
  }
  else {
    find_closest_beacon_index_carefully();
    bool can_hear2 = true; //whether_can_hear(1);
    if ( can_hear2) {
      light_led(2);
      rotate_sample(0, t_table[closest_beacon_index], FORWARD_DUTY_CYCLE, 550); //just rotate to certain position don't sample
      modify_direction(2500, 2250, MODIFY_DIR_DELAY_TIME); //turn the front wheel to straight direction
      //      delay(100);
      go_straight(false, 4
                 ); // breaks once beacon is too close
    }
    //else initialize_v_table();
  }
}
//--------------------------------whether_can_hear------------------------------------------------------------------
bool whether_can_hear(bool find_mode) {
  //initialize v_table, if we can't hear every unvisited beacon,magnitude of every freq is less than the magnitude in silence
  //so that we can avoid the situation that when we can
  bool can_hear_flag = false;
  //find_mode =0 is roughly,1 is carefully
  if (find_mode) {
    for (int i = 0; i < NUM_OF_BEACONS - 1; i++)
    {
      if (m_table[i] > mag_baseline[i]) {
        can_hear_flag = true;
        break;
      }
    }
  }
  else {
    for (int i = 0; i < NUM_OF_BEACONS - 1; i++)
    {
      if (m_table_copy[i] > mag_baseline[i]) {
        can_hear_flag = true;
        break;
      }
    }
  }
  return can_hear_flag;
}
//-------------------------------initialize_v_table-------------------------------------------
void initialize_v_table() {
  for (int index = 0; index < NUM_OF_BEACONS - 1; index++) {
    v_table[index] = 0;
  }
}
//---------------------------Find Closest Beacon Index roughly--------------------------
// Find the index of closest beacon excluding the target
// note that v_table[index] is 1 if beacon was already visited
void find_closest_beacon_index_roughly() {
  closest_beacon_index = 0;
  for (int index = 0; index < NUM_OF_BEACONS; index++) {
    if (v_table[index] == 0 ) {
      m_table_copy[index] = m_table[index];
    }
  }
  for (int index = 0; index < NUM_OF_BEACONS - 1; index++) {
    if ( m_table_copy[closest_beacon_index] < m_table_copy[index]) {
      closest_beacon_index = index;
    }
  }
  v_table[closest_beacon_index] = 1;
  if (closest_beacon_index == former_closest_beacon_index) {
    v_table[closest_beacon_index] = 1;
    m_table_copy[closest_beacon_index] = 0;
    // Update closest_beacon_index for second run through
    for (int index = 0; index < NUM_OF_BEACONS - 1; index++) {

      if ( m_table_copy[closest_beacon_index] < m_table_copy[index]) {
        closest_beacon_index = index;
      }
    }
  }
  former_closest_beacon_index = closest_beacon_index;
}


//void find_closest_beacon_index_roughly() {
//  closest_beacon_index = 0;
//  for (int index = 0; index < NUM_OF_BEACONS - 1; index++) {
//    if (v_table[index] == 0 ) {
//      m_table_copy[index] = m_table[index];
//    }
//  }
//    for (int index = 0; index < NUM_OF_BEACONS - 1; index++) {
//      if (m_table_copy[closest_beacon_index] < m_table_copy[index]) {
//        closest_beacon_index = index;
//      }
//    }
//    //don't care about the m_table_copy[10], which is the target,since we don't use this copy to find target beacon
//    v_table[closest_beacon_index] = 1;
//    //  m_table_copy[closest_beacon_index] = 0;
//
//}
//---------------------------Find Closest Beacon Index refinedly--------------------------
// Find the index of closest beacon excluding the target
// note that v_table[index] is 1 if beacon was already visited
void find_closest_beacon_index_carefully() {
  closest_beacon_index = 0;
  for (int index = 0; index < NUM_OF_BEACONS - 1; index++) {
    if (v_table[index] == 2) {
      m_table[index] = 0;
    }
  }
  for (int index = 0; index < NUM_OF_BEACONS - 1; index++) {
    if (m_table[closest_beacon_index] < m_table[index]) {
      closest_beacon_index = index;
    }
  }
}


//-------------------------------Go Straight------------------------------------
// when it go 1000ms, it'll stop and detect wether it's close to the beacon().
// this is used both in find_target_beacon and find_closest_beacon
void go_straight(bool visiting_final, int num_go_straight_step) {
  int num_turns_degree = 4;
  for (uint16_t i = 0; i < num_go_straight_step; i++) {
    listen(0);
    //    find_closest_beacon_index();
    if (!hit_warning)
    {
      detect_distance();
      detect_hitting_distance();
      if (m_table[beacon_freq_index[FINAL_BEACON_INDEX]] > 45)
      {
        break;
      }
      else if ((visiting_final) && ((distance <= DISTANCE_AVOID_THRESHOLD) || (distance1 <= DISTANCE_AVOID_THRESHOLD) || (distance1 <= DISTANCE_AVOID_THRESHOLD))) {
        for (int i = 0; i < num_turns_degree; i++) {
          detect_distance();
          if (distance < DISTANCE_FINAL_THRESHOLD) break;
          FTM0_C7V = 2500;
          FTM0_C4V = 2500;
          delay(100);
        }
        stop_forever();
      }
      else if (distance <= DISTANCE_AVOID_THRESHOLD) {
        v_table[closest_beacon_index] = 2;        //only when it avoid the beacon v_table will be set to 2, only passing by will be set to 1
        avoid_beacon();
        break;
      }

      FTM0_C7V = 2500;
      FTM0_C4V = 2500;
      delay(150); // 500, wood board,90 degree
    }
    else if (distance1 <= DISTANCE_AVOID_THRESHOLD) {
      FTM0_C7V = 2250;
      FTM0_C4V = 2500;
      delay(500);
    }
    else if (distance2 <= DISTANCE_AVOID_THRESHOLD) {
      FTM0_C7V = 2500;
      FTM0_C4V = 2250;
      delay(500);
    }
  }
  FTM0_C7V = 2250;
  FTM0_C4V = 2250;
}

//----------------------Rotate & sampling serveral times------------------------
void rotate_sample(bool sampling, int num_turns, int duty_cycle, int delay_time) {
  if (!sampling) {
    for (uint16_t turn_num = 0; turn_num < num_turns; turn_num++) {
      rotate_with_stop(duty_cycle, delay_time);
    }
    return;
  }
  for (uint16_t turn_num = 0; turn_num < num_turns; turn_num++) {
    listen(turn_num);
    if (hit_warning) {
      go_backward();
      return;
    }
    rotate_with_stop(duty_cycle, delay_time);
  }
}

//--------------------------------Rotate Once-----------------------------------
void rotate_with_stop(int duty_cycle, int delay_time) {
  FTM0_C7V = duty_cycle;
  FTM0_C4V = 2250;
  delay(delay_time);//500, wood board,90 degree
  FTM0_C7V = 2250;
  FTM0_C4V = 2250;
  delay(100);
}

//void rotate_without_stop(int duty_cycle, int num_turns) {
//  int left_duty_cycle;
//  int right_duty_cycle;
//  if (num_turns > 6) {
//    num_turns = 12 - num_turns;
//    left_duty_cycle = 2500;
//    right_duty_cycle = 2250;
//  } else {
//    left_duty_cycle = 2250;
//    right_duty_cycle = 2500;
//  }
//  int turn_delay = smooth_rotation_delay[num_turns];
//  if (turn_delay) {
//    FTM0_C4V = left_duty_cycle;
//    FTM0_C7V = right_duty_cycle;
//  }
//  delay(turn_delay);//500, wood board,90 degree
//  FTM0_C7V = 2250;
//  FTM0_C4V = 2250;
//  delay(200);
//}
void rotate_without_stop(int duty_cycle, int num_turns) {
  int turn_delay = smooth_rotation_delay[num_turns];
  if (turn_delay) {
    FTM0_C7V = duty_cycle;
    FTM0_C4V = 2250;
  }
  delay(turn_delay);//500, wood board,90 degree
  FTM0_C7V = 2250;
  FTM0_C4V = 2250;
  delay(100);
}

//--------------------------------Stop Forever----------------------------------
void stop_forever() {
  FTM0_C7V = 2250;
  FTM0_C4V = 2250;
  while (1) {
    light_led(1);
  }

}
//---------------------------------light_led-----------------------------------
void light_led(int num) {
  for (int i = 0; i < num; i++) {
    digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(300);               // wait for a second
    digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
    delay(300);
  }



}
//---------------------------------Avoid Beacon---------------------------------
void avoid_beacon() {
  // find_closest_beacon_with_ultra();
  while (distance > 20) {
    detect_distance();
    FTM0_C7V = 2500;
    FTM0_C4V = 2500;
    delay(100);
  }
  // modify direction
  FTM0_C7V = 2500;
  FTM0_C4V = 2250;
  delay(900);                 // turn about 45 degrees

  //  FTM0_C7V = 2500;
  //  FTM0_C4V = 2500;
  //  delay(150);                 // Go straight

  FTM0_C7V = 2450;
  FTM0_C4V = 2520;
  delay(5000);                // turnning around to avoid the beacon
  FTM0_C7V = 2250;
  FTM0_C4V = 2250;
  delay(500);
  FTM0_C7V = 2250;
  FTM0_C4V = 2000;
  delay(1500);                // modify direction to straight
  FTM0_C7V = 2250;
  FTM0_C4V = 2250;
  delay(500);
  detect_distance();
  if (distance > 30)
  {
    FTM0_C7V = 2500;
    FTM0_C4V = 2500;
    delay(500);                 // go straight
  }
  FTM0_C7V = 2500;
  FTM0_C4V = 2500;
  delay(500);
  FTM0_C7V = 2250;
  FTM0_C4V = 2250;
  delay(200);
}

//-------------------------Use Ultrasonic Seneor to find Beacon-----------------
void find_closest_beacon_with_ultra(int distance_threshold) {
  int num_turns_degree = 24;
  for (int i = 0; i < num_turns_degree; i++) {
    detect_distance();
    if (distance < distance_threshold) break;
    rotate_with_stop(FORWARD_DUTY_CYCLE, 270);
  }
  modify_direction(2500, 2250, MODIFY_DIR_DELAY_TIME);
}
//----------------------------Modify Direction----------------------------------
// Modify the direction of front wheel
void modify_direction(int left_duty_cycle, int right_duty_cycle, int delay_time)
{
  FTM0_C7V = right_duty_cycle;
  FTM0_C4V = left_duty_cycle;
  delay(delay_time);
  FTM0_C7V = 2250;
  FTM0_C4V = 2250;
}
//-------------------------------go_backward-----------------------------------
void go_backward()
{
  FTM0_C7V = 2000;
  FTM0_C4V = 2000;
  delay(800);
  FTM0_C7V = 2250;
  FTM0_C4V = 2250;
  delay(200);
}


//--------------------------------listen----------------------------------------
void listen(int num_turns) {
  for (int i = 0; i < 25; i++) {      //change this to 30, in case the buzzer is beeping to slow, we need to make sure of the interval
    call_fft();
    while (vReal[(samples >> 1) - 1] > 10) {
      call_fft();
    }
    average_mag += vReal[beacon_freq_index[DISTRIBUTION_INDEX]];
    delay(25);
    update_m_t_table(num_turns);
  }
  detect_hitting();     //detect hitting every listen
}

//---------------------------update_beacon_freqs_magnitude----------------------
void update_m_t_table(int num_turns) {
  for (int index = 0; index < NUM_OF_BEACONS; index++) {
    if (m_table[index] <  vReal[beacon_freq_index[index]]) {
      m_table[index] = vReal[beacon_freq_index[index]];
      t_table[index] = num_turns;
    }
  }

  for (int index = 0; index < 5; index++) {
    Serial.println(vReal[beacon_freq_index[index]]);
    Serial.println(m_table[index]);
    Serial.println(t_table[index]);
  }
  Serial.println(" ");
  print_fft();
  Serial.println(" ");

}
//--------------------------Sampling and FFT------------------------------------

void call_fft(void) {
  /*SAMPLING*/
  for (int i = 0; i < samples; i++) {
    microseconds = micros();
    vReal[i] = analogRead(CHANNEL);
    vImag[i] = 0;
    while (micros() < (microseconds + sampling_period_us)) {  }
  }
  /*FFT*/
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, samples);

}

void print_fft(void) {
  int scale = (samples >> 1);//length of the frequency table
  double f_value[scale];    //frequency value
  uint16_t i;
  Serial.println("frequency  magnitude");
  for (i = 0; i < scale ; i++) {
    f_value[i] = ((i * 1.0 * samplingFrequency) / samples);
    Serial.print(f_value[i] , 6);
    Serial.print("Hz");
    Serial.print(" ");
    Serial.print(vReal[i], 4);
    Serial.println(" ");

  }
  if (vReal[beacon_freq_index[DISTRIBUTION_INDEX]] > 2000) counter_9++;
  else if (vReal[beacon_freq_index[DISTRIBUTION_INDEX]] > 1500) counter_8++;
  else if (vReal[beacon_freq_index[DISTRIBUTION_INDEX]] > 1000) counter_7++;
  else if (vReal[beacon_freq_index[DISTRIBUTION_INDEX]] > 500) counter_6++;
  else if (vReal[beacon_freq_index[DISTRIBUTION_INDEX]] > 400) counter_5++;
  else if (vReal[beacon_freq_index[DISTRIBUTION_INDEX]] > 300) counter_4++;
  else if (vReal[beacon_freq_index[DISTRIBUTION_INDEX]] > 200) counter_3++;
  else if (vReal[beacon_freq_index[DISTRIBUTION_INDEX]] > 100) counter_2++;
  else counter_1++;
}

void print_results() {
  Serial.println("one loop magnitude table");
  Serial.println("--------------------------");
  for (int i = 0; i < NUM_OF_BEACONS; i++) {
    Serial.println(m_table[i]);
    Serial.println(t_table[i]);
  }
}

//-----------------------------print_counter------------------------------------
void print_counter() {
  Serial.println("counter:");
  Serial.println(counter_1);
  Serial.println(counter_2);
  Serial.println(counter_3);
  Serial.println(counter_4);
  Serial.println(counter_5);
  Serial.println(counter_6);
  Serial.println(counter_7);
  Serial.println(counter_8);
  Serial.println(counter_9);
  Serial.println("the above is well printed data");
  Serial.println(" ");
  double n = counter_1 + counter_2 + counter_3 + counter_4 + counter_5
             + counter_6 + counter_7 + counter_8 + counter_9;
  Serial.println("total times of fft");
  Serial.println(n);
  Serial.println("average_mag");
  Serial.println(average_mag / n);
  Serial.println(" ");
}

//--------------------------ultrasonic------------------------
void detect_distance() {
  int num_tests = 6;
  int min_dist = 5000;
  for (int index = 0; index < num_tests; index++) {
    // Clears the trigPin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    distance = duration * 0.034 / 2;
    if (distance < min_dist) min_dist = distance;
  }
  distance = min_dist;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);
}


void detect_hitting_distance() {
  int num_tests = 3;
  int min_dist_1 = 5000;
  int min_dist_2 = 5000;
  for (int index = 0; index < num_tests; index++) {
    //----------------------------1-----------------------------------
    // Clears the trigPin
    digitalWrite(trigPin1, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin1, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin1, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration1 = pulseIn(echoPin1, HIGH);
    // Calculating the distance
    distance1 = duration1 * 0.034 / 2;
    if (distance1 < min_dist_1) min_dist_1 = distance1;

    //-----------------------------2-----------------------------------
    // Clears the trigPin
    digitalWrite(trigPin2, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin2, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin2, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration2 = pulseIn(echoPin2, HIGH);
    // Calculating the distance
    distance2 = duration2 * 0.034 / 2;
    // Prints the distance on the Serial Monitor
    if (distance2 < min_dist_2) min_dist_2 = distance2;
  }
  distance1 = min_dist_1;
  distance2 = min_dist_2;
  Serial.print("Distance1: ");
  Serial.println(distance1);
}
void detect_hitting()
{
  int local_distance = distance1;
  if (distance1 > distance2)
  {
    local_distance = distance2;
  }
  detect_hitting_distance();
  if (local_distance < DISTANCE_HIT_THRESHOLD) {
    hit_warning = true;
  }
}
