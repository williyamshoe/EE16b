/*
 * integration.ino
 * Final sketch for SIXT33N Speech version
 *
 * EE16B Fall 2016
 * Emily Naviasky & Nathaniel Mailoa
 *
 * EE 16B Fall 2017
 * Andrew Blatner
 *
 */

/********************************************************/
/********************************************************/
/***                                                  ***/
/*** Constants and global variables from turning.ino. ***/
/***                                                  ***/
/********************************************************/
/********************************************************/

#define LEFT_MOTOR                  P2_0
#define LEFT_ENCODER                P6_1
#define RIGHT_MOTOR                 P1_5
#define RIGHT_ENCODER               P6_2

#define SAMPLING_INTERVAL           100
int sample_lens[4] = {0};

// Operation modes
#define MODE_LISTEN                 0
#define MODE_DRIVE                  1
int timer_mode = MODE_LISTEN;

#define DRIVE_FAR                   0
#define DRIVE_LEFT                  1
#define DRIVE_CLOSE                 2
#define DRIVE_RIGHT                 3

#define JOLT_STEPS                  2

boolean loop_mode = MODE_DRIVE;
int drive_mode = 0;

int step_num = 0;
volatile boolean do_loop = 0; // timer signal to increment timestep

typedef struct encoder {
  int pin;
  int pos;
  bool level;
  int avg;
} encoder_t;

encoder_t left_encoder = {LEFT_ENCODER, 0, LOW, 0};
encoder_t right_encoder = {RIGHT_ENCODER, 0, LOW, 0};

/*---------------------------*/
/*      CODE BLOCK CON1      */
/*    From closed_loop.ino   */
/*---------------------------*/

float theta_left = 0.1696;
float theta_right = 0.1842;
float beta_left = -45.65;
float beta_right = -41.54;
float v_star = 80;

// PWM inputs to jolt the car straight
int left_jolt = 260;
int right_jolt = 220;

// Control gains
float k_left = 0.2;
float k_right = 0.6;

/*---------------------------*/
/*      CODE BLOCK CON2      */
/*    From closed_loop.ino   */
/*---------------------------*/

float driveStraight_left(float delta) {
  return ((v_star + beta_left) - k_left * delta) / theta_left;
}

float driveStraight_right(float delta) {
  return ((v_star + beta_right) + k_right * delta) / theta_right;
}

/*---------------------------*/
/*      CODE BLOCK CON3      */
/*    From closed_loop.ino   */
/*---------------------------*/

float delta_ss = -7;

/*---------------------------*/
/*      CODE BLOCK CON4      */
/*---------------------------*/

#define CAR_WIDTH                   15.0 // in cm
#define TURN_RADIUS                 55 // in cm - 6 feet diameter = 3 tiles in 125 Cory
// #define TURN_RADIUS                 60 // in cm - 4 feet diameter = 2 tiles in 125 Cory

int run_times[4] = {7000, 2000, 2500, 2000};

float delta_reference(int k) {
  // YOUR CODE HERE
  if (drive_mode == DRIVE_RIGHT) {
    return -v_star / 5 * CAR_WIDTH * k / (TURN_RADIUS*0.65);
  }
  else if (drive_mode == DRIVE_LEFT) {
    return v_star / 5 * CAR_WIDTH * k / (TURN_RADIUS*1.8);
  }
  else { // DRIVE_FAR, DRIVE_CLOSE
    return 0;
  }
}

/*---------------------------*/
/*      CODE BLOCK CON5      */
/*---------------------------*/
#define INFINITY                    (3.4e+38)
#define STRAIGHT_RADIUS             INFINITY

float straight_correction(int k) {
  // YOUR CODE HERE
  return v_star / 5 * CAR_WIDTH * k / 800;
}

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/


/*********************************************************/
/*********************************************************/
/***                                                   ***/
/*** Constants and glboal variables from classify.ino. ***/
/***                                                   ***/
/*********************************************************/
/*********************************************************/

#define MIC_INPUT                   P6_0

#define SIZE                        2752
#define SIZE_AFTER_FILTER           172
#define ADC_TIMER_MS                0.35

/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*---------------------------*/

// Enveloping and K-means constants
#define SNIPPET_SIZE                80
#define PRELENGTH                   5
#define THRESHOLD                   0.5

#define KMEANS_THRESHOLD            0.03
#define LOUDNESS_THRESHOLD         750

/*---------------------------*/
/*      CODE BLOCK PCA2      */
/*---------------------------*/

float pca_vec1[SNIPPET_SIZE] = {-0.0475349522072, -0.0580143946896, -0.0765720830431, -0.0991553748921, -0.0895674291906, -0.0757201910464, -0.0728944381235, -0.0696636821199, -0.0710135558278, -0.111270680262, -0.143360089829, -0.159860791724, -0.157679615176, -0.159194477641, -0.184775900338, -0.202161548401, -0.201348613908, -0.247910961249, -0.248580460783, -0.238421991164, -0.231301624103, -0.250313984859, -0.226740898168, -0.170758858126, -0.108481804559, -0.0598961948129, -0.0247845213927, 0.0294172624419, 0.0817984413314, 0.100396598996, 0.10904698347, 0.121073903219, 0.139585252617, 0.14068419592, 0.136392579856, 0.131825591521, 0.121677204386, 0.127783936226, 0.109434676231, 0.105442350643, 0.0865255503831, 0.0827526279673, 0.0915049283185, 0.081465549168, 0.092487606155, 0.0909422348476, 0.0839455453295, 0.0931912831344, 0.0910873278617, 0.0764375820148, 0.0783772072881, 0.0718928405069, 0.0858545922019, 0.0837570607711, 0.0800355369788, 0.0709279914497, 0.0762621371058, 0.0683746673937, 0.0600665055448, 0.0592356088933, 0.0497279433914, 0.0552522024752, 0.0490456554767, 0.0437735274214, 0.0418157688487, 0.0432891524364, 0.036364305332, 0.0372492640534, 0.0330858306374, 0.0347113665175, 0.0350830193719, 0.0352290814244, 0.0237033055908, 0.0281119539829, 0.0389892842085, 0.0416737021335, 0.0209168847089, 0.0179612982488, 0.0275556633492, 0.033756547854};
float pca_vec2[SNIPPET_SIZE] = {-0.00499303584192, 0.000926336792702, -0.00675296291521, 0.00640864292431, 0.0625493997215, 0.156714148074, 0.214012444136, 0.238317504184, 0.252819345219, 0.268335469642, 0.244582028332, 0.238507222836, 0.21164725471, 0.184558507206, 0.158012483902, 0.0975415163304, 0.0238526904775, -0.0598557533837, -0.11088629261, -0.138118016924, -0.174441554424, -0.169926747246, -0.187059315656, -0.199578203907, -0.177704504508, -0.139672548847, -0.091677779369, -0.0213912137057, 0.0696454727667, 0.0683687148937, 0.0634833540888, 0.114312891812, 0.142238942695, 0.14353572099, 0.135843842108, 0.0707431585667, 0.080887275859, 0.111884077537, 0.102255177322, 0.0945686400849, 0.0713811166873, 0.025265370406, 0.00229777959928, -0.0251620615419, -0.0483479590478, -0.0700249838136, -0.0674672476363, -0.088522954107, -0.0864169451136, -0.0798469531207, -0.07349154867, -0.0793728708356, -0.0941774609992, -0.0864142976791, -0.0805334885134, -0.0724704837684, -0.0779260787015, -0.0644228991069, -0.0635302315722, -0.0531390683106, -0.0554159450035, -0.0423025629192, -0.049640532187, -0.0482875346303, -0.0386985460045, -0.0462181677305, -0.0491307793467, -0.0491922148775, -0.0469143413895, -0.0573250210953, -0.0558668989167, -0.0544886897988, -0.0485617420129, -0.0479092204832, -0.0579528088438, -0.0555760653017, -0.0369191222142, -0.0342777414667, -0.0419332039947, -0.0455599298135};
float mean_vec[SNIPPET_SIZE] = {0.00653750880888, 0.00653973960219, 0.00759112835697, 0.0107947187377, 0.0138223551281, 0.0228155088924, 0.024776870117, 0.0263553515493, 0.0276508204942, 0.0284750495334, 0.0278522543587, 0.0284509198583, 0.0279991386573, 0.0277498494847, 0.026714450339, 0.0253859944255, 0.0229161683346, 0.0213003533373, 0.0200687156925, 0.0185608931469, 0.018008812263, 0.018751000912, 0.0183513455329, 0.0176035438999, 0.0165923982643, 0.0159522050971, 0.0162424230941, 0.0162935891422, 0.0173442196735, 0.017442351321, 0.0170421483078, 0.0173027620692, 0.0172651266217, 0.0169834206173, 0.0156648428124, 0.0142897100061, 0.0139771554293, 0.014328831215, 0.0129035541406, 0.0120052313912, 0.0111935768743, 0.0106765918779, 0.0100941809684, 0.00881574265034, 0.00854732455245, 0.0077399581517, 0.00729528930697, 0.00731173001504, 0.00736107125013, 0.00702244526951, 0.00679573970411, 0.00667411704583, 0.00682120566777, 0.00668123401571, 0.00646805603737, 0.00617952259799, 0.00632019210598, 0.00617329600412, 0.00612400383929, 0.00599236658573, 0.00573224110835, 0.00594690323937, 0.00557432804393, 0.00511247771011, 0.00541162075132, 0.00512680365357, 0.00521957909653, 0.00505693751801, 0.0048764561718, 0.00525516967838, 0.0052081645412, 0.00517209962545, 0.00498290190767, 0.00502239701939, 0.0050624104803, 0.00515216410865, 0.00474803081708, 0.00464931437364, 0.00474698042042, 0.00495291454849};
float centroid1[2] = {-0.009203333081181396, -0.0097009933676278984};
float centroid2[2] = {-0.058830673231731599, -0.0078117185052879838};
float centroid3[2] = {0.020473111810182468, 0.048838453025517981};
float centroid4[2] = {0.047560894502730544, -0.031325741152602095};
float* centroids[4] = {
  (float *) &centroid1, (float *) &centroid2,
  (float *) &centroid3, (float *) &centroid4
};

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

float result[SNIPPET_SIZE] = {0};
float proj1 = 0;
float proj2 = 0;

// Data array and index pointer
int re[SIZE] = {0};
volatile int re_pointer = 0;

/*---------------------------------------------------*/
/*---------------------------------------------------*/
/*---------------------------------------------------*/

/*---------------------------*/
/*       Norm functions      */
/*---------------------------*/

// Compute the L2 norm of (dim1, dim2) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        centroid: size-2 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm(float dim1, float dim2, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2));
}

// Compute the L2 norm of (dim1, dim2, dim3) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        dim3: 3rd dimension coordinate
//        centroid: size-3 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm3(float dim1, float dim2, float dim3, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2) + pow(dim3-centroid[2],2));
}

void setup(void) {
  Serial.begin(38400);

  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_ENCODER, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(MIC_INPUT, INPUT);

  for (int i = 0; i <= 4; i++) {
    sample_lens[i] = run_times[i]/SAMPLING_INTERVAL;
  }

  write_pwm(0, 0);
  delay(2000); // Wait 2 seconds to put down car
  reset_blinker();
  start_listen_mode();
}

void loop(void) {
  check_encoders();
  if (timer_mode == MODE_LISTEN && re_pointer == SIZE){
    // Stop motor
    write_pwm(0, 0);
    digitalWrite(RED_LED, LOW);

    // if enveloped data is above some preset value
    if (envelope(re, result)) {

      // Reset projection result variables declared above
      proj1 = 0;
      proj2 = 0;

      /*---------------------------*/
      /*      CODE BLOCK PCA3      */
      /*     From classify.ino     */
      /*     with more changes     */
      /*---------------------------*/

  
      // Project 'result' on the principal components
      
      
      for (int i=0; i<SNIPPET_SIZE; i++){
        result[i] = result[i] - mean_vec[i];
      }
      for (int i=0; i<SNIPPET_SIZE; i++){
        proj1 += result[i]*pca_vec1[i];
        proj2 += result[i]*pca_vec2[i];
      }

      
      // Classification
      // Use the function l2_norm defined above
      // ith centroids: centroids[i]
      // YOUR CODE HERE
      int mind = 0;
      for (int i=1; i<4; i++) {
        if (l2_norm(proj1, proj2, centroids[i]) < l2_norm(proj1, proj2, centroids[mind])) {
          mind = i;
        }
      }
      
      // Check against KMEANS_THRESHOLD and print result over serial
      // YOUR CODE HERE
      if (l2_norm(proj1, proj2, centroids[mind])< KMEANS_THRESHOLD){
        Serial.println(mind);
        drive_mode = mind; // from 0-3, inclusive
        start_drive_mode();
      }
      else {
        Serial.println("Below K_THRESHOLD.");
      }
    }
    else {
      Serial.println("Below LOUDNESS_THRESHOLD.");
    }

    delay(2000);
    re_pointer = 0; // start recording from beginning if we don't start driving
  }

  else if (loop_mode == MODE_DRIVE && do_loop) {
    if (step_num < JOLT_STEPS) {
      write_pwm(left_jolt, right_jolt);
    }
    else {

      // Save positions because _left_position and _right_position
      // can change in the middle of one loop.
      int left_position = left_encoder.pos;
      int right_position = right_encoder.pos;

      /*---------------------------*/
      /*      CODE BLOCK CON0      */
      /*---------------------------*/

      float delta = left_position - right_position + delta_ss;
      delta = delta - delta_reference(step_num) - straight_correction(step_num);

      // Drive straight using feedback
      // Compute the needed pwm values for each wheel using delta and v_star
      int left_cur_pwm = driveStraight_left(delta);
      int right_cur_pwm = driveStraight_right(delta);
      write_pwm(left_cur_pwm, right_cur_pwm);

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }

    // Counter for how many times loop is executed since entering DRIVE MODE
    step_num++;

    if (step_num == sample_lens[drive_mode]) {
      // Completely stop and go back to listen MODE after 3 seconds
      start_listen_mode();
    }

    do_loop = 0;
  }
}

// Enveloping function with thresholding and normalizing,
// returns snippet of interest (containing speech)
bool envelope(int* data, float* data_out) {
  int32_t avg = 0;
  float maximum = 0;
  int32_t total = 0;
  int block;

  // Apply enveloping filter while finding maximum value
  for (block = 0; block < SIZE_AFTER_FILTER; block++) {
    avg = 0;
    for (int i = 0; i < 16; i++) {
      avg += data[i+block*16];
    }
    avg = avg >> 4;
    data[block] = abs(data[block*16] - avg);
    for (int i = 1; i < 16; i++) {
      data[block] += abs(data[i+block*16] - avg);
    }
    if (data[block] > maximum) {
      maximum = data[block];
    }
  }

  // If not loud enough, return false
  if (maximum < LOUDNESS_THRESHOLD) {
    return false;
  }

  // Determine threshold
  float thres = THRESHOLD * maximum;

  // Figure out when interesting snippet starts and write to data_out
  block = PRELENGTH;
  while (data[block++] < thres);
  if (block > SIZE_AFTER_FILTER - SNIPPET_SIZE) {
    block = SIZE_AFTER_FILTER - SNIPPET_SIZE;
  }
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data[block-PRELENGTH+i];
    total += data_out[i];
  }

  // Normalize data_out
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data_out[i] / total;
  }

  return true;
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void write_pwm(int pwm_left, int pwm_right) {
  analogWrite(LEFT_MOTOR, (int) min(max(0, pwm_left), 255));
  analogWrite(RIGHT_MOTOR, (int) min(max(0, pwm_right), 255));
}

void reset_blinker(void) {
  digitalWrite(RED_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(GREEN_LED, LOW);
}

void start_listen_mode(void) {
  re_pointer = 0;
  write_pwm(0, 0);
  delay(3000); // 3 seconds buffer for mic cap settling
  timer_mode = MODE_LISTEN;
  setTimer(MODE_LISTEN);
}

void start_drive_mode(void) {
  timer_mode = MODE_DRIVE;
  step_num = 0;
  left_encoder.pos = 0;
  right_encoder.pos = 0;
  setTimer(MODE_DRIVE);
}

/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

#define AVG_DECAY_RATE              0.3
#define LOW_THRESH                  ((int) (0.2*4096))
#define HIGH_THRESH                 ((int) (0.8*4096))

void check_encoder(encoder_t* enc) {
  int new_val = analogRead(enc->pin);
  enc->avg = (int) (AVG_DECAY_RATE*enc->avg + (1 - AVG_DECAY_RATE)*new_val);
  if ((enc->level == LOW && HIGH_THRESH < enc->avg) ||
      (enc->level == HIGH && enc->avg < LOW_THRESH)) {
    enc->pos++;
    enc->level = !enc->level;
  }
}

void check_encoders(void) {
  check_encoder(&left_encoder);
  check_encoder(&right_encoder);
}

// Set timer for timestep; use A2 since A0 & A1 are used by PWM
void setTimer(boolean mode) {
  if (mode == MODE_LISTEN) {
    // Set the timer based on 25MHz clock
    TA2CCR0 = (unsigned int) (25000*ADC_TIMER_MS);
    TA2CCTL0 = CCIE;
    __bis_SR_register(GIE);
    TA2CTL = TASSEL_2 + MC_1 + TACLR + ID_0;
  }
  else if (mode == MODE_DRIVE) {
    TA2CCR0 = (unsigned int) (32.768*SAMPLING_INTERVAL); // set the timer based on 32kHz clock
    TA2CCTL0 = CCIE; // enable interrupts for Timer A
    __bis_SR_register(GIE);
    TA2CTL = TASSEL_1 + MC_1 + TACLR + ID_0;
  }
  timer_mode = mode;
}

// ISR for timestep
#pragma vector=TIMER2_A0_VECTOR    // Timer A ISR
__interrupt void Timer2_A0_ISR(void) {
  if (timer_mode == MODE_LISTEN) {
    if (re_pointer < SIZE) {
      digitalWrite(RED_LED, HIGH);
      re[re_pointer] = (analogRead(MIC_INPUT) >> 4) - 128;
      re_pointer += 1;
    }
  }
  else if (timer_mode == MODE_DRIVE) {
    do_loop = 1;
  }
}
