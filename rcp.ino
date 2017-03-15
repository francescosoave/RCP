/*
* RCP prototype 
* by Francesco Soave
*/

#include <Servo.h> 
#include <SharpIR.h>

#define SERVO 9
#define SHARP A0
#define SCR 8 //screen butt
#define DIR_PIN 3 //stepper dir
#define STEP_PIN 4// stepper step
#define SERVO_SHARP_DIST (int)45
#define SIZE 10 //array length
#define NEXT_SONG 11 //butt
#define VOLUME 5 //pot


Servo my_servo;  //servo obj
SharpIR sharp(SHARP,25,93,1080);
                
int pos = 0;    //servo position 
int val = 0;    //sharp input val
int servo_upd = 0;
float avg = 0; //media
int scr_state= 0; //screen butt
int next_state = 0; //change song butt
boolean step_lock = false;
boolean next_lock = false;
boolean dir = true;
int old_avg = 0;
int raw_pot = 0;
int volume = 0;

int my_values[SIZE]; //IR val

void setup() {
    Serial.begin(9600); 
    
    my_servo.attach(SERVO);  // attaches the servo on pin 9 to the servo object 
    pinMode(SHARP, INPUT);
    pinMode(SCR, INPUT);
    pinMode(VOLUME, INPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(NEXT_SONG, INPUT);
    
    my_servo.write(180); //init servo
    
    //array init
    int i;
    for(i=0;i<SIZE;i++)
        my_values[i] = 0;
} 
    
void loop(){ 
    int int_avg;
    int i=0;
    
    scr_state = digitalRead(SCR); //button per schermo
    next_state = digitalRead(NEXT_SONG); //butt next song
    int dis = sharp.distance();//distance (cm)  
    dis+=2; //improve the library

    raw_pot = analogRead(VOLUME);
    volume = map(raw_pot, 0, 1023, 0, 100);
    Serial.print(volume);
    Serial.print(",");

    if(dis > 80 || dis < 0) //60 è il limite di max distanza del sensore
        dis = 80;

    //change song
    if(next_state == HIGH)
        Serial.println(200);
    else
        Serial.println();
        
    //stepper
    if(scr_state == HIGH){ 
        if(step_lock){
            if(dir){
                rotateDeg(1500,1); //ruota stepper +
                dir = false;
            }else{
                rotateDeg(-1500,1); //ruota stepper -  
                dir = true;
            }
            step_lock = false;
        }
    }else
        step_lock = true;
    
    //  calc avg
    for(;i<SIZE;i++)
        avg += my_values[i];
    
    int_avg = (int)avg/SIZE;
    
    if(int_avg > 30)
        int_avg+=12;
    else if(int_avg < 30 && int_avg > 10)
        int_avg+=13;
    else
        int_avg+=14;
    
    if(int_avg > old_avg || int_avg < old_avg){ //servo internal check
        servo_upd = calcAngle(int_avg);  
        my_servo.write(servo_upd);
        delay(15);
    }
       
    //get rid of glitches
    dis = removeGlitch(dis, int_avg);  
    
    //update array
    arrShift(my_values, SIZE);
    my_values[9] = dis;
    
    old_avg = int_avg;  
    avg = 0; //reset
    
    //delay(100);
    } 
    
int removeGlitch(int dis, int int_avg){
    int i;
    if(dis > int_avg+50){
        int massimo = 0;
        for(i=0;i<10;i++){
          if(my_values[i] > massimo)
            massimo = my_values[i];
        }
        dis = massimo;
    }else if(dis < int_avg-50){
        int minimo = 9999;
        for(i=0;i<10;i++){
          if(my_values[i] < minimo)
            minimo = my_values[i];
        }
        dis = minimo;
    }
    return dis;
}

int calcAngle(int val){        
    float ipoten = sqrt(sq(SERVO_SHARP_DIST)+sq(val));
    float cos_alpha = SERVO_SHARP_DIST/ipoten;
    int angle = (cos_alpha*4068)/71;
    
    return (int)angle; 
}
    
void arrShift(int *a, int n) {   
    int i;
    for(i=0;i!=n-1;i++){
      *(a+i)=*(a+i+1);
    }
}
    
void rotateDeg(float deg, float speed){ 
    int dir = (deg > 0)? HIGH:LOW;
    digitalWrite(DIR_PIN,dir); 
    
    int steps = abs(deg)*(1/0.225);
    float usDelay = (1/speed) * 70;
    
    for(int i=0; i < steps; i++){ 
        digitalWrite(STEP_PIN, HIGH); 
        delayMicroseconds(usDelay); 
        
        digitalWrite(STEP_PIN, LOW); 
        delayMicroseconds(usDelay); 
    } 
}

