#define L_Motor_F 10
#define R_Motor_F 11
#define L_Motor_B A2
#define R_Motor_B A1
#define Key8      3 
#define Key7      4
#define Key6      5
#define Key5      6
#define Key4      7
#define Key3      8
#define Key2      13
#define Key1      12 
#define Fan       9
#define Gled      A7
#define Rled      A5
#define Bled      2
#define V_Key     A4
#define Eco_Sport A3
#define Vol_Metor A0
int V_Key8      = 0;
int V_Key7      = 0;
int V_Key6      = 0;
int V_Key5      = 0;
int V_Key4      = 0;
int V_Key3      = 0;
int V_Key2      = 0;
int V_Key1      = 0;
int outline     = 0;
int previous_outline = 0;
int leftSpeed   = 0;
int rightSpeed  = 0;
#define speedPosito  140
int attacknoise = 0;
#define num_attacknoise  200
#define k_p  20
#define k_d  20
int P = 0, D = 0;
int Output = 0;
int error();
void robotRun();
void stopRobot();
void deBug();

//----------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------//
void setup()
{
    Serial.begin(9600);
    pinMode(L_Motor_F, OUTPUT);
    pinMode(R_Motor_F, OUTPUT);
    pinMode(L_Motor_B, OUTPUT);
    pinMode(R_Motor_B, OUTPUT);
    pinMode(Key8     , INPUT );
    pinMode(Key7     , INPUT );
    pinMode(Key6     , INPUT );
    pinMode(Key5     , INPUT );
    pinMode(Key4     , INPUT );
    pinMode(Key3     , INPUT );
    pinMode(Key2     , INPUT );
    pinMode(Key1     , INPUT );
    pinMode(Fan      , OUTPUT);
    pinMode(Gled     , OUTPUT);
    pinMode(Rled     , OUTPUT);
    pinMode(Bled     , OUTPUT);
    pinMode(V_Key    , OUTPUT);
    pinMode(Eco_Sport, OUTPUT);
    pinMode(Vol_Metor, INPUT );
    digitalWrite(Eco_Sport, HIGH);
    digitalWrite(V_Key, LOW);
    digitalWrite(R_Motor_B, LOW);
    digitalWrite(L_Motor_B, LOW);

    delay(1000);
}

void loop()
{  
    if (analogRead(Vol_Metor)>=400){
        robotRun();
    } else {
        stopRobot();
    }
    // deBug();
}
//----------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------//

int error()
{
    V_Key8 = digitalRead(Key8);
    V_Key7 = digitalRead(Key7);
    V_Key6 = digitalRead(Key6);
    V_Key5 = digitalRead(Key5);
    V_Key4 = digitalRead(Key4);
    V_Key3 = digitalRead(Key3);
    V_Key2 = digitalRead(Key2);
    V_Key1 = digitalRead(Key1);
    if (V_Key8 == 0) {V_Key8 = 1;} else {V_Key8 = 0;}
    if (V_Key7 == 0) {V_Key7 = 1;} else {V_Key7 = 0;}
    if (V_Key6 == 0) {V_Key6 = 1;} else {V_Key6 = 0;}
    if (V_Key5 == 0) {V_Key5 = 1;} else {V_Key5 = 0;}
    if (V_Key4 == 0) {V_Key4 = 1;} else {V_Key4 = 0;}
    if (V_Key3 == 0) {V_Key3 = 1;} else {V_Key3 = 0;}
    if (V_Key2 == 0) {V_Key2 = 1;} else {V_Key2 = 0;}
    if (V_Key1 == 0) {V_Key1 = 1;} else {V_Key1 = 0;}
    if (attacknoise > 0){
        attacknoise --;
        if (outline > 0) {
            return 10;
        } else {
            return -10;
        }
    }
    if (V_Key8==1&&V_Key7==1&&V_Key6==1&&V_Key5==1&&V_Key4==1&&V_Key3==1&&V_Key2==1&&V_Key1==1)
    {
        attacknoise = num_attacknoise;
        if (outline > 0) {
            return 10;
        } else {
            return -10;
        }
    } 
    else 
    {
        int GainSsLine = (V_Key8*(-10))+(V_Key7*(-8))+(V_Key6*(-3))+(V_Key5*0)+(V_Key4*0)+(V_Key3*3)+(V_Key2*8)+(V_Key1*10);
        int NumOfLine = 8-(V_Key8 + V_Key7 + V_Key6 + V_Key5 + V_Key4 + V_Key3 + V_Key2 + V_Key1);
        if (NumOfLine == 0) NumOfLine = 1;
        return GainSsLine/NumOfLine;
    }    
}

void robotRun()
{
    outline = error();
    if (outline == 0)
    {   leftSpeed = speedPosito;
        rightSpeed = speedPosito;
        analogWrite(L_Motor_F, leftSpeed);
        analogWrite(R_Motor_F, rightSpeed);       
    } 
    else 
    {
        // myPID.Compute();
        P = outline;
        D = outline - previous_outline;
        previous_outline = outline;
        Output = (P*k_p) + (D*k_d);

        leftSpeed  = speedPosito - Output;
        if (leftSpeed > 255) {leftSpeed = 255;}
        if (leftSpeed < 0) {leftSpeed = 0;}
        rightSpeed = speedPosito + Output;
        if (rightSpeed > 255) {rightSpeed = 255;}
        if (rightSpeed < 0) {rightSpeed =0;}
        analogWrite(L_Motor_F, leftSpeed );
        analogWrite(R_Motor_F, rightSpeed);
    }
}

void stopRobot()
{
    leftSpeed = 0;
    rightSpeed = 0;
    digitalWrite(V_Key, HIGH);
    analogWrite(L_Motor_F, leftSpeed);
    analogWrite(L_Motor_B, 0);
    analogWrite(R_Motor_F, rightSpeed);
    analogWrite(R_Motor_B, 0);
}

void deBug(){
    Serial.print("  "); Serial.print(V_Key8);   
    Serial.print("  "); Serial.print(V_Key7);   
    Serial.print("  "); Serial.print(V_Key6);   
    Serial.print("  "); Serial.print(V_Key5);   
    Serial.print("  "); Serial.print(V_Key4);   
    Serial.print("  "); Serial.print(V_Key3);   
    Serial.print("  "); Serial.print(V_Key2);   
    Serial.print("  "); Serial.print(V_Key1);
    Serial.print("       "); Serial.print(leftSpeed);   
    Serial.print("           "); Serial.print(rightSpeed);   
    Serial.print("  outline  "); Serial.println(outline);  
}
