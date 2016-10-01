// DCC Function Decoder AYA002-3 for DS-DCC decode
// By yaasan
// Based on Nicolas's sketch http://blog.nicolas.cx
// Inspired by Geoff Bunza and his 17 Function DCC Decoder & updated library
//
// Smile function decoder
//  for panta spark Ver1.0
//
// http://1st.geocities.jp/dcc_digital/
// O1:未使用
// O2:未使用
// O3:室内灯
// O4:パンタスパーク

// シリアルデバックの有効:1 / 無効:0
#define DEBUG 0

#include <NmraDcc.h>
#include <avr/eeprom.h>	 //required by notifyCVRead() function if enabled below

#if(DEBUG==1)
#include <SoftwareSerial.h>
#endif

//panta spark patern
//Cmd,Time,Val,Frq
//I:初期状態,O:出力,S:スイープ,L:ループ,E:終了
unsigned char ptn1[10][4]={{'I',0,5,1},{'O',1,5,1},  {'O',1,0,1},{'O',1,5,1},  {'O',1,0,1},{'O',1,5,1},  {'O',1,0,1},{'O',3,255,1},{'O',1,0,1},{'E',0,0,1}};
unsigned char ptn2[10][4]={{'I',0,5,1},{'O',1,255,1},{'O',1,0,1},{'O',1,255,1},{'O',1,0,1},{'O',1,255,1},{'O',1,0,1},{'O',3,255,1},{'O',1,0,1},{'E',0,0,1}};
unsigned char ptn3[8][4]= {{'I',0,5,1},{'O',1,5,1},  {'O',1,0,1},{'O',1,5,1},  {'O',1,0,1},{'O',1,5,1},  {'O',1,0,1},{'E',0,0,1}};
unsigned char ptn4[6][4]= {{'I',0,5,1},{'O',1,5,1},  {'O',1,0,1},{'O',1,5,1},  {'O',1,0,1},{'E',0,0,1}};
//unsigned char ptn5[4][4]= {{'I',0,5,1},{'O',3,255,1},{'O',1,0,1},{'E',0,0,1}};
unsigned char (*ptn)[4];

//panta spark table(速度による点滅間隔のMinとMax
//                           0-9   10-19     20-29     30-39     40-49     50-59    60-69    70-79    80-89   90-99    100-109 110-119 120-128
//long mtbl[13][2]={{0,0},{300,400},{200,350},{150,300},{100,250},{50,200},{30,150},{25,100},{25,80},{10,50}, {5,30}, {1,10}, {1,5}};

//各種設定、宣言
#define DECODER_ADDRESS 3
#define DCC_ACK_PIN 0   // Atiny85 PB0(5pin) if defined enables the ACK pin functionality. Comment out to disable.
//                      // Atiny85 DCCin(7pin)
#define O1 0            // Atiny85 PB0(5pin)
#define O2 1            // Atiny85 PB1(6pin) analogwrite
#define O3 3            // Atint85 PB3(2pin)
#define O4 4            // Atiny85 PB4(3pin) analogwrite

#define F0 0
#define F1 1
#define F2 2
#define F3 3
#define F4 4
#define F5 5
#define F6 6
#define F7 7
#define F8 8
#define F9 9
#define F10 10
#define F11 11
#define F12 12

#define ON 1
#define OFF 0

// ファンクション CV変換テーブル
#define CV_F0 33
#define CV_F1 35
#define CV_F2 36
#define CV_F3 37
#define CV_F4 38
#define CV_F5 39
#define CV_F6 40
#define CV_F7 41
#define CV_F8 42
#define CV_F9 43
#define CV_F10 44
#define CV_F11 45
#define CV_F12 46

// O1-O4 の ON/OFFステータス
uint8_t State_O[5][2] = {0,0 , 0,0 , 0,0 , 0,0 , 0,0}; // [0][n]は未使用

//使用クラスの宣言
NmraDcc	 Dcc;
DCC_MSG	 Packet;

#if(DEBUG==1)
SoftwareSerial mySerial(O1, O2); // RX, TX
#endif

//Task Schedule
unsigned long gPreviousL5 = 0;

//進行方向
uint8_t gDirection = 128;

//Function State
uint8_t gState_Fn[13][2];

//モータ制御関連の変数
uint32_t gSpeedRef = 1;

//CV related
uint8_t gCV1_SAddr = 3;
uint8_t gCVx_LAddr = 3;
uint8_t gCV29_Conf = 0;

//Internal variables and other.
#if defined(DCC_ACK_PIN)
const int DccAckPin = DCC_ACK_PIN ;
#endif

struct CVPair {
  uint16_t	CV;
  uint8_t	Value;
};
CVPair FactoryDefaultCVs [] = {
  {CV_MULTIFUNCTION_PRIMARY_ADDRESS, DECODER_ADDRESS}, // CV01
  {CV_ACCESSORY_DECODER_ADDRESS_MSB, 0},               // CV09 The LSB is set CV 1 in the libraries .h file, which is the regular address location, so by setting the MSB to 0 we tell the library to use the same address as the primary address. 0 DECODER_ADDRESS
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, 0},          // CV17 XX in the XXYY address
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, 0},          // CV18 YY in the XXYY address
  {CV_29_CONFIG, 2},                                   // CV29 Make sure this is 0 or else it will be random based on what is in the eeprom which could caue headaches
  {CV_F0 ,0},
  {CV_F1 ,0},
  {CV_F2 ,0},
  {CV_F3 ,3}, // Room Light LED [O3]
  {CV_F4 ,4}, // Panta Spark LED [O4]
  {CV_F5 ,0},
  {CV_F6 ,0},
  {CV_F7 ,0},
  {CV_F8 ,0},
  {CV_F9 ,0},
  {CV_F10 ,0},
  {CV_F11 ,0},
  {CV_F12 ,0},   
};

void(* resetFunc) (void) = 0;  //declare reset function at address 0
uint8_t FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs) / sizeof(CVPair);

void notifyCVResetFactoryDefault()
{
  //When anything is writen to CV8 reset to defaults.
  resetCVToDefault();
  //Serial.println("Resetting...");
  delay(1000);  //typical CV programming sends the same command multiple times - specially since we dont ACK. so ignore them by delaying

  resetFunc();
};

//------------------------------------------------------------------
// CVをデフォルトにリセット
// Serial.println("CVs being reset to factory defaults");
//------------------------------------------------------------------
void resetCVToDefault()
{
  for (int j = 0; j < FactoryDefaultCVIndex; j++ ) {
    Dcc.setCV( FactoryDefaultCVs[j].CV, FactoryDefaultCVs[j].Value);
  }
};

//------------------------------------------------------------------
// CVが変更された
//------------------------------------------------------------------
extern void	   notifyCVChange( uint16_t CV, uint8_t Value) {
  //CVが変更されたときのメッセージ
  //Serial.print("CV ");
  //Serial.print(CV);
  //Serial.print(" Changed to ");
  //Serial.println(Value, DEC);
};

//------------------------------------------------------------------
// CV Ack
// Smile Function Decoder は未対応
//------------------------------------------------------------------
void notifyCVAck(void)
{
  //Serial.println("notifyCVAck");
  digitalWrite(O3,HIGH);
  digitalWrite(O4,HIGH);
  delay( 6 );
  digitalWrite(O3,LOW);
  digitalWrite(O4,LOW);
}

//------------------------------------------------------------------
// Arduino固有の関数 setup() :初期設定
//------------------------------------------------------------------
void setup()
{
  uint8_t cv_value;

  TCCR1 = 0<<CTC1 | 0<<PWM1A | 0<<COM1A0 | 1<<CS10;
  
  pinMode(O1, OUTPUT);    // ソフトシリアル用 
//  pinMode(O2, OUTPUT);  //
  pinMode(O3, OUTPUT);
  pinMode(O4, OUTPUT);

#if(DEBUG==1)
  mySerial.begin(9600);
  mySerial.println("Hello,SmileFunctionDecoder");
#endif

  //DCCの応答用負荷ピン
#if defined(DCCACKPIN)
  //Setup ACK Pin
  pinMode(DccAckPin, OUTPUT);
  digitalWrite(DccAckPin, 0);
#endif

#if !defined(DECODER_DONT_DEFAULT_CV_ON_POWERUP)
  if ( Dcc.getCV(CV_MULTIFUNCTION_PRIMARY_ADDRESS) == 0xFF ) {	 //if eeprom has 0xFF then assume it needs to be programmed
    //Serial.println("CV Defaulting due to blank eeprom");
    notifyCVResetFactoryDefault();

  } else {
    //Serial.println("CV Not Defaulting");
  }
#else
  //Serial.println("CV Defaulting Always On Powerup");
  notifyCVResetFactoryDefault();
#endif

  // Setup which External Interrupt, the Pin it's associated with that we're using, disable pullup.
  Dcc.pin(0, 2, 0); // Atiny85 7pin(PB2)をDCC_PULSE端子に設定

  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init( MAN_ID_DIY, 100,   FLAGS_MY_ADDRESS_ONLY , 0 );

  //Reset task
  gPreviousL5 = millis();

  //Init CVs
  gCV1_SAddr = Dcc.getCV( CV_MULTIFUNCTION_PRIMARY_ADDRESS ) ;
  gCVx_LAddr = (Dcc.getCV( CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB ) << 8) + Dcc.getCV( CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB );
  gCV29_Conf = Dcc.getCV( CV_29_CONFIG ) ;
  gState_Fn[0][1] = Dcc.getCV( CV_F0 );
  gState_Fn[1][1] = Dcc.getCV( CV_F1 );
  gState_Fn[2][1] = Dcc.getCV( CV_F2 );
  gState_Fn[3][1] = Dcc.getCV( CV_F3 );
  gState_Fn[4][1] = Dcc.getCV( CV_F4 );
  gState_Fn[5][1] = Dcc.getCV( CV_F5 );
  gState_Fn[6][1] = Dcc.getCV( CV_F6 );
  gState_Fn[7][1] = Dcc.getCV( CV_F7 );
  gState_Fn[8][1] = Dcc.getCV( CV_F8 );
  gState_Fn[9][1] = Dcc.getCV( CV_F9 );
  gState_Fn[10][1] = Dcc.getCV( CV_F10 );
  gState_Fn[11][1] = Dcc.getCV( CV_F11 );
  gState_Fn[12][1] = Dcc.getCV( CV_F12 );

  for(int i=0;i<=12;i++){
    if(gState_Fn[i][1] != 0)
        State_O[ gState_Fn[i][1] ][0] = i;  // State_O に ファンクッション番号を格納
#if 0
    switch(gState_Fn[i][1]){
      case 1: //O1
              State_O[1][0] = i;  // State_O に ファンクッション番号を格納
              break;
      case 2: //O2
              State_O[2][0] = i;  // State_O に ファンクッション番号を格納
              break;
      case 3: //O3
              State_O[3][0] = i;  // State_O に ファンクッション番号を格納
              break;
      case 4: //O4
              State_O[4][0] = i;  // State_O に ファンクッション番号を格納
              break;
      default:
              break;
    }
#endif
  }
#if(DEBUG==1)
  mySerial.print("State_O:");
  mySerial.print(State_O[1][0], DEC);
  mySerial.print(":");
  mySerial.print(State_O[2][0], DEC);
  mySerial.print(":");
  mySerial.print(State_O[3][0], DEC);
  mySerial.print(":");
  mySerial.println(State_O[4][0], DEC);
#endif
}


//---------------------------------------------------------------------
// Arduino Main Loop
//---------------------------------------------------------------------
void loop() {
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();

  if ( (millis() - gPreviousL5) >= 10) // 100:100msec  10:10msec  Function decoder は 10msecにしてみる。
  {
    LightControl();

    //Reset task
    gPreviousL5 = millis();
  }
}


//---------------------------------------------------------------------
// HeadLight control Task (100Hz:10ms)
//---------------------------------------------------------------------
void LightControl()
{
  PantaSparkEffect_Control();

// O3の処理 室内灯
  if(State_O[3][1] == 0){
    digitalWrite(O3, LOW);
  } else {
    digitalWrite(O3, HIGH);
  }
#if 0
// O4の処理
  if(State_O[4][1] == 0){
    digitalWrite(O4, LOW);
  } else {
    digitalWrite(O4, HIGH);
  }
#endif
}


//---------------------------------------------------------------------
// PantographSpark効果ステートマシン
// 10ms周期で起動
// unsigned chart ptn[4][5]{{'I',0,0,1},{'S',20,255,1},{'S',40,0,1},{'E',0,0,1}};
//---------------------------------------------------------------------
void PantaSparkEffect_Control(){
  static char state = 0;    // ステート
  static char adr = 0;      // アドレス
  static int timeUp = 0;    // 時間
  static float delt_v = 0;  // 100msあたりの増加量 
  static float pwmRef =0;
  static int nextSparkWait =0;  // 点滅間隔 10ms
  long sparkSel = 0;
    
  if(State_O[4][1]== 0){ // PantaSpark O4:OFF
    state = 0; 
    adr = 0;
    timeUp = 0;
    pwmRef = 0;
    TCCR1 = 0<<CS10;  //分周比をセット
    //       OCR1B有効   high出力　
    GTCCR = 0 << PWM1B | 0 << COM1B0;
    analogWrite(O4, 0);
    digitalWrite(O4, LOW);               // 消灯
  }

  S00:                  // S00:idle
  switch(state){
    case 0:
      if(State_O[4][1] == 1){ // PantaSpark O4:ON
        adr = 0;
        timeUp = 0;
        pwmRef = 0;
        TCCR1 = 0<<CS10;  //分周比をセット 0827 0速でも点灯していた対策
        //       OCR1B有効   high出力　
        GTCCR = 1 << PWM1B | 2 << COM1B0;
        analogWrite(O4, 0);
        state = 1;
        goto S00;     // 100ms待たずに再度ステートマシーンに掛ける
      }
      break;

    case 1: // スピードによる点灯間隔算出 max 128 step?
        if(gSpeedRef <= 9){
            state = 0;
        } else if(gSpeedRef >= 10 && gSpeedRef <=19 ){
            nextSparkWait = random(200,250);//300,300
            state = 2;
        } else if(gSpeedRef >= 20 && gSpeedRef <=29 ){
            nextSparkWait = random(150,200);//200,250         
            state = 2;
        } else if(gSpeedRef >= 30 && gSpeedRef <=39 ){
            nextSparkWait = random(100,160);//150,200         
            state = 2;
        } else if(gSpeedRef >= 40 && gSpeedRef <=49 ){
            nextSparkWait = random(60,120);//100,160            
            state = 2;
        } else if(gSpeedRef >= 50 && gSpeedRef <=59 ){
            nextSparkWait = random(35,90);//60,120               
            state = 2;
        } else if(gSpeedRef >= 60 && gSpeedRef <=69 ){
            nextSparkWait = random(25,60);//35,90               
            state = 2;
        } else if(gSpeedRef >= 70 && gSpeedRef <=79 ){
            nextSparkWait = random(20,40);//25,60               
            state = 2;
        } else if(gSpeedRef >= 80 && gSpeedRef <=89 ){
            nextSparkWait = random(10,30);//20,40               
            state = 2;
        } else if(gSpeedRef >= 90 && gSpeedRef <=99 ){
            nextSparkWait = random(5,20);//10,30               
            state = 2;
        } else if(gSpeedRef >= 100 && gSpeedRef <=109 ){
            nextSparkWait = random(1,10);//5,20               
            state = 2;
        } else if(gSpeedRef >= 110 && gSpeedRef <=119 ){
            nextSparkWait = random(1,5);//1,10               
            state = 2;
        } else if(gSpeedRef >= 120 ){
            nextSparkWait = random(1,5);               
            state = 2;
        }
        break;
        
    case 2: // 点灯トリガ

      sparkSel = random(1,4);
      switch(sparkSel){
        case 1:
          ptn = ptn1;
          break;
        case 2:
          ptn = ptn2;
          break;
        case 3:
          ptn = ptn3;
          break;        
        case 4:
          ptn = ptn4;
          break;
//      case 5:
//        ptn = ptn5;
//        break;
        default:
          ptn = ptn1;
          break;
      }
      adr = 0; 
      state = 4;
      goto S00;   // 10ms待たずに再度ステートマシーンに掛ける
      break;

    case 3: // 次のスパークまでのウエイト処理

      nextSparkWait--;
      if(nextSparkWait <= 0){
        state = 1;
      }
      break;
      
    case 4: // S01:コマンド処理
        if( ptn[adr][0]=='I'){ // I:初期化
          timeUp = ptn[adr][1];
          pwmRef = ptn[adr][2];
          delt_v = 0; // 変化量0
          TCCR1 = ptn[adr][3]<<CS10;  //分周比をセット
          analogWrite(O4, (unsigned char)pwmRef); // 0〜255            
          adr++;
          state = 4;
          goto S00;   // 10ms待たずに再度ステートマシーンに掛ける
        } else if( ptn[adr][0]=='E'){ // E:end
          state = 3;
        } else if( ptn[adr][0]=='O' ){ // O:出力
          timeUp = ptn[adr][1];
          pwmRef = ptn[adr][2];
          TCCR1 = ptn[adr][3]<<CS10;  //分周比をセット
          delt_v = 0;
          state = 5;          
        } else if( ptn[adr][0]=='S' ){ // S:sweep
          timeUp = ptn[adr][1];
          TCCR1 = ptn[adr][3]<<CS10;  //分周比をセット      
          delt_v = (ptn[adr][2]-pwmRef)/timeUp;  // 変化量を算出
          state = 5;
        }
      break;
      
    case 5: // S02:時間カウント
      timeUp--;
      pwmRef = pwmRef + delt_v;
      if(pwmRef<=0){            // 下限、上限リミッタ
          pwmRef = 0;
      } else if(pwmRef>=255){
          pwmRef = 255;
      }
      analogWrite(O4, (unsigned char)pwmRef); // 0〜255         
 
      if( timeUp <= 0 ){
        adr ++;
        state = 4;  //次のコマンドへ
      }
      break;
      
      default:
      break;
  }
}

//---------------------------------------------------------------------------
// DCC速度信号の受信によるイベント 
//---------------------------------------------------------------------------
extern void notifyDccSpeed( uint16_t Addr, uint8_t Speed, uint8_t ForwardDir, uint8_t MaxSpeed )
{
  if ( gDirection != ForwardDir)
  {
    gDirection = ForwardDir;
  }
  gSpeedRef = Speed;
}

//---------------------------------------------------------------------------
// ファンクション信号受信のイベント
// FN_0_4とFN_5_8は常時イベント発生（DCS50KはF8まで）
// FN_9_12以降はFUNCTIONボタンが押されたときにイベント発生
// 前値と比較して変化あったら処理するような作り。
// 分かりづらい・・・
//---------------------------------------------------------------------------
extern void notifyDccFunc( uint16_t Addr, FN_GROUP FuncGrp, uint8_t FuncState)
{
#if(DEBUG==1)
   mySerial.print(FuncGrp,DEC);
   mySerial.print(",");
   mySerial.println(FuncState,DEC);
#endif      
  switch(FuncGrp){
    case FN_0_4:
      if( gState_Fn[F0][0] != (FuncState & FN_BIT_00)){     // 要素1:ファンクッション番号  要素2[0]:ファンクッションの状態
        gState_Fn[F0][0] = (FuncState & FN_BIT_00);
        if(gState_Fn[F0][1] !=0)                            // O1-O4が割り当てられている
          State_O[ gState_Fn[F0][1] ][1] = gState_Fn[F0][0] ? ON : OFF; // gState_Fn[F0][0]の値を見て、ON/OFFを決める
      }
      
      if( gState_Fn[F1][0] != (FuncState & FN_BIT_01)){
        gState_Fn[F1][0] = (FuncState & FN_BIT_01);
        if(gState_Fn[F1][1] !=0)                            // O1-O4が割り当てられている
          State_O[ gState_Fn[F1][1] ][1] = gState_Fn[F1][0] ? ON : OFF;
      }
      
      if( gState_Fn[F2][0] != (FuncState & FN_BIT_02)){
        gState_Fn[F2][0] = (FuncState & FN_BIT_02);
        if(gState_Fn[F2][1] !=0)                            // O1-O4が割り当てられている
          State_O[ gState_Fn[F2][1] ][1] = gState_Fn[F2][0] ? ON : OFF;
      }
      
      if( gState_Fn[F3][0]!= (FuncState & FN_BIT_03)){
        gState_Fn[F3][0] = (FuncState & FN_BIT_03);
        if(gState_Fn[F3][1] !=0)                            // O1-O4が割り当てられている
          State_O[ gState_Fn[F3][1] ][1] = gState_Fn[F3][0] ? ON : OFF;
      }
      
      if( gState_Fn[F4][0] != (FuncState & FN_BIT_04)){
        gState_Fn[F4][0] = (FuncState & FN_BIT_04);
        if(gState_Fn[F4][1] !=0)                            // O1-O4が割り当てられている
          State_O[ gState_Fn[F4][1] ][1] = gState_Fn[F4][0] ? ON : OFF;
      }
    break;

  case FN_5_8:
    if( gState_Fn[F5][0] != (FuncState & FN_BIT_05)){
        gState_Fn[F5][0] = (FuncState & FN_BIT_05);
        if(gState_Fn[F5][1] !=0)                            // O1-O4が割り当てられている
          State_O[ gState_Fn[F5][1] ][1] = gState_Fn[F5][0] ? ON : OFF;
    }
    
    if( gState_Fn[F6][0] != (FuncState & FN_BIT_06)){
        gState_Fn[F6][0] = (FuncState & FN_BIT_06);
        if(gState_Fn[F6][1] !=0)                            // O1-O4が割り当てられている
          State_O[ gState_Fn[F6][1] ][1] = gState_Fn[F6][0] ? ON : OFF;
    } 
    
    if( gState_Fn[F7][0] != (FuncState & FN_BIT_07)){
        gState_Fn[F7][0] = (FuncState & FN_BIT_07);
        if(gState_Fn[F7][1] !=0)                            // O1-O4が割り当てられている
          State_O[ gState_Fn[F7][1] ][1] = gState_Fn[F7][0] ? ON : OFF;
    }
    
    if( gState_Fn[F8][0] != (FuncState & FN_BIT_08)){
        gState_Fn[F8][0] = (FuncState & FN_BIT_08);
        if(gState_Fn[F8][1] !=0)                            // O1-O4が割り当てられている
          State_O[ gState_Fn[F8][1] ][1] = gState_Fn[F8][0] ? ON : OFF;
    }
    break;

  case FN_9_12:
    if( gState_Fn[F9][0] != (FuncState & FN_BIT_09)) {
        gState_Fn[F9][0] = (FuncState & FN_BIT_09);
        if(gState_Fn[F9][1] !=0)                            // O1-O4が割り当てられている
          State_O[ gState_Fn[F9][1] ][1] = gState_Fn[F9][0] ? ON : OFF;
    }
    if( gState_Fn[F10][0] != (FuncState & FN_BIT_10)){
        gState_Fn[F10][0] = (FuncState & FN_BIT_10);
        if(gState_Fn[F10][1] !=0)                            // O1-O4が割り当てられている
          State_O[ gState_Fn[F10][1] ][1] = gState_Fn[F10][0] ? ON : OFF;
    } 
    if( gState_Fn[F11][0] != (FuncState & FN_BIT_11)) {
        gState_Fn[F11][0] = (FuncState & FN_BIT_11);
        if(gState_Fn[F11][1] !=0)                            // O1-O4が割り当てられている
          State_O[ gState_Fn[F11][1] ][1] = gState_Fn[F11][0] ? ON : OFF;
    }
    if( gState_Fn[F12][0] != (FuncState & FN_BIT_12)) {
        gState_Fn[F12][0] = (FuncState & FN_BIT_12);
        if(gState_Fn[F12][1] !=0)                            // O1-O4が割り当てられている
          State_O[ gState_Fn[F12][1] ][1] = gState_Fn[F12][0] ? ON : OFF;
    }
    break;  

  default:
    break;
  }
}

#if(DEBUG==1)
//-----------------------------------------------------
// F4 を使って、メッセージを表示させる。
//上位ビットから吐き出す
// ex 5 -> 0101 -> ー・ー・
//-----------------------------------------------------
void LightMes( char sig ,char set)
{
  char cnt;
  for( cnt = 0 ; cnt<set ; cnt++ ){
    if( sig & 0x80){
      digitalWrite(O1, HIGH); // 短光
      delay(200);
      digitalWrite(O1, LOW);
      delay(200);
    } else {
      digitalWrite(O1, HIGH); // 長光
      delay(1000);
      digitalWrite(O1, LOW);            
      delay(200);
    }
    sig = sig << 1;
  }
      delay(400);
}

//-----------------------------------------------------
// Debug用Lﾁｶ
//-----------------------------------------------------
void pulse()
{
  digitalWrite(O1, HIGH); // 短光
  delay(100);
  digitalWrite(O1, LOW);
}
#endif

