// DCC Function Decoder AYA002-3 for DS-DCC decode
// By yaasan
// Based on Nicolas's sketch http://blog.nicolas.cx
// Inspired by Geoff Bunza and his 17 Function DCC Decoder & updated library
//
// Smile function decoder
//  for panta spark Ver1.1
//
// http://1st.geocities.jp/dcc_digital/
// O1:未使用
// O2:未使用
// O3:室内灯
// O4:パンタスパーク

// シリアルデバックの有効:1 / 無効:0

#include <NmraDcc.h>
#include <avr/eeprom.h>	 //required by notifyCVRead() function if enabled below
//#include "SS.h"

//panta spark patern
//Cmd,Time,Val
//O:出力,E:終了
byte ptn1[6][3]  = {{'O',1,100},{'O',1,0},{'O',1,255},{'E',1,0}};
byte ptn2[10][3] = {{'O',1,255},{'O',1,0},{'O',1,255},{'O',1,0},{'O',1,255},{'O',1,0},{'O',3,255},{'E',1,0}};
byte ptn3[6][3]  = {{'O',1,5},  {'O',1,0},{'O',1,5},  {'E',1,0}};
byte ptn4[4][3]  = {{'O',3,255},{'O',1,0},{'E',0,0}};
unsigned char (*ptn)[3];

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
#define CV_F0F 49   // F0/前進側(ヘッド)
#define CV_F0R 50   // F0/後進側(テール)
#define CV_farstStep 51 // 点灯開始STEP値
#define CV_ax1 52
#define CV_ax2 53
#define CV_ax3 54
#define CV_ax4 55

// O1-O4 の ON/OFFステータス
uint8_t State_O[5][2] = {0,0 , 0,0 , 0,0 , 0,0 , 0,0}; // [0][n]は未使用

//使用クラスの宣言
NmraDcc	 Dcc;
DCC_MSG	 Packet;

//SoftwareSerial mySerial(O1, O2); // RX, TX


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
uint8_t gCV49_Conf = 0x01;
uint8_t gCV50_Conf = 0x01;
uint8_t startStep = 5;
uint8_t ax1 = 100;
uint8_t ax2 = 1;
uint8_t ax3 = 200;
uint8_t ax4 = 5;

//Internal variables and other.
#if defined(DCC_ACK_PIN)
const int DccAckPin = DCC_ACK_PIN ;
#endif

struct CVPair {
  uint16_t	CV;
  uint8_t	Value;
};
CVPair FactoryDefaultCVs [] = {                        // 工場出荷時のCV値
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
  {CV_F0F ,0x01}, // 0:spark normal(0:nor,1:demo,2:snow) , 1:ON(0:OFF,1:ON)   
  {CV_F0R ,0x01}, // 0:spark normal(0:nor,1:demo,2:snow) , 1:ON(0:OFF,1:ON)   
  {CV_farstStep ,5}, 
  {CV_ax1,100},   
  {CV_ax2,1},
  {CV_ax3,200},   
  {CV_ax4,5},
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
  
// pinMode(O1, OUTPUT);  // ソフトシリアル用 TX 未使用
// pinMode(O2, OUTPUT);  // ソフトシリアル用 RX
  pinMode(O3, OUTPUT);
  pinMode(O4, OUTPUT);

//  mySerial.begin(9600);
//  mySerial.println("Hello,SmileFunctionDecoder");

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

  gCV49_Conf = Dcc.getCV( CV_F0F );
  gCV50_Conf = Dcc.getCV( CV_F0R );
  startStep = Dcc.getCV( CV_farstStep );
  ax1 = Dcc.getCV( CV_ax1 );
  ax2 = Dcc.getCV( CV_ax2 );
  ax3 = Dcc.getCV( CV_ax3 );
  ax4 = Dcc.getCV( CV_ax4 );    

  for(int i=0;i<=12;i++){
    if(gState_Fn[i][1] != 0)
        State_O[ gState_Fn[i][1] ][0] = i;  // State_O に ファンクッション番号を格納
  }
#if 0
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
// 
//---------------------------------------------------------------------
void PantaSparkEffect_Control(){

  static char state = 0;    // ステート
  static char adr = 0;      // アドレス
  static int timeUp = 0;    // 時間
  static float pwmRef =0;
  static int nextSparkWait =0;  // 点滅間隔 10ms
  static int sparkMode = 0;    // 点灯モード(0:Normal,1:DEMO,2:snow)

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

    if( gDirection == 0){                   // Reverse 後進(DCS50Kで確認)
      if(gCV50_Conf & 0x01 == 0x01){        // Reverse で点灯させる？
          sparkMode = gCV50_Conf / 10;      // 00:normal 10:demo 20:snowなので10で割ってmodeを決定させる
      } else {
          //       OCR1B有効   high出力　    薄く点灯しない様に　
          GTCCR = 0 << PWM1B | 0 << COM1B0;
          analogWrite(O4, 0);
        return;
      }
    } else {                                // Forward 前進
      if(gCV49_Conf & 0x01 == 0x01){        // Forward で点灯させる？
          sparkMode = gCV49_Conf /10;       // 00:normal 10:demo 20:snowなので10で割ってmodeを決定させる   
      } else {
          //       OCR1B有効   high出力      薄く点灯しない様に　
          GTCCR = 0 << PWM1B | 0 << COM1B0;
          analogWrite(O4, 0);   
         return;
      }
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
        if(gSpeedRef <= startStep){
          state = 0;
          TCCR1 = 0 << CS10;  //分周比をセット 0827 0速でも点灯していた対策
          GTCCR = 0 << PWM1B | 0 << COM1B0;
          analogWrite(O4, 0);
          state = 1;
             
        } else {
          switch(sparkMode){
            case 0:             // Normal 点滅
              nextSparkWait = random(map(gSpeedRef,1,127,ax1,ax2),map(gSpeedRef,1,127,ax3,ax4));
              break;
            case 1:             // Demo 点滅（常に激しく点滅）
            case 2:             // Snow
            case 3:       
               nextSparkWait = 1;
               break;
            default:
              break;
          }
          state = 2;
          TCCR1 = 1<<CS10;  //分周比をセット
          GTCCR = 1 << PWM1B | 2 << COM1B0;
        }
        break;
        
    case 2: // 光るパターンを４種類からランダムに算出
      switch(random(1,4)){
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
      
    case 4: // S04:コマンド処理
            timeUp = ptn[adr][1];
            pwmRef = ptn[adr][2];

            if(sparkMode == 2){       // 雪モードはPwmMaxを100にして、若干暗くする。
              if(pwmRef >= 100){
                pwmRef = 10;
              }
            }
                
            state = 5;          
          break;
      
    case 5: // S05:時間カウント
      analogWrite(O4, (unsigned char)pwmRef); // 0〜255         
      timeUp--;
      if( timeUp <= 0 ){
        state = 4;                            // 次のコマンドへ
        if( ptn[adr][0]=='E'){                // シーケンス終了
            state = 3;
            break;
        }
        adr ++;
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
  if(MaxSpeed == 28){               // 28stepで受信したときの処理
    Speed = map(Speed,1,29,1,126);  // 変換前 1～29 -> 変換後 1～126
  }

  if ( gDirection != ForwardDir){
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

#if 0
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

