/*======================================*/
/* インクルード                         */
/*======================================*/
#include <stdio.h>
#include "sfr_r838a.h"                  /* R8C/38A SFRの定義ファイル    */
#include "printf_lib.h"                 /* printf使用ライブラリ         */
#include "microsd_lib.h"                /* microSD制御ライブラリ        */
#include "lcd_lib.h"                    /* LCD表示用追加                */
#include "switch_lib.h"                 /* スイッチ追加                 */
#include "data_flash_lib.h"             /* データフラッシュライブラリ   */
#include "isCamera.h"
//#include "drive.h"

/* TAOS TSL1401CL */
#define	TAOS_SI_HIGH	p0_addr.bit.b1 = 1	/* Port P0_1 bit */
#define	TAOS_SI_LOW		p0_addr.bit.b1 = 0	/* Port P0_1 bit */
#define	TAOS_CLK_HIGH	p0_addr.bit.b2 = 1	/* Port P0_2 bit */
#define	TAOS_CLK_LOW	p0_addr.bit.b2 = 0	/* Port P0_2 bit */

#define White_min	9					/* 白色の最小値 			*/
#define	White_Max	150					/* ライン白色MAX値の設定 */

int 			ImageData[128];			/* カメラの値				*/
int				BinarizationData[130];	/* ２値化					*/
int   			EXPOSURE_timer = 1;	/* 露光時間					*/
int 			LineStart,LineStop;		/* 読み取り位置の始めと終わり */
int				Max,Min,Ave;			/*カメラ読み取り最大値、最小値、平均値*/
unsigned int 	Rsensor;				/* ラインの右端 */
unsigned int 	Lsensor;				/* ラインの左端 */
unsigned int 	Wide;					/* ラインの幅 */
int			 	Center;					/* ラインの重心 */
int 			White;					/* 白色の個数	*/
int				pid_angle;
int 			sensor8;

/************************************************************************/
/* センサー読み取り                                             */
/* 引数　 なし                                                          */
/* 戻り値 なし			                                                */
/************************************************************************/
void sensor_process(void) {
	
	int i;
	int s;
	
	ImageCapture();
	expose();
	ImageCapture();				/* イメージキャプチャ 	*/
	binarization();				/* ２値化				*/
	WhiteLineWide();			/* 白線の幅を測定		*/
	WhiteLineCenter();			/* 白線の中心を測定		*/

//	printf("Exposure = %d Max = %d Min = %d Ave = %d Wide = %d Center = %d\n",EXPOSURE_timer,Max,Min,Ave,WhiteLineWide(),WhiteLineCenter());
	pid_angle = PID();
	
	/* 8bit */
	sensor8 = 0;
	s = (LineStop - (LineStart -1))/8;
	for(i = (LineStart + s/2); i < LineStop; i=i+s) {	
		sensor8 |= BinarizationData[i];
		sensor8 = sensor8 << 1;
	} 
	p3 = ~sensor8;
}
/************************************************************************/
/* カメラの初期化                                                */
/* 引数　 開始位置、終了位置                                                          */
/* 戻り値 なし                                                 */
/************************************************************************/
void initCamera(int Start,int Stop)
{
	LineStart = Start;
	LineStop = Stop;
}
/************************************************************************/
/* イメージキャプチャ                                                 */
/* 引数　 開始位置、終了位置                                                          */
/* 戻り値 なし                                                 */
/************************************************************************/
void ImageCapture(void){	 
	
	unsigned char i;

	Max = 0,Min = 1024;

	TAOS_SI_HIGH;  
	TAOS_CLK_HIGH;  
	TAOS_SI_LOW;
	ImageData[0] = 0;//get_ad7();	// inputs data from camera (first pixel)
	TAOS_CLK_LOW;

	for(i = 1; i < LineStart; i++) {		
		TAOS_CLK_HIGH;		
		TAOS_CLK_LOW;
	}
	for(i = LineStart; i < LineStop; i++) {					 
		TAOS_CLK_HIGH;
		ImageData[i] = get_ad7();	// inputs data from camera (one pixel each time through loop) 
		TAOS_CLK_LOW;
		
		if(Max < ImageData[i]){
			Max = ImageData[i];
		}			
		if(Min > ImageData[i]){
			Min = ImageData[i];
		}	
	}
	for(i = LineStop; i < 128; i++) {		
		TAOS_CLK_HIGH;		
		TAOS_CLK_LOW;
	}

	TAOS_CLK_HIGH;
	TAOS_CLK_LOW;
}
/************************************************************************/
/* ２値化                                                               */
/* 引数　 なし                                                          */
/* 戻り値 なし			                                                */
/************************************************************************/
void binarization(void)
{
	int i;

	/* 最高値と最低値から間の値を求める */
	Ave = (Max + Min) * 2 / 3;
	/* 黒は０　白は１にする */
	White = 0;					/* 白の数を０にする */
	if( Max > 50 ){
		/* 白が一直線のとき */
		if(Min > 80){
			White = 128;
			for(i=1; i < 128; i++){
				BinarizationData[i] = 1;
			}
		}else{		
			for(i = LineStart ; i < LineStop; i++) {
				if(  ImageData[i] > Ave ){	
					White++;			
					BinarizationData[i] = 1;
				}else{
					BinarizationData[i] = 0;
				}
			}
		}
	/* 黒が一面のとき */
	}else{
		for(i=1; i < 128; i++){
			BinarizationData[i] = 0;
		}
	}

}
/************************************************************************/
/* 白線の幅を測定                                                       */
/* 引数　 なし                                                          */
/* 戻り値 なし			                                                 */
/************************************************************************/
void WhiteLineWide(void)
{
	int t=0,i;
		
	Lsensor = LineStart;
	Rsensor = LineStop;
		
	for(i = Lsensor ; i < LineStop; i++) {
		if(t==0){
			if( BinarizationData[i] ){					/* 左から最初の白 */
				Lsensor = i;
				t = 1;
			}
		}else if(t==1){
			if( !BinarizationData[i] ){					/* 左から最初の黒 */			
				Rsensor = i;
				t = 2;
			}
		}
	}
	if(White > White_min){
		Wide = Rsensor - Lsensor;					/* 幅を求める */	
	}else{
		Wide = 0;									/* 黒一面 */
	}
}
/************************************************************************/
/* 白線の中心を測定                                                       */
/* 引数　 なし                                                          */
/* 戻り値 なし			                                                 */
/************************************************************************/
void WhiteLineCenter(void)
{
	if(White > White_min){
		Center = (Lsensor + Rsensor)/2 - 64;				/* 重心を求める */	
	}else{
		Center = 0;								/* 黒一面 */
	}
}
/************************************************************************/
/* キャリブレーション                                                   */
/* 引数　 なし                                                          */
/* 戻り値 なし			                                                */
/************************************************************************/
void Calibration(void)
{
	int i;
	printf("Calibration now......\n");
	while(Max < White_Max){
		ImageCapture();
		for(i=0; i < EXPOSURE_timer ;i++) asm("nop");
		ImageCapture();
		EXPOSURE_timer += 1;
	}
	printf("EXPOSURE_timer = %d\n",EXPOSURE_timer);
}
/************************************************************************/
/* 露光時間調整                                                         */
/* 引数　 なし                                                          */
/* 戻り値 なし　　　　　                                                */
/************************************************************************/
void expose( void )
{
	int i;
	
//	if( Wide != 0 && !(White >= 90)){//黒でなく白でもない
//printf("White_Max = %d Max = %d\n",White_Max,Max);
//		if(Max < White_Max){
//			EXPOSURE_timer += (White_Max - Max)*10;
//			EXPOSURE_timer += 10;
//		}else{
//			EXPOSURE_timer -= (Max - White_Max)*10;
//			EXPOSURE_timer -= 10;
//		}
//		EXPOSURE_timer += (White_Max - Max);
//	}
//	if( EXPOSURE_timer > 1000) EXPOSURE_timer = 1000;
//	if( EXPOSURE_timer < 0 ) EXPOSURE_timer = 0;
	
	for(i=0;i<EXPOSURE_timer;i++) asm("nop");

}
/************************************************************************/
/* RAWデータの表示	                                                   */
/* 引数　 なし                                                          */
/* 戻り値 なし			                                                 */
/************************************************************************/
void raw_view(void)
{
	int i;
	for(i = LineStart;i < LineStop; i++){
		printf("%d ",ImageData[i]);
	}
	printf("\n\n");

}
/************************************************************************/
/* ２値化データの表示	                                                   */
/* 引数　 なし                                                          */
/* 戻り値 なし			                                                 */
/************************************************************************/
void bi_view(void)
{
	int i;
	for(i = LineStart;i < LineStop; i++){
		if(BinarizationData[i] == 1){
			printf("*");
		}else{
			printf(" ");
		}
	}
	printf("Exposure = %d Max = %d Min = %d Ave = %d Wide = %d Center = %d\n",EXPOSURE_timer,Max,Min,Ave,Wide,Center);
}
/************************************************************************/
/* センサ状態検出                                                       */
/* 引数　 マスク値                                                      */
/* 戻り値 センサ値                                                      */
/************************************************************************/
unsigned char sensor_inp( unsigned char mask )
{
    return (sensor8 & mask);
}
/************************************************************************/
/* PID				                                                    */
/* 引数　 なし                                                          */
/* 戻り値 なし			                                                */
/************************************************************************/
int PID(void)
{
	static float	iCenter = 0.0;
	static float	preCenter = 0.0;
	float			h;
//	Center /= 2;
	iCenter +=  (float)Center - preCenter;
	h = (float)Center * Kp + iCenter * Ki + ((float)Center - preCenter) * Kd;
	preCenter = (float)Center;
	
	return h;
}
/************************************************************************/
/* スタートバー検出センサ読み込み                                       */
/* 戻り値 センサ値 ON(バーあり):1 OFF(なし):0                           */
/************************************************************************/
unsigned char startbar_get( void )
{
	if (White > 80){
		 return 1;
	}else{
		return  0;
	}
}
/************************************************************************/
/* A/D値読み込み(AN7)                                                   */
/* 引数　 なし                                                          */
/* 戻り値 A/D値 0～1023                                                 */
/************************************************************************/
int get_ad7( void )
{
    int i;

    /* A/Dコンバータの設定 */
    admod   = 0x03;                     /* 単発モードに設定             */
    adinsel = 0x07;                     /* 入力端子AN7(P0_0)を選択      */
    adcon1  = 0x30;                     /* A/D動作可能                  */
    asm(" nop ");                       /* φADの1サイクルウエイト入れる*/
    adcon0  = 0x01;                     /* A/D変換スタート              */

    while( adcon0 & 0x01 );             /* A/D変換終了待ち              */

    i = ad7;

    return i;
}
