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
#include "drive.h"

/* 現在の状態保存用 */
int             handleBuff;             /* 現在のハンドル角度記録       */
int             leftMotorBuff;          /* 現在の左モータPWM値記録      */
int             rightMotorBuff;         /* 現在の右モータPWM値記録      */
int             angle_buff;             /* 現在ハンドル角度保持用   */
/* エンコーダ関連 */
int				iTimer10;			   	/* 10msカウント用		  	*/
int   			iEncoder;			   	/* 10ms毎の最新値		   	*/
unsigned long   lEncoderTotal;		  	/* 積算値保存用				*/
unsigned int	uEncoderBuff;		   	/* 計算用　割り込み内で使用 */
unsigned long	lEncoderCrank;		  	/* クロスライン検出時の積算値 	*/
unsigned long 	lEncoderCrank2;			/* クロスライン検出時の積算値2 	*/
unsigned long	lEncoderHarf;		  	/* ハーフライン検出時の積算値 	*/

int             servo_center;           /* サーボセンタ値               */


/************************************************************************/
/* モータ速度制御                                                       */
/* 引数　 左モータ:-100〜100、右モータ:-100〜100                        */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor( int accele_l, int accele_r )
{
    int    sw_data;

    accele_l = accele_l * data_buff[DF_PWM] / 100;
    accele_r = accele_r * data_buff[DF_PWM] / 100;

 	leftMotorBuff = accele_l;

    /* 左モータ制御 */
    if( accele_l >= 0 ) {
        p2 &= 0xfd;
        trdgrd0 = (long)( PWM_CYCLE - 1 ) * accele_l / 100;
    } else {
        p2 |= 0x02;
        trdgrd0 = (long)( PWM_CYCLE - 1 ) * ( -accele_l ) / 100;
    }
	
	rightMotorBuff = accele_r;

    /* 右モータ制御 */
    if( accele_r >= 0 ) {
        p2 &= 0xf7;
        trdgrc1 = (long)( PWM_CYCLE - 1 ) * accele_r / 100;
    } else {
        p2 |= 0x08;
        trdgrc1 = (long)( PWM_CYCLE - 1 ) * ( -accele_r ) / 100;
    }
}

/************************************************************************/
/* サーボハンドル操作                                                   */
/* 引数　 サーボ操作角度：-40〜40                                       */
/*        -40で左へ-40度、0でまっすぐ、40で右へ40度回転                  */
/************************************************************************/
void handle( int angle )
{
	if(angle >40)angle = 40;
	if(angle <-40)angle = -40;
	handleBuff = angle;
    angle_buff = angle;                 /* 現在の角度保存               */

    /* サーボが左右逆に動く場合は、「-」を「+」に替えてください */
    trdgrd1 = servo_center - angle * HANDLE_STEP;
}
