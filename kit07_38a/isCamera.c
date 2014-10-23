/*======================================*/
/* �C���N���[�h                         */
/*======================================*/
#include <stdio.h>
#include "sfr_r838a.h"                  /* R8C/38A SFR�̒�`�t�@�C��    */
#include "printf_lib.h"                 /* printf�g�p���C�u����         */
#include "microsd_lib.h"                /* microSD���䃉�C�u����        */
#include "lcd_lib.h"                    /* LCD�\���p�ǉ�                */
#include "switch_lib.h"                 /* �X�C�b�`�ǉ�                 */
#include "data_flash_lib.h"             /* �f�[�^�t���b�V�����C�u����   */
#include "isCamera.h"
//#include "drive.h"

/* TAOS TSL1401CL */
#define	TAOS_SI_HIGH	p0_addr.bit.b1 = 1	/* Port P0_1 bit */
#define	TAOS_SI_LOW		p0_addr.bit.b1 = 0	/* Port P0_1 bit */
#define	TAOS_CLK_HIGH	p0_addr.bit.b2 = 1	/* Port P0_2 bit */
#define	TAOS_CLK_LOW	p0_addr.bit.b2 = 0	/* Port P0_2 bit */

#define White_min	9					/* ���F�̍ŏ��l 			*/
#define	White_Max	150					/* ���C�����FMAX�l�̐ݒ� */

int 			ImageData[128];			/* �J�����̒l				*/
int				BinarizationData[130];	/* �Q�l��					*/
int   			EXPOSURE_timer = 1;	/* �I������					*/
int 			LineStart,LineStop;		/* �ǂݎ��ʒu�̎n�߂ƏI��� */
int				Max,Min,Ave;			/*�J�����ǂݎ��ő�l�A�ŏ��l�A���ϒl*/
unsigned int 	Rsensor;				/* ���C���̉E�[ */
unsigned int 	Lsensor;				/* ���C���̍��[ */
unsigned int 	Wide;					/* ���C���̕� */
int			 	Center;					/* ���C���̏d�S */
int 			White;					/* ���F�̌�	*/
int				pid_angle;
int 			sensor8;

/************************************************************************/
/* �Z���T�[�ǂݎ��                                             */
/* �����@ �Ȃ�                                                          */
/* �߂�l �Ȃ�			                                                */
/************************************************************************/
void sensor_process(void) {
	
	int i;
	int s;
	
	ImageCapture();
	expose();
	ImageCapture();				/* �C���[�W�L���v�`�� 	*/
	binarization();				/* �Q�l��				*/
	WhiteLineWide();			/* �����̕��𑪒�		*/
	WhiteLineCenter();			/* �����̒��S�𑪒�		*/

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
/* �J�����̏�����                                                */
/* �����@ �J�n�ʒu�A�I���ʒu                                                          */
/* �߂�l �Ȃ�                                                 */
/************************************************************************/
void initCamera(int Start,int Stop)
{
	LineStart = Start;
	LineStop = Stop;
}
/************************************************************************/
/* �C���[�W�L���v�`��                                                 */
/* �����@ �J�n�ʒu�A�I���ʒu                                                          */
/* �߂�l �Ȃ�                                                 */
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
/* �Q�l��                                                               */
/* �����@ �Ȃ�                                                          */
/* �߂�l �Ȃ�			                                                */
/************************************************************************/
void binarization(void)
{
	int i;

	/* �ō��l�ƍŒ�l����Ԃ̒l�����߂� */
	Ave = (Max + Min) * 2 / 3;
	/* ���͂O�@���͂P�ɂ��� */
	White = 0;					/* ���̐����O�ɂ��� */
	if( Max > 50 ){
		/* �����꒼���̂Ƃ� */
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
	/* ������ʂ̂Ƃ� */
	}else{
		for(i=1; i < 128; i++){
			BinarizationData[i] = 0;
		}
	}

}
/************************************************************************/
/* �����̕��𑪒�                                                       */
/* �����@ �Ȃ�                                                          */
/* �߂�l �Ȃ�			                                                 */
/************************************************************************/
void WhiteLineWide(void)
{
	int t=0,i;
		
	Lsensor = LineStart;
	Rsensor = LineStop;
		
	for(i = Lsensor ; i < LineStop; i++) {
		if(t==0){
			if( BinarizationData[i] ){					/* ������ŏ��̔� */
				Lsensor = i;
				t = 1;
			}
		}else if(t==1){
			if( !BinarizationData[i] ){					/* ������ŏ��̍� */			
				Rsensor = i;
				t = 2;
			}
		}
	}
	if(White > White_min){
		Wide = Rsensor - Lsensor;					/* �������߂� */	
	}else{
		Wide = 0;									/* ����� */
	}
}
/************************************************************************/
/* �����̒��S�𑪒�                                                       */
/* �����@ �Ȃ�                                                          */
/* �߂�l �Ȃ�			                                                 */
/************************************************************************/
void WhiteLineCenter(void)
{
	if(White > White_min){
		Center = (Lsensor + Rsensor)/2 - 64;				/* �d�S�����߂� */	
	}else{
		Center = 0;								/* ����� */
	}
}
/************************************************************************/
/* �L�����u���[�V����                                                   */
/* �����@ �Ȃ�                                                          */
/* �߂�l �Ȃ�			                                                */
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
/* �I�����Ԓ���                                                         */
/* �����@ �Ȃ�                                                          */
/* �߂�l �Ȃ��@�@�@�@�@                                                */
/************************************************************************/
void expose( void )
{
	int i;
	
//	if( Wide != 0 && !(White >= 90)){//���łȂ����ł��Ȃ�
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
/* RAW�f�[�^�̕\��	                                                   */
/* �����@ �Ȃ�                                                          */
/* �߂�l �Ȃ�			                                                 */
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
/* �Q�l���f�[�^�̕\��	                                                   */
/* �����@ �Ȃ�                                                          */
/* �߂�l �Ȃ�			                                                 */
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
/* �Z���T��Ԍ��o                                                       */
/* �����@ �}�X�N�l                                                      */
/* �߂�l �Z���T�l                                                      */
/************************************************************************/
unsigned char sensor_inp( unsigned char mask )
{
    return (sensor8 & mask);
}
/************************************************************************/
/* PID				                                                    */
/* �����@ �Ȃ�                                                          */
/* �߂�l �Ȃ�			                                                */
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
/* �X�^�[�g�o�[���o�Z���T�ǂݍ���                                       */
/* �߂�l �Z���T�l ON(�o�[����):1 OFF(�Ȃ�):0                           */
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
/* A/D�l�ǂݍ���(AN7)                                                   */
/* �����@ �Ȃ�                                                          */
/* �߂�l A/D�l 0�`1023                                                 */
/************************************************************************/
int get_ad7( void )
{
    int i;

    /* A/D�R���o�[�^�̐ݒ� */
    admod   = 0x03;                     /* �P�����[�h�ɐݒ�             */
    adinsel = 0x07;                     /* ���͒[�qAN7(P0_0)��I��      */
    adcon1  = 0x30;                     /* A/D����\                  */
    asm(" nop ");                       /* ��AD��1�T�C�N���E�G�C�g�����*/
    adcon0  = 0x01;                     /* A/D�ϊ��X�^�[�g              */

    while( adcon0 & 0x01 );             /* A/D�ϊ��I���҂�              */

    i = ad7;

    return i;
}
