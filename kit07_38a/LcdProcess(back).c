/*======================================*/
/* �C���N���[�h                         */
/*======================================*/
#include <stdio.h>
#include "lcd_lib.h"                    /* LCD�\���p�ǉ�                */
#include "switch_lib.h"                 /* �X�C�b�`�ǉ�                 */
#include "data_flash_lib.h"             /* �f�[�^�t���b�V�����C�u����   */

/* DataFlash�֘A */

#define DF_ADDR_START   0x3000          /* �������݊J�n�A�h���X         */
#define DF_ADDR_END     0x33ff          /* �������ݏI���A�h���X         */
#define DF_PARA_SIZE    64              /* DataFlash�p�����[�^��        baisuudefuyasu*/

#define DF_CHECK        0x00            /* DataFlash�`�F�b�N            */

extern unsigned char sensor_inp( unsigned char mask );
extern void handle( int angle );


extern signed char     data_buff[ DF_PARA_SIZE ];
extern unsigned long   cnt_lcd;                /* LCD�����Ŏg�p                */
extern int             pattern;                /* �p�^�[���ԍ�                 */
extern int             servo_center;           /* �T�[�{�Z���^�l               */

/* LCD�֘A */
int             lcd_pattern = 2;

/************************************************************************/
/* DataFlash�̃p�����[�^�ǂݍ���                                        */
/* ����         �Ȃ�                                                    */
/* �߂�l       �Ȃ�                                                    */
/************************************************************************/
void readDataFlashParameter( void )
{
    int             i;
    unsigned int    st = DF_ADDR_END + 1 - DF_PARA_SIZE;
    signed char     c;

    while( 1 ) {
        // �ǂݍ��ޔԒn��T��
        readDataFlash( st, &c, 1 );
        if( c == 0x11 ) {
            readDataFlash( st, data_buff, DF_PARA_SIZE );
            break;
        }
        st -= DF_PARA_SIZE;

        if( st < DF_ADDR_START ) {
            // �Y�������@���߂Ďg�p
            for( i=0; i<DF_PARA_SIZE; i++ ) data_buff[ i ] = 0;
            data_buff[DF_CHECK]     = 0x11;
            data_buff[DF_SERVO1]    = SERVO_CENTER >> 8;
            data_buff[DF_SERVO2]    = SERVO_CENTER & 0xff;
            data_buff[DF_PWM]       = 100;
            data_buff[DF_CRANK_PWM] = 40;
            data_buff[SERVO1_PWM] 	= 10;
            data_buff[SERVO2_PWM] 	= 25;
			data_buff[SERVO3_PWM]   = 35;
            data_buff[SERVO4_PWM] 	= 10;
            data_buff[SERVO5_PWM] 	= 25;
			data_buff[DF_MOTOR_R]   = 50;
			data_buff[DF_crank_motor1] = 0;
			data_buff[DF_crank_motor2] = 0;
			data_buff[DF_lane_PWM] = 0;
			data_buff[DF_lane_motorL] = 0;
			data_buff[DF_lane_motorR] = 0;
			data_buff[DF_lane_motorS] = 0;
			data_buff[CL_BR_TM] = 10;
			data_buff[DF_crank_handlepwm] = 0;
			data_buff[DF_CL_BR1] = 0;
			data_buff[DF_CL_BR2] = 0;
			data_buff[DF_CL_BR3] = 0;
			data_buff[DF_CL_BR4] = 0;
			data_buff[DF_CL_BR5] = 0;
			data_buff[DF_CL_SP1] = 0;
			data_buff[DF_CL_SP2] = 0;
			data_buff[DF_CL_SP3] = 0;
			data_buff[DF_CL_SP4] = 0;
			//data_buff[DF_MOTOR_L]      = 50;
			i = 3000;
            data_buff[DF_STOP1] 	= i >> 8;
            data_buff[DF_STOP2] 	= i & 0xff;

            blockEraseDataFlash( DF_ADDR_START );
            writeDataFlash( DF_ADDR_START, data_buff, DF_PARA_SIZE );
            break;
        }
    }
}
/************************************************************************/
/* DataFlash�փp�����[�^��������                                        */
/* ����         �Ȃ�                                                    */
/* �߂�l       �Ȃ�                                                    */
/************************************************************************/
void writeDataFlashParameter( void )
{
    unsigned int    st = DF_ADDR_START;
    signed char     c;

    while( 1 ) {
        // �������ޔԒn��T��
        readDataFlash( st, &c, 1 );
        if( c == -1 ) {
            writeDataFlash( st, data_buff, DF_PARA_SIZE );
            break;
        }

        st += DF_PARA_SIZE;

        if( st > DF_ADDR_END ) {
            // ���ׂĎg�p������A�C���[�Y���Đ擪�ɏ�������
            blockEraseDataFlash( DF_ADDR_START );
            writeDataFlash( DF_ADDR_START, data_buff, DF_PARA_SIZE );
            break;
        }
    }
}
/************************************************************************/
/* LCD�ƃX�C�b�`���g�����p�����[�^�Z�b�g����                            */
/* ����         �Ȃ�                                                    */
/* �߂�l       �Ȃ�                                                    */
/************************************************************************/
int lcdProcess( void )
{
    int i;

    if( pattern != 0 ) {
        if( cnt_lcd >= 250 ) {
            cnt_lcd = 0;
            lcdPosition( 0, 0 );
                     /* 0123456789abcbef 1�s16���� */
            lcdPrintf( "pattern = %3d   ", pattern );
                     /* 01234567..89abcde.f 1�s16���� */
            lcdPrintf( "sensor=%02x bar=%d ",
                        sensor_inp( 0xff ), startbar_get() );
        }
        return;
    }

    /* �X�C�b�`4�@�ݒ�l�ۑ� */
    if( getSwFlag(SW_4) ) {
        // �p�����[�^�ۑ�
        writeDataFlashParameter();
    }

    /* �X�C�b�`3�@���j���[�{�P */
    if( getSwFlag(SW_3) ) {
        lcd_pattern++;
		lcdPosition( 0, 0 );
		lcdPrintf( "                ", i );
		lcdPrintf( "                ", i );
        if( lcd_pattern == 40 ) lcd_pattern = 1;
    }

    /* �X�C�b�`2�@���j���[�|�P */
    if( getSwFlag(SW_2) ) {
        lcd_pattern--;
		lcdPosition( 0, 0 );
		lcdPrintf( "                ", i );
		lcdPrintf( "                ", i );

        if( lcd_pattern == 0 ) lcd_pattern = 39;
    }

    /* LCD�A�X�C�b�`���� */
    switch( lcd_pattern ) {
    case 1:
        /* �T�[�{�Z���^�l���� */
        if( getSwFlag(SW_1) ) {
            servo_center++;
            if( servo_center > 10000 ) servo_center = 10000;
        }
        if( getSwFlag(SW_0) ) {
            servo_center--;
            if( servo_center < 1000 ) servo_center = 1000;
        }
        data_buff[DF_SERVO1] = servo_center >> 8;
        data_buff[DF_SERVO2] = servo_center & 0xff;
        handle( 0 );

        /* LCD���� */
        lcdPosition( 0, 0 );
                 /* 0123456789ab..f 1�s16���� */
        lcdPrintf( "01 servo = %05d", servo_center );
                 /* 01234567..89abcde.f 1�s16���� */
        lcdPrintf( "sensor=%02x bar=%d ",
                    sensor_inp( 0xff ), startbar_get() );
        break;

    case 2:
        /* PWM�l���� */
        i = data_buff[DF_PWM];
        if( getSwFlag(SW_1) ) {
            i++;
            if( i > 100 ) i = 100;
        }
        if( getSwFlag(SW_0) ) {
            i--;
            if( i < 0 ) i = 0;
        }
        data_buff[DF_PWM] = i;

        /* LCD���� */
        lcdPosition( 0, 0 );
                 /* 0123456789..bcdef0123456789..bcdef 1�s16���� */
        lcdPrintf( "02 Max SPEED         %03d    ", i );
                 /* 01234567..89abcde.f 1�s16���� */
        break;

    case 3:	
        i = data_buff[Expose_Time];
        if( getSwFlag(SW_1) ) {
            i++;
            if( i > 120 ) i = 120;
        }
        if( getSwFlag(SW_0) ) {
            i--;
            if( i < 0 ) i = 0;
        }
        data_buff[Expose_Time] = i;
		
        /* LCD���� */
        lcdPosition( 0, 0 );
                 /* 0123456789abcdef0123456789abcdef 1�s16���� */
        lcdPrintf( "03 expose time      %d00      ", i );
//        lcdPrintf( "03 LaneChange   Right2   %03d ", i );
//        lcdPrintf( "03 not used                     ");
        break;

    case 4:
		i = data_buff[TH_Number];
        if( getSwFlag(SW_1) ) {
            i++;
            if( i > 8 ) i = 8;
        }
        if( getSwFlag(SW_0) ) {
            i--;
            if( i < 0 ) i = 0;
        }
        data_buff[TH_Number] = i;
		
        /* LCD���� */
        lcdPosition( 0, 0 );
                 /* 0123456789abcdef0123456789abcdef 1�s16���� */
        lcdPrintf( "04 th number         %d        ", i );
//        lcdPrintf( "04 LaneChange   Left 2   %03d ", i );
//        lcdPrintf( "04 not used                     ");
        break;

    case 5:
        i = data_buff[SERVO1_PWM];
        if( getSwFlag(SW_1) ) {
            i++;
            if( i > 45 ) i = 45;
        }
        if( getSwFlag(SW_0) ) {
            i--;
            if( i < 0 ) i = 0;
        }
        data_buff[SERVO1_PWM] = i;

        /* LCD���� */
        lcdPosition( 0, 0 );
                 /* 0123456789abcdef 1�s16���� */
        lcdPrintf( "05   *** OO*    " );
                 /* 0123456789abcdef 1�s16���� */
        lcdPrintf( "Little  %03d", i );
        break;
		
    case 6:
        i = data_buff[SERVO2_PWM];
        if( getSwFlag(SW_1) ) {
            i++;
            if( i > 45 ) i = 45;
        }
        if( getSwFlag(SW_0) ) {
            i--;
            if( i < 0 ) i = 0;
        }
        data_buff[SERVO2_PWM] = i;

        /* LCD���� */
        lcdPosition( 0, 0 );
                 /* 0123456789abcdef 1�s16���� */
        lcdPrintf( "06   *** *O*    " );
                 /* 0123456789abcdef 1�s16���� */
        lcdPrintf( "Middle     %03d", i );
        break;
	case 7:
        i = data_buff[SERVO3_PWM];
        if( getSwFlag(SW_1) ) {
            i++;
            if( i > 45 ) i = 45;
        }
        if( getSwFlag(SW_0) ) {
            i--;
            if( i < 0 ) i = 0;
        }
//        data_buff[SERVO3_PWM] = i;

        /* LCD���� */
        lcdPosition( 0, 0 );
                 /* 0123456789abcdef 1�s16���� */
        lcdPrintf( "07   *** OOO    " );
                 /* 0123456789abcdef 1�s16���� */
//        lcdPrintf( "mMiddle      %03d", i );
        lcdPrintf( "   not used     ");
        break;
		
	case 8:
		i = data_buff[SERVO4_PWM];
		if( getSwFlag(SW_1) ) {
            i++;
            if( i >  45) i = 45;
        }
        if( getSwFlag(SW_0) ) {
           	i--;
           	if( i < 0 ) i = 0;
       	}
		data_buff[SERVO4_PWM] = i;

        /* LCD���� */
        lcdPosition( 0, 0 );
                 /* 0123456789abcdef 1�s16���� */
        lcdPrintf( "08   *** *OO    " );
                 /* 0123456789abcdef 1�s16���� */
        lcdPrintf( "Big        %03d", i );
		break;
	case 9:
		i = data_buff[SERVO5_PWM];
		if( getSwFlag(SW_1) ) {
            i++;
            if( i >  45) i = 45;
        }
        if( getSwFlag(SW_0) ) {
           	i--;
           	if( i < 0 ) i = 0;
       	}
		data_buff[SERVO5_PWM] = i;
        /* LCD���� */
        lcdPosition( 0, 0 );
                 /* 0123456789abcdef 1�s16���� */
        lcdPrintf( "09   *** **O    " );
                 /* 0123456789abcdef 1�s16���� */
        lcdPrintf( "bBig       %03d", i );
		break;
		
    case 10:
        /* �^�C�}�[�l���� */
		i = ((unsigned char)data_buff[DF_STOP1]*0x100)|(unsigned char)data_buff[DF_STOP2];
        if( getSwFlag(SW_1) ) {
            i+=100;
            if( i > 9000 ) i = 9000;
        }
        if( getSwFlag(SW_0) ) {
            i-=100;
            if( i < 0 ) i = 0;
        }
        data_buff[DF_STOP1] = i >> 8;
        data_buff[DF_STOP2] = i & 0xff;

        /* LCD���� */
        lcdPosition( 0, 0 );
                 /* 0123456789abcdef0123456789abcdef 1�s16���� */
        lcdPrintf( "10    stop =       %04d ", i );
        break;
	
	case 11://�N�����N�����[�^�[�@OUT
			i = data_buff[DF_crank_motor1];
        	if( getSwFlag(SW_1) ) {
            	i++;
            	if( i > 100 ) i = 100;
        	}
        	if( getSwFlag(SW_0) ) {
            	i--;
            	if( i < 0 ) i = 0;
        	}
        	data_buff[DF_crank_motor1] = i;

        /* LCD���� */
        lcdPosition( 0, 0 );
                 /* 0123456789abcdef0123456789abcdef 1�s16���� */
        lcdPrintf( "11 Crank motor  OUT   %03d", i );
        break;
	case 12://�N�����N�����[�^�[�@IN
			i = data_buff[DF_crank_motor2];
        	if( getSwFlag(SW_1) ) {
            	i++;
            	if( i > 100 ) i = 100;
        	}
        	if( getSwFlag(SW_0) ) {
            	i--;
            	if( i < -100 ) i = -100;
        	}
        	data_buff[DF_crank_motor2] = i;

        /* LCD���� */
        lcdPosition( 0, 0 );
                 /* 0123456789abcdef0123456789abcdef 1�s16���� */
        lcdPrintf( "12 Crank motor  IN   %03d", i );
        break;
	case 13:
			i = data_buff[DF_lane_motorR];
        	if( getSwFlag(SW_1) ) {
            	i++;
            	if( i > 100 ) i = 100;
        	}
        	if( getSwFlag(SW_0) ) {
            	i--;
            	if( i < 0 ) i = 0;
        	}
        	data_buff[DF_lane_motorR] = i;

        /* LCD���� */
        lcdPosition( 0, 0 );
                 /* 0123456789abcdef0123456789abcdef 1�s16���� */
        lcdPrintf( "13 LaneChange   Motor IN  %03d", i );
        break;
		
	case 14:
			i = data_buff[DF_lane_motorL];
        	if( getSwFlag(SW_1) ) {
            	i++;
            	if( i > 100 ) i = 100;
        	}
        	if( getSwFlag(SW_0) ) {
            	i--;
            	if( i < 0 ) i = 0;
        	}
        	data_buff[DF_lane_motorL] = i;

        /* LCD���� */
        lcdPosition( 0, 0 );
                 /* 0123456789abcdef0123456789abcdef 1�s16���� */
        lcdPrintf( "14 LaneChange   Motor OUT %03d", i );
        break;

	case 15:
			i = data_buff[DF_lane_motorS];
        	if( getSwFlag(SW_1) ) {
            	i++;
            	if( i > 100 ) i = 100;
        	}
        	if( getSwFlag(SW_0) ) {
            	i--;
            	if( i < 0 ) i = 0;
        	}
        	data_buff[DF_lane_motorS] = i;

        /* LCD���� */
        lcdPosition( 0, 0 );
                 /* 0123456789abcdef0123456789abcdef 1�s16���� */
        lcdPrintf( "15 LaneChange   Speed    %3d", i );
        break;
	case 16:
			i = data_buff[DF_lane_PWM];
        	if( getSwFlag(SW_1) ) {
            	i++;
            	if( i > 45 ) i = 45;
        	}
        	if( getSwFlag(SW_0) ) {
            	i--;
            	if( i < 0 ) i = 0;
        	}
        	data_buff[DF_lane_PWM] = i;

        /* LCD���� */
        lcdPosition( 0, 0 );
                 /* 0123456789abcdef0123456789abcdef 1�s16���� */
        lcdPrintf( "16 LaneChange   handle   %03d", i );
        break;
	case 17:
		i = data_buff[CL_BR_TM];
        if( getSwFlag(SW_1) ) {
            i++;
            if( i > 100 ) i = 100;
        }
        if( getSwFlag(SW_0) ) {
            i--;
            if( i < 0 ) i = 0;
        }
//        data_buff[CL_BR_TM] = i;
         /* LCD���� */
        lcdPosition( 0, 0 );
                 /* 0123456789abcdef0123456789abcdef 1�s16���� */
//        lcdPrintf( "16 Crank Brak   Time     %03d", i );
        lcdPrintf( "17 not used                    ");
       break;
	case 18:
		i = data_buff[DF_crank_handlepwm];
        if( getSwFlag(SW_1) ) {
            i++;
            if( i > 45 ) i = 45;
        }
        if( getSwFlag(SW_0) ) {
           	i--;
           	if( i < 0 ) i = 0;
       	}
        	data_buff[DF_crank_handlepwm] = i;

        /* LCD���� */
        lcdPosition( 0, 0 );
                 /* 0123456789abcdef0123456789abcdef 1�s16���� */
        lcdPrintf( "18 Crank handle     %03d", i );
        break;
	case 19://
		i = data_buff[DF_CL_BR1];
		if( getSwFlag(SW_1) ) {
            i++;
            if( i >  0) i = 0;
        }
        if( getSwFlag(SW_0) ) {
           	i--;
           	if( i < -100 ) i = -100;
       	}
		data_buff[DF_CL_BR1] = i;
		lcdPosition( 0, 0 );
                 /* 0123456789abcdef0123456789abcdef 1�s16���� */
		lcdPrintf( "19 Crank Brake  01   %03d      ", i );
		break;
		
	case 20:
		i = data_buff[DF_CL_BR2];
		if( getSwFlag(SW_1) ) {
            i++;
            if( i >  0) i = 0;
        }
        if( getSwFlag(SW_0) ) {
           	i--;
           	if( i < -100 ) i = -100;
       	}
		data_buff[DF_CL_BR2] = i;
		lcdPosition( 0, 0 );
                 /* 0123456789abcdef0123456789abcdef 1�s16���� */
		lcdPrintf( "20 Crank Brake  02   %03d      ", i );
		break;
	case 21:
		i = data_buff[DF_CL_BR3];
		if( getSwFlag(SW_1) ) {
            i++;
            if( i >  0) i = 0;
        }
        if( getSwFlag(SW_0) ) {
           	i--;
           	if( i < -100 ) i = -100;
       	}
		data_buff[DF_CL_BR3] = i;
		lcdPosition( 0, 0 );
                 /* 0123456789abcdef0123456789abcdef 1�s16���� */
		lcdPrintf( "21 Crank Brake  03   %03d      ", i );
		break;
	case 22:
		i = data_buff[DF_CL_BR4];
		if( getSwFlag(SW_1) ) {
            i++;
            if( i >  0) i = 0;
        }
        if( getSwFlag(SW_0) ) {
           	i--;
           	if( i < -100 ) i = -100;
       	}
		data_buff[DF_CL_BR4] = i;
		lcdPosition( 0, 0 );
                 /* 0123456789abcdef0123456789abcdef 1�s16���� */
		lcdPrintf( "22 Crank Brake  04   %03d      ", i );
		break;
	case 23:
		i = data_buff[DF_CL_BR5];
		if( getSwFlag(SW_1) ) {
            i++;
            if( i >  0) i = 0;
        }
        if( getSwFlag(SW_0) ) {
           	i--;
           	if( i < -100 ) i = -100;
       	}
		data_buff[DF_CL_BR5] = i;
		lcdPosition( 0, 0 );
                 /* 0123456789abcdef0123456789abcdef 1�s16���� */
		lcdPrintf( "23 Crank Brake  05   %03d      ", i );
		break;
	case 24:
		i = data_buff[DF_CL_SP1];
		if( getSwFlag(SW_1) ) {
            i++;
            if( i >  100) i = 100;
        }
        if( getSwFlag(SW_0) ) {
           	i--;
           	if( i < 0 ) i = 0;
       	}
//		data_buff[DF_CL_SP1] = i;
		lcdPosition( 0, 0 );
                 /* 0123456789abcdef0123456789abcdef 1�s16���� */
//		lcdPrintf( "24 Crank Speed  01   %03d      ", i );
        lcdPrintf( "24 not used                     ");
		break;
	case 25:
		i = data_buff[DF_CL_SP2];
		if( getSwFlag(SW_1) ) {
            i++;
            if( i >  100) i = 100;
        }
        if( getSwFlag(SW_0) ) {
           	i--;
           	if( i < 0 ) i = 0;
       	}
//		data_buff[DF_CL_SP2] = i;
		lcdPosition( 0, 0 );
                 /* 0123456789abcdef0123456789abcdef 1�s16���� */
//		lcdPrintf( "25 Crank Speed  02   %03d      ", i );
        lcdPrintf( "25 not used                     ");
		break;
	case 26:
		i = data_buff[DF_CL_SP3];
		if( getSwFlag(SW_1) ) {
            i++;
            if( i >  100) i = 100;
        }
        if( getSwFlag(SW_0) ) {
           	i--;
           	if( i < 0 ) i = 0;
       	}
//		data_buff[DF_CL_SP3] = i;
		lcdPosition( 0, 0 );
                 /* 0123456789abcdef0123456789abcdef 1�s16���� */
//		lcdPrintf( "26 Crank Speed  03   %03d      ", i );
        lcdPrintf( "26 not used                     ");
		break;
	case 27:
		i = data_buff[DF_CL_SP4];
		if( getSwFlag(SW_1) ) {
            i++;
            if( i >  100) i = 100;
        }
        if( getSwFlag(SW_0) ) {
           	i--;
           	if( i < 0 ) i = 0;
       	}
//		data_buff[DF_CL_SP4] = i;
		lcdPosition( 0, 0 );
                 /* 0123456789abcdef0123456789abcdef 1�s16���� */
//		lcdPrintf( "27 Crank Speed  04   %03d      ", i );
        lcdPrintf( "27 not used                     ");
		break;
	case 28:
		i = data_buff[DF_CL_SP5];
		if( getSwFlag(SW_1) ) {
            i++;
            if( i >  100) i = 100;
        }
        if( getSwFlag(SW_0) ) {
           	i--;
           	if( i < 0 ) i = 0;
       	}
//		data_buff[DF_CL_SP5] = i;
		lcdPosition( 0, 0 );
                 /* 0123456789abcdef0123456789abcdef 1�s16���� */
//		lcdPrintf( "28 Crank Speed  05   %03d      ", i );
        lcdPrintf( "28 not used                     ");
		break;
	case 29:
		i = data_buff[DF_CL_NUM];
		if( getSwFlag(SW_1) ) {
            i++;
            if( i >  5) i = 5;
        }
        if( getSwFlag(SW_0) ) {
           	i--;
           	if( i < 1 ) i = 1;
       	}
		data_buff[DF_CL_NUM] = i;
		lcdPosition( 0, 0 );
                 /* 0123456789abcdef0123456789abcdef 1�s16���� */
		lcdPrintf( "29 Crank Number    %03d      ", i );
		break;
		
	case 30:
		i = data_buff[DF_MOTOR1];
		if( getSwFlag(SW_1) ) {
            i++;
            if( i >  100) i = 100;
        }
        if( getSwFlag(SW_0) ) {
           	i--;
           	if( i < -100 ) i = -100;
       	}
		data_buff[DF_MOTOR1] = i;
		lcdPosition( 0, 0 );
                 /* 0123456789abcdef0123456789abcdef 1�s16���� */
		lcdPrintf( "Littel Curve OUT    %03d   ", i );
		break;
	case 31:
		i = data_buff[DF_MOTOR2];
		if( getSwFlag(SW_1) ) {
            i++;
            if( i >  100) i = 100;
        }
        if( getSwFlag(SW_0) ) {
           	i--;
           	if( i < -100 ) i = -100;
       	}
		data_buff[DF_MOTOR2] = i;
		lcdPosition( 0, 0 );
                 /* 0123456789abcdef0123456789abcdef 1�s16���� */
		lcdPrintf( "Littel Curve IN     %03d   ", i );
		break;
	case 32:
		i = data_buff[DF_MOTOR3];
		if( getSwFlag(SW_1) ) {
            i++;
            if( i >  100) i = 100;
        }
        if( getSwFlag(SW_0) ) {
           	i--;
           	if( i < -100 ) i = -100;
       	}
		data_buff[DF_MOTOR3] = i;
		lcdPosition( 0, 0 );
                 /* 0123456789abcdef0123456789abcdef 1�s16���� */
		lcdPrintf( "Middle Curve     OUT   %03d   ", i );
		break;
	case 33:
		i = data_buff[DF_MOTOR4];
		if( getSwFlag(SW_1) ) {
            i++;
            if( i >  100) i = 100;
        }
        if( getSwFlag(SW_0) ) {
           	i--;
           	if( i < -100 ) i = -100;
       	}
		data_buff[DF_MOTOR4] = i;
		lcdPosition( 0, 0 );
                 /* 0123456789abcdef0123456789abcdef 1�s16���� */
		lcdPrintf( "Middle Curve    IN     %03d   ", i );
		break;
	case 34:
		i = data_buff[DF_MOTOR5];
		if( getSwFlag(SW_1) ) {
            i++;
            if( i >  100) i = 100;
        }
        if( getSwFlag(SW_0) ) {
           	i--;
           	if( i < -100 ) i = -100;
       	}
		data_buff[DF_MOTOR5] = i;
		lcdPosition( 0, 0 );
                 /* 0123456789abcdef0123456789abcdef 1�s16���� */
		lcdPrintf( "mMiddle Curve   OUT    %03d", i );
		break;
	case 35:
		i = data_buff[DF_MOTOR6];
		if( getSwFlag(SW_1) ) {
            i++;
            if( i >  100) i = 100;
        }
        if( getSwFlag(SW_0) ) {
           	i--;
           	if( i < -100 ) i = -100;
       	}
		data_buff[DF_MOTOR6] = i;
		lcdPosition( 0, 0 );
                 /* 0123456789abcdef0123456789abcdef 1�s16���� */
		lcdPrintf( "mMiddle Curve   IN     %03d", i );
		break;
	case 36:
		i = data_buff[DF_MOTOR7];
		if( getSwFlag(SW_1) ) {
            i++;
            if( i >  100) i = 100;
        }
        if( getSwFlag(SW_0) ) {
           	i--;
           	if( i < -100 ) i = -100;
       	}
		data_buff[DF_MOTOR7] = i;
		lcdPosition( 0, 0 );
                 /* 0123456789abcdef0123456789abcdef 1�s16���� */
		lcdPrintf( "Big Curve OUT       %03d  ", i );
		break;
	case 37:
		i = data_buff[DF_MOTOR8];
		if( getSwFlag(SW_1) ) {
            i++;
            if( i >  100) i = 100;
        }
        if( getSwFlag(SW_0) ) {
           	i--;
           	if( i < -100 ) i = -100;
       	}
		data_buff[DF_MOTOR8] = i;
		lcdPosition( 0, 0 );
                 /* 0123456789abcdef0123456789abcdef 1�s16���� */
		lcdPrintf( "Big Curve IN       %03d  ", i );
		break;
	case 38:
		i = data_buff[DF_MOTOR9];
		if( getSwFlag(SW_1) ) {
            i++;
            if( i >  100) i = 100;
        }
        if( getSwFlag(SW_0) ) {
           	i--;
           	if( i < -100 ) i = -100;
       	}
		data_buff[DF_MOTOR9] = i;
		lcdPosition( 0, 0 );
                 /* 0123456789abcdef0123456789abcdef 1�s16���� */
		lcdPrintf( "bBig Curve OUT      %03d  ", i );
		break;
	case 39:
		i = data_buff[DF_MOTOR10];
		if( getSwFlag(SW_1) ) {
            i++;
            if( i >  100) i = 100;
        }
        if( getSwFlag(SW_0) ) {
           	i--;
           	if( i < -100 ) i = -100;
       	}
		data_buff[DF_MOTOR10] = i;
		lcdPosition( 0, 0 );
                 /* 0123456789abcdef0123456789abcdef 1�s16���� */
		lcdPrintf( "bBig Curve IN       %03d", i );
		break;
	}
}