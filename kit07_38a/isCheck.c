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
#include "isCheck.h"

/************************************************************************/
/* �N���X���C�����o����                                                 */
/* �߂�l 0:�N���X���C���Ȃ� 1:����                                     */
/************************************************************************/
int check_crossline( void )
{

    if( Wide > 80 ) {
        return 1;	/* �N���X���C�������I */
    }else{
		return 0;	/* �N���X���C���Ȃ�  */
	}
}

/************************************************************************/
/* �E�n�[�t���C�����o����                                               */
/* �߂�l 0:�Ȃ� 1:����                                                 */
/************************************************************************/
int check_rightline( void )
{
/*	printf("sval[14] = ");float_printf(sval[14],3);
	printf("sval[15] = ");float_printf(sval[15],3);
	printf("\n\n");
*/  if( (Wide > 30) && (Center > 0) ) {
        return 1;	/* �E�n�[�t���C�������I */
    }else{
		return 0;	/* �E�n�[�t���C���Ȃ�  */
	}
}

/************************************************************************/
/* ���n�[�t���C�����o����                                               */
/* �߂�l 0:�Ȃ� 1:����                                                 */
/************************************************************************/
int check_leftline( void )
{
/*	printf("sval[0] = ");float_printf(sval[0],3);
	printf("sval[1] = ");float_printf(sval[1],3);
	printf("\n\n");
*/  if( (Wide > 30) && (Center < 0) ) {
        return 1;	/* ���n�[�t���C�������I */
    }else{
		return 0;	/* ���n�[�t���C���Ȃ�  */
	}
}
