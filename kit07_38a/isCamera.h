#ifndef	ISCAMERA_H
#define	ISCAMERA_H

#define Kp -1.5
#define Ki -1.0
#define Kd -1.0

/* �}�X�N�l�ݒ� �~�F�}�X�N����(����)�@���F�}�X�N����(�L��) */
#define MASK2_2         0x66            /* �~�����~�~�����~             */
#define MASK2_0         0x60            /* �~�����~�~�~�~�~             */
#define MASK0_2         0x06            /* �~�~�~�~�~�����~             */
#define MASK3_3         0xe7            /* �������~�~������             */
#define MASK0_3         0x07            /* �~�~�~�~�~������             */
#define MASK3_0         0xe0            /* �������~�~�~�~�~             */
#define MASK4_0         0xf0            /* ���������~�~�~�~             */
#define MASK0_4         0x0f            /* �~�~�~�~��������             */
#define MASK4_4         0xff            /* ����������������             */
#define MASK_4_			0x3c			/* �~�~���������~�~				*/

extern int 				ImageData[128];
extern int 				sensor8;
extern unsigned int 	Wide;					/* ���C���̕� */
extern int			 	Center;					/* ���C���̏d�S */
extern int				pid_angle;

/* �v���g�^�C�v�錾 */
void			sensor_process(void);			/* �Z���T�[�ǂݎ��		*/
void			initCamera(int,int);			/* �J�����̏�����		*/
void			ImageCapture(void);				/* �C���[�W�L���v�`�� 	*/
void			binarization(void);				/* �Q�l��				*/
void 			WhiteLineWide(void);			/* �����̕��𑪒�		*/
void 			WhiteLineCenter(void);			/* �����̒��S�𑪒�		*/
void 			Calibration(void);				/* �L�����u���[�V����	*/
void 			expose( void );					/* �I�����Ԓ��� 		*/
void			raw_view(void);					/* RAW�f�[�^�̕\��		*/
void			bi_view(void);					/* �Q�l���f�[�^�̕\��	*/
unsigned char	sensor_inp(unsigned char);		/* �Z���T��Ԍ��o		*/
int 			PID(void);						/* PID					*/
unsigned char 	startbar_get( void );			/* �X�^�[�g�o�[���o		*/
int				get_ad7( void );				/* A/D�l�ǂݍ���(AN7)	*/

#endif