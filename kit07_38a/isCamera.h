#ifndef	ISCAMERA_H
#define	ISCAMERA_H

#define Kp -1.5
#define Ki -1.0
#define Kd -1.0

/* }XNlÝè ~F}XN è(³ø)@F}XN³µ(Lø) */
#define MASK2_2         0x66            /* ~~~~             */
#define MASK2_0         0x60            /* ~~~~~~             */
#define MASK0_2         0x06            /* ~~~~~~             */
#define MASK3_3         0xe7            /* ~~             */
#define MASK0_3         0x07            /* ~~~~~             */
#define MASK3_0         0xe0            /* ~~~~~             */
#define MASK4_0         0xf0            /* ~~~~             */
#define MASK0_4         0x0f            /* ~~~~             */
#define MASK4_4         0xff            /*              */
#define MASK_4_			0x3c			/* ~~~~				*/

extern int 				ImageData[128];
extern int 				sensor8;
extern unsigned int 	Wide;					/* CÌ */
extern int			 	Center;					/* CÌdS */
extern int				pid_angle;

/* vg^Cvé¾ */
void			sensor_process(void);			/* ZT[ÇÝæè		*/
void			initCamera(int,int);			/* JÌú»		*/
void			ImageCapture(void);				/* C[WLv` 	*/
void			binarization(void);				/* Ql»				*/
void 			WhiteLineWide(void);			/* üÌðªè		*/
void 			WhiteLineCenter(void);			/* üÌSðªè		*/
void 			Calibration(void);				/* Lu[V	*/
void 			expose( void );					/* IõÔ²® 		*/
void			raw_view(void);					/* RAWf[^Ì\¦		*/
void			bi_view(void);					/* Ql»f[^Ì\¦	*/
unsigned char	sensor_inp(unsigned char);		/* ZTóÔo		*/
int 			PID(void);						/* PID					*/
unsigned char 	startbar_get( void );			/* X^[go[o		*/
int				get_ad7( void );				/* A/DlÇÝÝ(AN7)	*/

#endif