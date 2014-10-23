#ifndef	ISCAMERA_H
#define	ISCAMERA_H

#define Kp -1.5
#define Ki -1.0
#define Kd -1.0

/* マスク値設定 ×：マスクあり(無効)　○：マスク無し(有効) */
#define MASK2_2         0x66            /* ×○○××○○×             */
#define MASK2_0         0x60            /* ×○○×××××             */
#define MASK0_2         0x06            /* ×××××○○×             */
#define MASK3_3         0xe7            /* ○○○××○○○             */
#define MASK0_3         0x07            /* ×××××○○○             */
#define MASK3_0         0xe0            /* ○○○×××××             */
#define MASK4_0         0xf0            /* ○○○○××××             */
#define MASK0_4         0x0f            /* ××××○○○○             */
#define MASK4_4         0xff            /* ○○○○○○○○             */
#define MASK_4_			0x3c			/* ××○○○○××				*/

extern int 				ImageData[128];
extern int 				sensor8;
extern unsigned int 	Wide;					/* ラインの幅 */
extern int			 	Center;					/* ラインの重心 */
extern int				pid_angle;

/* プロトタイプ宣言 */
void			sensor_process(void);			/* センサー読み取り		*/
void			initCamera(int,int);			/* カメラの初期化		*/
void			ImageCapture(void);				/* イメージキャプチャ 	*/
void			binarization(void);				/* ２値化				*/
void 			WhiteLineWide(void);			/* 白線の幅を測定		*/
void 			WhiteLineCenter(void);			/* 白線の中心を測定		*/
void 			Calibration(void);				/* キャリブレーション	*/
void 			expose( void );					/* 露光時間調整 		*/
void			raw_view(void);					/* RAWデータの表示		*/
void			bi_view(void);					/* ２値化データの表示	*/
unsigned char	sensor_inp(unsigned char);		/* センサ状態検出		*/
int 			PID(void);						/* PID					*/
unsigned char 	startbar_get( void );			/* スタートバー検出		*/
int				get_ad7( void );				/* A/D値読み込み(AN7)	*/

#endif