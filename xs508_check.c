#ifdef WIN32 

int xs508_check()
{
	return -1;
}

#else

#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <stdarg.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/time.h> 

#include <linux/i2c.h>
#include <linux/i2c-dev.h>


int     XS_flag=0;

#define XS508_DT1    1         
#define W_XS508_Addr_Byte 0x29  
#define R_XS508_Addr_Byte 0x30 

int HI2C_write(unsigned char slave_addr, unsigned char *data, int len, int *retlen)
{
	int fdI2C = -1;
	int i;
	int res;
	
	unsigned char addr = 0x29;
	unsigned char write_buf[26];

	if( data == NULL || retlen == NULL )
		return -1;

	fdI2C = open("/dev/i2c-0",O_RDWR);
	if( fdI2C == -1 )
	{
		printf("open write /dev/i2c-0 error !\n"); 
		return -1;
	}

	//printf("open write /dev/i2c-0 success !\n"); 

	res = ioctl(fdI2C,I2C_TENBIT,0);  
	//printf("i2c write tenbit = %d\n",res);

	if(ioctl(fdI2C, I2C_SLAVE, addr)<0) 
	{    
		printf("fail to set i2c device slave address!\n");
		close(fdI2C);
		return -1;
	}

	//printf("set write slave address to 0x%x success!\n", addr);

	write_buf[0] = 0x07;
	for(i=0;i<25;i++)
	{
		write_buf[1+i] = data[i];
	}
	res = write(fdI2C, write_buf, 26);
	if(res < 0)
	{
		printf("[troad-lyp] write i2c data error, res = %d -----------------%d\n", res,i);
		return res;
	}
	
	close(fdI2C);

    return 0;
}

int HI2C_read(unsigned char slave_addr, unsigned char *data, int len, int *retlen)
{
	int fdI2C = -1;
	int i;
	int res;
	
	unsigned char addr = 0x29;
	unsigned char reg[1] = {0x00};

	if( data == NULL || retlen == NULL )
		return -1;

	fdI2C = open("/dev/i2c-0",O_RDWR);
	if( fdI2C == -1 )
	{
		printf("open read /dev/i2c-0 error !\n");  
		return -1;
	}

	//printf("open read /dev/i2c-0 success !\n");   

	res = ioctl(fdI2C,I2C_TENBIT,0);   
	//printf("i2c read tenbit = %d\n",res);

	if(ioctl(fdI2C, I2C_SLAVE, addr)<0) 
	{    
		printf("fail to set i2c device slave address!\n");
		close(fdI2C);
		return -1;
	}

	//printf("set read slave address to 0x%x success!\n", addr);

	res = write(fdI2C, reg, 1);
	if(res < 0)
	{
		printf("[troad-lyp] read_write i2c data error, res = %d -----------------%d\n", res,i);
		return res;
	}
	//usleep(10000);
	res = read(fdI2C, data, 32);
	if(res < 0)
	{
		printf("[troad-lyp] read i2c data error, res = %d -----------------%d\n", res,i);
		return res;
	}

	close(fdI2C);

	return 0;
}

void XS508_delay_ms(int D_TIME)
{
	usleep(D_TIME*1000); 
	return;
}

int get_xs_srand(void)
{
	volatile int Xstimeus; 
	
	struct timeval tv;
	struct timezone tz;
	gettimeofday (&tv , &tz);
	Xstimeus += (int)(tv.tv_sec*1000000+tv.tv_usec);
	srand(Xstimeus);
	Xstimeus+=rand();	
	return Xstimeus;
}

extern int XS508_Handshake(unsigned char *XS508_16B_Ukey, unsigned char *XS508_16B_ID);

int XS508_I2C_Handshake(unsigned char *XS508_16B_Ukey,unsigned char *XS508_16B_ID)
{
	unsigned char wr_data_25_byte[25];
	unsigned char rd_data_32_byte[32]={0};
	int i,check_cnt,act_long;
	volatile int xs_curr_time;
	int debugset=0xa8;
	int t1;
	
	if (XS_flag ==0) 
	{
	
		t1 = xs_curr_time & 0xFF;
		xs_curr_time += get_xs_srand();
		srand(xs_curr_time);
		while (t1--)
		{
			rand();
		}
		XS_flag=1;
	}  
 	
	//---------初始化------------------
	if (debugset==0xa8)	 printf("T_I2C write	BYTES =\r\n"); 

					 
	for(i=1;i<25;i++)
	{
		//wr_data_25_byte[i]=i;
		wr_data_25_byte[i]+=(char)rand();
	}
	
	for(i=0;i<16;i++)
	{
		wr_data_25_byte[i+4]+=XS508_16B_Ukey[i];
	}
	
	if ((wr_data_25_byte[23]==0x5a)|| (wr_data_25_byte[23]==0xa5) )
		wr_data_25_byte[23]=0x7a;
   
	wr_data_25_byte[0]=0XF2;
	if (debugset==0xa8)	
	{
		for(i=0;i<25;i++)
			printf("0x%02x,",wr_data_25_byte[i]);
		printf("\r\n");
	}
	//------step1：准备25字节数据wr_data_25_byte[]-----------------------

	check_cnt=0;
	while(HI2C_write(W_XS508_Addr_Byte,wr_data_25_byte,25,&act_long)) 
	{
		XS508_delay_ms(XS508_DT1);//
		check_cnt++;
		if (check_cnt>100)
			{
					if (debugset==0xa8)	 printf("X508 I2C Write  error \n");
					return -1;  
			}
	}
	//-------step2：写到X508芯片中-----

	XS508_delay_ms(10);//delay 10ms
	//-------step3：等待X508处理-----			
				
	check_cnt=0;
	while(HI2C_read(R_XS508_Addr_Byte,rd_data_32_byte,32,&act_long))
	{
		XS508_delay_ms(XS508_DT1);//
		check_cnt++;
		if (check_cnt>100)
		{
			if (debugset==0xa8)	 printf("XS508 I2C Read  error \n");
		 
			return -1; 
		}
	}
	//-------step4：从X508芯片中读取32字节数据-----
	
	XS508_delay_ms(100);  //delay 100ms 
	
	if (debugset==0xa8)
	{

		printf("XS508_I2C READ 32 BYTES =\r\n"); 
		for(i=0;i<32;i++)
			printf("0x%02x,",rd_data_32_byte[i]);
		printf("\r\n"); 

	}
	
	
	
	for(i=0;i<24;i++)
	{
		if (wr_data_25_byte[1+i]!=(rd_data_32_byte[8+i]^0xaa) )
		{
			if (debugset==0xa8)	 printf("no(%d): 0x%2x _xor_0xaa != 0x%2x	",i,wr_data_25_byte[1+i],rd_data_32_byte[8+i]);
			if (debugset==0xa8)	 printf("IIC_DATA_ERR\r\n"); 
			return -2; //验证错误
		}
	}
			
	if (debugset==0xa8)	printf ("---XS508E_I2C success!---\r\n");
	
	return 0;	
}



int xs508_check()
{
	/*ÓÃ»§×Ô¶¨Òå16×Ö½Úkey,£¨Ä¬ÈÏÖµÎª16¸ö×Ö½Ú0xff£©,½¨ÒéÊ¹ÓÃÍê±ÏÁ¢¿Ì¸ü¸ÄÆäÄÚÈÝ*/
	//unsigned char XS508_KEY[16]={0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF, 0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF};	
	unsigned char XS508_KEY[16]={0x34, 0x28, 0x75, 0xFD, 0x12, 0xD5, 0x9A, 0x76, 0x02, 0xAB, 0x21, 0x6D, 0x5C, 0x88, 0x1A, 0x38};
	/*ÉùÃ÷16BYTE¿Õ¼ä £¬½¨ÒéÊ¹ÓÃÍê±ÏÁ¢¿Ì¸ü¸ÄÆäÄÚÈÝ¡£*/
	unsigned char XS508_DAT[16]={0};//Ð¾Æ¬ÄÚ³ö³§ÖµÎª16×Ö½Ú0XFF

	//unsigned char XS508_id0[16]={0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF, 0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF};		//¼Ù¶¨XS508Ô¤ÖÃ16×Ö½ÚidµÄÖµ
	unsigned char XS508_id0[16]={0x8D, 0x4C, 0x6D, 0xDF, 0x2F, 0xFC, 0x37, 0x7F, 0x5F, 0x99, 0x66, 0x3D, 0x13, 0x0A, 0x2A, 0x64};
	
	int i, i1, i2, nRet;

	i1=XS508_I2C_Handshake(XS508_KEY,XS508_DAT);
	if( i1!=0 )
	{
		printf("\r\n XS508_I2C_Handshake() err=%d:\r\n",i1); 
		i1=-1;
	}
	else
	{
		printf("************[troad-lyp] start XS508_Handshake*************\n");
		i1=XS508_Handshake(XS508_KEY,XS508_DAT);
		printf("key = \n");
		for(i=0;i<16;i++)
		{
			printf(" 0x%02x ", XS508_KEY[i]);
		}
		printf("\ndat = \n");
		for(i=0;i<16;i++)
		{
			printf(" 0x%02x ", XS508_DAT[i]);
		}
		printf("\n [troad-lyp] test ACK = %d\n", i1);
		if( i1 != 0 )
		{	
			nRet = -1;
			goto ON_RETURN;
		}

		if( 0 == memcmp(XS508_DAT, XS508_id0, 16) )
		{
			printf("[troad-lyp] test succeed!!!\n");
			nRet = 0;
		}
		else
		{
			printf("[troad-lyp] test error!!!\n");
			nRet = -2;
		}

		return 0;
	}

ON_RETURN:
	for(i2=0;i2<16;i2++)
	{
		XS508_KEY[i2]+=rand();//ÓÃÍêÔòÏú»Ù¡£
		XS508_DAT[i2]+=rand();
		XS508_id0[i2]+=rand();
	}

	return nRet;
}

#endif

