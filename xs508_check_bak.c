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

//#define I2C_SLAVE     	0x0703
//#define I2C_BUS_MODE 	0x0780
int     XS_flag=0;

#define XS508_DT1    1          //1ms
#define W_XS508_Addr_Byte 0x29  //XS508 i2c slave address = 0x28
#define R_XS508_Addr_Byte 0x30 


/*int HI2C_write(unsigned char slave_addr, unsigned char *data, int len, int *retlen)
{
#if 1
	int fdI2C = -1;
	int i;
	int res;
	
	//unsigned char addr = (slave_addr >> 1);
	unsigned char addr = W_XS508_Addr_Byte;
	unsigned char reg[] = {0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F};
	//unsigned char addr = slave_addr;

	if( data == NULL || retlen == NULL )
		return -1;

	fdI2C = open("/dev/i2c-0",O_RDWR);
	if( fdI2C == -1 )
		return -1;

	printf("open /dev/i2c-0 success !\n");   //´ò¿ªiicÉè±¸ÎÄ¼þ³É¹¦

	res = ioctl(fdI2C,I2C_TENBIT,0);   //not 10bit
	printf("i2c write tenbit = %d\n",res);

	//if(ioctl(fdI2C, I2C_SLAVE_FORCE,addr)<0) 
	if(ioctl(fdI2C, I2C_SLAVE, addr)<0) 
	{    
		//ÉèÖÃiic´ÓÆ÷¼þµØÖ·
		printf("fail to set i2c device slave address!\n");
		close(fdI2C);
		return -1;
	}

	printf("set write slave address to 0x%x success!\n", addr);

#endif

	//write(fdI2C, data+7, 18);
	//
	// printf("write data len = %d, : \n",len);
	// for(i=0; i<len; i++)
	// {
	// 	printf(" %02x ",data[i]);
	// }
	// printf("\n\n");

	// res = write(fdI2C, data+i+7 ,18);
	// if(0 != res)
	// {
	// 	printf("write i2c data error, res = %d -----------------%d\n", res,i);
	// 	close(fdI2C);
	// 	return res;
	// }
	//write(fdI2C, reg, 1);
	//write(fdI2C, data, 25);
	
	for(i=0; i<len; i++)
	{
		res = write(fdI2C, reg+i, 1);
		res = write(fdI2C, data+i, 1); 
		//usleep(10000);
  		if(1 != res)
  		{
  			printf("write i2c data error, res = %d -----------------%d\n", res,i);
  			close(fdI2C);
  			return res;
  		}
	}

	// *retlen = i;

	//write(fdI2C, &Reg, 1);  //
	//usleep(100);              //ÑÓÊ±100us
	//*retlen = write(fdI2C, data, len);
	//usleep(1000*4);             //ÑÓÊ± 4ms
	close(fdI2C);

	return 0;
}




int HI2C_read(unsigned char slave_addr, unsigned char *data, int len, int *retlen)
{
#if 1
	int fdI2C = -1;
	int i;
	int res;
	
	//unsigned char addr = (slave_addr >> 1);
	unsigned char addr = W_XS508_Addr_Byte;
	unsigned char reg[] = {0x00};
	//unsigned char addr = slave_addr;

	if( data == NULL || retlen == NULL )
		return -1;

	fdI2C = open("/dev/i2c-0",O_RDWR);
	if( fdI2C == -1 )
		return -1;

	printf("open /dev/i2c-0 success !\n");   //´ò¿ªiicÉè±¸ÎÄ¼þ³É¹¦

	res = ioctl(fdI2C,I2C_TENBIT,0);   //not 10bit
	printf("i2c read tenbit = %d\n",res);

	//if(ioctl(fdI2C, I2C_SLAVE_FORCE, addr)<0) 
	if(ioctl(fdI2C, I2C_SLAVE, addr)<0) 
	{    
		//ÉèÖÃiic´ÓÆ÷¼þµØÖ·
		printf("fail to set i2c device slave address!\n");
		close(fdI2C);
		return -1;
	}

	printf("set read slave address to 0x%x success!\n", addr);

#endif

	write(fdI2C, reg, 1);
	//usleep(10000);
	res = read(fdI2C, data, 32);
	if(res < 0)
	{
		printf("------read i2c data error, res = %d -----------------%d\n", res,i);
	}
	// printf("read  data len = %d, : \n",res);
	// for(i=0; i<len; i++)
	// {
	// 	printf(" %02x ",data[i]);
	// }
	// printf("\n\n");
	// for(i=0; i<32; i++)
	// {
	// 	res = read(fdI2C, data+i, 1);  
	// 	if(1 != res)
 //  		{
 //  			printf("------read i2c data error, res = %d -----------------%d\n", res,i);
 //  			close(fdI2C);
 //  			return res;
 //  		}
 //  		usleep(100);              //ÑÓÊ±100us
	// }

	// *retlen = i;

	//write(fdI2C, &Reg, 1);  //
	//usleep(100);              //ÑÓÊ±100us
	//*retlen = read(fdI2C, data, len);
	//usleep(1000*4);             //ÑÓÊ± 4ms

	close(fdI2C);

	return 0;
}*/

int HI2C_write(unsigned char slave_addr, unsigned char *data, int len, int *retlen)
{
#if 1
	int fdI2C = -1;
	int i;
	int res;
	
	//unsigned char addr = (slave_addr >> 1);
	unsigned char addr = 0x29;
	unsigned char write_buf[26];
	//unsigned char addr = slave_addr;

	if( data == NULL || retlen == NULL )
		return -1;

	fdI2C = open("/dev/i2c-0",O_RDWR);
	if( fdI2C == -1 )
		return -1;

	printf("open write /dev/i2c-0 success !\n");   //´ò¿ªiicÉè±¸ÎÄ¼þ³É¹¦

	res = ioctl(fdI2C,I2C_TENBIT,0);   //not 10bit
	printf("i2c write tenbit = %d\n",res);

	//if(ioctl(fdI2C, I2C_SLAVE_FORCE, addr)<0) 
	if(ioctl(fdI2C, I2C_SLAVE_FORCE, addr)<0) 
	{    
		//ÉèÖÃiic´ÓÆ÷¼þµØÖ·
		printf("fail to set i2c device slave address!\n");
		close(fdI2C);
		return -1;
	}

	printf("set write slave address to 0x%x success!\n", addr);

	write_buf[0] = 0x07;
	for(i=0;i<25;i++)
	{
		write_buf[1+i] = data[i];
	}
	res = write(fdI2C, write_buf, 26);
	if(res < 0)
	{
		printf("------write i2c data error, res = %d -----------------%d\n", res,i);
		return res;
	}
	//usleep(10000);
	
	close(fdI2C);
#else
	int i, ret, length;  
	int fdI2C = -1;
	unsigned char addr = 0x29;  //读写地址  

	struct i2c_rdwr_ioctl_data wr_data; 

	if ((fdI2C = open("/dev/i2c-0", O_RDWR)) < 0)
	{
		printf("!!!!!!!!!!!  open /dev/i2c-0 error !\n");
		return fdI2C;
	}

	printf("open /dev/i2c-0 success !\n");   //´ò¿ªiicÉè±¸ÎÄ¼þ³É¹¦

	printf("HI2C_write start!\n");

	wr_data.nmsgs = 2;
	wr_data.msgs = (struct i2c_msg *)malloc(wr_data.nmsgs * sizeof(struct i2c_msg));
	if(!wr_data.msgs)
	{
		printf("msgs memery alloc error\n");
		//close(i2c_fd);
		return 0;
	} 
	if ((wr_data.msgs[0].buf = (unsigned char *)malloc(26 * sizeof(unsigned char))) == NULL)
	{
		printf("buf memery alloc error...\n");
		//close(i2c_fd);
		return 0;
	}

	wr_data.msgs[0].addr = addr;
	wr_data.msgs[0].flags = 0;
	wr_data.msgs[0].len = 26;
	wr_data.msgs[0].buf[0] = 0x07;
	for(i=0;i<len;i++)
	{
		wr_data.msgs[0].buf[1+i] = data[i];
	}

	wr_data.nmsgs = 1;

	ret = ioctl(fdI2C, I2C_RDWR, (unsigned long)&wr_data) ;
	if(ret <0)  
    {  
        printf("write data error\n");  
                return -1;  
    }  
    usleep(10000);
    free(wr_data.msgs[0].buf);
	free(wr_data.msgs);

	close(fdI2C);
	printf("close write /dev/i2c-0 success !\n");


#endif

    return 0;
}

int HI2C_read(unsigned char slave_addr, unsigned char *data, int len, int *retlen)
{
#if 1
	int fdI2C = -1;
	int i;
	int res;
	
	//unsigned char addr = (slave_addr >> 1);
	unsigned char addr = 0x29;
	unsigned char reg[1] = {0x00};
	//unsigned char addr = slave_addr;

	if( data == NULL || retlen == NULL )
		return -1;

	fdI2C = open("/dev/i2c-0",O_RDWR);
	if( fdI2C == -1 )
		return -1;

	printf("open read /dev/i2c-0 success !\n");   //´ò¿ªiicÉè±¸ÎÄ¼þ³É¹¦

	res = ioctl(fdI2C,I2C_TENBIT,0);   //not 10bit
	printf("i2c read tenbit = %d\n",res);

	//if(ioctl(fdI2C, I2C_SLAVE_FORCE, addr)<0) 
	if(ioctl(fdI2C, I2C_SLAVE_FORCE, addr)<0) 
	{    
		//ÉèÖÃiic´ÓÆ÷¼þµØÖ·
		printf("fail to set i2c device slave address!\n");
		close(fdI2C);
		return -1;
	}

	printf("set read slave address to 0x%x success!\n", addr);

	res = write(fdI2C, reg, 1);
	if(res < 0)
	{
		printf("------read_write i2c data error, res = %d -----------------%d\n", res,i);
		return res;
	}
	//usleep(10000);
	res = read(fdI2C, data, 32);
	if(res < 0)
	{
		printf("------read i2c data error, res = %d -----------------%d\n", res,i);
		return res;
	}

	close(fdI2C);
#else
	int i, ret, length;  
	//int fdI2C = -1;
	unsigned char addr = 0x29;  //读写地址  
	unsigned char reg;

	printf("HI2C_read start!\n");

	struct i2c_rdwr_ioctl_data rd_data;

	// if ((fdI2C = open("/dev/i2c-0", O_RDWR)) < 0)
	// {
	// 	printf("!!!!!!!!!!!  open /dev/i2c-0 error !\n");
	// 	return fdI2C;
	// }

	// printf("open /dev/i2c-0 success !\n");   //´ò¿ªiicÉè±¸ÎÄ¼þ³É¹¦

	rd_data.nmsgs = 2;
	rd_data.msgs = (struct i2c_msg *)malloc(rd_data.nmsgs * sizeof(struct i2c_msg));
	if(!rd_data.msgs)
	{
		printf("msgs memery alloc error\n");
		//close(i2c_fd);
		return 0;
	}
	printf("HI2C_read start1111111!\n");
	if ((rd_data.msgs[0].buf = (unsigned char *)malloc(sizeof(unsigned char))) == NULL)
	{
		printf("buf[1] memery alloc error...\n");
		//close(i2c_fd);
		return 0;
	} 
	if ((rd_data.msgs[1].buf = (unsigned char *)malloc(len * sizeof(unsigned char))) == NULL)
	{
		printf("buf[1] memery alloc error...\n");
		//close(i2c_fd);
		return 0;
	}
	(rd_data.msgs[0]).addr = 0x29;
	(rd_data.msgs[0]).flags = 0;
	(rd_data.msgs[0]).len = 1;
	(rd_data.msgs[0]).buf[0] = 0x00;

	(rd_data.msgs[1]).addr = 0x29;
	(rd_data.msgs[1]).flags = I2C_M_RD;
	(rd_data.msgs[1]).len = 32;	
	for(i=0;i<32;i++)
	{
		(rd_data.msgs[1]).buf[i] = 0;
	}

	printf("HI2C_read start22222222!\n");

	ret = ioctl(fdI2C, I2C_RDWR, (unsigned long)&rd_data) ;
	if(ret <0)  
    {  
        printf("read data error\n");
        return ret;
    }  
    printf("HI2C_read start333333333!\n");

	for(i = 0 ;i < rd_data.msgs[1].len; i++)
		data[i] = rd_data.msgs[1].buf[i];

    free(rd_data.msgs[0].buf);
	free(rd_data.msgs[1].buf);
	free(rd_data.msgs);

	//close(fdI2C);
	//
#endif

	return 0;
}

void XS508_delay_ms(int D_TIME)
{
	usleep(D_TIME*1000); 
	return;
}

//ÐèÒªÓÃ»§Ìá¹©¸¨ÖúËæ»úº¯Êý
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
	/* 
	if (XS_flag ==0) 
	{
		SysTick->CTRL&=0xfffffffb;
		xs_curr_time+=SysTick->VAL;
		srand(xs_curr_time);
		XS_flag=1;
	}	
 
 	*/    
 	
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

	int i;
	int res;


	//unsigned char addr = (slave_addr >> 1);
	//unsigned char addr = slave_addr;	
	
	int i1, i2, nRet;

	// if ((fdI2C = open("/dev/i2c-0", O_RDWR)) < 0)
	// {
	// 	printf("!!!!!!!!!!!  open /dev/i2c-0 error !\n");
	// 	return fdI2C;
	// }

	//printf("open /dev/i2c-0 success !\n");   //´ò¿ªiicÉè±¸ÎÄ¼þ³É¹¦

	i1=XS508_I2C_Handshake(XS508_KEY,XS508_DAT);
	if( i1!=0 )
	{
		printf("\r\n XS508_I2C_Handshake() err=%d:\r\n",i1); 
		i1=-1;
	}
	else
	{
		printf("************start XS508_Handshake*************\n");
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
		printf("\n lyp test = %d\n", i1);
		if( i1 != 0 )
		{	
			nRet = -1;
			goto ON_RETURN;
		}

		if( 0 == memcmp(XS508_DAT, XS508_id0, 16) )
		{
			printf("lyp test succeed!!!");
			nRet = 0;
		}
		else
		{
			printf("lyp test error!!!");
			nRet = -2;
		}

		return 0;
	}

ON_RETURN:
	/*for(i2=0;i2<16;i2++)
	{
		XS508_KEY[i2]+=rand();//ÓÃÍêÔòÏú»Ù¡£
		XS508_DAT[i2]+=rand();
		XS508_id0[i2]+=rand();
	}*/

	return nRet;
}

#endif

