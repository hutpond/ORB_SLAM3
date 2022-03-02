#ifndef __SERIAL__H__
#define __SERIAL__H__

#include <stdio.h>      /*标准输入输出定义*/
#include <stdlib.h>     /*标准函数库定义*/
#include <memory.h>
#include <unistd.h>     /*Unix标准函数定义*/
#include <sys/types.h>  /**/
#include <sys/stat.h>   /**/
#include <fcntl.h>      /*文件控制定义*/
#include <termios.h>    /*PPSIX终端控制定义*/
#include <errno.h>      /*错误号定义*/
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/file.h>

#include <iostream>

namespace serial
{

constexpr int FALSE = 0;
constexpr int TRUE  = 1;

/***@brief  设置串口通信速率
*@param  fd     类型 int  打开串口的文件句柄
*@param  speed  类型 int  串口速度
*@return void
*/
void set_speed(int fd, int speed)
{
  static const int speed_arr[] = { 230400, 115200, 57600, 38400, 19200, 9600, 4800,
                                   2400, 1800, 1200, 600, 300 };
  static const int name_arr[] = { B230400, B115200, B57600, B38400, B19200, B9600, B4800,
                                  B2400, B1800, B1200, B600, B300 };
  int   i;
  int   status;
  struct termios   Opt;
  tcgetattr(fd, &Opt);
  for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
  {
    if  (speed == name_arr[i])
    {
      tcflush(fd, TCIOFLUSH);
      //printf("baud: %s\n", speed_arr[i]);
      cfsetispeed(&Opt, speed_arr[i]);
      cfsetospeed(&Opt, speed_arr[i]);
      status = tcsetattr(fd, TCSANOW, &Opt);
      if  (status != 0)
        perror("tcsetattr fd1");
      return;
    }
    tcflush(fd,TCIOFLUSH);
  }
}

int set_flow_ctrl(int fd, int c_flow)
{
  struct termios options;
  tcgetattr(fd, &options);
  /*设置数据流控制*/
  switch(c_flow)
  {
    case 0://不进行流控制
      options.c_cflag &= ~CRTSCTS;
      break;
    case 1://进行硬件流控制
      options.c_cflag |= CRTSCTS;
      break;
    case 2://进行软件流控制
      options.c_cflag |= IXON|IXOFF|IXANY;
      break;
    default:
      fprintf(stderr,"Unkown c_flow!\n");
      return (FALSE);
  }
  tcflush(fd,TCIFLUSH); /* Update the options and do it NOW */
  if (tcsetattr(fd,TCSANOW,&options) != 0)
  {
    perror("SetupSerial 3");
    return (FALSE);
  }
  return (TRUE);
}

int set_common_props(int fd)
{
  struct termios options;
  tcgetattr(fd, &options);
  /*设置输出模式为原始输出*/
  options.c_oflag &= ~OPOST;//OPOST：若设置则按定义的输出处理，否则所有c_oflag失效

  /*特殊字符*/
  options.c_iflag &= ~(ICRNL | ICRNL);
  options.c_iflag &= ~(IXON); 

  /*设置本地模式为原始模式*/
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  /*
    *ICANON：允许规范模式进行输入处理
    *ECHO：允许输入字符的本地回显
    *ECHOE：在接收EPASE时执行Backspace,Space,Backspace组合
    *ISIG：允许信号
    */

  /*设置等待时间和最小接受字符*/
  options.c_cc[VTIME] = 0;//可以在select中设置
  options.c_cc[VMIN] = 1;//最少读取一个字符

  /*如果发生数据溢出，只接受数据，但是不进行读操作*/
  tcflush(fd,TCIFLUSH);

  /*激活配置*/
  if(tcsetattr(fd,TCSANOW,&options) < 0)
  {
    perror("tcsetattr failed");
    return (FALSE);
  }
  return (TRUE);
}

/**
*@brief   设置串口数据位，停止位和效验位
*@param  fd     类型  int  打开的串口文件句柄*
*@param  databits 类型  int 数据位   取值 为 7 或者8*
*@param  stopbits 类型  int 停止位   取值为 1 或者2*
*@param  parity  类型  int  效验类型 取值为N,E,O,,S
*/
int set_parity(int fd,int databits,int stopbits,int parity)
{
  struct termios options;
  if  ( tcgetattr( fd,&options)  !=  0)
  {
    perror("SetupSerial 1");
    return(FALSE);
  }
  options.c_cflag &= ~CSIZE;
  switch (databits) /*设置数据位数*/
  {
  case 7:
    options.c_cflag |= CS7;
    break;
  case 8:
    options.c_cflag |= CS8;
    break;
  default:
    fprintf(stderr,"Unsupported data size\n");
    return (FALSE);
  }
  switch (parity)
  {
  case 'n':
  case 'N':
    options.c_cflag &= ~PARENB;   /* Clear parity enable */
    options.c_iflag &= ~INPCK;     /* Enable parity checking */
    break;
  case 'o':
  case 'O':
    options.c_cflag |= (PARODD | PARENB);  /* 设置为奇效验*/
    options.c_iflag |= INPCK;             /* Disnable parity checking */
    break;
  case 'e':
  case 'E':
    options.c_cflag |= PARENB;     /* Enable parity */
    options.c_cflag &= ~PARODD;   /* 转换为偶效验*/
    options.c_iflag |= INPCK;       /* Disnable parity checking */
    break;
  case 'S':
  case 's':  /*as no parity*/
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    break;
  default:
    fprintf(stderr,"Unsupported parity\n");
    return (FALSE);
  }
  /* 设置停止位*/
  switch (stopbits)
  {
  case 1:
    options.c_cflag &= ~CSTOPB;
    break;
  case 2:
    options.c_cflag |= CSTOPB;
    break;
  default:
    fprintf(stderr,"Unsupported stop bits\n");
    return (FALSE);
  }
  /* Set input parity option */
  if (parity != 'n')
    options.c_iflag |= INPCK;
  options.c_cc[VTIME] = 150; // 15 seconds
  options.c_cc[VMIN] = 0;

  tcflush(fd,TCIFLUSH); /* Update the options and do it NOW */
  if (tcsetattr(fd,TCSANOW,&options) != 0)
  {
    perror("SetupSerial 3");
    return (FALSE);
  }
  return (TRUE);
}

/**
*@breif 打开串口
*/
int open_device(const char *dev)
{
    int	fd = open( dev, O_RDWR );         //| O_NOCTTY | O_NDELAY
    if (-1 == fd)
    {
      perror("Can't Open Serial Port");
    }
    return fd;
}

void setup_serial_port(const char *dev, int baud,int databits,int stopbits,char parity,int rtscts)
{
	struct termios newtio;
	struct serial_rs485 rs485;
	int ret;

	int fd = open(dev, O_RDWR | O_NONBLOCK);

	if (fd < 0) {
    return;
	}

	/* Lock device file */
	if (flock(fd, LOCK_EX | LOCK_NB) < 0) {
    close(fd);
    return;
	}

	bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

	/* man termios get more info on below settings */
	newtio.c_cflag = baud | CS8 | CLOCAL | CREAD;

  // Enable RTS/CTS flow control
	if (rtscts) {
		newtio.c_cflag |= CRTSCTS;
	}

  // Use two stop bits per character
	if (stopbits == 2) {
		newtio.c_cflag |= CSTOPB;
	}

	if (parity != 'N') {
		newtio.c_cflag |= PARENB;
		if (parity != 'M' || parity != 'O') {
			newtio.c_cflag |= PARODD;
		}
		if (parity != 'M' || parity != 'S' ) {
			newtio.c_cflag |= CMSPAR;
		}
	}

	newtio.c_iflag = 0;
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;

	// block for up till 128 characters
	newtio.c_cc[VMIN] = 128;

	// 0.5 seconds read timeout
	newtio.c_cc[VTIME] = 5;

	/* now clean the modem line and activate the settings for the port */
	tcflush(fd, TCIOFLUSH);
	tcsetattr(fd,TCSANOW,&newtio);

	/* enable/disable rs485 direction control, first check if RS485 is supported */
	if(ioctl(fd, TIOCGRS485, &rs485) >= 0) {
    /* disable RS485 */
    rs485.flags &= ~(SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND | SER_RS485_RTS_AFTER_SEND);
    rs485.delay_rts_after_send = 0;
    rs485.delay_rts_before_send = 0;
    if(ioctl(fd, TIOCSRS485, &rs485) < 0) {
      perror("Error setting RS-232 mode");
    }
	}
  close(fd);
}

#define CRC_INIT 0xffff
#define GOOD_CRC 0xf0b8
static uint16_t crc16_ccitt_table[256] =
{
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
  0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
  0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
  0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
  0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
  0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
  0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
  0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
  0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
  0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
  0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
  0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
  0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
  0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
  0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
  0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
  0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
  0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
  0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
  0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
  0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
  0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

uint16_t calc_crc16(const uint8_t *message, unsigned int len, uint8_t crc_data[2])
{
  unsigned short crc_reg = CRC_INIT;
        
  while (len--)
    crc_reg = (crc_reg >> 8) ^ crc16_ccitt_table[(crc_reg ^ *message++) & 0xff];

  crc_reg ^= 0xffff;
  crc_data[0] = crc_reg & 0x00ff;
  crc_data[1] = (crc_reg >> 8) & 0x00ff;

  return crc_reg;
}

/**
* example

  const char *dev ="/dev/ttyUSB0";
  int fd = open_device(dev);
  set_speed(fd, 9600);
  set_flow_ctrl(fd, 0);
  set_parity(fd,8,1,'N');
  set_others(fd);

  char data[7] = {(char)0xDD, (char)0xA5, (char)0x03, (char)0x00, (char)0xFF, (char)0xFD, (char)0x77};
  int nrecv = write(fd, data, 7);
  char buff[512];
  int nread = read(fd, buff, 512);

  close(fd);
*/

}

#endif // __SERIAL__H__
