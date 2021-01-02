/*
 * Copyright (C) 2017 C-SKY Microsystems Co., Ltd. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/******************************************************************************
 * @file     minilibc_port.c
 * @brief    minilibc port
 * @version  V1.0
 * @date     26. Dec 2017
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <csi_config.h>
#include "wm_regs.h"

int sendchar(int ch)
{
	//tls_reg_write32(HR_UART0_INT_MASK, 0x3);
    if(ch == '\n')
	{
		while (tls_reg_read32(HR_UART0_FIFO_STATUS) & 0x3F);
		tls_reg_write32(HR_UART0_TX_WIN, '\r');
    }
    while(tls_reg_read32(HR_UART0_FIFO_STATUS) & 0x3F);
    tls_reg_write32(HR_UART0_TX_WIN, (char)ch);
    //tls_reg_write32(HR_UART0_INT_MASK, 0x0);
    return ch;
}

int sendchar1(int ch)
{
	//tls_reg_write32(HR_UART1_INT_MASK, 0x3);
    if(ch == '\n')
	{
		while (tls_reg_read32(HR_UART1_FIFO_STATUS) & 0x3F);
		tls_reg_write32(HR_UART1_TX_WIN, '\r');
    }
    while(tls_reg_read32(HR_UART1_FIFO_STATUS) & 0x3F);
    tls_reg_write32(HR_UART1_TX_WIN, (char)ch);
    //tls_reg_write32(HR_UART1_INT_MASK, 0x0);
    return ch;
}

int fputc(int ch, FILE *stream)
{
    (void)stream;
#if USE_UART0_PRINT
    sendchar(ch);
#else
    sendchar1(ch);
#endif

    return 0;
}

int fgetc(FILE *stream)
{
    (void)stream;

    return 0;
}

//function:	������ת�����ַ���
//paramter:
//str	:	ת��֮����ڴ�buffer
//num	:	Ҫת��������
//base	:	ʮ���ƣ�ʮ������
//width	:	������
//opflag :	��������bit��������

#define	P_ALIGN_BIT	(0x01<<0)	// bit=1 ����� bit=0�Ҷ���
#define  P_FILL_BIT		(0x01<<1)	//bit = 1���'0'���������' '
#define  P_BIG_BIT		(0x01<<2)	//bit=1����д������Сд

int Int2Str(char *str,int num,char base,char width,int opflag) 
{   
	char temp; 
	int len = 0;
	signed char k = 0;
	char *str_bk;
	signed char k_bk;

	if(num <0) 
	{ 
		num = -num;   
		*str='-';
		str++;
		len++;  
	}
	if(0 == num)
	{
		*str = '0';
		str ++;
		k ++;
	}
	while(num) 
	{	
		temp= num%base; 
		if(temp > 9) // insert hexdecimal--ABCDEF-- 
		{  
			temp-=10;  
			if(opflag & P_BIG_BIT)
				*str = temp + 'A';  	
			else
				*str = temp + 'a';  
		} 
		else 
		{  
			*str = temp + '0'; 
		}
		num=num/base; 
		str++;
		k++; 
	}  

	if(opflag&P_ALIGN_BIT)	//�����
	{
		str_bk = str;
		k_bk = k;		//�ȱ���ָ��ͳ��ȣ�����֮�����
		str --;
		k --;
		while(k>0) 
		{
			temp = *str; 
			*str = *(str-k); 
			*(str-k) = temp; 
			str--; 
			k-=2; 
		}  
		k = k_bk;
		str = str_bk;
	}	
 
	//�����ȵ���' '����
	while(width>k) 
	{  
		if(opflag&P_FILL_BIT)
		{
			*str++ ='0';
		}
		else
		{
			*str++ =' ';
		}
		k++; 
	}

	len=len+k; 
	*str-- = '\0'; 
	k--; 
 	if(0 == (opflag&P_ALIGN_BIT))	//�Ҷ���
 	{
		//���� 
		while(k>0) 
		{
			temp = *str; 
			*str = *(str-k); 
			*(str-k) = temp; 
			str--; 
			k-=2; 
		}  
 	} 
	return len; 
}  

static int IP2Str(unsigned char v4v6, unsigned int *inuint, char *outtxt)
{
    unsigned char i;
    unsigned char j = 0;
    unsigned char k;
    unsigned char h;
    unsigned char m;
    unsigned char l;
    unsigned char bit;

    if (4 == v4v6)
    {
        for(i = 0; i < 4; i++)
        {
            bit = (*inuint >> (8 * i)) & 0xff;
            h = bit / 100;
            if (h)
                outtxt[j++] = '0' + h;
            m = (bit % 100) / 10;
            if (m)
            {
                outtxt[j++] = '0' + m;
            }
            else
            {
                if (h)
                    outtxt[j++] = '0';
            }
            l = (bit % 100) % 10;
            outtxt[j++] = '0' + l;
            outtxt[j++] = '.';
        }
    }
    else
    {
        for (k = 0; k < 4; k++)
        {
            for(i = 0; i < 4; i++)
            {
                m = (*inuint >> (8 * i)) & 0xff;
                h = m >> 4;
                l = m & 0xf;
                if (h > 9)
                    outtxt[j++] = 'A' + h - 10;
                else 
                    outtxt[j++]= '0' + h;
                if (l > 9)
                    outtxt[j++] = 'A' + l - 10;
                else
                    outtxt[j++] = '0' + l;
                if (0 != (i % 2))
                    outtxt[j++] = ':';
            }
            inuint++;
        }
    }

    outtxt[j - 1] = 0;
    return j - 1;
}

static int Mac2Str(unsigned char *inchar, char *outtxt)
{
    unsigned char hbit,lbit;
    unsigned int i;

    for(i = 0; i < 6; i++)/* mac length */
    {
        hbit = (*(inchar + i) & 0xf0) >> 4;
        lbit = *(inchar + i ) & 0x0f;
        if (hbit > 9)
            outtxt[3 * i] = 'A' + hbit - 10;
        else 
            outtxt[3 * i]= '0' + hbit;
        if (lbit > 9)
            outtxt[3 * i + 1] = 'A' + lbit - 10;
        else
            outtxt[3 * i + 1] = '0' + lbit;
        outtxt[3 * i + 2] = '-';
    }

    outtxt[3 * (i - 1) + 2] = 0;

    return 3 * (i - 1) + 2;
}

int wm_vprintf(const char *fmt, va_list arg_ptr)
{
	unsigned char width=0; 	//�������
	unsigned int len; 			//���ݿ��
	char *fp = (char *)fmt;  
	//va_list arg_ptr; 
	char *pval;
	int opflag = 0;
	char store[40];
	char c;
	int i;
	char* str;
	
	//va_start(arg_ptr, fmt); //arg_ptr ָ���һ������
	while (*fp !='\0') 
	{
		c = *fp++; 
		if (c != '%') 
		{
			sendchar(c);
		} 
		else 
		{ 
			width = 0;  //��ȡ���ݿ��
			opflag = 0;
			if('-' == *fp)
			{
				opflag |= P_ALIGN_BIT;//�����
				fp ++;
			}
			if('0' == *fp)	//ǰ�油��
			{
				opflag |= P_FILL_BIT;	//����
				fp ++;
			}

			while(*fp>='0'&&*fp<='9') 
			{  
				width = width * 10 + (*fp) - '0'; 
				fp++; 
			} 
			if('.' == *fp)	//����������ʱû�ã�������
			{
				fp ++;
				while(*fp>='0'&&*fp<='9') 
				{  
					fp++; 
				}
			}

			while('l' == *fp || 'h' == *fp)
			{
				fp ++;
			}			
			
			switch (*fp) 
			{  
				case 'c': 
				case 'C': 
					c = (char)va_arg(arg_ptr, int);
			             sendchar(c);
					break; 
				case 'd': 
				case 'i':  
				case 'u':	
			              i = va_arg(arg_ptr, int);
			              str = store;
					Int2Str(store,i,10,width,opflag); 
      		                    while( *str != '\0') sendchar(*str++);
					break; 
				case 'x': 
				case 'X':  
			              i = va_arg(arg_ptr, int);
			              str = store;
					if('X' == *fp)
					{
						opflag |= P_BIG_BIT;
					}					 
					 Int2Str(store,i,16,width,opflag); 		   
			                while( *str != '\0') sendchar(*str++);
					break; 
				case 'o':
			             i = va_arg(arg_ptr, int);
			             str = store;
					Int2Str(store,i,8,width,opflag); 		   
       			      while( *str != '\0') sendchar(*str++);
					break;
				case 's': 
				case 'S':
					pval=va_arg(arg_ptr,char*);
					len = strlen(pval);
					if((width > len) && (0 == (opflag&P_ALIGN_BIT)))		//�Ҷ���
					{
						for(i = 0;i < (width - len);i ++)	//��߲��ո�
						{
							sendchar(' ');
						}
					}
                                for(i=0;i < len;i++)
                                {
						sendchar(pval[i]);					
                                }
					if((width > len) && (opflag&P_ALIGN_BIT))		//�����
					{
						for(i = 0;i < (width - len);i ++)	//�ұ߲��ո�
						{
							sendchar(' ');
						}
					}
					break; 
                case 'v':/* ip v4 address */
                    i = va_arg(arg_ptr, int);
                    len = IP2Str(4, (unsigned int *)&i, store);
					if((width > len) && (0 == (opflag&P_ALIGN_BIT)))		//�Ҷ���
					{
						for(i = 0;i < (width - len);i ++)	//��߲��ո�
						{
							sendchar(' ');
						}
					}
                    str = store;
                    while( *str != '\0') sendchar(*str++);
					if((width > len) && (opflag&P_ALIGN_BIT))		//�����
					{
						for(i = 0;i < (width - len);i ++)	//�ұ߲��ո�
						{
							sendchar(' ');
						}
					}
			        break;
			    case 'V':/* ip v6 address */
                    pval=va_arg(arg_ptr,char*);
                    len = IP2Str(6, (unsigned int *)pval, store);
					if((width > len) && (0 == (opflag&P_ALIGN_BIT)))		//�Ҷ���
					{
						for(i = 0;i < (width - len);i ++)	//��߲��ո�
						{
							sendchar(' ');
						}
					}
                    str = store;
                    while( *str != '\0') sendchar(*str++);
					if((width > len) && (opflag&P_ALIGN_BIT))		//�����
					{
						for(i = 0;i < (width - len);i ++)	//�ұ߲��ո�
						{
							sendchar(' ');
						}
					}
			        break;
				case 'M':/* mac address */
                    pval = va_arg(arg_ptr, char*);
                    len = Mac2Str((unsigned char *)pval, store);
					if((width > len) && (0 == (opflag&P_ALIGN_BIT)))		//�Ҷ���
					{
						for(i = 0;i < (width - len);i ++)	//��߲��ո�
						{
							sendchar(' ');
						}
					}
                    str = store;
                    while( *str != '\0') sendchar(*str++);
					if((width > len) && (opflag&P_ALIGN_BIT))		//�����
					{
						for(i = 0;i < (width - len);i ++)	//�ұ߲��ո�
						{
							sendchar(' ');
						}
					}
                    break;
				case '%':  
					sendchar('%');
					break; 
				default: 
					break; 
			}
			fp++; 
		} 
	}  
	//va_end(arg_ptr); 
	return 0; 
	
}

int wm_printf(const char *fmt,...) 
{
	va_list ap;

	va_start(ap, fmt);
	wm_vprintf(fmt,ap);
	va_end(ap);
	return 0;
}

int wm_vsnprintf(char *outstr, size_t size, const char *fmt, va_list arg_ptr)
{
	unsigned char width=0; 	//�������
	unsigned int len; 			//���ݿ��
	char *fp = (char *)fmt;  
	char *pval;
	int opflag = 0;
	char store[40];
	char c;
	int i;
	char* str;
	int ret = 0;

	while (*fp !='\0') 
	{
		c = *fp++; 
		if (c != '%') 
		{
			if ((ret + 1) >= size)
			    break;
			outstr[ret++] = c;
		} 
		else 
		{ 
			width = 0;  //��ȡ���ݿ��
			opflag = 0;
			if('-' == *fp)
			{
				opflag |= P_ALIGN_BIT;//�����
				fp ++;
			}
			if('0' == *fp)	//ǰ�油��
			{
				opflag |= P_FILL_BIT;	//����
				fp ++;
			}

			while(*fp>='0'&&*fp<='9') 
			{  
				width = width * 10 + (*fp) - '0'; 
				fp++; 
			} 
			if('.' == *fp)	//����������ʱû�ã�������
			{
				fp ++;
				while(*fp>='0'&&*fp<='9') 
				{  
					fp++; 
				}
			}

			while('l' == *fp || 'h' == *fp)
			{
				fp ++;
			}			
			
			switch (*fp) 
			{  
				case 'c': 
				case 'C': 
					c = (char)va_arg(arg_ptr, int);
			        if ((ret + 1) >= size)
        			    break;
        			outstr[ret++] = c;
					break; 
				case 'd': 
				case 'i':  
				case 'u':	
                    i = va_arg(arg_ptr, int);
                    str = store;
                    Int2Str(store,i,10,width,opflag); 
                    while( *str != '\0')
                    {
                        if ((ret + 1) >= size)
        			        break;
        			    outstr[ret++] = *str++;
                    }
					break; 
				case 'x': 
				case 'X':  
                    i = va_arg(arg_ptr, int);
                    str = store;
                    if('X' == *fp)
                    {
                    opflag |= P_BIG_BIT;
                    }					 
                    Int2Str(store,i,16,width,opflag); 		   
                    while( *str != '\0')
                    {
                        if ((ret + 1) >= size)
        			        break;
        			    outstr[ret++] = *str++;
                    }
					break; 
				case 'o':
                    i = va_arg(arg_ptr, int);
                    str = store;
                    Int2Str(store,i,8,width,opflag); 		   
                    while( *str != '\0')
                    {
                        if ((ret + 1) >= size)
        			        break;
        			    outstr[ret++] = *str++;
                    }
					break;
				case 's': 
				case 'S':
					pval=va_arg(arg_ptr,char*);
					len = strlen(pval);
					if((width > len) && (0 == (opflag&P_ALIGN_BIT)))		//�Ҷ���
					{
						for(i = 0;i < (width - len);i ++)	//��߲��ո�
						{
							if ((ret + 1) >= size)
        			            break;
        			        outstr[ret++] = ' ';
						}
					}
                                for(i=0;i < len;i++)
                                {
						            if ((ret + 1) >= size)
                			            break;
                			        outstr[ret++] = pval[i];
                                }
					if((width > len) && (opflag&P_ALIGN_BIT))		//�����
					{
						for(i = 0;i < (width - len);i ++)	//�ұ߲��ո�
						{
							if ((ret + 1) >= size)
        			            break;
        			        outstr[ret++] = ' ';
						}
					}
					break; 
                case 'v':/* ip v4 address */
                    i = va_arg(arg_ptr, int);
                    len = IP2Str(4, (unsigned int *)&i, store);
					if((width > len) && (0 == (opflag&P_ALIGN_BIT)))		//�Ҷ���
					{
						for(i = 0;i < (width - len);i ++)	//��߲��ո�
						{
							if ((ret + 1) >= size)
        			            break;
        			        outstr[ret++] = ' ';
						}
					}
                    str = store;
                    while( *str != '\0')
                    {
                        if ((ret + 1) >= size)
    			            break;
    			        outstr[ret++] = *str++;
                    }
					if((width > len) && (opflag&P_ALIGN_BIT))		//�����
					{
						for(i = 0;i < (width - len);i ++)	//�ұ߲��ո�
						{
							if ((ret + 1) >= size)
        			            break;
        			        outstr[ret++] = ' ';
						}
					}
			        break;
			    case 'V':/* ip v6 address */
                    pval=va_arg(arg_ptr,char*);
                    len = IP2Str(6, (unsigned int *)pval, store);
					if((width > len) && (0 == (opflag&P_ALIGN_BIT)))		//�Ҷ���
					{
						for(i = 0;i < (width - len);i ++)	//��߲��ո�
						{
							if ((ret + 1) >= size)
        			            break;
        			        outstr[ret++] = ' ';
						}
					}
                    str = store;
                    while( *str != '\0')
                    {
                        if ((ret + 1) >= size)
    			            break;
    			        outstr[ret++] = *str++;
                    }
					if((width > len) && (opflag&P_ALIGN_BIT))		//�����
					{
						for(i = 0;i < (width - len);i ++)	//�ұ߲��ո�
						{
							if ((ret + 1) >= size)
        			            break;
        			        outstr[ret++] = ' ';
						}
					}
			        break;
				case 'M':/* mac address */
                    pval = va_arg(arg_ptr, char*);
                    len = Mac2Str((unsigned char *)pval, store);
					if((width > len) && (0 == (opflag&P_ALIGN_BIT)))		//�Ҷ���
					{
						for(i = 0;i < (width - len);i ++)	//��߲��ո�
						{
							if ((ret + 1) >= size)
        			            break;
        			        outstr[ret++] = ' ';
						}
					}
                    str = store;
                    while( *str != '\0')
                    {
                        if ((ret + 1) >= size)
    			            break;
    			        outstr[ret++] = *str++;
                    }
					if((width > len) && (opflag&P_ALIGN_BIT))		//�����
					{
						for(i = 0;i < (width - len);i ++)	//�ұ߲��ո�
						{
							if ((ret + 1) >= size)
        			            break;
        			        outstr[ret++] = ' ';
						}
					}
                    break;
				case '%':  
					if ((ret + 1) >= size)
			            break;
			        outstr[ret++] = '%';
					break; 
				default: 
					break; 
			}
			fp++; 
		} 
	}  

	outstr[ret] = '\0';

	return ret;
}

