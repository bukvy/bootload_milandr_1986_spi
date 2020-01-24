#include "s19.h"
#include "main.h"


static uint8_t GetHexValue(uint8_t text)
{
    switch (text)
    {
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
            return (uint8_t)(text - '0');
        case 'A':
        case 'a':
            return 10;
        case 'B':
        case 'b':
            return 11;
        case 'C':
        case 'c':
            return 12;
        case 'D':
        case 'd':
            return 13;
        case 'E':
        case 'e':
            return 14;
        case 'F':
        case 'f':
            return 15;
        default:
            return 0xFF;
    } /* EndSwitch */
}
// room420 convert two AsciiChar to one uint8_t value
static uint8_t GetSpair(uint8_t *arr, uint8_t point, int *res)
{
    /* Body */
    uint8_t ch;
    uint8_t upper,lower;
    
    ch = arr[point];
    upper = (uint8_t)(GetHexValue(ch));
    
    if (upper == 0xFF) {
        /* Not a proper S19 file */
        *res = -1;
    } /* EndIf */
    
    upper = (uint8_t)(upper << 4);
    ch = arr[point+1];
    lower = (uint8_t)(GetHexValue(ch));
    if (lower == 0xFF) {
        /* Not a proper S19 file */
    	*res = -1;
    } /* EndIf */
    return (uint8_t)(upper | lower);
}

int res_burn_s19_line = 0;
int burn_s19_line(uint8_t *Line)
{
    /* Body */
	static uint8_t length;
    static uint8_t checksum;
    static uint8_t i, offset,temp,c_temp,j;
    static uint8_t type;
    static uint8_t data;
    static uint8_t cur_point; /* current pointer */
    static uint32_t S19Address;
    uint8_t buffer_to_flash[256];
    int res;
    uint32_t fl_addr_res;
    
//room420    if (!s19_sem.VALID) _lwsem_create(&s19_sem, 1);
    
//room420    _lwsem_wait(&s19_sem);
    
    c_temp = Line[0];
    if (c_temp != 'S') {
    	res_burn_s19_line = 1;
    	goto err_unlock;
    } /* EndIf */
    /* Get record length */
    cur_point = 2;
    length = GetSpair(Line, cur_point, &res); // if *res =-1 then could nod find length room420
    if (res < 0) {
    	res_burn_s19_line = 2;
    	goto err_unlock;
    } /* EndIf */
    cur_point--;
    checksum = length;
    type = Line[1];

    /* Take appropriate action */
    switch (type)
    {
    case '1':
    case '2':
    case '3':
    	S19Address = 0;
    	type -= '0';
    	cur_point = 4;
    	for (i = 0; i <= type; i++) {
    		/* Formulate address */
    		/* Address needs to be word aligned for successful flash program */
    		data = GetSpair(Line, cur_point, &res);
    		if (res < 0) { 
    	    	res_burn_s19_line = 3;
    			goto err_unlock;
    		} /* EndIf */
    		S19Address = (S19Address << 8) | data;
    		/* Maintain 8-bit checksum */
    		checksum = (unsigned char)((data + checksum) & 0x00FF);
    		cur_point += 2;
    	} /* EndFor */

    	/* 32-bit cores program flash in 32-bit words */
    	/* Therefore S19 address needs to be adjusted to be word aligned */
    	/* Pad beginning of buffer if address not word aligned */
    	offset = (uint8_t)(S19Address & 0x0003);
    	S19Address = (uint32_t)(S19Address & 0xFFFFFFFC);
    	length += offset;
// room420 if address is not *0x03 then address is from  S19Address & 0xFFFFFFFC  but fist not filled bytes are zero
    	for (i = 0; i < offset; i++) {
    		buffer_to_flash[i] = 0xFF; 
    	} /* EndFor */
    	/* Get data and put into buffer */
    	for (i = offset; i < (length - (type + 2)); i++) {
    		data = GetSpair(Line,cur_point, &res);
    		buffer_to_flash[i] = data;
    		cur_point += 2;
    		if (res < 0) {
    	    	res_burn_s19_line = 4;
    			goto err_unlock;
    		} /* EndIf */
    	} /* EndFor */

    	/* Calculate checksum */
    	for (i = offset; i < (length - (type+2)); i++) {
    		checksum = (unsigned char)((buffer_to_flash[i] + checksum) & 0x00FF);
    	} /* EndFor */
    	/* Get checksum byte */
    	data = GetSpair(Line, cur_point, &res);
    	cur_point += 2;
    	if (res < 0) {
        	res_burn_s19_line = 5;
			goto err_unlock;
    	} /* EndIf */

    	if (((data - ~checksum) & 0x00FF) != 0) {
        	res_burn_s19_line = 6;
			goto err_unlock;
    	} /* EndIf */
    	/* For 32-bit cores Flash_Prog writes 32-bit words, not bytes */
    	/* if last 32-bit word in s-record is not complete, finish word */
    	if ((i & 0x0003) != 0x0000) {
    		/* 32-bit word not complete */
    		buffer_to_flash[i++] = 0xFF;         /* pad end of word */
    		buffer_to_flash[i++] = 0xFF;         /* pad end of word */
    		buffer_to_flash[i++] = 0xFF;         /* pad end of word */
    	} /* EndIf */
        
/* room420 that is from keenetic
    	if (S19Address < ((uint32_t)(__FLASHX_END_ADDR)) / 2) {
    		flash_file = flash_file0;
    		fl_addr_res = S19Address;
    	} else 
    	if ((S19Address>= ((uint32_t)(__FLASHX_END_ADDR)) / 2) && 
    		(S19Address <  (uint32_t)(__FLASHX_END_ADDR))) {
    		flash_file = flash_file1;
    		fl_addr_res = S19Address - ((uint32_t)(__FLASHX_END_ADDR))/2;
    	} else {
        	res_burn_s19_line = 7;
    		goto err_unlock;
    	}
*/
//room420 no need		fseek(flash_file, fl_addr_res, IO_SEEK_SET);
/* room420 keenetic        
		if (write(flash_file, (uint32_t*)buffer_to_flash, i) != i) */
        fl_addr_res=S19Address;
//=======================  now programm flash =============================
                                __disable_interrupt();
    if(Program_block(S19Address,(uint32_t*)(buffer_to_flash),i) !=0){
            asm (" NOP");
                                    __enable_interrupt();
	    	res_burn_s19_line = (uint32_t) i;
			goto err_unlock;
		}
                                        __enable_interrupt();
//room420 no need		ioctl(flash_file, FLASH_IOCTL_FLUSH_BUFFER, NULL);
		break;
    case '7':
    case '8':
    case '9':
    	S19Address = 0;//(uint32_t) NULL; 
    	type = (unsigned char)(type - '0');
    	type = (unsigned char)(10 - type);
    	cur_point = 4;
    	checksum = length;
    	/* Get Address */
    	for (i = 0; i <= type; i++) {
    		for (j = 0; j < length - 1; j++) {
    			data = GetSpair(Line, cur_point, &res);
    			if (res < 0) {
    		    	res_burn_s19_line = 9;
        			goto err_unlock;
    			} /* EndIf */
    			checksum = (unsigned char)((data + checksum) & 0x00FF);
    			cur_point+=2;
    		} /* EndFor */

    		/* Read checksum value */
    		data = GetSpair(Line, cur_point, &res);
    		if (res < 0) {
    	    	res_burn_s19_line = 10;
    			goto err_unlock;
    		} /* EndIf */

    		/* Check checksum */
    		if (((data - ~checksum) & 0x00FF) != 0)	{
    	    	res_burn_s19_line = 11;
    			goto err_unlock;
    		} else {
    			return 0;
    		} /* EndIf */
    	} /* EndFor */
    	break;
    case '0':
    case '4':
    case '5':
    case '6':
    default:
    	break;
    } /* EndSwitch */
    return 0;
    
err_unlock:
	return -1;
}



