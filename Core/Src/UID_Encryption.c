#include "UID_Encryption.h"

/*
P100: 继承于最新的Minipro脱机下载器UID加密方法为V1.2版本


V1.2:
    
    更新：
        修正配置密钥长度为4字节、8字节时，取用户自定义 ID越界导致运算的密钥结果不正确的问题。
        MINI-Pro脱机下载器 V1.20B 版本固件后使用V1.2版本代码；
        MINI脱机下载器 V1.4A2 版本固件后使用V1.2版本代码；
        
    影响：
        前提条件：
            (1).MINI版脱机下载器在升级了固件到V1.4A2后，使得MINI脱机下载器内置的是V1.2版本代码;
            (2).MINI-Pro版脱机下载器在升级了固件到V1.20B后，使得MINI脱机下载器内置的是V1.2版本代码;
            (3).密钥长度配置为4字节、8字节的情况；
        导致现象：
            (1).由于修正了算法里密钥长度为 4/8字节时的运算逻辑，会导致运算出的密钥结果（仅密钥长度为4/8字节受影响）
            与V1.1版本前的代码运算结果不同！
        解决办法：
            (1).MINI-Pro版脱机下载器已经升级到V1.20B版本固件的用户可以回退到旧版本固件继续使用。
            (2).MINI版脱机下载器已经升级到V1.4A2版本固件的用户可以回退到旧版本固件继续使用。
        不受影响的情况：
            (1).只要密钥长度为12字节均不受影响；
V1.1:
    
    更新：
        MINI版脱机下载器V1,3A5版固件使用V1.1版本代码，V1.1版本代码修复了V1.0版本
        调用UID_Encryption_Key_Check(),UID_Encryption_Key_Calculate()使用大端
        模式有可能出现的内存越界问题。
    影响：
        前提条件：
            (1).在升级了MINI版脱机下载器的固件到V1.3A5后，使得MINI脱机下载器内置的是V1.1版本代码;
            (2).此前 使用V1.3A4及之前版本固件的MINI脱机下载器 配置为大端模式，且用户程序使用的是最初的V1.0版本代码；
        导致现象：
            (1).后续使用V1.3A5版本固件的MINI脱机下载器，开启了UID自定义加密功能，然后继续烧录包含了V1.0版本代码的用户程序，
            由于V1.1版本计算的大端模式密钥与V1.0版本计算的密钥不一致，因此会导致芯片中会出现会造成密钥验证不通过的问题！
        解决办法：
            (1).将用户程序中的V1.0版代码同步更新到V1.1版代码。
        不受影响的情况：
            (1).此前使用 小端模式 配置的用户产品不会受到V1.1版本代码更新以及MINI脱机下载器固件更新的影响；
            (2).如果已经使用V1.0代码的产品后续使用IAP方式升级则不会受到影响；
            (3).如果已经使用V1.0代码的产品后续使用MINI脱机下载器升级产品 且 不开启UID自定义加密功能重新更新密钥不会受到影响，
            但是如果使用脱机下载器升级产品的话，建议更新用户代码的了的UID自定义加密代码为V1.1，同时开启UID自定义加密重新更新密钥；
            (4)MINI-Pro版脱机下载器首版固件中使用的是V1.1版本的代码，不受影响；
V1.0：
    MINI版脱机下载器V1.3A4版固件及之前的固件中均使用了V1.0版本的代码；
*/

   
/****************************************** UID加密算法 start ******************************************/
/* UID加密算法0 */
static void Algorithm_0(char *pCustomID, char *pUID, char KeyLength, char *pKey)
{
    pKey[0] = (pUID[0]    ^ (pCustomID[3] & pCustomID[2])) ^ pCustomID[1];
    pKey[1] = pCustomID[1] | (pUID[2]    ^ (pCustomID[3] & pCustomID[3]));
    pKey[2] = (pCustomID[2] ^ pCustomID[1]) | (pUID[0]    ^ pCustomID[0]);
    pKey[3] = ((pCustomID[3] & pCustomID[0]) ^ pCustomID[1]) | pUID[2];
    
    if(KeyLength == 8)
    {
        pKey[4] = (pUID[4]    ^ (pCustomID[7] & pCustomID[5])) ^ pCustomID[5];
        pKey[5] = pCustomID[5] | (pUID[6]    ^ (pCustomID[4] & pCustomID[7]));
        pKey[6] = (pCustomID[6] ^ pCustomID[5]) | (pUID[7]    ^ pCustomID[4]);
        pKey[7] = ((pCustomID[7] & pCustomID[4]) ^ pCustomID[6]) | pUID[6];
    }
    if(KeyLength == 12)
    {
        pKey[0] = (pUID[0]    ^ (pCustomID[1] & pCustomID[2])) ^ pCustomID[3];
        pKey[1] = pCustomID[1] | (pUID[1]    ^ (pCustomID[3] & pCustomID[4]));
        pKey[2] = (pCustomID[2] ^ pCustomID[1]) | (pUID[2]    ^ pCustomID[5]);
        pKey[3] = ((pCustomID[3] & pCustomID[1]) ^ pCustomID[5]) | pUID[3];
        pKey[4] = (pUID[4]    ^ (pCustomID[5] & pCustomID[6])) ^ pCustomID[7];
        pKey[5] = pCustomID[5] | (pUID[5]    ^ (pCustomID[7] & pCustomID[8]));
        pKey[6] = (pCustomID[6] ^ pCustomID[7]) | (pUID[6]    ^ pCustomID[9]);
        pKey[7] = ((pCustomID[7] & pCustomID[8]) ^ pCustomID[9]) | pUID[7];
        pKey[8] = (pUID[8]    ^ (pCustomID[9] & pCustomID[10])) ^ pCustomID[11];
        pKey[9] = pCustomID[9] | (pUID[9]    ^ (pCustomID[11] & pCustomID[0]));
        pKey[10]= (pCustomID[10] ^ pCustomID[11]) | (pUID[0]   ^ pCustomID[1]);
        pKey[11]= ((pCustomID[11] & pCustomID[0]) ^ pCustomID[1]) | pUID[2];
    }
       
}



/* UID加密算法1 */
static void Algorithm_1(char *pCustomID, char *pUID, char KeyLength, char *pKey)
{
    pKey[0] = ((pUID[0]    ^ pCustomID[1]) ^ pCustomID[2]) ^ pCustomID[3];
    pKey[1] = ((pCustomID[1] ^ pUID[1])    ^ pCustomID[3]) ^ pCustomID[0];
    pKey[2] = ((pCustomID[2] ^ pCustomID[1]) ^ pUID[2])    ^ pCustomID[3];
    pKey[3] = ((pCustomID[3] ^ pCustomID[1]) ^ pCustomID[0]) ^ pUID[3];
    
    if(KeyLength == 8)
    {
        pKey[4] = ((pUID[4]    ^ pCustomID[5]) ^ pCustomID[6]) ^ pCustomID[7];
        pKey[5] = ((pCustomID[5] ^ pUID[5])    ^ pCustomID[7]) ^ pCustomID[2];
        pKey[6] = ((pCustomID[6] ^ pCustomID[7]) ^ pUID[6])    ^ pCustomID[3];
        pKey[7] = ((pCustomID[7] ^ pCustomID[4]) ^ pCustomID[5]) ^ pUID[4];
    }
    if(KeyLength == 12)
    {
        pKey[0] = ((pUID[0]    ^ pCustomID[1]) ^ pCustomID[2]) ^ pCustomID[3];
        pKey[1] = ((pCustomID[1] ^ pUID[1])    ^ pCustomID[3]) ^ pCustomID[4];
        pKey[2] = ((pCustomID[2] ^ pCustomID[1]) ^ pUID[2])    ^ pCustomID[5];
        pKey[3] = ((pCustomID[3] ^ pCustomID[1]) ^ pCustomID[5]) ^ pUID[3];
        pKey[4] = ((pUID[4]    ^ pCustomID[5]) ^ pCustomID[6]) ^ pCustomID[7];
        pKey[5] = ((pCustomID[5] ^ pUID[5])    ^ pCustomID[7]) ^ pCustomID[8];
        pKey[6] = ((pCustomID[6] ^ pCustomID[7]) ^ pUID[6])    ^ pCustomID[9];
        pKey[7] = ((pCustomID[7] ^ pCustomID[8]) ^ pCustomID[9]) ^ pUID[7];
        pKey[8] = ((pUID[8]    ^ pCustomID[9]) ^ pCustomID[10]) ^ pCustomID[11];
        pKey[9] = ((pCustomID[9] ^ pUID[9])    ^ pCustomID[11]) ^ pCustomID[0];
        pKey[10]= ((pCustomID[10] ^ pCustomID[11]) ^ pUID[0])   ^ pCustomID[1];
        pKey[11]= ((pCustomID[11] ^ pCustomID[0]) ^ pCustomID[1]) ^ pUID[2];
    }
}


/* UID加密算法2 */
static void Algorithm_2(char *pCustomID, char *pUID, char KeyLength, char *pKey)
{
    pKey[0] = ((pUID[0]    & pCustomID[1]) ^ pCustomID[2]) ^ pCustomID[3];
    pKey[1] = (pCustomID[1] ^ (pUID[1]    & pCustomID[3])) ^ pCustomID[0];
    pKey[2] = (pCustomID[2] | (pCustomID[1] ^ pUID[2]))    | pCustomID[3];
    pKey[3] = (pCustomID[3] ^ pCustomID[1]) | (pCustomID[0] ^ pUID[3]);
    
    if(KeyLength == 8)  
    {
        pKey[4] = (pUID[4]    ^ (pCustomID[5] & pCustomID[6])) ^ pCustomID[7];
        pKey[5] = pCustomID[5] | (pUID[5]    ^ (pCustomID[7] & pCustomID[2]));
        pKey[6] = ((pCustomID[6] & pCustomID[7]) ^ pUID[6])    | pCustomID[3];
        pKey[7] = (pCustomID[7] ^ pCustomID[4]) | (pCustomID[1] ^ pUID[2]);
    }
    if(KeyLength == 12)
    {
        pKey[0] = ((pUID[0]    & pCustomID[1]) ^ pCustomID[2]) ^ pCustomID[3];
        pKey[1] = (pCustomID[1] ^ (pUID[1]    & pCustomID[3])) ^ pCustomID[4];
        pKey[2] = (pCustomID[2] | (pCustomID[1] ^ pUID[2]))    | pCustomID[5];
        pKey[3] = (pCustomID[3] ^ pCustomID[1]) | (pCustomID[5] ^ pUID[3]);
        pKey[4] = (pUID[4]    ^ (pCustomID[5] & pCustomID[6])) ^ pCustomID[7];
        pKey[5] = pCustomID[5] | (pUID[5]    ^ (pCustomID[7] & pCustomID[8]));
        pKey[6] = ((pCustomID[6] & pCustomID[7]) ^ pUID[6])    | pCustomID[9];
        pKey[7] = (pCustomID[7] ^ pCustomID[8]) | (pCustomID[9] ^ pUID[7]);
        pKey[8] = (pUID[8]    ^ (pCustomID[9] & pCustomID[10])) ^ pCustomID[11];
        pKey[9] = (pCustomID[9] ^ pUID[9])    | (pCustomID[11] & pCustomID[0]);
        pKey[10]= ((pCustomID[10] & pCustomID[11]) ^ pUID[0])   ^ pCustomID[1];
        pKey[11]= (pCustomID[11] ^ (pCustomID[0] & pCustomID[1])) ^ pUID[2];
    }
}

/* UID加密算法3 */
static void Algorithm_3(char *pCustomID, char *pUID, char KeyLength, char *pKey)
{
    pKey[0] = ((pUID[0] & pCustomID[1]) ^ pCustomID[2]) ^ pCustomID[3];
    pKey[1] = (pUID[1] ^ (pCustomID[2] & pCustomID[3])) ^ pCustomID[4];
    pKey[2] = (pUID[2] | (pCustomID[3] ^ pCustomID[4])) | pCustomID[5];
    pKey[3] = (pUID[3] ^ pCustomID[4]) | (pCustomID[5] ^ pCustomID[6]);
    
    if(KeyLength == 8)  
    {
        pKey[4] = (pCustomID[4] ^ (pUID[5] & pCustomID[6])) ^ pCustomID[7];
        pKey[5] = pCustomID[5] | (pUID[6] ^ (pCustomID[7] & pCustomID[4]));
        pKey[6] = ((pCustomID[6] & pUID[7]) ^ pCustomID[6]) | pCustomID[3];
        pKey[7] = (pCustomID[7] ^ pUID[4]) | (pCustomID[5] ^ pCustomID[6]);
    }
    if(KeyLength == 12)
    {
        pKey[0] = ((pUID[0] & pCustomID[1]) ^ pCustomID[2]) ^ pCustomID[3];
        pKey[1] = (pUID[1] ^ (pCustomID[2] & pCustomID[3])) ^ pCustomID[4];
        pKey[2] = (pUID[2] | (pCustomID[3] ^ pCustomID[4])) | pCustomID[5];
        pKey[3] = (pUID[3] ^ pCustomID[4]) | (pCustomID[5] ^ pCustomID[6]);
        pKey[4] = (pCustomID[4] ^ (pUID[5] & pCustomID[6])) ^ pCustomID[7];
        pKey[5] = pCustomID[5] | (pUID[6] ^ (pCustomID[7] & pCustomID[8]));
        pKey[6] = ((pCustomID[6] & pUID[7]) ^ pCustomID[6]) | pCustomID[9];
        pKey[7] = (pCustomID[7] ^ pUID[8]) | (pCustomID[9] ^ pCustomID[10]);
        pKey[8] = (pCustomID[8]  ^ (pCustomID[9]  & pUID[10])) ^ pCustomID[11];
        pKey[9] = (pCustomID[9]  ^ pCustomID[10]) | (pUID[11] & pCustomID[0]);
        pKey[10]= ((pCustomID[10] & pCustomID[11]) ^ pUID[0])  ^ pCustomID[1];
        pKey[11]= (pCustomID[11] ^ (pCustomID[0]  & pUID[1]))  ^ pCustomID[2];
    }
}


/* UID加密算法4 */
static void Algorithm_4(char *pCustomID, char *pUID, char KeyLength, char *pKey) 
{
    pKey[0] = (pUID[0] & pCustomID[1]) ^ (pUID[2] & pCustomID[3]);
    pKey[1] = (pCustomID[1] ^ (pUID[2] & pCustomID[3])) ^ pUID[4];
    pKey[2] = (pCustomID[2] | (pUID[3] ^ pUID[4])) | pCustomID[3];
    pKey[3] = (pUID[3] - pCustomID[0]) | (pCustomID[2] ^ pUID[3]);
    
    if(KeyLength == 8)  
    {
        pKey[4] = (pUID[4] ^ (pCustomID[5] & pUID[6])) ^ pCustomID[7];
        pKey[5] = pCustomID[5] | (pUID[6] ^ (pCustomID[7] & pUID[8]));
        pKey[6] = ((pCustomID[6] & pUID[7]) ^ pCustomID[6]) | pUID[9];
        pKey[7] = (pUID[7] - pCustomID[3]) | (pUID[9] ^ pCustomID[5]);
    }
    if(KeyLength == 12)
    {
        pKey[0] = (pUID[0] & pCustomID[1]) ^ (pUID[2] & pCustomID[3]);
        pKey[1] = (pCustomID[1] ^ (pUID[2] & pCustomID[3])) ^ pUID[4];
        pKey[2] = (pCustomID[2] | (pUID[3] ^ pUID[4])) | pCustomID[5];
        pKey[3] = (pUID[3] - pCustomID[4]) | (pCustomID[5] ^ pUID[6]);
        pKey[4] = (pUID[4] ^ (pCustomID[5] & pUID[6])) ^ pCustomID[7];
        pKey[5] = pCustomID[5] | (pUID[6] ^ (pCustomID[7] & pUID[8]));
        pKey[6] = ((pCustomID[6] & pUID[7]) ^ pCustomID[6]) | pUID[9];
        pKey[7] = (pUID[7] - pCustomID[8]) | (pUID[9] ^ pCustomID[10]);
        pKey[8] = (pCustomID[8] ^ (pUID[9] & pCustomID[10])) ^ pUID[11];
        pKey[9] = (pUID[9] ^ pCustomID[10]) | (pUID[11] & pCustomID[0]);
        pKey[10]= ((pUID[10] & pCustomID[11]) ^ pUID[0])  ^ pCustomID[1];
        pKey[11]= ((pCustomID[11] - pUID[0]) & pCustomID[1])  ^ pUID[2];
    }
}



/* 定义UID加密算法 函数指针类型 */
typedef void (*Algorithm_Fun_Typedef)(char *pCustomID, char *pUID, char KeyLength, char *pKey);
/* 定义UID加密算法 函数指针数组 */
static const Algorithm_Fun_Typedef Algorithm_Fun_Array[5] = 
{
    Algorithm_0,
    Algorithm_1,
    Algorithm_2,
    Algorithm_3,
    Algorithm_4,
};
/****************************************** UID加密算法 end ******************************************/    



/* 大小端序切换 */      
#define BigLittleSwap32(A)          ((((unsigned int)(A) & 0xFF000000) >> 24) | \
                                     (((unsigned int)(A) & 0x00FF0000) >> 8)  | \
                                     (((unsigned int)(A) & 0x0000FF00) << 8)  | \
                                     (((unsigned int)(A) & 0x000000FF) << 24)) 
                                        
void LittleEndia_BigEndia_Interconvert_32(unsigned int* pBuffer, unsigned int WordLen)
{
    unsigned int i;
    
    for(i = 0; i < WordLen; i ++)
    {
        pBuffer[i] = BigLittleSwap32(pBuffer[i]);
    }
    return;
}  



static Algorithm_Fun_Typedef   pAlgorithm_Fun;
/****************** UID 算法加密密钥验证 ******************/
char UID_Encryption_Key_Check(void *pKey,
                              void *pUID,
                              void *pCustomID,
                              eKeyLengthType keyLength, 
                              eEndiaType endiaType, 
                              eAlgorithmType AlgorithmNum)
{   
    char KeyBuf[12] = {0};//临时存储计算得到的Key
    
    
    /* 选择算法公式 */
    pAlgorithm_Fun = Algorithm_Fun_Array[AlgorithmNum];
    /* 计算KEY */
    (*pAlgorithm_Fun)(pCustomID, pUID, keyLength, KeyBuf);   

    if(endiaType == BIG_ENDIA)
    {
        LittleEndia_BigEndia_Interconvert_32((unsigned int*)KeyBuf, keyLength/sizeof(unsigned int));
    }
    /* 比较脱机下载器烧录的Key与使用相同算法计算得到的key是否相同 */
    if(memcmp(pKey, KeyBuf, keyLength)) 
    {
        return 1;
    }
    
    return 0; 
}
/****************** UID 算法加密密钥计算 ******************/
void UID_Encryption_Key_Calculate(void *pKey,
                                 void *pUID,
                                 void *pCustomID,
                                 eKeyLengthType keyLength, 
                                 eEndiaType endiaType, 
                                 eAlgorithmType AlgorithmNum)
{    
    /* 选择算法公式 */
    pAlgorithm_Fun = Algorithm_Fun_Array[AlgorithmNum];
    /* 计算KEY */
    (*pAlgorithm_Fun)(pCustomID, pUID, keyLength, pKey);   

    if(endiaType == BIG_ENDIA)
    {
        LittleEndia_BigEndia_Interconvert_32((unsigned int*)pKey, keyLength/sizeof(unsigned int));
    }
    
    return; 
}








