# 1 "./src/main/io/asyncfatfs/fat_standard.c"
# 1 "<built-in>"
# 1 "<command-line>"
# 1 "./src/main/io/asyncfatfs/fat_standard.c"
# 18 "./src/main/io/asyncfatfs/fat_standard.c"
# 1 "./src/main/ctype.h" 1
# 18 "./src/main/ctype.h"
int isalnum (int __c);
int isalpha (int __c);
int iscntrl (int __c);
int isdigit (int __c);
int isgraph (int __c);
int islower (int __c);
int isprint (int __c);
int ispunct (int __c);
int isspace (int __c);
int isupper (int __c);
int isxdigit (int __c);
int tolower (int __c);
int toupper (int __c);
int isblank (int __c);
# 19 "./src/main/io/asyncfatfs/fat_standard.c" 2

# 1 "./src/main/io/asyncfatfs/fat_standard.h" 1
# 18 "./src/main/io/asyncfatfs/fat_standard.h"
       

# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/lib/gcc/arm-none-eabi/7.2.1/include/stdint.h" 1 3 4
# 9 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/lib/gcc/arm-none-eabi/7.2.1/include/stdint.h" 3 4
# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/stdint.h" 1 3 4
# 12 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/stdint.h" 3 4
# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/machine/_default_types.h" 1 3 4







# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/features.h" 1 3 4
# 28 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/features.h" 3 4
# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/_newlib_version.h" 1 3 4
# 29 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/features.h" 2 3 4
# 9 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/machine/_default_types.h" 2 3 4
# 41 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/machine/_default_types.h" 3 4

# 41 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/machine/_default_types.h" 3 4
typedef signed char __int8_t;

typedef unsigned char __uint8_t;
# 55 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/machine/_default_types.h" 3 4
typedef short int __int16_t;

typedef short unsigned int __uint16_t;
# 77 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/machine/_default_types.h" 3 4
typedef long int __int32_t;

typedef long unsigned int __uint32_t;
# 103 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/machine/_default_types.h" 3 4
typedef long long int __int64_t;

typedef long long unsigned int __uint64_t;
# 134 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/machine/_default_types.h" 3 4
typedef signed char __int_least8_t;

typedef unsigned char __uint_least8_t;
# 160 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/machine/_default_types.h" 3 4
typedef short int __int_least16_t;

typedef short unsigned int __uint_least16_t;
# 182 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/machine/_default_types.h" 3 4
typedef long int __int_least32_t;

typedef long unsigned int __uint_least32_t;
# 200 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/machine/_default_types.h" 3 4
typedef long long int __int_least64_t;

typedef long long unsigned int __uint_least64_t;
# 214 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/machine/_default_types.h" 3 4
typedef long long int __intmax_t;







typedef long long unsigned int __uintmax_t;







typedef int __intptr_t;

typedef unsigned int __uintptr_t;
# 13 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/stdint.h" 2 3 4
# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/_intsup.h" 1 3 4
# 35 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/_intsup.h" 3 4
       
       
       
       
       
       
       
# 187 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/_intsup.h" 3 4
       
       
       
       
       
       
       
# 14 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/stdint.h" 2 3 4
# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/_stdint.h" 1 3 4
# 20 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/_stdint.h" 3 4
typedef __int8_t int8_t ;



typedef __uint8_t uint8_t ;







typedef __int16_t int16_t ;



typedef __uint16_t uint16_t ;







typedef __int32_t int32_t ;



typedef __uint32_t uint32_t ;







typedef __int64_t int64_t ;



typedef __uint64_t uint64_t ;






typedef __intmax_t intmax_t;




typedef __uintmax_t uintmax_t;




typedef __intptr_t intptr_t;




typedef __uintptr_t uintptr_t;
# 15 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/stdint.h" 2 3 4






typedef __int_least8_t int_least8_t;
typedef __uint_least8_t uint_least8_t;




typedef __int_least16_t int_least16_t;
typedef __uint_least16_t uint_least16_t;




typedef __int_least32_t int_least32_t;
typedef __uint_least32_t uint_least32_t;




typedef __int_least64_t int_least64_t;
typedef __uint_least64_t uint_least64_t;
# 51 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/stdint.h" 3 4
  typedef int int_fast8_t;
  typedef unsigned int uint_fast8_t;
# 61 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/stdint.h" 3 4
  typedef int int_fast16_t;
  typedef unsigned int uint_fast16_t;
# 71 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/stdint.h" 3 4
  typedef int int_fast32_t;
  typedef unsigned int uint_fast32_t;
# 81 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/stdint.h" 3 4
  typedef long long int int_fast64_t;
  typedef long long unsigned int uint_fast64_t;
# 10 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/lib/gcc/arm-none-eabi/7.2.1/include/stdint.h" 2 3 4
# 21 "./src/main/io/asyncfatfs/fat_standard.h" 2
# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/lib/gcc/arm-none-eabi/7.2.1/include/stdbool.h" 1 3 4
# 22 "./src/main/io/asyncfatfs/fat_standard.h" 2
# 53 "./src/main/io/asyncfatfs/fat_standard.h"

# 53 "./src/main/io/asyncfatfs/fat_standard.h"
typedef enum {
    FAT_FILESYSTEM_TYPE_INVALID,
    FAT_FILESYSTEM_TYPE_FAT12,
    FAT_FILESYSTEM_TYPE_FAT16,
    FAT_FILESYSTEM_TYPE_FAT32
} fatFilesystemType_e;

typedef struct mbrPartitionEntry_t {
    uint8_t bootFlag;
    uint8_t chsBegin[3];
    uint8_t type;
    uint8_t chsEnd[3];
    uint32_t lbaBegin;
    uint32_t numSectors;
} __attribute__((packed)) mbrPartitionEntry_t;

typedef struct fat16Descriptor_t {
    uint8_t driveNumber;
    uint8_t reserved1;
    uint8_t bootSignature;
    uint32_t volumeID;
    char volumeLabel[11];
    char fileSystemType[8];
} __attribute__((packed)) fat16Descriptor_t;

typedef struct fat32Descriptor_t {
    uint32_t FATSize32;
    uint16_t extFlags;
    uint16_t fsVer;
    uint32_t rootCluster;
    uint16_t fsInfo;
    uint16_t backupBootSector;
    uint8_t reserved[12];
    uint8_t driveNumber;
    uint8_t reserved1;
    uint8_t bootSignature;
    uint32_t volumeID;
    char volumeLabel[11];
    char fileSystemType[8];
} __attribute__((packed)) fat32Descriptor_t;

typedef struct fatVolumeID_t {
    uint8_t jmpBoot[3];
    char oemName[8];
    uint16_t bytesPerSector;
    uint8_t sectorsPerCluster;
    uint16_t reservedSectorCount;
    uint8_t numFATs;
    uint16_t rootEntryCount;
    uint16_t totalSectors16;
    uint8_t media;
    uint16_t FATSize16;
    uint16_t sectorsPerTrack;
    uint16_t numHeads;
    uint32_t hiddenSectors;
    uint32_t totalSectors32;
    union {
        fat16Descriptor_t fat16;
        fat32Descriptor_t fat32;
    } fatDescriptor;
} __attribute__((packed)) fatVolumeID_t;

typedef struct fatDirectoryEntry_t {
    char filename[11];
    uint8_t attrib;
    uint8_t ntReserved;
    uint8_t creationTimeTenths;
    uint16_t creationTime;
    uint16_t creationDate;
    uint16_t lastAccessDate;
    uint16_t firstClusterHigh;
    uint16_t lastWriteTime;
    uint16_t lastWriteDate;
    uint16_t firstClusterLow;
    uint32_t fileSize;
} __attribute__((packed)) fatDirectoryEntry_t;

uint32_t fat32_decodeClusterNumber(uint32_t clusterNumber);


# 132 "./src/main/io/asyncfatfs/fat_standard.h" 3 4
_Bool 
# 132 "./src/main/io/asyncfatfs/fat_standard.h"
    fat32_isEndOfChainMarker(uint32_t clusterNumber);

# 133 "./src/main/io/asyncfatfs/fat_standard.h" 3 4
_Bool 
# 133 "./src/main/io/asyncfatfs/fat_standard.h"
    fat16_isEndOfChainMarker(uint16_t clusterNumber);


# 135 "./src/main/io/asyncfatfs/fat_standard.h" 3 4
_Bool 
# 135 "./src/main/io/asyncfatfs/fat_standard.h"
    fat_isFreeSpace(uint32_t clusterNumber);


# 137 "./src/main/io/asyncfatfs/fat_standard.h" 3 4
_Bool 
# 137 "./src/main/io/asyncfatfs/fat_standard.h"
    fat_isDirectoryEntryTerminator(fatDirectoryEntry_t *entry);

# 138 "./src/main/io/asyncfatfs/fat_standard.h" 3 4
_Bool 
# 138 "./src/main/io/asyncfatfs/fat_standard.h"
    fat_isDirectoryEntryEmpty(fatDirectoryEntry_t *entry);

void fat_convertFilenameToFATStyle(const char *filename, uint8_t *fatFilename);
# 21 "./src/main/io/asyncfatfs/fat_standard.c" 2


# 22 "./src/main/io/asyncfatfs/fat_standard.c" 3 4
_Bool 
# 22 "./src/main/io/asyncfatfs/fat_standard.c"
    fat16_isEndOfChainMarker(uint16_t clusterNumber)
{
    return clusterNumber >= 0xFFF8;
}



# 28 "./src/main/io/asyncfatfs/fat_standard.c" 3 4
_Bool 
# 28 "./src/main/io/asyncfatfs/fat_standard.c"
    fat32_isEndOfChainMarker(uint32_t clusterNumber)
{
    return clusterNumber >= 0x0FFFFFF8;
}






uint32_t fat32_decodeClusterNumber(uint32_t clusterNumber)
{
    return clusterNumber & 0x0FFFFFFF;
}



# 44 "./src/main/io/asyncfatfs/fat_standard.c" 3 4
_Bool 
# 44 "./src/main/io/asyncfatfs/fat_standard.c"
    fat_isFreeSpace(uint32_t clusterNumber)
{
    return clusterNumber == 0;
}


# 49 "./src/main/io/asyncfatfs/fat_standard.c" 3 4
_Bool 
# 49 "./src/main/io/asyncfatfs/fat_standard.c"
    fat_isDirectoryEntryTerminator(fatDirectoryEntry_t *entry)
{
    return entry->filename[0] == 0x00;
}


# 54 "./src/main/io/asyncfatfs/fat_standard.c" 3 4
_Bool 
# 54 "./src/main/io/asyncfatfs/fat_standard.c"
    fat_isDirectoryEntryEmpty(fatDirectoryEntry_t *entry)
{
    return (unsigned char) entry->filename[0] == 0xE5;
}






void fat_convertFilenameToFATStyle(const char *filename, uint8_t *fatFilename)
{
    for (int i = 0; i < 8; i++) {
        if (*filename == '\0' || *filename == '.') {
            *fatFilename = ' ';
        } else {
            *fatFilename = toupper((unsigned char)*filename);
            filename++;
        }
        fatFilename++;
    }

    if (*filename == '.') {
        filename++;
    }

    for (int i = 0; i < 3; i++) {
         if (*filename == '\0') {
             *fatFilename = ' ';
         } else {
             *fatFilename = toupper((unsigned char)*filename);
             filename++;
         }
         fatFilename++;
     }
}
