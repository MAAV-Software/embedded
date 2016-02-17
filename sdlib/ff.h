/*---------------------------------------------------------------------------/
/  FatFs - FAT file system module include R0.11     (C)ChaN, 2015
/----------------------------------------------------------------------------/
/ FatFs module is a free software that opened under license policy of
/ following conditions.
/
/ Copyright (C) 2015, ChaN, all right reserved.
/
/ 1. Redistributions of source code must retain the above copyright notice,
/    this condition and the following disclaimer.
/
/ This software is provided by the copyright holder and contributors "AS IS"
/ and any warranties related to this software are DISCLAIMED.
/ The copyright owner or contributors be NOT LIABLE for any damages caused
/ by use of this software.
/---------------------------------------------------------------------------*/


#ifndef _FATFS
#define _FATFS	32020	/**< @brief Revision ID */

#ifdef __cplusplus
extern "C" {
#endif

#include "integer.h"	/* Basic integer types */
#include "ffconf.h"		/* FatFs configuration options */
#if _FATFS != _FFCONF
#error Wrong configuration file (ffconf.h).
#endif



/* Definitions of volume management */

#if _MULTI_PARTITION		/* Multiple partition configuration */
typedef struct {
	BYTE pd;	/* Physical drive number */
	BYTE pt;	/* Partition: 0:Auto detect, 1-4:Forced partition) */
} PARTITION;
extern PARTITION VolToPart[];	/* Volume - Partition resolution table */
#define LD2PD(vol) (VolToPart[vol].pd)	/**< @brief Get physical drive number */
#define LD2PT(vol) (VolToPart[vol].pt)	/**< @brief Get partition index */

#else							/* Single partition configuration */
#define LD2PD(vol) (BYTE)(vol)	/**< @brief Each logical drive is bound to the same physical drive number */
#define LD2PT(vol) 0			/**< @brief Find first valid partition or in SFD */

#endif



/* Type of path name strings on FatFs API */

#if _LFN_UNICODE			/**< @brief Unicode string */
#if !_USE_LFN
#error _LFN_UNICODE must be 0 at non-LFN cfg.
#endif
#ifndef _INC_TCHAR
typedef WCHAR TCHAR;
#define _T(x) L ## x
#define _TEXT(x) L ## x
#endif

#else						/**< @brief ANSI/OEM string */
#ifndef _INC_TCHAR
typedef char TCHAR;
#define _T(x) x
#define _TEXT(x) x
#endif

#endif



/* File system object structure (FATFS) */

typedef struct {
	BYTE	fs_type;	/**< @brief FAT sub-type (0:Not mounted) */
	BYTE	drv;		/**< @brief Physical drive number */
	BYTE	csize;		/**< @brief Sectors per cluster (1,2,4...128) */
	BYTE	n_fats;		/**< @brief Number of FAT copies (1 or 2) */
	BYTE	wflag;		/**< @brief win[] flag (b0:dirty) */
	BYTE	fsi_flag;	/**< @brief FSINFO flags (b7:disabled, b0:dirty) */
	WORD	id;		/**< @brief File system mount ID */
	WORD	n_rootdir;	/**< @brief Number of root directory entries (FAT12/16) */
#if _MAX_SS != _MIN_SS
	WORD	ssize;		/**< @brief Bytes per sector (512, 1024, 2048 or 4096) */
#endif
#if _FS_REENTRANT
	_SYNC_t	sobj;		/**< @brief Identifier of sync object */
#endif
#if !_FS_READONLY
	DWORD	last_clust;	/**< @brief Last allocated cluster */
	DWORD	free_clust;	/**< @brief Number of free clusters */
#endif
#if _FS_RPATH
	DWORD	cdir;		/**< @brief Current directory start cluster (0:root) */
#endif
	DWORD	n_fatent;	/**< @brief Number of FAT entries, = number of clusters + 2 */
	DWORD	fsize;		/**< @brief Sectors per FAT */
	DWORD	volbase;	/**< @brief Volume start sector */
	DWORD	fatbase;	/**< @brief FAT start sector */
	DWORD	dirbase;	/**< @brief Root directory start sector (FAT32:Cluster#) */
	DWORD	database;	/**< @brief Data start sector */
	DWORD	winsect;	/**< @brief Current sector appearing in the win[] */
	BYTE	win[_MAX_SS];	/**< @brief Disk access window for Directory, FAT (and file data at tiny cfg) */
} FATFS;



/* File object structure (FIL) */

typedef struct {
	FATFS*	fs;		/**< @brief Pointer to the related file system object (**do not change order**) */
	WORD	id;		/**< @brief Owner file system mount ID (**do not change order**) */
	BYTE	flag;		/**< @brief Status flags */
	BYTE	err;		/**< @brief Abort flag (error code) */
	DWORD	fptr;		/**< @brief File read/write pointer (Zeroed on file open) */
	DWORD	fsize;		/**< @brief File size */
	DWORD	sclust;		/**< @brief File start cluster (0:no cluster chain, always 0 when fsize is 0) */
	DWORD	clust;		/**< @brief Current cluster of fpter (not valid when fprt is 0) */
	DWORD	dsect;		/**< @brief Sector number appearing in buf[] (0:invalid) */
#if !_FS_READONLY
	DWORD	dir_sect;	/**< @brief Sector number containing the directory entry */
	BYTE*	dir_ptr;	/**< @brief Pointer to the directory entry in the win[] */
#endif
#if _USE_FASTSEEK
	DWORD*	cltbl;		/**< @brief Pointer to the cluster link map table (Nulled on file open) */
#endif
#if _FS_LOCK
	UINT	lockid;		/**< @brief File lock ID origin from 1 (index of file semaphore table Files[]) */
#endif
#if !_FS_TINY
	BYTE	buf[_MAX_SS];	/**< @brief File private data read/write window */
#endif
} FIL;



/* Directory object structure (DIR) */

typedef struct {
	FATFS*	fs;		/**< @brief Pointer to the owner file system object (**do not change order**) */
	WORD	id;		/**< @brief Owner file system mount ID (**do not change order**) */
	WORD	index;		/**< @brief Current read/write index number */
	DWORD	sclust;		/**< @brief Table start cluster (0:Root dir) */
	DWORD	clust;		/**< @brief Current cluster */
	DWORD	sect;		/**< @brief Current sector */
	BYTE*	dir;		/**< @brief Pointer to the current SFN entry in the win[] */
	BYTE*	fn;		/**< @brief Pointer to the SFN (in/out) {file[8],ext[3],status[1]} */
#if _FS_LOCK
	UINT	lockid;		/**< @brief File lock ID (index of file semaphore table Files[]) */
#endif
#if _USE_LFN
	WCHAR*	lfn;		/**< @brief Pointer to the LFN working buffer */
	WORD	lfn_idx;	/**< @brief Last matched LFN index number (0xFFFF:No LFN) */
#endif
#if _USE_FIND
	const TCHAR*	pat;	/**< @brief Pointer to the name matching pattern */
#endif
} DIR;



/* File information structure (FILINFO) */

typedef struct {
	DWORD	fsize;		/**< @brief File size */
	WORD	fdate;		/**< @brief Last modified date */
	WORD	ftime;		/**< @brief Last modified time */
	BYTE	fattrib;	/**< @brief Attribute */
	TCHAR	fname[13];	/**< @brief Short file name (8.3 format) */
#if _USE_LFN
	TCHAR*	lfname;		/**< @brief Pointer to the LFN buffer */
	UINT 	lfsize;		/**< @brief Size of LFN buffer in TCHAR */
#endif
} FILINFO;



/* File function return code (FRESULT) */

typedef enum {
	FR_OK = 0,		/**< @brief (0) Succeeded */
	FR_DISK_ERR,		/**< @brief (1) A hard error occurred in the low level disk I/O layer */
	FR_INT_ERR,		/**< @brief (2) Assertion failed */
	FR_NOT_READY,		/**< @brief (3) The physical drive cannot work */
	FR_NO_FILE,		/**< @brief (4) Could not find the file */
	FR_NO_PATH,		/**< @brief (5) Could not find the path */
	FR_INVALID_NAME,	/**< @brief (6) The path name format is invalid */
	FR_DENIED,		/**< @brief (7) Access denied due to prohibited access or directory full */
	FR_EXIST,		/**< @brief (8) Access denied due to prohibited access */
	FR_INVALID_OBJECT,	/**< @brief (9) The file/directory object is invalid */
	FR_WRITE_PROTECTED,	/**< @brief (10) The physical drive is write protected */
	FR_INVALID_DRIVE,	/**< @brief (11) The logical drive number is invalid */
	FR_NOT_ENABLED,		/**< @brief (12) The volume has no work area */
	FR_NO_FILESYSTEM,	/**< @brief (13) There is no valid FAT volume */
	FR_MKFS_ABORTED,	/**< @brief (14) The f_mkfs() aborted due to any parameter error */
	FR_TIMEOUT,		/**< @brief (15) Could not get a grant to access the volume within defined period */
	FR_LOCKED,		/**< @brief (16) The operation is rejected according to the file sharing policy */
	FR_NOT_ENOUGH_CORE,	/**< @brief (17) LFN working buffer could not be allocated */
	FR_TOO_MANY_OPEN_FILES,	/**< @brief (18) Number of open files **< _FS_SHARE */
	FR_INVALID_PARAMETER	/**< @brief (19) Given parameter is invalid */
} FRESULT;



/*--------------------------------------------------------------*/
/* FatFs module application interface                           */

FRESULT f_open (FIL* fp, const TCHAR* path, BYTE mode);	/**< @brief Open or create a file */
FRESULT f_close (FIL* fp);				/**< @brief Close an open file object */
FRESULT f_read (FIL* fp, void* buff, UINT btr, UINT* br);	/**< @brief Read data from a file */
FRESULT f_write (FIL* fp, const void* buff, UINT btw, UINT* bw); /**< @brief Write data to a file */
FRESULT f_forward (FIL* fp, UINT(*func)(const BYTE*,UINT), UINT btf, UINT* bf);	/**< @brief Forward data to the stream */
FRESULT f_lseek (FIL* fp, DWORD ofs); /**< @brief Move file pointer of a file object */
FRESULT f_truncate (FIL* fp); /**< @brief Truncate file */
FRESULT f_sync (FIL* fp); /**< @brief Flush cached data of a writing file */
FRESULT f_opendir (DIR* dp, const TCHAR* path); /**< @brief Open a directory */
FRESULT f_closedir (DIR* dp); /**< @brief Close an open directory */
FRESULT f_readdir (DIR* dp, FILINFO* fno); /**< @brief Read a directory item */
FRESULT f_findfirst (DIR* dp, FILINFO* fno, const TCHAR* path, const TCHAR* pattern); /**< @brief Find first file */
FRESULT f_findnext (DIR* dp, FILINFO* fno); /**< @brief Find next file */
FRESULT f_mkdir (const TCHAR* path); /**< @brief Create a sub directory */
FRESULT f_unlink (const TCHAR* path); /**< @brief Delete an existing file or directory */
FRESULT f_rename (const TCHAR* path_old, const TCHAR* path_new); /**< @brief Rename/Move a file or directory */
FRESULT f_stat (const TCHAR* path, FILINFO* fno); /**< @brief Get file status */
FRESULT f_chmod (const TCHAR* path, BYTE attr, BYTE mask); /**< @brief Change attribute of the file/dir */
FRESULT f_utime (const TCHAR* path, const FILINFO* fno); /**< @brief Change times-tamp of the file/dir */
FRESULT f_chdir (const TCHAR* path); /**< @brief Change current directory */
FRESULT f_chdrive (const TCHAR* path); /**< @brief Change current drive */
FRESULT f_getcwd (TCHAR* buff, UINT len); /**< @brief Get current directory */
FRESULT f_getfree (const TCHAR* path, DWORD* nclst, FATFS** fatfs); /**< @brief Get number of free clusters on the drive */
FRESULT f_getlabel (const TCHAR* path, TCHAR* label, DWORD* vsn); /**< @brief Get volume label */
FRESULT f_setlabel (const TCHAR* label); /**< @brief Set volume label */
FRESULT f_mount (FATFS* fs, const TCHAR* path, BYTE opt); /**< @brief Mount/Unmount a logical drive */
FRESULT f_mkfs (const TCHAR* path, BYTE sfd, UINT au); /**< @brief Create a file system on the volume */
FRESULT f_fdisk (BYTE pdrv, const DWORD szt[], void* work); /**< @brief Divide a physical drive into some partitions */
int f_putc (TCHAR c, FIL* fp); /**< @brief Put a character to the file */
int f_puts (const TCHAR* str, FIL* cp); /**< @brief Put a string to the file */
int f_printf (FIL* fp, const TCHAR* str, ...); /**< @brief Put a formatted string to the file */
TCHAR* f_gets (TCHAR* buff, int len, FIL* fp); /**< @brief Get a string from the file */

#define f_eof(fp) ((int)((fp)->fptr == (fp)->fsize))
#define f_error(fp) ((fp)->err)
#define f_tell(fp) ((fp)->fptr)
#define f_size(fp) ((fp)->fsize)
#define f_rewind(fp) f_lseek((fp), 0)
#define f_rewinddir(dp) f_readdir((dp), 0)

#ifndef EOF
#define EOF (-1)
#endif




/*--------------------------------------------------------------*/
/* Additional user defined functions                            */

/* RTC function */
#if !_FS_READONLY && !_FS_NORTC
DWORD get_fattime (void);
#endif

/* Unicode support functions */
#if _USE_LFN							/* Unicode - OEM code conversion */
WCHAR ff_convert (WCHAR chr, UINT dir);	/* OEM-Unicode bidirectional conversion */
WCHAR ff_wtoupper (WCHAR chr);			/* Unicode upper-case conversion */
#if _USE_LFN == 3						/* Memory functions */
void* ff_memalloc (UINT msize);			/* Allocate memory block */
void ff_memfree (void* mblock);			/* Free memory block */
#endif
#endif

/* Sync functions */
#if _FS_REENTRANT
int ff_cre_syncobj (BYTE vol, _SYNC_t* sobj);	/* Create a sync object */
int ff_req_grant (_SYNC_t sobj);				/* Lock sync object */
void ff_rel_grant (_SYNC_t sobj);				/* Unlock sync object */
int ff_del_syncobj (_SYNC_t sobj);				/* Delete a sync object */
#endif




/*--------------------------------------------------------------*/
/* Flags and offset address                                     */


/* File access control and file status flags (FIL.flag) */

#define	FA_READ				0x01
#define	FA_OPEN_EXISTING	0x00

#if !_FS_READONLY
#define	FA_WRITE			0x02
#define	FA_CREATE_NEW		0x04
#define	FA_CREATE_ALWAYS	0x08
#define	FA_OPEN_ALWAYS		0x10
#define FA__WRITTEN			0x20
#define FA__DIRTY			0x40
#endif


/* FAT sub type (FATFS.fs_type) */

#define FS_FAT12	1
#define FS_FAT16	2
#define FS_FAT32	3


/* File attribute bits for directory entry */

#define	AM_RDO	0x01	/* Read only */
#define	AM_HID	0x02	/* Hidden */
#define	AM_SYS	0x04	/* System */
#define	AM_VOL	0x08	/* Volume label */
#define AM_LFN	0x0F	/* LFN entry */
#define AM_DIR	0x10	/* Directory */
#define AM_ARC	0x20	/* Archive */
#define AM_MASK	0x3F	/* Mask of defined bits */


/* Fast seek feature */
#define CREATE_LINKMAP	0xFFFFFFFF



/*--------------------------------*/
/* Multi-byte word access macros  */

#if _WORD_ACCESS == 1	/* Enable word access to the FAT structure */
#define	LD_WORD(ptr)		(WORD)(*(WORD*)(BYTE*)(ptr))
#define	LD_DWORD(ptr)		(DWORD)(*(DWORD*)(BYTE*)(ptr))
#define	ST_WORD(ptr,val)	*(WORD*)(BYTE*)(ptr)=(WORD)(val)
#define	ST_DWORD(ptr,val)	*(DWORD*)(BYTE*)(ptr)=(DWORD)(val)
#else					/* Use byte-by-byte access to the FAT structure */
#define	LD_WORD(ptr)		(WORD)(((WORD)*((BYTE*)(ptr)+1)<<8)|(WORD)*(BYTE*)(ptr))
#define	LD_DWORD(ptr)		(DWORD)(((DWORD)*((BYTE*)(ptr)+3)<<24)|((DWORD)*((BYTE*)(ptr)+2)<<16)|((WORD)*((BYTE*)(ptr)+1)<<8)|*(BYTE*)(ptr))
#define	ST_WORD(ptr,val)	*(BYTE*)(ptr)=(BYTE)(val); *((BYTE*)(ptr)+1)=(BYTE)((WORD)(val)>>8)
#define	ST_DWORD(ptr,val)	*(BYTE*)(ptr)=(BYTE)(val); *((BYTE*)(ptr)+1)=(BYTE)((WORD)(val)>>8); *((BYTE*)(ptr)+2)=(BYTE)((DWORD)(val)>>16); *((BYTE*)(ptr)+3)=(BYTE)((DWORD)(val)>>24)
#endif

#ifdef __cplusplus
}
#endif

#endif /* _FATFS */
