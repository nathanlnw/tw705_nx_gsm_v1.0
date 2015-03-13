




extern DSTATUS disk_initialize_SD (
    BYTE drv				/* Physical drive nmuber (0..) */
);
extern DSTATUS disk_status_SD (
    BYTE drv		/* Physical drive nmuber (0..) */
);
extern DRESULT disk_write_SD (
    BYTE drv,			/* Physical drive nmuber (0..) */
    const BYTE *buff,	/* Data to be written */
    DWORD sector,		/* Sector address (LBA) */
    BYTE count			/* Number of sectors to write (1..255) */
);

extern DRESULT disk_ioctl_SD (
    BYTE drv,		/* Physical drive nmuber (0..) */
    BYTE ctrl,		/* Control code */
    void *buff		/* Buffer to send/receive control data */
);
extern DWORD get_fattime (void);






