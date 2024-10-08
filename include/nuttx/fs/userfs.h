/****************************************************************************
 * include/nuttx/fs/userfs.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_FS_USERFSFS_H
#define __INCLUDE_NUTTX_FS_USERFSFS_H

/* UserFS is implemented by two components:
 *
 * 1. A component as part of the internal OS file system logic.  This
 *    file system receives the file system requests and marshals the request
 *    as described by the following structures, and sends this marshaled
 *    request on a FIFO that was created by userfs_run().  It also receives
 *    the marshal led response by the application file system, unmarshals the
 *    response, and provides the file system response to the caller.
 * 2. userfs_run() is part of the NuttX C library.  It receives the marshaled
 *    operating system requests on the FIFO, unmarshals it, and calls the
 *    application file system methods on behalf of the OS.  It the marshals
 *    the application response data and sends this back to the waiting
 *    OS file system logic.
 *
 * Overview of general operation flow:
 *
 * 1. The UserFS OS support will be instantiated when the UserFS is mounted
 *    based upon the configuration passed in the optional data of the
 *    mount command.
 * 2. The UserFS server port number will be configured to communicate on a
 *    LocalHost UDP socket with the server portof 0x83nn where nn is the
 *    value that was provided when file system was created.
 * 3. The UserFs will receive system file system requests and forward them
 *    on the the MqUfsReqN to the user-space file system server
 *    (userfs_run()). These requests may be accompanied by additional data in
 *    an provided request buffer that was provided when the UserFS was
 *    created.  This buffer would hold, for example, the data to be
 *    written that would accompany a write request.
 * 4. The user-space logic of userfs_run() listens at the other end of the
 *    LocalHost socket.  It will receive the requests and forward them
 *    to the user file system implementation via the methods of struct
 *    userfs_operations_s
 * 5. Responses generated by the struct userfs_operations_s method will be
 *    returned to UserFS via the LocalHost socket.
 * 6. The UserFS kernel thread will listen on the LocalHost socket
 *    and will receive the user file system responses and forward them to
 *    the kernel-space file system client.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/statfs.h>
#include <sys/stat.h>
#include <dirent.h>

#include <nuttx/fs/fs.h>

#ifdef CONFIG_FS_USERFS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* UserFS IOCTLs are defined in included/nuttx/fs/ioctl.h. There is only one:
 *
 * FIONUSERFS.  The is the IOCTL that is used with the dev/userfs factory to
 *   create a UserFS instance.
 *
 *   Input:  This function receives an pointer to a read-only instance of
 *           struct userfs_config_s that contains information needed to
 *           configure the UserFS instance.
 *   Output: On success the UserFS nn instance is created. nn is non-negative
 *           and will be provided as the IOCTL return value on success.  On
 *           failure, ioctl() will return -1 with the errno variable set to
 *           indicate the cause of the failure.
 */

/* This is the base value of the server port number.  The actual range is
 * 0x8300 through 0x83ff.
 */

#define USERFS_SERVER_PORTBASE 0x8300

/* It looks like the maximum size of a request is 16 bytes.  We will allow a
 * little more for the maximum size of a request structure.
 */

#define USERFS_REQ_MAXSIZE   (32)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This enumeration provides the type of each request sent from the OS file
 * system client to the user file system.
 */

enum userfs_req_e
{
  USERFS_REQ_OPEN = 0,
  USERFS_REQ_CLOSE,
  USERFS_REQ_READ,
  USERFS_REQ_WRITE,
  USERFS_REQ_SEEK,
  USERFS_REQ_IOCTL,
  USERFS_REQ_SYNC,
  USERFS_REQ_DUP,
  USERFS_REQ_FSTAT,
  USERFS_REQ_TRUNCATE,
  USERFS_REQ_OPENDIR,
  USERFS_REQ_CLOSEDIR,
  USERFS_REQ_READDIR,
  USERFS_REQ_REWINDDIR,
  USERFS_REQ_STATFS,
  USERFS_REQ_UNLINK,
  USERFS_REQ_MKDIR,
  USERFS_REQ_RMDIR,
  USERFS_REQ_RENAME,
  USERFS_REQ_STAT,
  USERFS_REQ_DESTROY,
  USERFS_REQ_FCHSTAT,
  USERFS_REQ_CHSTAT
};

/* This enumeration provides the type of each response returned from the
 * user file system to OS file system client.
 */

enum userfs_resp_e
{
  USERFS_RESP_OPEN = 0,
  USERFS_RESP_CLOSE,
  USERFS_RESP_READ,
  USERFS_RESP_WRITE,
  USERFS_RESP_SEEK,
  USERFS_RESP_IOCTL,
  USERFS_RESP_SYNC,
  USERFS_RESP_DUP,
  USERFS_RESP_FSTAT,
  USERFS_RESP_OPENDIR,
  USERFS_RESP_CLOSEDIR,
  USERFS_RESP_READDIR,
  USERFS_RESP_REWINDDIR,
  USERFS_RESP_STATFS,
  USERFS_RESP_UNLINK,
  USERFS_RESP_MKDIR,
  USERFS_RESP_RMDIR,
  USERFS_RESP_RENAME,
  USERFS_RESP_STAT,
  USERFS_RESP_DESTROY,
  USERFS_RESP_FCHSTAT,
  USERFS_RESP_CHSTAT
};

/* These structures are used by internal UserFS implementation and should not
 * be of interest to application level logic.
 *
 * This is passed to the mount() function as optional data when the UserFS
 * file system is mounted.
 */

struct userfs_config_s
{
  size_t mxwrite;        /* The max size of a write data */
  uint16_t portno;       /* The server port number (host order) */
};

/* This structure identifies the user-space file system operations. */

struct stat;   /* Forward reference */
struct statfs; /* Forward reference */

struct userfs_operations_s
{
  int     (*open)(FAR void *volinfo, FAR const char *relpath,
            int oflags, mode_t mode, FAR void **openinfo);
  int     (*close)(FAR void *volinfo, FAR void *openinfo);
  ssize_t (*read)(FAR void *volinfo, FAR void *openinfo,
            FAR char *buffer, size_t buflen);
  ssize_t (*write)(FAR void *volinfo, FAR void *openinfo,
            FAR const char *buffer, size_t buflen);
  off_t   (*seek)(FAR void *volinfo, FAR void *openinfo, off_t offset,
            int whence);
  int     (*ioctl)(FAR void *volinfo, FAR void *openinfo, int cmd,
            unsigned long arg);
  int     (*sync)(FAR void *volinfo, FAR void *openinfo);
  int     (*dup)(FAR void *volinfo, FAR void *oldinfo, FAR void **newinfo);
  int     (*fstat)(FAR void *volinfo, FAR void *openinfo,
            FAR struct stat *buf);
  int     (*truncate)(FAR void *volinfo, FAR void *openinfo, off_t length);
  int     (*opendir)(FAR void *volinfo, FAR const char *relpath,
            FAR void **dir);
  int     (*closedir)(FAR void *volinfo, FAR void *dir);
  int     (*readdir)(FAR void *volinfo, FAR void *dir,
            FAR struct dirent *entry);
  int     (*rewinddir)(FAR void *volinfo, FAR void *dir);
  int     (*statfs)(FAR void *volinfo, FAR struct statfs *buf);
  int     (*unlink)(FAR void *volinfo, FAR const char *relpath);
  int     (*mkdir)(FAR void *volinfo, FAR const char *relpath, mode_t mode);
  int     (*rmdir)(FAR void *volinfo, FAR const char *relpath);
  int     (*rename)(FAR void *volinfo, FAR const char *oldrelpath,
            FAR const char *newrelpath);
  int     (*stat)(FAR void *volinfo, FAR const char *relpath,
            FAR struct stat *buf);
  int     (*destroy)(FAR void *volinfo);
  int     (*fchstat)(FAR void *volinfo, FAR void *openinfo,
            FAR const struct stat *buf, int flags);
  int     (*chstat)(FAR void *volinfo, FAR const char *relpath,
            FAR const struct stat *buf, int flags);
};

/* The following structures describe the header on the marshaled data sent
 * on the FIFOs.  See struct userfs_operations_s for the form of the
 * marshaled function calls.
 */

struct userfs_open_request_s
{
  uint8_t req;              /* Must be USERFS_REQ_OPEN */
  int oflags;               /* Open flags */
  mode_t mode;              /* Open mode */
  char relpath[1];          /* Mountpoint relative path to the file to open */
};

#define SIZEOF_USERFS_OPEN_REQUEST_S(n) (sizeof(struct userfs_open_request_s) + (n) - 1)

struct userfs_open_response_s
{
  uint8_t resp;             /* Must be USERFS_RESP_OPEN */
  FAR void *openinfo;       /* Open file info for use in other operations */
  int ret;                  /* Result of the operation */
};

struct userfs_close_request_s
{
  uint8_t req;              /* Must be USERFS_REQ_CLOSE */
  FAR void *openinfo;       /* Open file info as returned by open() */
};

struct userfs_close_response_s
{
  uint8_t resp;             /* Must be USERFS_RESP_CLOSE */
  int ret;                  /* Result of the operation */
};

struct userfs_read_request_s
{
  uint8_t req;              /* Must be USERFS_REQ_READ */
  FAR void *openinfo;       /* Open file info as returned by open() */
  size_t readlen;           /* Maximum number of bytes to read */
};

struct userfs_read_response_s
{
  uint8_t resp;             /* Must be USERFS_RESP_READ */
  ssize_t nread;            /* Result of the operation */
  char rddata[1];           /* Read data follows.  Actual size is nread */
};

#define SIZEOF_USERFS_READ_RESPONSE_S(n) (sizeof(struct userfs_read_response_s) + (n) - 1)

struct userfs_write_request_s
{
  uint8_t req;              /* Must be USERFS_REQ_WRITE */
  FAR void *openinfo;       /* Open file info as returned by open() */
  size_t writelen;          /* Number of bytes to write */
  char wrdata[1];           /* Read data follows.  Actual size is wrsize */
};

#define SIZEOF_USERFS_WRITE_REQUEST_S(n) (sizeof(struct userfs_write_request_s) + (n) - 1)

struct userfs_write_response_s
{
  uint8_t resp;             /* Must be USERFS_RESP_WRITE */
  ssize_t nwritten;         /* Result of the operation */
};

struct userfs_seek_request_s
{
  uint8_t req;              /* Must be USERFS_REQ_SEEK */
  FAR void *openinfo;       /* Open file info as returned by open() */
  off_t offset;             /* Seek offset */
  int whence;               /* Determines how offset is interpreted */
};

struct userfs_seek_response_s
{
  uint8_t resp;             /* Must be USERFS_RESP_SEEK */
  off_t ret;                /* Result of the operation */
};

struct userfs_ioctl_request_s
{
  uint8_t req;              /* Must be USERFS_REQ_IOCTL */
  FAR void *openinfo;       /* Open file info as returned by open() */
  int cmd;                  /* IOCTL command */
  unsigned long arg;        /* Argument that accompanies the command */
};

struct userfs_ioctl_response_s
{
  uint8_t resp;             /* Must be USERFS_RESP_IOCTL */
  int ret;                  /* Result of the operation */
};

struct userfs_sync_request_s
{
  uint8_t req;              /* Must be USERFS_REQ_SYNC */
  FAR void *openinfo;       /* Open file info as returned by open() */
};

struct userfs_sync_response_s
{
  uint8_t resp;             /* Must be USERFS_RESP_SYNC */
  int ret;                  /* Result of the operation */
};

struct userfs_dup_request_s
{
  uint8_t req;              /* Must be USERFS_REQ_DUP */
  FAR void *openinfo;       /* Open file info as returned by open() */
};

struct userfs_dup_response_s
{
  uint8_t resp;             /* Must be USERFS_RESP_DUP */
  FAR void *openinfo;       /* Open file info for the dup'ed file */
  int ret;                  /* Result of the operation */
};

struct userfs_fstat_request_s
{
  uint8_t req;              /* Must be USERFS_REQ_FSTAT */
  FAR void *openinfo;       /* Open file info as returned by open() */
};

struct userfs_fstat_response_s
{
  uint8_t resp;             /* Must be USERFS_RESP_FSTAT */
  int ret;                  /* Result of the operation */
  struct stat buf;          /* Returned file system status */
};

struct userfs_truncate_request_s
{
  uint8_t req;              /* Must be USERFS_REQ_TRUNCATE */
  FAR void *openinfo;       /* Open file info as returned by open() */
  off_t length;             /* New length of the file */
};

struct userfs_truncate_response_s
{
  uint8_t resp;             /* Must be USERFS_RESP_FSTAT */
  int ret;                  /* Result of the operation */
};

struct userfs_opendir_request_s
{
  uint8_t req;              /* Must be USERFS_REQ_OPENDIR */
  char relpath[1];          /* Mountpoint relative path to the directory to open */
};

#define SIZEOF_USERFS_OPENDIR_REQUEST_S(n) (sizeof(struct userfs_opendir_request_s) + (n) - 1)

struct userfs_opendir_response_s
{
  uint8_t resp;             /* Must be USERFS_RESP_OPENDIR */
  int ret;                  /* Result of the operation */
  FAR void *dir;            /* Opaque pointer to directory information */
};

struct userfs_closedir_request_s
{
  uint8_t req;              /* Must be USERFS_REQ_CLOSEDIR */
  FAR void *dir;            /* Opaque pointer to directory information */
};

struct userfs_closedir_response_s
{
  uint8_t resp;             /* Must be USERFS_RESP_CLOSEDIR */
  int ret;                  /* Result of the operation */
};

struct userfs_readdir_request_s
{
  uint8_t req;              /* Must be USERFS_REQ_READDIR */
  FAR void *dir;            /* Opaque pointer to directory information */
};

struct userfs_readdir_response_s
{
  uint8_t resp;             /* Must be USERFS_RESP_READDIR */
  int ret;                  /* Result of the operation */
  struct dirent entry;      /* Directory entry that was read */
};

struct userfs_rewinddir_request_s
{
  uint8_t req;              /* Must be USERFS_REQ_REWINDDIR */
  FAR void *dir;            /* Opaque pointer to directory information */
};

struct userfs_rewinddir_response_s
{
  uint8_t resp;             /* Must be USERFS_RESP_REWINDDIR */
  int ret;                  /* Result of the operation */
};

struct userfs_statfs_request_s
{
  uint8_t req;              /* Must be USERFS_REQ_STATFS */
};

struct userfs_statfs_response_s
{
  uint8_t resp;             /* Must be USERFS_RESP_STATFS */
  int ret;                  /* Result of the operation */
  FAR struct statfs buf;    /* Returned file system status */
};

struct userfs_unlink_request_s
{
  uint8_t req;              /* Must be USERFS_REQ_UNLINK */
  char relpath[1];          /* Relative path to the entry to unlink */
};

#define SIZEOF_USERFS_UNLINK_REQUEST_S(n) (sizeof(struct userfs_unlink_request_s) + (n) - 1)

struct userfs_unlink_response_s
{
  uint8_t resp;             /* Must be USERFS_RESP_UNLINK */
  int ret;                  /* Result of the operation */
};

struct userfs_mkdir_request_s
{
  uint8_t req;              /* Must be USERFS_REQ_MKDIR */
  mode_t mode;              /* Directory mode flags */
  char relpath[1];          /* Relative path to the directory to create */
};

#define SIZEOF_USERFS_MKDIR_REQUEST_S(n) (sizeof(struct userfs_mkdir_request_s) + (n) - 1)

struct userfs_mkdir_response_s
{
  uint8_t resp;             /* Must be USERFS_RESP_MKDIR */
  int ret;                  /* Result of the operation */
};

struct userfs_rmdir_request_s
{
  uint8_t req;              /* Must be USERFS_REQ_RMDIR */
  char relpath[1];          /* Relative path to the directory to remove */
};

#define SIZEOF_USERFS_RMDIR_REQUEST_S(n) (sizeof(struct userfs_rmdir_request_s) + (n) - 1)

struct userfs_rmdir_response_s
{
  uint8_t resp;             /* Must be USERFS_RESP_RMDIR */
  int ret;                  /* Result of the operation */
};

struct userfs_rename_request_s
{
  uint8_t req;              /* Must be USERFS_REQ_RENAME */
  uint16_t newoffset;       /* Offset from old to new relpath */
  char oldrelpath[1];       /* Old relative path to be renamed */
};

#define SIZEOF_USERFS_RENAME_REQUEST_S(o,n) (sizeof(struct userfs_rename_request_s) + (o) + (n))

struct userfs_rename_response_s
{
  uint8_t resp;             /* Must be USERFS_RESP_RENAME */
  int ret;                  /* Result of the operation */
};

struct userfs_stat_request_s
{
  uint8_t req;              /* Must be USERFS_REQ_STAT */
  char relpath[1];          /* Relative path to the directory entry to be queried */
};

#define SIZEOF_USERFS_STAT_REQUEST_S(n) (sizeof(struct userfs_stat_request_s) + (n) - 1)

struct userfs_stat_response_s
{
  uint8_t resp;             /* Must be USERFS_RESP_STAT */
  int ret;                  /* Result of the operation */
  struct stat buf;          /* Returned status of the directory entry */
};

struct userfs_destroy_request_s
{
  uint8_t req;              /* Must be USERFS_REQ_DESTROY */
};

struct userfs_destroy_response_s
{
  uint8_t resp;             /* Must be USERFS_RESP_DESTROY */
  int ret;                  /* Result of the operation */
};

struct userfs_fchstat_request_s
{
  uint8_t req;              /* Must be USERFS_REQ_FCHSTAT */
  FAR void *openinfo;       /* Open file info as returned by open() */
  struct stat buf;          /* File system status */
  int flags;                /* The status to be change */
};

struct userfs_fchstat_response_s
{
  uint8_t resp;             /* Must be USERFS_RESP_FCHSTAT */
  int ret;                  /* Result of the operation */
};

struct userfs_chstat_request_s
{
  uint8_t req;              /* Must be USERFS_REQ_CHSTAT */
  struct stat buf;          /* File system status */
  int flags;                /* The status to be change */
  char relpath[1];          /* Relative path to the directory entry to be changed */
};

#define SIZEOF_USERFS_CHSTAT_REQUEST_S(n) (sizeof(struct userfs_chstat_request_s) + (n) - 1)

struct userfs_chstat_response_s
{
  uint8_t resp;             /* Must be USERFS_RESP_CHSTAT */
  int ret;                  /* Result of the operation */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: userfs_register
 *
 * Description:
 *   Register the UserFS factory driver at dev/userfs.
 *
 *   NOTE:  This is an OS internal function that should not be called from
 *   application logic.
 *
 * Input Parameters:
 *   None
 *
 *  Returned Value:
 *    Zero (OK) is returned if the dev/userfs driver was initialized and
 *    registered properly.  Otherwise, a negated errno value is returned
 *    to indicate the nature of the failure.
 *
 ****************************************************************************/

int userfs_register(void);

/****************************************************************************
 * Name: userfs_run
 *
 * Description:
 *   Start the UserFS server on the current thread.  This function will mount
 *   the UserFS file system and will not return until that file system has
 *   been unmounted.
 *
 *   userfs_run() implements the UserFS server. It performs there operations:
 *
 *   1. It configures and creates the UserFS file system and
 *   2. Mounts the user file system at the provide mount point path.
 *   2. Receives file system requests on the LocalHost socket with
 *      server port 0x83nn where nn is the same as above,
 *   3. Received file system requests are marshaled and dispatch to the
 *      user file system via callbacks to the operations provided by
 *      "userops", and
 *   3. Returns file system responses generated by the callbacks to the
 *      LocalHost client socket.
 *
 *   NOTE:  This is a user function that is implemented as part of the
 *   NuttX C library and is intended to be called by application logic.
 *
 * Input Parameters:
 *   mountpt  - Mountpoint path
 *   userops  - The caller operations that implement the file system
 *              interface.
 *   volinfo  - Private volume data that will be provided in all struct
 *              userfs_operations_s methods.
 *   mxwrite  - The max size of a write data
 *
 *  Returned Value:
 *    This function does not return unless the file system is unmounted (OK)
 *    or unless an error is encountered.  In the event of an error, the
 *    returned value is a negated errno value indicating the nature of the
 *    error.
 *
 ****************************************************************************/

int userfs_run(FAR const char *mountpt,
               FAR const struct userfs_operations_s *userops,
               FAR void *volinfo, size_t mxwrite);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_FS_USERFS */
#endif /* __INCLUDE_NUTTX_FS_USERFSFS_H */
