/*
 *  V4L2 video capture example
 *
 *  This program can be used and distributed without restrictions.
 *
 *      This program is provided with the V4L2 API
 * see http://linuxtv.org/docs.php for more information
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <string>
#include <iostream>

#include <getopt.h>             /* getopt_long() */

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>
#include <linux/usb/video.h>

/* Kernel API interface changed since 4.16  */

/* Meta-data formats */
#define V4L2_META_FMT_VSP1_HGO    v4l2_fourcc('V', 'S', 'P', 'H') /* R-Car VSP1 1-D Histogram */
#define V4L2_META_FMT_VSP1_HGT    v4l2_fourcc('V', 'S', 'P', 'T') /* R-Car VSP1 2-D Histogram */
#define V4L2_META_FMT_UVC         v4l2_fourcc('U', 'V', 'C', 'H') /* UVC Payload Header metadata */
#ifndef V4L2_CAP_META_CAPTURE

#define V4L2_CAP_META_CAPTURE		0x00800000  /* Is a metadata capture device */

#define V4L2_BUF_TYPE_META_CAPTURE 13 // as part of v4l2_buf_type

//enum v4l2_buf_type {
//    V4L2_BUF_TYPE_VIDEO_CAPTURE        = 1,
//    V4L2_BUF_TYPE_VIDEO_OUTPUT         = 2,
//    V4L2_BUF_TYPE_VIDEO_OVERLAY        = 3,
//    V4L2_BUF_TYPE_VBI_CAPTURE          = 4,
//    V4L2_BUF_TYPE_VBI_OUTPUT           = 5,
//    V4L2_BUF_TYPE_SLICED_VBI_CAPTURE   = 6,
//    V4L2_BUF_TYPE_SLICED_VBI_OUTPUT    = 7,e
//    V4L2_BUF_TYPE_VIDEO_OUTPUT_OVERLAY = 8,
//    V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE = 9,
//    V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE  = 10,
//    V4L2_BUF_TYPE_SDR_CAPTURE          = 11,
//    V4L2_BUF_TYPE_SDR_OUTPUT           = 12,
//    V4L2_BUF_TYPE_META_CAPTURE         = 13,
//    /* Deprecated, do not use */
//    V4L2_BUF_TYPE_PRIVATE              = 0x80,
//};

/**
 * struct v4l2_meta_format - metadata format definition
 * @dataformat:		little endian four character code (fourcc)
 * @buffersize:		maximum size in bytes required for data
 */
//struct v4l2_meta_format {
//    __u32				dataformat;
//    __u32				buffersize;
//} __attribute__ ((packed));

/**
 * struct v4l2_format - stream data format
 * @type:	enum v4l2_buf_type; type of the data stream
 * @pix:	definition of an image format
 * @pix_mp:	definition of a multiplanar image format
 * @win:	definition of an overlaid image
 * @vbi:	raw VBI capture or output parameters
 * @sliced:	sliced VBI capture or output parameters
 * @raw_data:	placeholder for future extensions and custom formats
 */
//struct v4l2_format {
//    __u32	 type;
//    union {
//        struct v4l2_pix_format		pix;     /* V4L2_BUF_TYPE_VIDEO_CAPTURE */
//        struct v4l2_pix_format_mplane	pix_mp;  /* V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE */
//        struct v4l2_window		win;     /* V4L2_BUF_TYPE_VIDEO_OVERLAY */
//        struct v4l2_vbi_format		vbi;     /* V4L2_BUF_TYPE_VBI_CAPTURE */
//        struct v4l2_sliced_vbi_format	sliced;  /* V4L2_BUF_TYPE_SLICED_VBI_CAPTURE */
//        struct v4l2_sdr_format		sdr;     /* V4L2_BUF_TYPE_SDR_CAPTURE */
//        struct v4l2_meta_format		meta;    /* V4L2_BUF_TYPE_META_CAPTURE */
//        __u8	raw_data[200];                   /* user-defined */
//    } fmt;

//};
#endif


// Guennady only
#define V4L2_META_FMT_D4XX        v4l2_fourcc('D', '4', 'X', 'X') /* D400 Payload Header metadata */

#define CLEAR(x) memset(&(x), 0, sizeof(x))

#define max(a,b) \
  ({ __typeof__ (a) _a = (a); \
      __typeof__ (b) _b = (b); \
    _a > _b ? _a : _b; })


enum io_method {
    IO_METHOD_READ,
    IO_METHOD_MMAP,
    IO_METHOD_USERPTR,
};

struct buffer {
    void   *start;
    size_t  length;
};

std::string            dev_name_0; // video streaming
std::string            dev_name_1; // metadata node
static enum io_method   io = IO_METHOD_MMAP;
static int              fd_0 = -1;
struct buffer          *buffers_0;
static unsigned int     n_buffers;
static int              out_buf;
static int              force_format = 0;
static int              frame_count = 70;

static void errno_exit(const char *s)
{
    fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
    exit(EXIT_FAILURE);
}

static int xioctl(int fh, unsigned long int request, void *arg)
{
    int r;

    do {
        r = ioctl(fh, request, arg);
    } while (-1 == r && EINTR == errno);

    return r;
}

struct uvc_ms_metadata {
    uint32_t	id;
    uint32_t	length;
    uint8_t		buf[];
} __packed;

struct uvc_meta_buf {
    uint64_t ns;
    uint16_t sof;
    uint8_t length;
    uint8_t flags;
    uint8_t buf[];
} __attribute__((packed));

#define UVC_STREAM_SCR					(1 << 3)
#define UVC_STREAM_PTS					(1 << 2)

static void parse_metadata(const uint8_t **p, int &size)
{
    const struct uvc_meta_buf *meta = (const struct uvc_meta_buf *)*p;
    const struct uvc_ms_metadata *ms_meta;
    bool has_pts, has_scr;
    size_t var_sz, block_len, total_sz = sizeof(meta->ns) + sizeof(meta->sof);
    const uint8_t *scr;

    if (size < sizeof(*meta)) {
        size = 0;
        return;
    }

    has_pts = meta->flags & UVC_STREAM_PTS;
    has_scr = meta->flags & UVC_STREAM_SCR;

    if (has_pts) {
        var_sz = 4;
        scr = meta->buf + 4;
    } else {
        var_sz = 0;
        scr = meta->buf;
    }

    if (has_scr)
        var_sz += 6;

    // Evgeni - this doesn't work. explicit full size only
    total_sz += force_format ? meta->length :
        sizeof(meta->length) + sizeof(meta->flags) + var_sz;

    if (size < total_sz) {
        fprintf(stderr, "%s(): %u is too large!", __func__, meta->length);
        size = 0;
        return;
    }

    fprintf(stderr,
        "%s(%u): t-sys %lu.%09lus, SOF %u, len %u, flags 0x%x, PTS %u, STC %u frame SOF %u",
        __func__, size, meta->ns / 1000000000, meta->ns % 1000000000, meta->sof,
        meta->length, meta->flags, has_pts ? *(uint32_t *)meta->buf : 0,
        has_scr ? *(uint32_t *)scr : 0,
        has_scr ? *(uint32_t *)(scr + 4) & 0x7ff : 0);

    for (ms_meta = (const struct uvc_ms_metadata *)(meta->buf + var_sz), block_len = total_sz;
         block_len > var_sz + sizeof(*meta) && ms_meta->length;
         ms_meta = (const struct uvc_ms_metadata *)((uint8_t *)ms_meta + ms_meta->length)) {
        fprintf(stderr, "MS metadata type 0x%x of %u bytes\n", ms_meta->id, ms_meta->length);
        block_len -= ms_meta->length;
    }

    // Evgeni
//    for (size_t i=12; i< 60; i++)
//        fprintf(stderr,"%x",*(meta->buf+i));

    size -= total_sz;
    *p += total_sz;
}

static void process_image(const void *p, int size, struct v4l2_buffer *buf)
{
    fprintf(stderr, "type: %u  %lu.%06lu, %u (%u): ",
            buf->type,
            buf->timestamp.tv_sec, buf->timestamp.tv_usec,
            buf->sequence, size);

    if (out_buf)
    {

        if ((size >0) && (size< 2048)) // Evgeni for metadata only - will not print frame
        {
            while (size)
                parse_metadata((const uint8_t **)&p, size);
    //		fwrite(p, size, 1, stdout);
        }
    }

    fprintf(stderr,"\n");
    fflush(stderr);
}

static int read_frame(int fd, const std::string& dev_name, struct buffer *buffers)
{
    struct v4l2_buffer buf;
    unsigned int i;
    __u32 buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    switch (io) {
    case IO_METHOD_READ:
        if (-1 == read(fd, buffers[0].start, buffers[0].length)) {
            switch (errno) {
            case EAGAIN:
                return 0;

            case EIO:
                /* Could ignore EIO, see spec. */

                /* fall through */

            default:
                errno_exit("read");
            }
        }

        process_image(buffers[0].start, buffers[0].length, &buf);
        break;

    case IO_METHOD_MMAP:
        CLEAR(buf);

        buf.type = buf_type;
        buf.memory = V4L2_MEMORY_MMAP;

        if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
            switch (errno) {
            case EAGAIN:
                return 0;

            case EIO:
                /* Could ignore EIO, see spec. */

                /* fall through */

            default:
                errno_exit("VIDIOC_DQBUF");
            }
        }

        assert(buf.index < n_buffers);

        process_image(buffers[buf.index].start, buf.bytesused, &buf);

        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
            errno_exit("VIDIOC_QBUF");
        break;

    case IO_METHOD_USERPTR:
        CLEAR(buf);

        buf.type = buf_type;
        buf.memory = V4L2_MEMORY_USERPTR;

        if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
            switch (errno) {
            case EAGAIN:
                return 0;

            case EIO:
                /* Could ignore EIO, see spec. */

                /* fall through */

            default:
                errno_exit("VIDIOC_DQBUF");
            }
        }

        for (i = 0; i < n_buffers; ++i)
            if (buf.m.userptr == (unsigned long)buffers[i].start
                && buf.length == buffers[i].length)
                break;

        assert(i < n_buffers);

        process_image((void *)buf.m.userptr, buf.bytesused, &buf);

        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
            errno_exit("VIDIOC_QBUF");
        break;
    }

    return 1;
}


static void mainloop(void)
{
    unsigned int count;

    count = frame_count;

    while (count-- > 0) {
        for (;;) {
            fd_set fds;         // only for video streaming Evgeni to expand for video_1
            struct timeval tv;
            int r;

            FD_ZERO(&fds);
            FD_SET(fd_0, &fds);
            int max_fd = fd_0;

            /* Timeout. */
            tv.tv_sec = 2;
            tv.tv_usec = 0;

            if ((r = select(fd_0 + 1, &fds, NULL, NULL, &tv)) < 0)
            {
                if (EINTR == errno)
                    continue;
                errno_exit("select");
            }

            if (0 == r) {
                fprintf(stderr, "\nselect timeout");
                //exit(EXIT_FAILURE);
            }

            if (r>0) // One or more fd's were signalled
            {
                int cnt =0;
                // http://jhshi.me/2013/11/02/use-select-to-monitor-multiple-file-descriptors/index.html#.WyiqmBxRXmE
                /* check which fd is avaialbe for read */
                for (int fd = 0; fd <= max_fd; fd++)
                {
                    if (FD_ISSET(fd, &fds))
                    {
                        cnt++;
                        // monitoring console commands
                        if (fd == STDIN_FILENO) {
                            printf("Console command received");
                            //handle_command();
                        }
    //                    else if (fd == server_sock) {
    //                        printf("\n");
    //                        handle_new_connection();
    //                    }
                        else
                        {
                            //handle_message(fd);
                            std::string dev_name = dev_name_0;
                            struct buffer * buf = buffers_0;
                            fprintf(stderr,"Frame received for : %s: ",dev_name.data());
                            if (0==read_frame(fd,dev_name.data(),buf))
                            {
                                printf("read frame failed");
                                //break;
                            }
                        }
                    }
                }
            }

//            if (read_frame(fd_0,dev_name_0,buffers_0))
//                break;
            /* EAGAIN - continue select loop. */
        }
    }
}

static void stop_capturing(int fd,const std::string& dev_name)
{
    enum v4l2_buf_type type = (dev_name=="/dev/video0") ? V4L2_BUF_TYPE_VIDEO_CAPTURE : V4L2_BUF_TYPE_META_CAPTURE;

    switch (io) {
    case IO_METHOD_READ:
        /* Nothing to do. */
        break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
        if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
            errno_exit("VIDIOC_STREAMOFF");
        break;
    }
}

static void start_capturing(int fd,struct buffer *buffers,const std::string& dev_name)
{
    enum v4l2_buf_type type = (dev_name=="/dev/video0") ? V4L2_BUF_TYPE_VIDEO_CAPTURE : V4L2_BUF_TYPE_META_CAPTURE;
    unsigned int i;

    switch (io) {
    case IO_METHOD_READ:
        /* Nothing to do. */
        break;

    case IO_METHOD_MMAP:
        for (i = 0; i < n_buffers; ++i) {
            struct v4l2_buffer buf;

            CLEAR(buf);
            buf.type = type;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;

            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                errno_exit("VIDIOC_QBUF");
        }

        if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
            errno_exit("VIDIOC_STREAMON");
        break;

    case IO_METHOD_USERPTR:
        for (i = 0; i < n_buffers; ++i) {
            struct v4l2_buffer buf;

            CLEAR(buf);
            buf.type = type;
            buf.memory = V4L2_MEMORY_USERPTR;
            buf.index = i;
            buf.m.userptr = (unsigned long)buffers[i].start;
            buf.length = buffers[i].length;

            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                errno_exit("VIDIOC_QBUF");
        }

        if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
            errno_exit("VIDIOC_STREAMON");
        break;
    }
}

static void uninit_device(int fd, const std::string &dev_name,struct buffer *buffers)
{
    unsigned int i;

    switch (io) {
    case IO_METHOD_READ:
        free(buffers[0].start);
        break;

    case IO_METHOD_MMAP:
        for (i = 0; i < n_buffers; ++i)
            if (-1 == munmap(buffers[i].start, buffers[i].length))
                errno_exit("munmap");
        break;

    case IO_METHOD_USERPTR:
        for (i = 0; i < n_buffers; ++i)
            free(buffers[i].start);
        break;
    }

    free(buffers);
}

static void init_read(unsigned int buffer_size,struct buffer **buffers)
{
    *buffers = (buffer*)calloc(1, sizeof(*buffers));

    if (!buffers) {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }

    (*buffers)[0].length = buffer_size;
    (*buffers)[0].start = malloc(buffer_size);

    if (!(*buffers)[0].start) {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }
}

static void init_mmap(int fd,const std::string& dev_name,struct buffer **buffers)
{
    __u32 type = (dev_name=="/dev/video0") ? V4L2_BUF_TYPE_VIDEO_CAPTURE : V4L2_BUF_TYPE_META_CAPTURE;

    struct v4l2_requestbuffers req;
    req.type = type;
    req.memory = V4L2_MEMORY_MMAP;
    req.count = 4;

    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
        if (EINVAL == errno) {
            fprintf(stderr, "%s does not support "
                 "memory mapping\n", dev_name.data());
            exit(EXIT_FAILURE);
        } else {
            errno_exit("VIDIOC_REQBUFS");
        }
    }

    if (req.count < 2) {
        fprintf(stderr, "Insufficient buffer memory on %s\n", dev_name.data());
        exit(EXIT_FAILURE);
    }

    *buffers = (buffer*)calloc(req.count, sizeof(**buffers));

    if (!(*buffers)) {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }

    for (n_buffers = 0; n_buffers < req.count; ++n_buffers)
    {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type        = type;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = n_buffers;

        if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
            errno_exit("VIDIOC_QUERYBUF");

        (*buffers)[n_buffers].length = buf.length;
        (*buffers)[n_buffers].start =
            mmap(NULL /* start anywhere */,
                  buf.length,
                  PROT_READ | PROT_WRITE /* required */,
                  MAP_SHARED /* recommended */,
                  fd, buf.m.offset);

        if (MAP_FAILED == (*buffers)[n_buffers].start)
            errno_exit("mmap");
    }
}

static void init_userp(int fd, const std::string &dev_name, unsigned int buffer_size,struct buffer **buffers)
{
    struct v4l2_requestbuffers req;

    CLEAR(req);

    __u32 type = (dev_name=="/dev/video0") ? V4L2_BUF_TYPE_VIDEO_CAPTURE : V4L2_BUF_TYPE_META_CAPTURE;

    req.count  = 4;
    req.type   = type;
    req.memory = V4L2_MEMORY_USERPTR;

    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
        if (EINVAL == errno) {
            fprintf(stderr, "%s does not support "
                 "user pointer i/o\n", dev_name.data());
            exit(EXIT_FAILURE);
        } else {
            errno_exit("VIDIOC_REQBUFS");
        }
    }


    *buffers = (buffer*)calloc(4, sizeof(**buffers));

    if (!(*buffers)) {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }

    for (n_buffers = 0; n_buffers < 4; ++n_buffers) {
        (*buffers)[n_buffers].length = buffer_size;
        (*buffers)[n_buffers].start = malloc(buffer_size);

        if (!(*buffers)[n_buffers].start) {
            fprintf(stderr, "Out of memory\n");
            exit(EXIT_FAILURE);
        }
    }
}

static void init_device(int fd, const std::string &dev_name)
{
    struct v4l2_capability cap;
    __u32 type = (dev_name=="/dev/video0") ? V4L2_BUF_TYPE_VIDEO_CAPTURE : V4L2_BUF_TYPE_META_CAPTURE;
    struct v4l2_cropcap cropcap = { .type = type,  };
    struct v4l2_crop crop       = { .type = type,  };
    struct v4l2_format fmt      = { .type = type,  };

    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
        if (EINVAL == errno) {
            fprintf(stderr, "%s is no V4L2 device\n",
                 dev_name.data());
            exit(EXIT_FAILURE);
        } else {
            errno_exit("VIDIOC_QUERYCAP");
        }
    }

//    if (!(cap.capabilities & V4L2_CAP_META_CAPTURE)) {
//        fprintf(stderr, "%s is not a video metadata device\n",
//             dev_name);
//        exit(EXIT_FAILURE);
//    }

    switch (io) {
    case IO_METHOD_READ:
        if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
            fprintf(stderr, "%s does not support read i/o\n",
                 dev_name.data());
            exit(EXIT_FAILURE);
        }
        break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
        if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
            fprintf(stderr, "%s does not support streaming i/o\n",
                 dev_name.data());
            exit(EXIT_FAILURE);
        }
        break;
    }


    /* Select video input, video standard and tune here. */

    if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
        crop.c = cropcap.defrect; /* reset to default */

        if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop)) {
            switch (errno) {
            case EINVAL:
                /* Cropping not supported. */
                break;
            default:
                /* Errors ignored. */
                break;
            }
        }
    } else {
        /* Errors ignored. */
    }


    /* Preserve original settings as set by v4l2-ctl for example */
    if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
        errno_exit("VIDIOC_G_FMT");

//    fmt.fmt.pix.width = 640;
//    fmt.fmt.pix.height = 480;
//    fmt.fmt.pix.bytesperline = fmt.fmt.pix.width * sizeof(uint16_t);
//    /* Reapply this format */
//    if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
//        errno_exit("VIDIOC_S_FMT");

    // Select FPS
//    v4l2_streamparm parm = {};
//    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
//    if(xioctl(fd, VIDIOC_G_PARM, &parm) < 0)
//        errno_exit("xioctl(VIDIOC_G_PARM) failed");

//    parm.parm.capture.timeperframe.numerator = 1;
//    parm.parm.capture.timeperframe.denominator = 30;
//    if(xioctl(fd, VIDIOC_S_PARM, &parm) < 0)
//        errno_exit("xioctl(VIDIOC_S_PARM) failed");


    struct buffer **bufs = &buffers_0;

    switch (io) {
    case IO_METHOD_READ:
        init_read(fmt.fmt.pix.sizeimage,bufs);
        break;

    case IO_METHOD_MMAP:
        init_mmap(fd,dev_name,bufs);
        break;

    case IO_METHOD_USERPTR:
        init_userp(fd,dev_name,fmt.fmt.pix.sizeimage,bufs);
        break;
    }
}

static void close_device(int fd, const std::string &dev_name)
{
    if (-1 == close(fd))
        errno_exit("close");

    fd = -1;
}

static void open_device(int *fd, const std::string &dev_name)
{
    struct stat st;

    if (-1 == stat(dev_name.data(), &st)) {
        fprintf(stderr, "Cannot identify '%s': %d, %s\n",
             dev_name.data(), errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    if (!S_ISCHR(st.st_mode)) {
        fprintf(stderr, "%s is no device\n", dev_name.data());
        exit(EXIT_FAILURE);
    }

    *fd = open(dev_name.data(), O_RDWR /* required */ | O_NONBLOCK, 0);

    if (-1 == *fd) {
        fprintf(stderr, "Cannot open '%s': %d, %s\n",
             dev_name.data(), errno, strerror(errno));
        exit(EXIT_FAILURE);
    }
}

static void usage(FILE *fp, int argc, char **argv)
{
    fprintf(fp,
         "Usage: %s [options]\n\n"
         "Version 1.3\n"
         "Options:\n"
         "-d | --device name   Video device name [%s]\n"
         "-h | --help          Print this message\n"
         "-m | --mmap          Use memory mapped buffers [default]\n"
         "-r | --read          Use read() calls\n"
         "-u | --userp         Use application allocated buffers\n"
         "-o | --output        Outputs stream to stdout\n"
         "-f | --format        Force format to 640x480 YUYV\n"
         "-c | --count         Number of frames to grab [%i]\n"
         "",
         argv[0], dev_name_1.data(), frame_count);
}

static const char short_options[] = "d:hmruofc:";

static const struct option
long_options[] = {
    { "device", required_argument, NULL, 'd' },
    { "help",   no_argument,       NULL, 'h' },
    { "mmap",   no_argument,       NULL, 'm' },
    { "read",   no_argument,       NULL, 'r' },
    { "userp",  no_argument,       NULL, 'u' },
    { "output", no_argument,       NULL, 'o' },
    { "format", no_argument,       NULL, 'f' },
    { "count",  required_argument, NULL, 'c' },
    { 0, 0, 0, 0 }
};

int main(int argc, char **argv)
{
    dev_name_0 = "/dev/video0";

    for (;;) {
        int idx;
        int c;

        c = getopt_long(argc, argv,
                short_options, long_options, &idx);

        if (-1 == c)
            break;

        switch (c) {
        case 0: /* getopt_long() flag */
            break;

        case 'd':
            dev_name_1 = optarg;
            break;

        case 'h':
            usage(stdout, argc, argv);
            exit(EXIT_SUCCESS);

        case 'm':
            io = IO_METHOD_MMAP;
            break;

        case 'r':
            io = IO_METHOD_READ;
            break;

        case 'u':
            io = IO_METHOD_USERPTR;
            break;

        case 'o':
            out_buf++;
            break;

        case 'f':
            force_format++;
            break;

        case 'c':
            errno = 0;
            frame_count = strtol(optarg, NULL, 0);
            if (errno)
                errno_exit(optarg);
            break;

        default:
            usage(stderr, argc, argv);
            exit(EXIT_FAILURE);
        }
    }

    open_device(&fd_0,dev_name_0);
    init_device(fd_0,dev_name_0);
    start_capturing(fd_0,buffers_0,dev_name_0);
    mainloop();
    stop_capturing(fd_0,dev_name_0);
    uninit_device(fd_0,dev_name_0,buffers_0);
    close_device(fd_0,dev_name_0);
    fprintf(stderr, "\n");
    return 0;
}
