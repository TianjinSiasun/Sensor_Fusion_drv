#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <string.h>
#include <sys/mman.h>
#include <errno.h>
#include "../lib/libsfconfig.h"

int fd;

struct videobuf
{
    unsigned char *start;
    size_t len;
};

int main(int argc, const char * argv[])
{
    int ret = 0;
    int i = 0;
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    struct v4l2_capability cap;
    struct v4l2_requestbuffers reqbuf;
    struct videobuf framebuf[2];
    struct v4l2_buffer buf;
    struct v4l2_format fmt;
    struct timeval tv;

    int fd_cfg = FPGA_Open();
    if (fd_cfg < 0)
    {
        printf("open fpga_cfg failed\n");
    }

    IRQ_Mask_Enable(fd_cfg, 0xf);

    fd = open("/dev/video0", O_RDWR);
    if (fd < 0)
    {
        printf("open failed\n");
    }

    ret = ioctl(fd, VIDIOC_QUERYCAP, &cap);
    if (ret < 0)
    {
        perror("query cap failed\n");
        return -1;
    }

    if ((cap.capabilities & V4L2_CAP_STREAMING) && (cap.capabilities & V4L2_BUF_TYPE_VIDEO_CAPTURE))
        printf("%s:%s:%s, %#x\n", cap.driver, cap.card, cap.bus_info, cap.capabilities);

    ioctl(fd, VIDIOC_STREAMOFF, &type);
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = 1600;
    fmt.fmt.pix.height = 1200;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
    ret = ioctl(fd, VIDIOC_S_FMT, &fmt);
    if (ret < 0)
    {
        printf("set fmt failed[%s]\n", strerror(errno));
        return ret;
    }

    reqbuf.count = 2;
    reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqbuf.memory = V4L2_MEMORY_MMAP;
    ret = ioctl(fd, VIDIOC_REQBUFS, &reqbuf);
    if (ret < 0)
    {
        printf("reqbuf failed[%s]\n", strerror(errno));
    }
    else
    {
        printf("buf count = %d\n", reqbuf.count);
    }

    for (i = 0; i < 2; i++)
    {
        buf.index = i;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        ret = ioctl(fd, VIDIOC_QUERYBUF, &buf);
        if (ret)
        {
            printf("query buf failed[%s]\n", strerror(errno));
            return ret;
        }

        framebuf[i].len = buf.length;
        framebuf[i].start = (unsigned char *)mmap(0, buf.length, PROT_READ, MAP_SHARED, fd, buf.m.offset);
        if (framebuf[i].start == MAP_FAILED)
        {
            printf("mmap[%d] failed:[%s]\n", i, strerror(errno));
            return -1;
        }
        printf("mmap[%d] offset = %#x\n", i, buf.m.offset);
    }

    buf.index = 0;
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    ret = ioctl(fd, VIDIOC_QBUF, &buf);
    if (ret)
    {
        perror("qbuf failed");
        return ret;
    }

    ret = ioctl(fd, VIDIOC_STREAMON, &type);
    if (ret)
    {
        perror("stream on failed");
        return ret;
    }

    for(i = 0; i < 50; i++)
    {
        ret = ioctl(fd, VIDIOC_DQBUF, &buf);
        if (ret)
        {
            printf("dqbuf failed[%d]\n", i);
            return ret;
        }
        gettimeofday(&tv, NULL);
        printf("ID:%d time:[%d]-[%d]-[%d] [%d]:[%d]:[%d].[%03d][03%d]\n", *(framebuf[1].start + 2*1024*1024 - 1),
			   *(framebuf[0].start + 9), *(framebuf[0].start + 8),
               *(framebuf[0].start + 7), *(framebuf[0].start + 6), *(framebuf[0].start + 5), 
               *(framebuf[0].start + 4), ((*(framebuf[0].start + 3) & 0xff)<<8)|(*(framebuf[0].start + 2)&0xff),
               (*(framebuf[0].start + 1)&0xff << 8) | (*(framebuf[0].start + 0)&0xff));
        printf("\t\ts:[%ld] \t ms:[%ld] us:[%ld]\n", tv.tv_sec ,  tv.tv_usec/1000 ,  tv.tv_usec - tv.tv_usec/1000 * 1000 );

        ret = ioctl(fd, VIDIOC_QBUF, &buf);
        if (ret)
        {
            printf("qbuf failed[%d]\n", i);
            return ret;
        }
    }

    ret = ioctl(fd, VIDIOC_STREAMOFF, &type);
    if (ret)
    {
        perror("stream off failed");
        return ret;
    }

    close(fd);
}
