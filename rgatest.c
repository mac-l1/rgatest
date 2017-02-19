#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <pthread.h>
#include <rockchip_drm.h>
#include <rockchip_drmif.h>
#include <rockchip_rga.h>
#include <fcntl.h>
#include <drm_fourcc.h>
#include <sys/mman.h>

#define DBG(fmt, ...) do { printf("D(%p): %s:%d " fmt "\n", (void*)pthread_self(),  __func__, __LINE__, ##__VA_ARGS__); fflush(stdout); } while (0)
#define ERROR(fmt, ...) do { printf("E(%p): %s:%d " fmt "\n", (void*)pthread_self(),  __func__, __LINE__, ##__VA_ARGS__); fflush(stdout); } while (0)
#define INFO(fmt, ...) do { printf("I(%p): %s:%d " fmt "\n", (void*)pthread_self(), __func__, __LINE__, ##__VA_ARGS__); fflush(stdout); } while (0)

#define FIN() do { printf("->F(%p): %s#%d \n", (void*)pthread_self(), __PRETTY_FUNCTION__,__LINE__); fflush(stdout); } while(0)
#define FOUT() do { printf("<-F(%p): %s#%d \n", (void*)pthread_self(), __PRETTY_FUNCTION__,__LINE__); fflush(stdout); } while(0)
#define FUN() do { printf("==F(%p): %s#%d \n", (void*)pthread_self(), __PRETTY_FUNCTION__,__LINE__); fflush(stdout); } while(0)

struct rga_context *rga_ctx = 0;
int rga_fd = 0;

int drm_alloc(size_t size, int *dma_fd, uint8_t **dma_vaddr)
{
        int ret = 0;
        int map_fd, handle;

        if (size < 4096) size = 4096;

        struct drm_mode_create_dumb dmcd;
        memset(&dmcd, 0, sizeof(dmcd));
        dmcd.bpp = 4096;
        dmcd.width = ((size + 4095) & (~4095)) >> 12;
        dmcd.height = 8;
        dmcd.size = dmcd.width * dmcd.bpp;

        ret = drmIoctl(rga_fd, DRM_IOCTL_MODE_CREATE_DUMB, &dmcd);
        handle = dmcd.handle;
        size = dmcd.size;

        if (ret) {
                INFO("drm alloc failed\n");
                return ret;
        }

        ret = drmPrimeHandleToFD(rga_fd, handle, 0, &map_fd);
        if (ret) {
                INFO("prime handle to dma fd failed\n");
                struct drm_mode_destroy_dumb dmdb;
                dmdb.handle = handle;
                drmIoctl(rga_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dmdb);
                return ret;
        }

        struct drm_mode_map_dumb dmmd;
        memset(&dmmd, 0, sizeof(dmmd));
        dmmd.handle = handle;
        ret = drmIoctl(rga_fd, DRM_IOCTL_MODE_MAP_DUMB, &dmmd);
        if (ret) {
                INFO("drm mode map failed\n");
                struct drm_mode_destroy_dumb dmdb;
                dmdb.handle = handle;
                drmIoctl(rga_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dmdb);
                return ret;
        }
#if 1
        *dma_vaddr = (uint8_t*)mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, 
                          rga_fd, dmmd.offset);
        if (dma_vaddr == MAP_FAILED) {
                ERROR("drm map failed\n");
                struct drm_mode_destroy_dumb dmdb;
                dmdb.handle = handle;
                drmIoctl(rga_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dmdb);
                return ret;
        }
#endif
        *dma_fd = map_fd;
        return size;
}

int initRGA() {
  rga_fd = open("/dev/dri/card0", O_RDWR);
  if (rga_fd < 0) {
    ERROR("failed to open");
    return -1;
  }

  rga_ctx = rga_init(rga_fd);
  if (!rga_ctx) {
    ERROR("rga init failed");
    return -1;
  }

  INFO("RGA initialized" );
  return 0;
}

int rga_convert_copy(struct rga_context *rga_ctx,
                     int src_fd,
                     int src_width,
                     int src_height,
                     uint32_t src_stride,
                     uint32_t src_fmt,
                     int dst_fd,
                     int dst_width,
                     int dst_height,
                     uint32_t dst_stride,
                     uint32_t dst_fmt,
                     enum e_rga_buf_type type,
		     int virtual_width, 
		     int virtual_height) {
    struct rga_image src_img = { 0 }, dst_img = { 0 };

    dst_img.bo[0] = dst_fd;
    src_img.bo[0] = src_fd;

    src_img.width = src_width;
    src_img.height = src_height;
    src_img.stride = src_stride;
    src_img.buf_type = type;
    src_img.color_mode = src_fmt;

    dst_img.width = dst_width;
    dst_img.height = dst_height;
    dst_img.stride = dst_stride;
    dst_img.buf_type = type;
    dst_img.color_mode = dst_fmt;

    INFO("src fd %d stride %d dst stride %d fd %d", src_fd, src_img.stride, dst_img.stride, dst_fd);

    rga_multiple_transform(rga_ctx, &src_img, &dst_img,
                           0, 0, src_img.width, src_img.height,
                           0, 0, dst_img.width, dst_img.height,
                           0, 0, 0);
    rga_exec(rga_ctx);

    return 0;
}

int read_yuv(const char* filename,uint8_t* pdata,int width,int height) {
    FILE *fp = fopen(filename, "rb");
    if ((pdata == NULL) || (fp == NULL)) return 0;
    printf("read yuv-frame(%dx%d) data from %s\n", width,height,filename );
    fread(pdata, width*height*3/2, 1, fp);
    fclose(fp);
    return 1;
}

int write_truecolor_tga( int* data, int width, int height, int number ) {
    char filename[256];
    sprintf( filename, "frame.%d.tga", number );
    FILE *fp = fopen(filename, "wb");
    if (fp == NULL) return 0;

    printf("write %d frame(tga) data to %s\n", number, filename );
    char header[ 18 ] = { 0 }; // char = byte
    header[ 2 ] = 2; // truecolor
    header[ 12 ] = width & 0xFF;
    header[ 13 ] = (width >> 8) & 0xFF;
    header[ 14 ] = height & 0xFF;
    header[ 15 ] = (height >> 8) & 0xFF;
    header[ 16 ] = 24; // bits per pixel

    fwrite((const char*)&header, 1, sizeof(header), fp);
    int x,y;
    for (y = height -1; y >= 0; y--)
        for (x = 0; x < width; x++)
        {
            char b = (data[x+(y*width)] & 0x0000FF);
            char g = (data[x+(y*width)] & 0x00FF00) >> 8;
            char r = (data[x+(y*width)] & 0xFF0000) >> 16;
            putc((int)(r & 0xFF),fp);
            putc((int)(g & 0xFF),fp);
            putc((int)(b & 0xFF),fp);
        }

    static const char footer[ 26 ] =
        "\0\0\0\0" // no extension area
        "\0\0\0\0" // no developer directory
        "TRUEVISION-XFILE" // yep, this is a TGA file
        ".";
    fwrite((const char*)&footer, 1, sizeof(footer), fp);

    fclose(fp);
    return 1;
}

void decodeYUV420SP(int *rgb, unsigned char *yuv420sp, int width, int height, int aligned_width, int aligned_height) {
    int frameSize = aligned_width * aligned_height;
    int j, yp, uvp, u, v, i, y, y1192, r, g, b, rgbp;

    for (j = 0, yp = 0, rgbp = 0; j < height; j++) {
        uvp = frameSize + (j >> 1) * aligned_width, u = 0, v = 0;
        yp = j * aligned_width;
        for (i = 0; i < width; i++, yp++, rgbp++) {
            y = (0xff & ((int) yuv420sp[yp])) - 16;
            if (y < 0) {
                y = 0;
            }
            if ((i & 1) == 0) {
                v = (0xff & yuv420sp[uvp++]) - 128;
                u = (0xff & yuv420sp[uvp++]) - 128;
            }

            y1192 = 1192 * y;
            r = (y1192 + 1634 * v);
            g = (y1192 - 833 * v - 400 * u);
            b = (y1192 + 2066 * u);

            if (r < 0) {
                r = 0;
            } else if (r > 262143) {
                r = 262143;
            }
            if (g < 0) {
                g = 0;
            } else if (g > 262143) {
                g = 262143;
            }
            if (b < 0) {
                b = 0;
            } else if (b > 262143) {
                b = 262143;
            }

            rgb[rgbp] = 0xff000000 | ((r << 6) & 0xff0000) | ((g >> 2) & 0xff00)
                        | ((b >> 10) & 0xff);
        }
    }
}

void testRGA() {
    int rga_fd_1, rga_fd_2;
    uint8_t *rga_vaddr_1, *rga_vaddr_2;

    // alloc two drm buffers
    if( drm_alloc(1088 * 1920 * 4, &rga_fd_1, &rga_vaddr_1 ) <= 0 ) {
	ERROR("drm_alloc() failed!");
	exit(-1);
    }
    if( drm_alloc(1088 * 1920 * 4, &rga_fd_2, &rga_vaddr_2 ) <= 0 ) {
	ERROR("drm_alloc() failed!");
	exit(-1);
    }
    INFO("rga1: fd=%d,vaddr=%p", rga_fd_1, rga_vaddr_1);
    INFO("rga2: fd=%d,vaddr=%p", rga_fd_2, rga_vaddr_2);

    // load nv12 image in rga buffer 1 (file is handy rgb/yuv test picture)
    if(read_yuv("rgb1920x1088.nv12",rga_vaddr_1,1920,1088) == 0) {
	ERROR("read_yuv() failed!");
	exit(-1);
    }
    INFO("yuv/nv12 image loaded at rga_vaddr_1");

    // convert nv12 image at rga buffer 1 to rgba image at rga buffer 2
    if( rga_convert_copy(rga_ctx, 
		rga_fd_1, 1920, 1088, 1920, DRM_FORMAT_NV12,
		rga_fd_2, 1920, 1088, 1920 * 4, DRM_FORMAT_ABGR8888,
                RGA_IMGBUF_GEM, 
	        1920, 1088) != 0 ) {
	ERROR("rga_convert_copy() failed!");
	exit(-1);
    }
    INFO("rga1/nv12 converted-copied to rga2/rgba");

    // write converted rgba image to file 1 (in tga format)
    write_truecolor_tga((int*)rga_vaddr_2, 1920, 1088, 1 );
    INFO("rga_vaddr_2 saved to rgba image 1");

    // also convert nv12 image using sw conversion and write to tga file 1
    int* rgbsw_addr = (int*)malloc(1920*1088*4);

    decodeYUV420SP(rgbsw_addr, rga_vaddr_1, 1920, 1088, 1920, 1088);
    INFO("rga1/nv12 converted-copied by sw to rgbsw/rgba");

    write_truecolor_tga(rgbsw_addr, 1920, 1088, 2 );
    INFO("rgbsw saved to rgba image 2");

    free(rgbsw_addr);
}

int main(int argc, char *argv[]) {
    initRGA();
    testRGA();
}
