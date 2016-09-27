 /* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2016, OwlReality Co., Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of OwlReality Co., Ltd. nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <math.h>
#include "360tools.h"
#include "360tools_ssp.h"

/*-------------------------------   FROM ERP TO SSP   -----------------------------*/


static void erp_to_ssp_sph2point(double  lng, double lat, double* x, \
                                double* y, int w_src, int h_src)
{
    *x = lng/M_2PI * w_src + w_src / 2;
    *y = h_src/2 - lat/M_PI_2 * h_src / 2;
}


static int erp_to_ssp_plane(void * src, int w_src, int h_src, int s_src, \
                           int w_dst, int h_dst, int s_dst, void * dst, int opt, int cs)
{
    void(*fn_resample)(void * src, int w_start, int w_end, int h_src, int s_src,
                       double x, double y, void * dst, int x_dst);
    double  lng, lat, x, y;
    int     i, j;
    
    if (cs == S360_COLORSPACE_YUV420)
    {
        fn_resample = resample_2d;
    }
    else if (cs == S360_COLORSPACE_YUV420_10)
    {
        fn_resample = resample_2d_10b;
        s_dst <<= 1;
    }
    double w_center = w_dst;
    double w_square = h_dst / 3.0;
    double pole_x, pole_y, pole_d;
    for (j = 0; j<h_dst; j++)
    {
        for (i = 0; i<w_dst; i++)
        {
            if (j >= w_square && j < w_square*2) {
                lng = i / w_center * M_2PI - PI;
                lat = M_PI_2 / 2 - (j - w_square) * M_PI_2 / w_square;
            }
            else if (j < w_square) {
                if (i < (w_center - w_square) / 2 || i>= (w_center + w_square) / 2)
                    continue;
                pole_x = i - w_center / 2;
                pole_y = j - w_square / 2;
                pole_d = sqrt(pole_x * pole_x + pole_y * pole_y);
                lat = M_PI_2 - pole_d / w_square * M_PI_2;
                lng = (pole_d > 0) ? acos(pole_y / pole_d) : 0;
                lng = (pole_x < 0) ? -lng : lng;
            }
            else {
                if (i < (w_center - w_square) / 2 || i >= (w_center + w_square) / 2)
                    continue;
                pole_x = i - w_center / 2;
                pole_y = w_square * 5 / 2 - j;
                pole_d = sqrt(pole_x * pole_x + pole_y * pole_y);
                lat = pole_d / w_square * M_PI_2 - M_PI_2;
                lng = (pole_d > 0) ? acos(pole_y / pole_d) : 0;
                lng = (pole_x < 0) ? -lng : lng;
            }
            
            erp_to_ssp_sph2point(lng, lat, &x, &y, w_src, h_src);
            fn_resample(src, 0, w_src, h_src, s_src, x, y, dst, i);
        }
        dst = (void *)((uint8 *)dst + s_dst);
    }
    
    return S360_OK;
}


int o360_erp_to_ssp(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, S360_MAP * map)
{
    int w_src, h_src, w_dst, h_dst;
    
    w_src = img_src->width;
    h_src = img_src->height;
    
    w_dst = img_dst->width;
    h_dst = img_dst->height;
    
    s360_img_reset(img_dst);
    if (opt & S360_OPT_PAD)
        s360_pad_erp(img_src);
    
    if (IS_VALID_CS(img_src->colorspace))
    {
        erp_to_ssp_plane(img_src->buffer[0], w_src, h_src, img_src->stride[0], w_dst, h_dst, img_dst->stride[0], img_dst->buffer[0], opt, img_src->colorspace);
        w_src >>= 1;
        h_src >>= 1;
        w_dst >>= 1;
        h_dst >>= 1;
        erp_to_ssp_plane(img_src->buffer[1], w_src, h_src, img_src->stride[1], w_dst, h_dst, img_dst->stride[1], img_dst->buffer[1], opt, img_src->colorspace);
        erp_to_ssp_plane(img_src->buffer[2], w_src, h_src, img_src->stride[2], w_dst, h_dst, img_dst->stride[2], img_dst->buffer[2], opt, img_src->colorspace);
    }
    else
    {
        return S360_ERR_UNSUPPORTED_COLORSPACE;
    }
    return S360_OK;
}


/*-------------------------------   FROM SSP TO ERP    ---------------------------------*/


static void ssp_to_erp_sph2point(double  lng, double lat, double* x, \
                                double* y, int w_src, int h_src, int* x_begin, int* x_end, int* y_begin, int* y_end)
{
    double w_center = w_src;
    double w_square = h_src / 3;

    if (lat > M_PI_2 / 2) {
        *x = w_center/2 + w_square * sin(lng) * (M_PI_2 - lat) / M_PI_2;
        *y = w_square / 2 * (1 + cos(lng) * 2 * (M_PI_2 - lat) / M_PI_2);
        *x_begin = (w_center - w_square) / 2;
        *x_end   = (w_center + w_square) / 2;
        *y_begin = 0;
        *y_end   = w_square;
    }
    else if (lat < -M_PI_2 / 2){
        *x = w_center/2 + w_square * sin(lng) * (M_PI_2 + lat) / M_PI_2;
        *y = w_square * 2 + w_square / 2 * (1 - cos(lng) * 2 * (M_PI_2 + lat) / M_PI_2);
        *x_begin = (w_center - w_square) / 2;
        *x_end   = (w_center + w_square) / 2;
        *y_begin = w_square*2;
        *y_end   = h_src;
    }
    else
    {
        *x = lng / M_2PI * w_center + w_center / 2;
        *y = w_square / 2 - lat / M_PI_2 * w_square + w_square;
        *x_begin = 0;
        *x_end   = w_src;
        *y_begin = w_square;
        *y_end   = w_square*2;
    }
    
}

static int ssp_to_erp_plane(void * src, int w_src, int h_src, int s_src, \
                           int w_dst, int h_dst, int s_dst, void * dst, int opt, int cs)
{
    void(*fn_resample)(void * src, int w_start, int w_end, int h_src, int s_src,
                       double x, double y, void * dst, int x_dst, int x_begin, int x_end, int y_start, int y_end);
    double  lng, lat, x, y;
    int     i, j,
            x_begin, x_end, y_begin, y_end;
    
    if (cs == S360_COLORSPACE_YUV420)
    {
        fn_resample = resample_2d_bounded;
    }
    else if (cs == S360_COLORSPACE_YUV420_10)
    {
        fn_resample = resample_2d_10b_bounded;
        s_dst <<= 1;
    }
    
    for (j = 0; j<h_dst; j++)
    {
        for (i = 0; i<w_dst; i++)
        {
            lng = M_2PI * i / w_dst - PI;
            lat = M_PI_2 - PI * j / h_dst;
            
            ssp_to_erp_sph2point(lng, lat, &x, &y, w_src, h_src, &x_begin, &x_end, &y_begin, &y_end);//where w_src=h_src
            fn_resample(src, 0, w_src, h_src, s_src, x, y, dst, i, x_begin, x_end, y_begin, y_end);
        }
        dst = (void *)((uint8 *)dst + s_dst);
    }
    
    return S360_OK;
}

int o360_ssp_to_erp(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, S360_MAP * map)
{
    int w_src, h_src, w_dst, h_dst;
    
    w_src = img_src->width;
    h_src = img_src->height;
    
    w_dst = img_dst->width;
    h_dst = img_dst->height;
    
    if (IS_VALID_CS(img_src->colorspace))
    {
        ssp_to_erp_plane(img_src->buffer[0], w_src, h_src, img_src->stride[0], w_dst, h_dst, img_dst->stride[0], img_dst->buffer[0], opt, img_src->colorspace);
        w_src >>= 1;
        h_src >>= 1;
        w_dst >>= 1;
        h_dst >>= 1;
        ssp_to_erp_plane(img_src->buffer[1], w_src, h_src, img_src->stride[1], w_dst, h_dst, img_dst->stride[1], img_dst->buffer[1], opt, img_src->colorspace);
        ssp_to_erp_plane(img_src->buffer[2], w_src, h_src, img_src->stride[2], w_dst, h_dst, img_dst->stride[2], img_dst->buffer[2], opt, img_src->colorspace);
    }
    else
    {
        return S360_ERR_UNSUPPORTED_COLORSPACE;
    }
    
    return S360_OK;
}
