/*
 * GStreamer
 * Copyright (C) 2015 Matthew Waters <matthew@centricular.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <vector>
#include <stdio.h>

#include <gst/video/video.h>
#include <gst/gl/gl.h>

#if GST_GL_HAVE_VIV_DIRECTVIV
#define GL_GLEXT_PROTOTYPES
#include <gst/allocators/imx/phys_mem_meta.h>
#endif

#include <gst/gl/gstglfuncs.h>
#include "gstqsgtexture.h"


#define GST_CAT_DEFAULT gst_qsg_texture_debug
GST_DEBUG_CATEGORY_STATIC (GST_CAT_DEFAULT);

GstQSGTexture::GstQSGTexture ()
{
  static volatile gsize _debug;

  initializeOpenGLFunctions();

  if (g_once_init_enter (&_debug)) {
    GST_DEBUG_CATEGORY_INIT (GST_CAT_DEFAULT, "qtqsgtexture", 0,
        "Qt Scenegraph Texture");
    g_once_init_leave (&_debug, 1);
  }

  gst_video_info_init (&this->v_info);
  this->buffer_ = NULL;
  this->qt_context_ = NULL;
  this->sync_buffer_ = gst_buffer_new ();
  this->dummy_tex_id_ = 0;
#if GST_GL_HAVE_VIV_DIRECTVIV
  QOpenGLContext *qglcontext = QOpenGLContext::currentContext ();

  if (qglcontext) {
    QOpenGLFunctions *funcs = qglcontext->functions ();
    this->video_texture_ = 0;
    funcs->glGenTextures (1, &this->video_texture_);
    funcs->glBindTexture (GL_TEXTURE_2D, this->video_texture_);
    funcs->glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    funcs->glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  }

  this->video_info_updated = true;
#endif
}

GstQSGTexture::~GstQSGTexture ()
{
  gst_buffer_replace (&this->buffer_, NULL);
  gst_buffer_replace (&this->sync_buffer_, NULL);
  if (QOpenGLContext::currentContext ()) {
    if (this->video_texture_) {
      QOpenGLContext::currentContext ()->functions ()->glDeleteTextures (1,
        &this->video_texture_);
    }

    if (this->dummy_tex_id_) {
      QOpenGLContext::currentContext ()->functions ()->glDeleteTextures (1,
        &this->dummy_tex_id_);
    }
  }
}

/* only called from the streaming thread with scene graph thread blocked */
void
GstQSGTexture::setCaps (GstCaps * caps)
{
  GST_LOG ("%p setCaps %" GST_PTR_FORMAT, this, caps);

  gst_video_info_from_caps (&this->v_info, caps);
#if GST_GL_HAVE_VIV_DIRECTVIV
  this->video_info_updated = true;
#endif
}

/* only called from the streaming thread with scene graph thread blocked */
gboolean
GstQSGTexture::setBuffer (GstBuffer * buffer)
{
  GST_LOG ("%p setBuffer %" GST_PTR_FORMAT, this, buffer);
  /* FIXME: update more state here */
  if (!gst_buffer_replace (&this->buffer_, buffer))
    return FALSE;

  this->qt_context_ = gst_gl_context_get_current ();
#if GST_GL_HAVE_VIV_DIRECTVIV
  this->video_info_updated = true;
#endif

  return TRUE;
}

/* only called from qt's scene graph render thread */
#if GST_GL_HAVE_VIV_DIRECTVIV
void
GstQSGTexture::bind ()
{
    if (!(this->viv_planes[0] == NULL || this->video_info_updated))
    {
        GST_LOG("video frame did not change - not doing anything");
        return;
    }

    auto gst_fmt_to_gl_format = [] (GstVideoFormat format) {
        switch (format)
        {
        case GST_VIDEO_FORMAT_I420:  return GL_VIV_I420;
        case GST_VIDEO_FORMAT_YV12:  return GL_VIV_YV12;
        case GST_VIDEO_FORMAT_NV12:  return GL_VIV_NV12;
        case GST_VIDEO_FORMAT_NV21:  return GL_VIV_NV21;
        case GST_VIDEO_FORMAT_YUY2:  return GL_VIV_YUY2;
        case GST_VIDEO_FORMAT_UYVY:  return GL_VIV_UYVY;
        case GST_VIDEO_FORMAT_RGB16: return GL_RGB565;
        case GST_VIDEO_FORMAT_RGBA:  return GL_RGBA;
        case GST_VIDEO_FORMAT_BGRA:  return GL_BGRA_EXT;
        case GST_VIDEO_FORMAT_RGBx:  return GL_RGBA;
        case GST_VIDEO_FORMAT_BGRx:  return GL_BGRA_EXT;
        default: return 0;
        }
    };

    auto check_gl_error = [this](char const *category, char const * label) {
        GLenum err = glGetError();
        if (err == GL_NO_ERROR)
            return true;

        switch (err)
        {
        case GL_INVALID_ENUM:                  GST_ERROR("[%s] [%s] error: invalid enum", category, label); break;
        case GL_INVALID_VALUE:                 GST_ERROR("[%s] [%s] error: invalid value", category, label); break;
        case GL_INVALID_OPERATION:             GST_ERROR("[%s] [%s] error: invalid operation", category, label); break;
        case GL_INVALID_FRAMEBUFFER_OPERATION: GST_ERROR("[%s] [%s] error: invalid framebuffer operation", category, label); break;
        case GL_OUT_OF_MEMORY:                 GST_ERROR("[%s] [%s] error: out of memory", category, label); break;
        default:                               GST_ERROR("[%s] [%s] error: unknown GL error 0x%x", category, label, err);
        }

        return false;
    };

    auto bpp = [](GstVideoFormat fmt)
    {
        switch (fmt)
        {
        case GST_VIDEO_FORMAT_RGB16: return 2;
        case GST_VIDEO_FORMAT_RGB: return 3;
        case GST_VIDEO_FORMAT_RGBA: return 4;
        case GST_VIDEO_FORMAT_BGRA: return 4;
        case GST_VIDEO_FORMAT_RGBx: return 4;
        case GST_VIDEO_FORMAT_BGRx: return 4;
        case GST_VIDEO_FORMAT_YUY2: return 2;
        case GST_VIDEO_FORMAT_UYVY: return 2;
        default: return 1;
        }
    };

    const GstGLFuncs *gl;
    GstMapInfo map_info;
    gboolean use_dummy_tex = TRUE;
    GstVideoMeta *video_meta;
    guint num_extra_lines, stride[3], offset[3];
    GstImxPhysMemMeta *phys_mem_meta;
    guint is_phys_buf;
    GLuint phys_addr;
    GLuint w, h, total_w, total_h;
    GLvoid *virt_addr;

    if (!this->qt_context_)
        return;

    if (!this->buffer_)
        goto out;
    if (GST_VIDEO_INFO_FORMAT (&this->v_info) == GST_VIDEO_FORMAT_UNKNOWN)
        goto out;

    this->mem_ = gst_buffer_peek_memory (this->buffer_, 0);
    if (!this->mem_)
        goto out;

    g_assert (this->qt_context_);
    gl = this->qt_context_->gl_vtable;

    phys_mem_meta = GST_IMX_PHYS_MEM_META_GET(this->buffer_);
    is_phys_buf = (phys_mem_meta != NULL) && (phys_mem_meta->phys_addr != 0);
    phys_addr = (GLuint)(phys_mem_meta->phys_addr);
    w = this->v_info.width;
    h = this->v_info.height;

    num_extra_lines = is_phys_buf ? phys_mem_meta->y_padding : 0;

    /* Get the stride and number of extra lines */
    video_meta = gst_buffer_get_video_meta(this->buffer_);
    if (video_meta != NULL)
    {
        for (guint i = 0; i < MIN(video_meta->n_planes, 3); ++i)
        {
            stride[i] = video_meta->stride[i];
            offset[i] = video_meta->offset[i];
        }
    }
    else
    {
        for (guint i = 0; i < MIN(GST_VIDEO_INFO_N_PLANES(&(this->v_info)), 3); ++i)
        {
            stride[i] = GST_VIDEO_INFO_PLANE_STRIDE(&(this->v_info), i);
            offset[i] = GST_VIDEO_INFO_PLANE_OFFSET(&(this->v_info), i);
        }
    }

    total_w = stride[0] / bpp(this->v_info.finfo->format);
    total_h = h + num_extra_lines;

    GST_LOG("w/h: %u/%u total_w/h: %u/%u num extra lines: %u", w, h, total_w, total_h, num_extra_lines);

    QOpenGLContext::currentContext ()->functions()->glBindTexture(GL_TEXTURE_2D, this->video_texture_);
    QOpenGLContext::currentContext ()->functions()->glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    QOpenGLContext::currentContext ()->functions()->glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    if (is_phys_buf)
    {
        /* Safeguard to catch data loss if in any future i.MX version the types do not match */
        g_assert(((gst_imx_phys_addr_t)(phys_addr)) == phys_mem_meta->phys_addr);

        gst_buffer_map(this->buffer_, &map_info, GST_MAP_READ);
        virt_addr = map_info.data;

        GST_LOG("mapping physical address %" GST_IMX_PHYS_ADDR_FORMAT " of video frame in buffer %p into VIV texture", phys_mem_meta->phys_addr, (gpointer)this->buffer_);

        glTexDirectVIVMap(
                    GL_TEXTURE_2D,
                    total_w, total_h,
                    gst_fmt_to_gl_format(this->v_info.finfo->format),
                    (GLvoid **)(&virt_addr), &phys_addr
                    );

        gst_buffer_unmap(this->buffer_, &map_info);

        if (!check_gl_error("render", "glTexDirectVIVMap"))
            goto out;
    }
    else
    {
        glTexDirectVIV(
                    GL_TEXTURE_2D,
                    total_w, total_h,
                    gst_fmt_to_gl_format(this->v_info.finfo->format),
                    (GLvoid **) &(viv_planes)
                    );

        if (!check_gl_error("render", "glTexDirectVIV"))
            goto out;

        gst_buffer_map(this->buffer_, &map_info, GST_MAP_READ);

        GST_LOG("copying pixels into VIV direct texture buffer");

        switch (this->v_info.finfo->format)
        {
        case GST_VIDEO_FORMAT_I420:
        case GST_VIDEO_FORMAT_YV12:
            memcpy(this->viv_planes[0], map_info.data + offset[0], stride[0] * total_h);
            memcpy(this->viv_planes[1], map_info.data + offset[1], stride[1] * total_h / 2);
            memcpy(this->viv_planes[2], map_info.data + offset[2], stride[2] * total_h / 2);
            break;
        case GST_VIDEO_FORMAT_NV12:
        case GST_VIDEO_FORMAT_NV21:
            memcpy(this->viv_planes[0], map_info.data + offset[0], stride[0] * total_h);
            memcpy(this->viv_planes[1], map_info.data + offset[1], stride[1] * total_h / 2);
            break;
        default:
            memcpy(this->viv_planes[0], map_info.data, stride[0] * total_h);
        }

        gst_buffer_unmap(this->buffer_, &map_info);
    }

    glTexDirectInvalidateVIV(GL_TEXTURE_2D);

    if (!check_gl_error("render", "glTexDirectInvalidateVIV"))
        goto out;

    /* Texture was successfully bound, so we do not need
   * to use the dummy texture */
    use_dummy_tex = FALSE;

out:
    if (G_UNLIKELY (use_dummy_tex)) {
        QOpenGLContext *qglcontext = QOpenGLContext::currentContext ();
        QOpenGLFunctions *funcs = qglcontext->functions ();

         GST_ERROR("Unable to bind texture, using dummy texture instead");

        /* Create dummy texture if not already present.
     * Use the Qt OpenGL functions instead of the GstGL ones,
     * since we are using the Qt OpenGL context here, and we must
     * be able to delete the texture in the destructor. */
        if (this->dummy_tex_id_ == 0) {
            /* Make this a black 64x64 pixel RGBA texture.
       * This size and format is supported pretty much everywhere, so these
       * are a safe pick. (64 pixel sidelength must be supported according
       * to the GLES2 spec, table 6.18.)
       * Set min/mag filters to GL_LINEAR to make sure no mipmapping is used. */
            const int tex_sidelength = 64;
            std::vector < guint8 > dummy_data (tex_sidelength * tex_sidelength * 4, 0);

            funcs->glGenTextures (1, &this->dummy_tex_id_);
            funcs->glBindTexture (GL_TEXTURE_2D, this->dummy_tex_id_);
            funcs->glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            funcs->glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            funcs->glTexImage2D (GL_TEXTURE_2D, 0, GL_RGBA, tex_sidelength,
                                 tex_sidelength, 0, GL_RGBA, GL_UNSIGNED_BYTE, &dummy_data[0]);
        }

        g_assert (this->dummy_tex_id_ != 0);

        funcs->glBindTexture (GL_TEXTURE_2D, this->dummy_tex_id_);
    }

    this->video_info_updated = false;
}

/* can be called from any thread */
int
GstQSGTexture::textureId () const
{
  return video_texture_;
}

#else
void
GstQSGTexture::bind ()
{
  const GstGLFuncs *gl;
  GstGLContext *context;
  GstGLSyncMeta *sync_meta;
  GstMemory *mem;
  guint tex_id;
  gboolean use_dummy_tex = TRUE;

  if (!this->qt_context_)
    return;

  if (!this->buffer_)
    goto out;
  if (GST_VIDEO_INFO_FORMAT (&this->v_info) == GST_VIDEO_FORMAT_UNKNOWN)
    goto out;

  this->mem_ = gst_buffer_peek_memory (this->buffer_, 0);
  if (!this->mem_)
    goto out;

  g_assert (this->qt_context_);
  gl = this->qt_context_->gl_vtable;

  /* FIXME: should really lock the memory to prevent write access */
  if (!gst_video_frame_map (&this->v_frame, &this->v_info, this->buffer_,
        (GstMapFlags) (GST_MAP_READ | GST_MAP_GL))) {
    g_assert_not_reached ();
    goto out;
  }

  mem = gst_buffer_peek_memory (this->buffer_, 0);
  g_assert (gst_is_gl_memory (mem));

  context = ((GstGLBaseMemory *)mem)->context;

  sync_meta = gst_buffer_get_gl_sync_meta (this->sync_buffer_);
  if (!sync_meta)
    sync_meta = gst_buffer_add_gl_sync_meta (context, this->sync_buffer_);

  gst_gl_sync_meta_set_sync_point (sync_meta, context);

  gst_gl_sync_meta_wait (sync_meta, this->qt_context_);

  tex_id = *(guint *) this->v_frame.data[0];
  GST_LOG ("%p binding Qt texture %u", this, tex_id);

  gl->BindTexture (GL_TEXTURE_2D, tex_id);

  gst_video_frame_unmap (&this->v_frame);

  /* Texture was successfully bound, so we do not need
   * to use the dummy texture */
  use_dummy_tex = FALSE;

out:
  if (G_UNLIKELY (use_dummy_tex)) {
    QOpenGLContext *qglcontext = QOpenGLContext::currentContext ();
    QOpenGLFunctions *funcs = qglcontext->functions ();

    /* Create dummy texture if not already present.
     * Use the Qt OpenGL functions instead of the GstGL ones,
     * since we are using the Qt OpenGL context here, and we must
     * be able to delete the texture in the destructor. */
    if (this->dummy_tex_id_ == 0) {
      /* Make this a black 64x64 pixel RGBA texture.
       * This size and format is supported pretty much everywhere, so these
       * are a safe pick. (64 pixel sidelength must be supported according
       * to the GLES2 spec, table 6.18.)
       * Set min/mag filters to GL_LINEAR to make sure no mipmapping is used. */
      const int tex_sidelength = 64;
      std::vector < guint8 > dummy_data (tex_sidelength * tex_sidelength * 4, 0);

      funcs->glGenTextures (1, &this->dummy_tex_id_);
      funcs->glBindTexture (GL_TEXTURE_2D, this->dummy_tex_id_);
      funcs->glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      funcs->glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      funcs->glTexImage2D (GL_TEXTURE_2D, 0, GL_RGBA, tex_sidelength,
          tex_sidelength, 0, GL_RGBA, GL_UNSIGNED_BYTE, &dummy_data[0]);
    }

    g_assert (this->dummy_tex_id_ != 0);

    funcs->glBindTexture (GL_TEXTURE_2D, this->dummy_tex_id_);
  }
}

/* can be called from any thread */
int
GstQSGTexture::textureId () const
{
  int tex_id = 0;

  if (this->buffer_) {
    GstMemory *mem = gst_buffer_peek_memory (this->buffer_, 0);

    tex_id = ((GstGLMemory *) mem)->tex_id;
  }

  GST_LOG ("%p get texture id %u", this, tex_id);

  return tex_id;
}
#endif


/* can be called from any thread */
QSize
GstQSGTexture::textureSize () const
{
  if (GST_VIDEO_INFO_FORMAT (&this->v_info) == GST_VIDEO_FORMAT_UNKNOWN)
    return QSize (0, 0);

  GST_TRACE ("%p get texture size %ux%u", this, this->v_info.width,
      this->v_info.height);

  return QSize (this->v_info.width, this->v_info.height);
}

/* can be called from any thread */
bool
GstQSGTexture::hasAlphaChannel () const
{
  /* FIXME: support RGB textures */
  return true;
}

/* can be called from any thread */
bool
GstQSGTexture::hasMipmaps () const
{
  return false;
}
