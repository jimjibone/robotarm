#ifndef __robotarm__Freenect__
#define __robotarm__Freenect__

#include <libfreenect.hpp>
#include <libfreenect/libfreenect-registration.h>
#include "Mutex.hpp"

// Kinect Hardware Connection Class
// Thanks to Yoda---- from IRC (libfreenect)
class MyFreenectDevice : public Freenect::FreenectDevice {
	public:
		MyFreenectDevice(freenect_context *_ctx, int _index):
			Freenect::FreenectDevice(_ctx, _index),
			depth(freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM,FREENECT_DEPTH_REGISTERED).bytes),
			m_buffer_video(freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM,FREENECT_VIDEO_RGB).bytes),
			m_new_rgb_frame(false), m_new_depth_frame(false)
			{
			}
	//~MyFreenectDevice(){}
		// Do not call directly even in child
		void VideoCallback(void* _rgb, uint32_t timestamp) {
			Mutex::ScopedLock lock(m_rgb_mutex);
			uint8_t* rgb = static_cast<uint8_t*>(_rgb);
			std::copy(rgb, rgb+getVideoBufferSize(), m_buffer_video.begin());
			m_new_rgb_frame = true;
		};
		// Do not call directly even in child
		void DepthCallback(void* _depth, uint32_t timestamp) {
			Mutex::ScopedLock lock(m_depth_mutex);
			depth.clear();
			uint16_t* call_depth = static_cast<uint16_t*>(_depth);
			for (size_t i = 0; i < 640*480 ; i++) depth.push_back(call_depth[i]);
			m_new_depth_frame = true;
		}
		bool getRGB(std::vector<uint8_t>&buffer) {
			//printf("Getting RGB!\n");
			Mutex::ScopedLock lock(m_rgb_mutex);
			if (!m_new_rgb_frame) {
				//printf("No new RGB Frame.\n");
				return false;
			}
			buffer.swap(m_buffer_video);
			m_new_rgb_frame = false;
			return true;
		}
		bool getDepth(std::vector<uint16_t>&buffer) {
			Mutex::ScopedLock lock(m_depth_mutex);
			if (!m_new_depth_frame) return false;
			buffer.swap(depth);
			m_new_depth_frame = false;
			return true;
		}
	private:
		std::vector<uint16_t> depth;
		std::vector<uint8_t> m_buffer_video;
		Mutex m_rgb_mutex;
		Mutex m_depth_mutex;
		bool m_new_rgb_frame;
		bool m_new_depth_frame;
};

#endif /* defined(__robotarm__Freenect__) */
