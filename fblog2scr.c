// fblog2scr - Boilerplate code to get framebuffer graphics with working keyboard and mouse/touch
// Author: Anup Jayapal Rao, 2024

// License: Apache License

// SDL based Emulation Build
// 
// gcc -o fblog2scr_SDL -D BACKEND_SDL -D SCALED_OUTPUT fblog2scr.c -I /usr/include/SDL2 -l SDL2 -l m

/////////////////////////////////////////////////////////////////////////////////////////

// FrameBuffer Build
// 
// export PATH=/mnt/workspace/sdks/armv7l-linux-musleabihf-cross/bin:$PATH 
// export PATH=/run/media/anup/ext_sdks/armv7l-linux-musleabihf-cross/bin/armv7l-linux-musleabihf-gcc:$PATH
// 
// armv7l-linux-musleabihf-gcc -o fblog2scr_fb -D BACKEND_FB fblog2scr.c
// 
// Output binary size if 18KB
// requires replacing the ld-musl-armhf.so.1 in the /lib (in rfs,or,initrd)
// You may need to make the ld-musl-armhf.so.1 a symbolic link to 
// libc.so from the toolchain's "sdks/armv7l-linux-musleabihf-cross/armv7l-linux-musleabihf/lib" folder
// The libc.so needs to be copied to the /lib folder (in rfs,or,initrd)

// or,

// export PATH=$PATH:/mnt/workspace/sdks/linaro_7_hf/toolchain/bin
//
// arm-linux-gnueabihf-gcc -o fblog2scr_fb -D BACKEND_FB fblog2scr.c --static -lpthread
//
// Output binary size if 4.6 MB

#include <errno.h>
#include <inttypes.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>

#include <fcntl.h>
#include <linux/input.h>

#include <sys/mman.h>
#include <sys/time.h>
#include <sys/epoll.h>
#include <sys/timerfd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#include <termios.h>

#include <pthread.h>

//

#define STB_IMAGE_IMPLEMENTATION
#define STBI_NO_PSD
#define STBI_NO_TGA
#define STBI_NO_HDR
#define STBI_NO_PIC
#define STBI_NO_LINEAR
#include "stb_image.h"

#define STB_IMAGE_RESIZE_IMPLEMENTATION
#define STBIR_SATURATE_INT
#include "stb_image_resize2.h"

struct rgba {
	uint8_t r;
	uint8_t g;
	uint8_t b;
	uint8_t a;
} __attribute__((packed));

#define STB_TRUETYPE_IMPLEMENTATION 
#include "stb_truetype.h" /* http://nothings.org/stb/stb_truetype.h */

//

#define FB_WIDTH  720
#define FB_HEIGHT 1280

#define MAX_EVENTS 32

#define FB_KEY_ESC	0x1B

#define READ_SIZE			160

//

#ifdef BACKEND_SDL

	#define FN_INPUT_FIFO		"INITWRAPPER_INPUT"
	#define FN_INPUT_READ		FN_INPUT_FIFO
	#define FN_INPUT_WRITE		FN_INPUT_FIFO

	#define FN_OUTPUT_FIFO		"INITWRAPPER_OUTPUT"
	#define FN_OUTPUT_READ		FN_OUTPUT_FIFO
	#define FN_OUTPUT_WRITE		FN_OUTPUT_FIFO

#endif // BACKEND_SDL

#ifdef BACKEND_FB

	#define FN_INPUT_FIFO		"/INITWRAPPER_INPUT"
	#define FN_INPUT_READ		FN_INPUT_FIFO
	#define FN_INPUT_WRITE		FN_INPUT_FIFO

	#define FN_OUTPUT_FIFO		"/INITWRAPPER_OUTPUT"
	#define FN_OUTPUT_READ		FN_OUTPUT_FIFO
	#define FN_OUTPUT_WRITE		FN_OUTPUT_FIFO

#endif // BACKEND_FB

//

struct termios prev_tty_cfg;

int epoll_fd;

int fdTimerScrRefresh;
struct itimerspec ts;

unsigned char inputKeys[MAX_EVENTS];

int fdMouseTouch;
struct input_event  evInputEvents[MAX_EVENTS];

pthread_t thGraphics;

pthread_mutex_t lockBackBuffer;

bool bUpdate = false;

//

typedef struct stBufferAttr {
	int BytesPerPixel;
	unsigned char *pixels;
	int pitch;
	int width;
	int height;	
} stBufferAttr;

int graphicsMain( void* ptrData );

//

int fd_input_read = -1;
int fd_output_write = -1;

/////////////////////////////////////////////////////////////////////////////////////////

//

#ifdef BACKEND_SDL

#include <SDL.h>
#include <SDL_render.h>

//

#ifdef SCALED_OUTPUT

	#define SCALE_M     3
	#define SCALE_D     4

	#define WINDOW_WIDTH  (FB_WIDTH*SCALE_M)/SCALE_D
	#define WINDOW_HEIGHT (FB_HEIGHT*SCALE_M)/SCALE_D
#else
	#define WINDOW_WIDTH  FB_WIDTH
	#define WINDOW_HEIGHT FB_HEIGHT
#endif

//

SDL_Window* frontWindow;
SDL_Surface* frontSurface;
SDL_Renderer* frontRenderer;

SDL_Surface* backSurface;

size_t screensize;

//

void blankBuffer(unsigned char *buffer, int nSize)
{
	memset(buffer, 0x00, nSize); // ?? Change to SDL Operations ??
}

void blankFrontBuffer()
{
	SDL_SetRenderDrawColor(frontRenderer, 0, 0, 0, 255);
	SDL_RenderClear(frontRenderer);
}

void blitBackBufferToFrontBuffer()
{
	SDL_BlitScaled(backSurface, NULL, frontSurface, NULL);
}

void updateFrontBuffer()
{
	SDL_UpdateWindowSurface(frontWindow);	
}

void initGraphics(stBufferAttr* ptrBufferAttr)
{
	int return_code;

	if(NULL == ptrBufferAttr)
	{
		return;
	}

	//

	// Initialize the SDL librarys
	return_code = SDL_Init(SDL_INIT_VIDEO);

	if(return_code < 0) 
	{
		fprintf(stderr, "Unable to initialize SDL: %s\n", SDL_GetError());
		exit(1);
	}

	atexit(SDL_Quit);

	// Set the video mode we want

	int width = WINDOW_WIDTH; // 0 means any width
	int height = WINDOW_HEIGHT; // 0 means any height
	int bits_per_pixel = 32; // 0 means use the current bpp
	Uint32 flags = 0; //SDL_RESIZABLE; // make a resizable window

	frontWindow = SDL_CreateWindow("fblog2scr", -1, -1, width, height, flags);
	if(NULL == frontWindow) 
	{
		fprintf(stderr, "Unable to obtain frontWindow: %s\n", SDL_GetError());
		exit(2);
	}

	frontSurface = SDL_GetWindowSurface(frontWindow);
	if(NULL == frontSurface) 
	{
		fprintf(stderr, "Unable to get screen surface: %s\n", SDL_GetError());
		exit(3);
	}

	frontRenderer = SDL_GetRenderer(frontWindow);
	if(NULL == frontRenderer) 
	{
		fprintf(stderr, "Unable to get screen renderer: %s\n", SDL_GetError());
		exit(3);
	}

	//

	printf("%dx%d, %d bpp\n", width, height, 8*frontSurface->format->BytesPerPixel);

	//printf("rotate=%d\n", vinfo.rotate);
	//printf("activate=%d\n", vinfo.activate);
	printf("line_length=%d\n", frontSurface->pitch);

	screensize = height * frontSurface->pitch;
	printf("screensize = %lu\n", screensize);

	// Create a 32-bit surface with the bytes of each pixel in R,G,B,A order, as expected by OpenGL for textures 
	
	Uint32 rmask, gmask, bmask, amask;

#if 0
	rmask = 0xff000000;
	gmask = 0x00ff0000;
	bmask = 0x0000ff00;
	amask = 0x000000ff;
#else
	rmask = 0x000000ff;
	gmask = 0x0000ff00;
	bmask = 0x00ff0000;
	amask = 0xff000000;
#endif

	//

	backSurface = SDL_CreateRGBSurface(SDL_SWSURFACE, FB_WIDTH, FB_HEIGHT, bits_per_pixel, rmask, gmask, bmask, amask);
	if(backSurface == NULL) 
	{
		fprintf(stderr, "CreateRGBSurface failed: %s\n", SDL_GetError());
		exit(1);
	}

	//

	ptrBufferAttr->BytesPerPixel = backSurface->format->BytesPerPixel;
	ptrBufferAttr->pixels = backSurface->pixels;
	ptrBufferAttr->pitch = backSurface->pitch;	
	ptrBufferAttr->width = FB_WIDTH;
	ptrBufferAttr->height = FB_HEIGHT;	
}

void deInitGraphics()
{

}

#endif // BACKEND_SDL

/////////////////////////////////////////////////////////////////////////////////////////

//

#ifdef BACKEND_FB

#include <linux/fb.h>

//

unsigned char *arFrontBuffer = NULL;
int fdFB;

unsigned char *arrBackBuffer = NULL;

size_t screensize;
int nPitch_BackBuffer = 0;

//

void blankBuffer(unsigned char *buffer, int nSize)
{
	memset(buffer, 0x00, nSize);
}

void blankFrontBuffer()
{
	memset(arFrontBuffer, 0x00, screensize);
}

void blitBackBufferToFrontBuffer()
{
	memcpy(arFrontBuffer, arrBackBuffer, screensize);
}

void updateFrontBuffer()
{

}

void initGraphics(stBufferAttr* ptrBufferAttr)
{
	int width, height, posx, posy;

	struct fb_var_screeninfo vinfo;
	struct fb_fix_screeninfo finfo;

	//

	if(NULL == ptrBufferAttr)
	{
		return;
	}

	//

	fdFB = getenv("FRAMEBUFFER") ? open(getenv("FRAMEBUFFER"), O_RDWR) : open("/dev/fb0", O_RDWR);
	if (-1 == fdFB) {
		printf("Failed to open Framebuffer device: %m\n");
		return;
	}

	// Get fixed screen information. 
	if (ioctl(fdFB, FBIOGET_FSCREENINFO, &finfo) == -1)
	{
		close(fdFB);
		perror("Failed to read fixed information");
		exit(2);
	}

	// Get variable screen information. 
	if (ioctl(fdFB, FBIOGET_VSCREENINFO, &vinfo) == -1)
	{
		close(fdFB);
		perror("Failed to read variable information");
		exit(3);
	}

	if (getenv("WIDTH")) 
	{
		width = atoi(getenv("WIDTH"));
	}
	else 
	{
		width = vinfo.xres;
	}

	if (getenv("HEIGHT")) 
	{
		height = atoi(getenv("HEIGHT"));
	}
	else 
	{
		height = vinfo.yres;
	}

	if (getenv("POSX")) 
	{
		posx = atoi(getenv("POSX"));
	}
	else 
	{
		posx = 0;
	}

	if (getenv("POSY")) 
	{
		posy = atoi(getenv("POSY"));
	}
	else 
	{
		posy = 0;
	}

	//
	printf("%dx%d, %d bpp\n", vinfo.xres, vinfo.yres, vinfo.bits_per_pixel);
	printf("rotate=%d\n", vinfo.rotate);
	printf("activate=%d\n", vinfo.activate);
	printf("line_length=%d\n", finfo.line_length);

	// Figure out the size of the screen in bytes.
	//screensize = vinfo.xres * vinfo.yres * vinfo.bits_per_pixel / 8;
	screensize = vinfo.yres * finfo.line_length;
	printf("screensize = %lu\n", screensize);

	// Map the device to memory. 
	arFrontBuffer = (unsigned char *) mmap(0, screensize, PROT_WRITE, MAP_SHARED, fdFB, 0);
	if (-1 == (long) arFrontBuffer)
	{
		close(fdFB);
		perror("Failed to map framebuffer device to memory");
		exit(4);
	}

	printf("The framebuffer device was mapped to memory successfully.\n");
	printf("arFrontBuffer = %p\r\n", arFrontBuffer);

	//

	arrBackBuffer = (unsigned char *) malloc(screensize);
	if(NULL == arrBackBuffer)
	{
		close(fdFB);
		perror("Failed to obtain drawing buffer");
		exit(5);
	}

	//

	ptrBufferAttr->BytesPerPixel = vinfo.bits_per_pixel / 8;
	ptrBufferAttr->pixels = arrBackBuffer;
	ptrBufferAttr->pitch = finfo.line_length;	
	ptrBufferAttr->width = vinfo.xres;
	ptrBufferAttr->height = vinfo.yres;	
}

void deInitGraphics()
{
	// free back buffer
	free(arrBackBuffer);
	arrBackBuffer = NULL;

	// Release screen surface
	munmap(arFrontBuffer, screensize);
	close(fdFB);
}

#endif // BACKEND_FB


/////////////////////////////////////////////////////////////////////////////////////////

void initTerminal()
{
	// TERMINAL SETUP
	struct termios tty_cfg;

	// Check that fd is a TTY
	if (!isatty(STDIN_FILENO)) 
	{
		fprintf(stderr, "Standard input is not a terminal.\n");
		exit(11);
	}

	// Save old terminal configuration
	if (tcgetattr(STDIN_FILENO, &prev_tty_cfg) == -1 || tcgetattr(STDIN_FILENO, &tty_cfg) == -1) 
	{
		fprintf(stderr, "Cannot get terminal settings: %s.\n", strerror(errno));
		exit(12);
	}

	// Set new terminal configuration
	tty_cfg.c_iflag &= ~(IGNBRK | BRKINT | PARMRK);
	tty_cfg.c_lflag &= ~(ICANON | ISIG | ECHO | IEXTEN | TOSTOP);
	tty_cfg.c_cc[VMIN] = 0;
	tty_cfg.c_cc[VTIME] = 0;
	tty_cfg.c_cc[VSTART] = 0;
	tty_cfg.c_cc[VSTOP] = 0;

	if (tcsetattr(STDIN_FILENO, TCSANOW, &tty_cfg) == -1) 
	{
		const int  saved_errno = errno;
		tcsetattr(STDIN_FILENO, TCSANOW, &prev_tty_cfg);
		fprintf(stderr, "Cannot set terminal settings: %s.\n", strerror(saved_errno));
		exit(13);
	}
}

void lockSurface(void)
{
	// Mutex lock
	pthread_mutex_lock(&lockBackBuffer); 
}

void unlockSurface(void)
{
	// Mutex unlock
	pthread_mutex_unlock(&lockBackBuffer); 
}


void* fxn_graphicsMain(void* ptrData)
{
	graphicsMain(ptrData);
	
	return NULL;
}

void launchGraphicsThread(stBufferAttr* ptrBufferAttr)
{
	if(NULL == ptrBufferAttr)
	{
		return;
	}

	pthread_create(&thGraphics, NULL, fxn_graphicsMain, ptrBufferAttr);
}

void msgGraphicsThreadToQuit(void)
{
	pthread_cancel(thGraphics);
}

void waitForGraphicsThread(void)
{
	pthread_join(thGraphics, NULL);
}

void checkIfQuit(void)
{
	pthread_testcancel();
}

/*
static void cleanupGraphicsThread(void *arg)
{
	printf("Cleanup !!!\n");
	//
}
*/

//

/*
static char * itimerspec_dump(struct itimerspec *ts)
{
	static char buf[1024];

	snprintf(buf, sizeof(buf),
			"itimer: [ interval=%lu s %lu ns, next expire=%lu s %lu ns ]",
			ts->it_interval.tv_sec,
			ts->it_interval.tv_nsec,
			ts->it_value.tv_sec,
			ts->it_value.tv_nsec
		   );

	return (buf);
}
*/

void addScreenRefreshTimer(void)
{
	struct epoll_event epollEvent_ScrRefresh;

	int msec = 33; // timer fires after 33 msec ie 30 Hz or 30 FPS

	// Skip the fllowing two lines to make code work like a simple delay
	//event.events = EPOLLIN;
	//event.data.fd = 0; // stdin

	//

	fdTimerScrRefresh = timerfd_create(CLOCK_MONOTONIC, 0);
	if (-1 == fdTimerScrRefresh) 
	{
		printf("timerfd_create() failed: errno=%d\n", errno);
		exit(6);
	}
	printf("created timerfd %d\n", fdTimerScrRefresh);

	bzero(&ts, sizeof(ts));

	ts.it_value.tv_sec = msec / 1000;
	ts.it_value.tv_nsec = (msec % 1000) * 1000000;    
	ts.it_interval.tv_sec = msec / 1000;
	ts.it_interval.tv_nsec = (msec % 1000) * 1000000;   

	if (timerfd_settime(fdTimerScrRefresh, 0, &ts, NULL) < 0) {
		printf("timerfd_settime() failed: errno=%d\n", errno);
		close(fdTimerScrRefresh);
		exit(7);
	}
	
	//printf("set timerfd time=%s\n", itimerspec_dump(&ts));
	
	//

	epollEvent_ScrRefresh.events = EPOLLIN;
	epollEvent_ScrRefresh.data.fd = fdTimerScrRefresh; 
	if(-1 == epoll_ctl(epoll_fd, EPOLL_CTL_ADD, fdTimerScrRefresh, &epollEvent_ScrRefresh))
	{
		printf("epoll_ctl(ADD) failed: errno=%d\n", errno);
		close(epoll_fd);
		close(fdTimerScrRefresh);
		exit(8);
	}

	//

	printf("added timerfd to epoll set\n");
}

void addKeyBoardEvents(void)
{
	// Add input event for standard input 

	struct epoll_event epollEvent_KB;

	epollEvent_KB.events = EPOLLIN;
	epollEvent_KB.data.fd = STDIN_FILENO;
	if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, STDIN_FILENO, &epollEvent_KB) == -1)
	{
		printf("epoll_ctl(ADD) failed: errno=%d\n", errno);
		close(epoll_fd);
		exit(10);
	}	
}

void addMouseEvents(void)
{
	// Add input event for mouse

	if(0 < fdMouseTouch)
	{
		struct epoll_event epollEvent_Mouse;

		epollEvent_Mouse.events = EPOLLIN;
		epollEvent_Mouse.data.fd = fdMouseTouch;
		if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, fdMouseTouch, &epollEvent_Mouse) == -1)
		{
			printf("epoll_ctl(ADD) failed: errno=%d\n", errno);
			close(epoll_fd);
			exit(10);
		}	
	}
}

void addRedirEvents(void)
{
	//

	fd_input_read = open(FN_INPUT_READ, O_RDONLY | O_NONBLOCK);
	if(0 < fd_input_read)
	{
		printf("Opened  fd_input_read = %d\n", fd_input_read);
	}
	else
	{
		perror("Could not open fd_input_read");
	}

	// USe RDWR since, as WRONLY can only be used if the pipe is known to be aleady opened for reading
	fd_output_write = open(FN_OUTPUT_WRITE, O_RDWR | O_NONBLOCK);
	if(0 < fd_output_write)
	{			
		printf("Opened  fd_output_write = %d\n", fd_output_write);
	}
	else
	{
		perror("Could not open fd_output_write");
	}	

	//
	{
		struct epoll_event epollEvent_KB;

		epollEvent_KB.events = EPOLLIN;
		epollEvent_KB.data.fd = fd_input_read;
		if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, fd_input_read, &epollEvent_KB) == -1)
		{
			printf("epoll_ctl(ADD) failed: errno=%d\n", errno);
			close(epoll_fd);
			exit(10);
		}				
	}
}

void initEvents(void)
{
	if (pthread_mutex_init(&lockBackBuffer, NULL) != 0) 
	{ 
		printf("\n mutex init has failed\n"); 
		exit(9); 
	} 	

	epoll_fd = epoll_create(1);
	if(-1 == epoll_fd)
	{
		fprintf(stderr, "Failed to create epoll file descriptor\n");
		exit(5);
	}

	//

	// Init Timer
	addScreenRefreshTimer();
	
	addKeyBoardEvents();

	addMouseEvents();

	addRedirEvents();
}

//

void eventLoop()
{
	int i;
	int quit = 0;

	int event_count;
	struct epoll_event newEvents[MAX_EVENTS];

	unsigned char strRecd[128];

	while(0 == quit)
	{		
		//printf("In poll_wait ... \n");

		// Running epoll_wait with empty list works like a simple delay
		memset(&newEvents, 0, sizeof(newEvents));
		event_count = epoll_wait(epoll_fd, newEvents, MAX_EVENTS, -1);
		
		//printf("%d ready events\n", event_count);
		for(i = 0; i < event_count; i++)
		{
			if(newEvents[i].events & EPOLLIN )
			{
				// Read FDs in Event list to reset them and prevent repeat triggering
				
				if(STDIN_FILENO == newEvents[i].data.fd)
				{
					int j, n, nEvents;

					n = read(STDIN_FILENO, inputKeys, sizeof(inputKeys));
					if (n > 0) 
					{
						nEvents = n / sizeof(unsigned char);
						for (j = 0; j < nEvents; j++) 
						{
							printf("inputKeys[%d] = 0x%02X !!!\n", j, inputKeys[j]);

							// Quit if 'q' or 'Q' is pressed
							// if (inputKeys[j] == 'q' || inputKeys[j] == 'Q')
							if (FB_KEY_ESC == inputKeys[j])
							{
								printf("Quit !!!\n");
								msgGraphicsThreadToQuit();
								quit = 1;
							}
						}
					}
				}

				if(fdMouseTouch == newEvents[i].data.fd)
				{
					int j, n, nEvents;
					struct input_event* pSample;

					n = read(fdMouseTouch, evInputEvents, sizeof(evInputEvents));
					//printf("Sz input_event %ld\n", sizeof(struct input_event));
					//printf("Sz evInputEvents %ld\n", sizeof(evInputEvents));
					//printf("Received %d bytes\n", n);
					printf("Received %ld input_event\n", n / sizeof(struct input_event));
					if (n > 0) 
					{
						pSample = &evInputEvents[0];
						nEvents = n / sizeof(struct input_event);
						for (j = 0; j < nEvents; j++) 
						{
							// Mouse events

							if ((EV_KEY == pSample->type) && (BTN_LEFT == pSample->code)) 
							{
								if (pSample->value > 0)
								{
									//printf("Left mouse button pressed\n");
								}
								else
								{
									//printf("Left mouse button released\n");
								}
							}

							if ((EV_KEY == pSample->type) && (BTN_RIGHT == pSample->code)) 
							{
								if (pSample->value > 0)
								{
									//printf("Right mouse button pressed\n");
								}
								else
								{
									//printf("Right mouse button released\n");
								}
							}

							// Single-Touch screen events

							if ((EV_ABS == pSample->type) && (ABS_X == pSample->code)) 
							{
								//printf("ABS_X : %d\n", pSample->value);
							}

							if ((EV_ABS == pSample->type) && (ABS_Y == pSample->code)) 
							{
								//printf("ABS_Y : %d\n", pSample->value);
							}

							// Multi-Touch screen events

							if ((EV_ABS == pSample->type) && (ABS_MT_SLOT == pSample->code)) 
							{
								//printf("ABS_MT_SLOT : %d\n", pSample->value);
							}

							if ((EV_ABS == pSample->type) && (ABS_MT_POSITION_X == pSample->code)) 
							{
								//printf("ABS_MT_POSITION_X : %d\n", pSample->value);
							}

							if ((EV_ABS == pSample->type) && (ABS_MT_POSITION_Y == pSample->code)) 
							{
								//printf("ABS_MT_POSITION_Y : %d\n", pSample->value);
							}

							//

							pSample++;	
						}
					}
				}

				if(fdTimerScrRefresh == newEvents[i].data.fd)
				{
					int ret;
					uint64_t res;

					// Read and clear accumulated timer events
					ret = read(fdTimerScrRefresh, &res, sizeof(res));
					//printf("read() returned %d, res=%" PRIu64 "\n", ret, res);

					// Optional
					// Blank the buffer contents before draw operations
					// blankFrontBuffer();

					//printf( "RefreshEvent ...\n");

					if(true == bUpdate)
					{
						//
						lockSurface();
						//

						blitBackBufferToFrontBuffer();

						//
						unlockSurface();

						//
						bUpdate = false;
					}
					//                    
				}

				if(fd_input_read == newEvents[i].data.fd)
				{
					if(newEvents[i].events & EPOLLIN )
					{
						// Read FDs in Event list to reset them and prevent repeat triggering
						if(fd_input_read == newEvents[i].data.fd)
						{
							int n;

							do
							{
								memset(strRecd,0,sizeof(strRecd));
								n = read(fd_input_read, strRecd, sizeof(strRecd));
								if (n > 0) 
								{
									//printf("Recd [%d]: %s\n", n, strRecd);
									printf("%s", strRecd);
									//puts(strRecd);

									//

									// Add a line of text to the screen
									// ??
								}
							} while (n > 0);
						}
					}

					//printf("EPOLLHUP | EPOLLOUT | EPOLLPRI : %d\n", EPOLLHUP | EPOLLOUT | EPOLLPRI);
					//if(newEvents[i].events & (EPOLLHUP | EPOLLOUT | EPOLLPRI))
					if(newEvents[i].events & (EPOLLHUP | EPOLLPRI))
					{
						quit = 1;
					}
				}
			}
		}

		//
		// Additional drawing code

		//
		//SDL_RenderPresent(frontRenderer);

		//Update screen
		updateFrontBuffer();
	}	
}

void deInitEvents()
{
	//

	// send EOF so child can continue (child blocks until all input has been processed):
	close(fd_input_read);

	close(fd_output_write);

	//

	if(-1 == close(fdTimerScrRefresh)) 
	{
		fprintf(stderr, "Failed to close timerfd: errno=%d\n", errno);
		exit(8);
	}

	if(-1 == close(epoll_fd))
	{
		fprintf(stderr, "Failed to close epoll file descriptor: errno=%d\n", errno);
		exit(8);
	}

	pthread_mutex_destroy(&lockBackBuffer); 
}

void restoreTerminal()
{
	tcsetattr(STDIN_FILENO, TCSAFLUSH, &prev_tty_cfg);
}

/////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
	stBufferAttr tBA;

	if(2 > argc) 
	{
		printf("Mouse/Touch input device not specified !\n");
		fdMouseTouch = -1;	
	} 
	else if(2 == argc)
	{
		printf("Mouse/Touch input device : %s\n", argv[1]);
		fdMouseTouch = open(argv[1], O_RDONLY | O_NOCTTY | O_NONBLOCK);
		printf("Mouse/Touch fd : %d\n", fdMouseTouch);
	}
	else 
	{

	}

	// Init Terminal Settings
	initTerminal();

	// Init Graphics
	initGraphics(&tBA);

	// Blank the buffer contents before draw operations
	blankFrontBuffer();

	// Init Events
	initEvents();

	// Launch Thread
	launchGraphicsThread(&tBA);

	// Event Loop
	eventLoop();

	// Wait for end of Graphics thread
	waitForGraphicsThread();

	// DeInit Events
	deInitEvents();

	// DeInit Graphics
	deInitGraphics();	

	// Restore Terminal Settings
	restoreTerminal();

	printf("Program done !!!\r\n");

	return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////

int graphicsMain( void* ptrData )
{
	stBufferAttr* ptrBufferAttr;

	//
	
	int bytes_per_pixel, nPitch;	
	unsigned char *pBackBuffer;
	int nWidth, nHeight;

	//

	unsigned char *pDstData;
	unsigned char *pDstLine;
	unsigned char *pSrcData;
	unsigned char *pSrcLine;

	unsigned char r, g ,b;
	unsigned int w, h, i, j, x, y, start, end; 
	struct timeval tv;

	int posx, posy;

	posx = 0;
	posy = 0;

	//

	ptrBufferAttr = (stBufferAttr*) ptrData ;

	if(NULL == ptrBufferAttr)
	{
		return -1;
	}

	if((NULL == ptrBufferAttr->pixels) || (0 == ptrBufferAttr->BytesPerPixel))
	{
		return -2;
	}

	if(0 == ptrBufferAttr->pitch)
	{
		return -3;
	}

	//

	bytes_per_pixel = ptrBufferAttr->BytesPerPixel;
	pBackBuffer = ptrBufferAttr->pixels;
	nPitch = ptrBufferAttr->pitch;
	nWidth = ptrBufferAttr->width;
	nHeight = ptrBufferAttr->height;

	//

	//

	// Blank the buffer contents before draw operations
	blankBuffer(pBackBuffer, nHeight*nPitch);

	//

	{
		int x,y,n;

		const char* fnImage = "img/myanimal.png";
		unsigned char *im_data;

		unsigned char *resized_data;
		
		int new_x;
		int new_y;		
		uint32_t red_len;
		uint32_t blue_len;
		uint32_t green_len;
		uint32_t transp_len;
		uint32_t *pixel;
		size_t line_len;
		struct rgba *rgba;	
		int len;	
		int nReqChannels;

		nReqChannels = 4;
		im_data = stbi_load(fnImage, &x, &y, &n, nReqChannels);
		len = nReqChannels * y;

		if (NULL != im_data)
		{
			if ((nWidth >= x) && (nHeight >= y))
			{
				printf("Avoiding resize !\n");
				resized_data = im_data;
			}
			else 
			{
				printf("Resizing !\n");
				resized_data = malloc(len);

				// shrink the image and keep the ratio 
				if (x > y) 
				{
					new_x = (int) nWidth;
					new_y = (int) roundf((float) (y * nWidth / x));
				}
				else 
				{
					new_y = (int) nHeight;
					new_x = (int) roundf((float) (x * nHeight / y));
				}

				if (1 != stbir_resize_uint8_linear(im_data, x, y, 0, resized_data, new_x, new_y, 0, n))
					goto free_resized;

				x = new_x;
				y = new_y;
			}

			// convert the pixels from RGBA to the framebuffer format 
			/*
			red_len = 0; //8 - var.red.length;
			green_len = 0; //8 - var.green.length;
			blue_len = 0; //8 - var.blue.length;
			transp_len = 0; //8 - var.transp.length;

			for (pixel = (uint32_t *) resized_data;  (uint32_t *) (resized_data + (x * y * n)) > pixel;  ++pixel) 
			{
				rgba = (struct rgba *) pixel;

				*pixel = 
					((rgba->r >> red_len) << var.red.offset) | 
					((rgba->g >> green_len) << var.green.offset) |
					((rgba->b >> blue_len) << var.blue.offset) |
					((rgba->a >> transp_len) << var.transp.offset);				
			}
			*/

			//printf("x = %d\n", x);
			//printf("y = %d\n", y);
			//printf("n = %d\n", n);
			line_len = (size_t) (x * nReqChannels);
			//printf("line_len = %d\n", line_len);
			pDstLine = pBackBuffer;
			pSrcLine = resized_data;

			for (int m =0; m < y; m++) 
			{
				(void) memcpy((void *) pDstLine, (void *) pSrcLine, line_len);

				pDstLine += nPitch;
				pSrcLine += line_len;
			}

			stbi_image_free((void *) im_data);

		free_resized:
			if (im_data != resized_data)
				free(resized_data);
		}
	}

	{
		long size;
		unsigned char* fontBuffer;	

		const char* fnFont = "font/FiraCode-Regular.ttf";
		//const char* fnFont = "font/FiraCode-Medium.ttf";
		//const char* fnFont = "font/Hack-Regular.ttf";
		//const char* fnFont = "font/Cascadia.ttf";
		//const char* fnFont = "font/Monda-Regular.ttf";		
		//const char* fnFont = "font/VictorMono-Regular.ttf";	

		FILE* fontFile = fopen(fnFont, "rb");	

		fseek(fontFile, 0, SEEK_END);
		size = ftell(fontFile); /* how long is the file ? */
		fseek(fontFile, 0, SEEK_SET); /* reset */

		fontBuffer = malloc(size);

		fread(fontBuffer, size, 1, fontFile);
		fclose(fontFile);		

		stbtt_fontinfo info;
		if (!stbtt_InitFont(&info, fontBuffer, 0))
		{
			printf("failed\n");
		}

		int b_w = 720; /* bitmap width */
		int b_h = 22; /* bitmap height */
		int l_h = 20; //24; /* line height */

		/* create a bitmap for the phrase */
		unsigned char* bitmap = calloc(b_w * b_h, sizeof(unsigned char));

		/* calculate font scaling */
		float scale = stbtt_ScaleForPixelHeight(&info, l_h);

		//char* word = "Now that is what I call ... Cheese !!!";
		char* word = "abcdefghijklmnopqrstuvwxyz ABCDEFGHIJKLMNOPQRSTUVWXYZ 0123456789";

		int x = 0;
			
		int ascent, descent, lineGap;
		stbtt_GetFontVMetrics(&info, &ascent, &descent, &lineGap);

		ascent = roundf(ascent * scale);
		descent = roundf(descent * scale);

		int i;
		for (i = 0; i < strlen(word); ++i)
		{
			/* how wide is this character */
			int ax;
			int lsb;

			stbtt_GetCodepointHMetrics(&info, word[i], &ax, &lsb);
			/* (Note that each Codepoint call has an alternative Glyph version which caches the work required to lookup the character word[i].) */

			/* get bounding box for character (may be offset to account for chars that dip above or below the line) */
			int c_x1, c_y1, c_x2, c_y2;
			stbtt_GetCodepointBitmapBox(&info, word[i], scale, scale, &c_x1, &c_y1, &c_x2, &c_y2);
			
			/* compute y (different characters have different heights) */
			int y = ascent + c_y1;
			
			/* render character (stride and offset is important here) */
			int byteOffset = x + roundf(lsb * scale) + (y * b_w);
			stbtt_MakeCodepointBitmap(&info, bitmap + byteOffset, c_x2 - c_x1, c_y2 - c_y1, b_w, scale, scale, word[i]);

			/* advance x */
			x += roundf(ax * scale);
			
			/* add kerning */
			int kern;
			kern = stbtt_GetCodepointKernAdvance(&info, word[i], word[i + 1]);
			x += roundf(kern * scale);
		}

		/*
			Note that this example writes each character directly into the target image buffer.
			The "right thing" to do for fonts that have overlapping characters is
			MakeCodepointBitmap to a temporary buffer and then alpha blend that onto the target image.
			See the stb_truetype.h header for more info.
		*/

		//

		unsigned char fr = 64;
		unsigned char fg = 192;
		unsigned char fb = 240;
		unsigned char fc = 0;

		int nReqChannels = 1;
		int line_len = (size_t) (x * nReqChannels);
		//printf("line_len = %d\n", line_len);
		pDstLine = pBackBuffer;
		pSrcLine = bitmap;
		pSrcData = pSrcLine;

		for (int m =0; m < b_h; m++) 
		{
			//(void) memcpy((void *) pDstLine, (void *) pSrcLine, line_len);

			pDstData = pDstLine;
			

			for (int n =0; n < b_w; n++) 
			{
				fc = *pSrcData;

				*pDstData = (unsigned char) ((fr * fc)>>8);
				pDstData++;

				*pDstData = (unsigned char) ((fg * fc)>>8);
				pDstData++;

				*pDstData = (unsigned char) ((fb * fc)>>8);
				pDstData++;

				*pDstData = 255;//a;
				pDstData++;

				pSrcData++;
			}

			pDstLine += nPitch;
			//pSrcLine += line_len;
		}		

		//

		free(fontBuffer);
		free(bitmap);	

		//

		bUpdate = true;	
	}

	//

	printf("graphicsMain Done !!!\r\n");	

	//

	return 0;
}

