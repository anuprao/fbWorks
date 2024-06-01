// fbWorks - Boilerplate code to get framebuffer graphics with working keyboard and mouse/touch
// Author: Anup Jayapal Rao, 2024

// License: Apache License

// SDL based Emulation Build
// 
// gcc -o fbworks_SDL -D BACKEND_SDL -D SCALED_OUTPUT fbworks.c -I /usr/include/SDL2 -l SDL2

/////////////////////////////////////////////////////////////////////////////////////////

// FrameBuffer Build
// 
// export PATH=/mnt/workspace/sdks/armv7l-linux-musleabihf-cross/bin:$PATH 
// 
// armv7l-linux-musleabihf-gcc -o fbworks_fb -D BACKEND_FB fbworks.c
// 
// Output binary size if 18KB
// requires replacing the ld-musl-armhf.so.1 in the /lib (in rfs,or,initrd)
// You may need to make the ld-musl-armhf.so.1 a symbolic link to 
// libc.so from the toolchain's "sdks/armv7l-linux-musleabihf-cross/armv7l-linux-musleabihf/lib" folder
// The libc.so needs to be copied to the /lib folder (in rfs,or,initrd)

// or,

// export PATH=$PATH:/mnt/workspace/sdks/linaro_7_hf/toolchain/bin
//
// arm-linux-gnueabihf-gcc -o fbworks_fb -D BACKEND_FB fbworks.c --static -lpthread
//
// Output binary size if 4.6 MB

#include <errno.h>
#include <inttypes.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
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

//

#include <pthread.h>

//

#define FB_WIDTH  720
#define FB_HEIGHT 1280

#define MAX_EVENTS 5
#define READ_SIZE 10

#define FB_KEY_ESC	0x1B

//

struct termios prev_tty_cfg;

int epoll_fd;

int fdTimerScrRefresh;
struct itimerspec ts;

unsigned char inputKeys[16];

int fdMouseTouch;
struct input_event  evInputEvents[16];

pthread_t thGraphics;

pthread_mutex_t lockBackBuffer;

//

typedef struct stBufferAttr {
	int BytesPerPixel;
	unsigned char *pixels;
	int pitch;
	int width;
	int height;	
} stBufferAttr;

int graphicsMain( void* ptrData );

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
	SDL_BlitSurface(backSurface, NULL, frontSurface, NULL);
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

	frontWindow = SDL_CreateWindow("fbWorks", -1, -1, width, height, flags);
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

static void cleanupGraphicsThread(void *arg)
{
	printf("Cleanup !!!\n");
	//
}

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

	// Init Timer
	addScreenRefreshTimer();
	
	addKeyBoardEvents();

	addMouseEvents();
}

//

void eventLoop()
{
	int quit = 0;

	int event_count;
	struct epoll_event newEvents[MAX_EVENTS];
	
	int i;
	size_t bytes_read;
	char read_buffer[READ_SIZE + 1];

	while(0 == quit)
	{		
		//printf("In poll_wait ... \n");
		// Running epoll_wait with empty list works like a simple delay
		memset(&newEvents, 0, sizeof(newEvents));
		event_count = epoll_wait(epoll_fd, newEvents, MAX_EVENTS, -1);
		
		//printf("%d ready events\n", event_count);
		for(i = 0; i < event_count; i++)
		{
			if(newEvents[i].events && EPOLLIN )
			{
				// Read FDs in Event list to reset them and prevent repeat triggering
				//printf("Reading file descriptor '%d' -- ", newEvents[i].data.fd);
				//bytes_read = read(newEvents[i].data.fd, read_buffer, READ_SIZE);
				
				if(STDIN_FILENO == newEvents[i].data.fd)
				{
					int i, n;

					n = read(STDIN_FILENO, inputKeys, sizeof(inputKeys));
					if (n > 0) 
					{
						for (i = 0; i < n; i++) 
						{
							printf("inputKeys[%d] = 0x%02X !!!\n", i, inputKeys[i]);

							// Quit if 'q' or 'Q' is pressed
							// if (inputKeys[i] == 'q' || inputKeys[i] == 'Q')
							if (FB_KEY_ESC == inputKeys[i])
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
					int i, n;
					struct input_event* pSample;

					n = read(fdMouseTouch, evInputEvents, sizeof(evInputEvents));
					if (n > 0) 
					{
						pSample = &evInputEvents[0];
						for (i = 0; i < n; i++) 
						{
							//printf("keys[%d] = 0x%02X !!!\n", i, keys[i]);

							// Mouse events

							if ((EV_KEY == pSample->type) && (BTN_LEFT == pSample->code)) 
							{
								if (pSample->value > 0)
								{
									printf("Left mouse button pressed\n");
								}
								else
								{
									printf("Left mouse button released\n");
								}
							}

							if ((EV_KEY == pSample->type) && (BTN_RIGHT == pSample->code)) 
							{
								if (pSample->value > 0)
								{
									printf("Right mouse button pressed\n");
								}
								else
								{
									printf("Right mouse button released\n");
								}
							}

							// Single-Touch screen events

							if ((EV_ABS == pSample->type) && (ABS_X == pSample->code)) 
							{
								printf("ABS_X : %d\n", pSample->value);
							}

							if ((EV_ABS == pSample->type) && (ABS_Y == pSample->code)) 
							{
								printf("ABS_Y : %d\n", pSample->value);
							}

							// Multi-Touch screen events

							if ((EV_ABS == pSample->type) && (ABS_MT_SLOT == pSample->code)) 
							{
								printf("ABS_MT_SLOT : %d\n", pSample->value);
							}

							if ((EV_ABS == pSample->type) && (ABS_MT_POSITION_X == pSample->code)) 
							{
								printf("ABS_MT_POSITION_X : %d\n", pSample->value);
							}

							if ((EV_ABS == pSample->type) && (ABS_MT_POSITION_Y == pSample->code)) 
							{
								printf("ABS_MT_POSITION_Y : %d\n", pSample->value);
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

					//
					lockSurface();
					//

					blitBackBufferToFrontBuffer();

					//
					unlockSurface();
					//                    
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

int graphicsMain( void* ptrData )
{
	stBufferAttr* ptrBufferAttr;

	//
	
	int bytes_per_pixel, nPitch;	
	unsigned char *buffer;
	int width, height;

	//

	unsigned char *data;
	unsigned char *pline;

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
	buffer = ptrBufferAttr->pixels;
	nPitch = ptrBufferAttr->pitch;
	width = ptrBufferAttr->width;
	height = ptrBufferAttr->height;

	//

	//

	// Blank the buffer contents before draw operations
	blankBuffer(buffer, height*nPitch);

	//

	w = width >> 2;
	h = height >> 2;

	gettimeofday(&tv, NULL);
	start = tv.tv_sec * 1000 + tv.tv_usec / 1000;

	do
	{
		checkIfQuit();

		r = rand() % 256;
		g = rand() % 256;
		b = rand() % 256;

		x = rand() % (width - w);
		y = rand() % (height - h);

		data = buffer + (y + posy) * (nPitch) + (x + posx) * (bytes_per_pixel);

		//
		lockSurface();
		//

		for (i = 0; i < h; i++) 
		{
			pline = data;
			for (j = 0; j < w; j++) 
			{
				*pline = (unsigned char)r;
				pline++;

				*pline = (unsigned char)g;
				pline++;

				*pline = (unsigned char)b;
				pline++;

				*pline = 255;//a;
				pline++;
			}

			data += nPitch;
		}

		//
		unlockSurface();
		//

		gettimeofday(&tv, NULL);
		end = tv.tv_sec * 1000 + tv.tv_usec / 1000;

		//
		//sleep(1);
		//sleep(0.1);	
		sleep(0.2);	

	} while (end < (start + 50000));

	printf("graphicsMain Done !!!\r\n");	

	//

	return 0;
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

